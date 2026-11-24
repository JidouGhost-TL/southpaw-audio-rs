//! Zero-allocation parser for Southpaw 16-byte `SouthpawSampleDesc` sample descriptors
//! and contiguous sample tables.

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use crate::error::{Error, Result};
use crate::freq::timer_reload_to_hz;

/// GBA ROM bus window base address (`0x0800_0000`).
pub const GBA_ROM_BUS_BASE: u32 = 0x0800_0000;

/// Size of a Southpaw `SouthpawSampleDesc` descriptor in bytes (`0x10`).
pub const SOUTHPAW_DESC_SIZE: usize = 16;

/// Size of one GBA DirectSound DMA/FIFO block in bytes (`4` 32-bit words).
pub const DMA_BLOCK_BYTES: usize = 16;

/// Expected upper 16 bits of `timer0_cnt_reload` (`TMxCNT_H = 0x0080`, Timer enable).
pub const TIMER_ENABLE_HI16: u32 = 0x0080;

/// Minimum accepted `num_blocks` value when scanning ROM tables (`16` bytes).
pub const MIN_SCAN_BLOCKS: u32 = 1;

/// Maximum accepted `num_blocks` value when scanning ROM tables (`256 KiB`).
pub const MAX_SCAN_BLOCKS: u32 = 0x4000;

/// Minimum accepted Timer0 reload value (`0xE000`, `~2,048 Hz`).
pub const MIN_SCAN_TIMER_RELOAD: u32 = 0xE000;

/// Maximum accepted Timer0 reload value (`0xFE80`, `~43,690 Hz`).
pub const MAX_SCAN_TIMER_RELOAD: u32 = 0xFE80;

/// Minimum consecutive valid descriptors required for a ROM sample table match.
pub const MIN_TABLE_ENTRIES: usize = 5;

/// Parsed 16-byte Southpaw GBA sample descriptor (`SouthpawSampleDesc`).
///
/// Binary layout (little-endian):
/// - `+0x00`: `sample_ptr: u32` — GBA ROM pointer (`0x08xxxxxx`) to raw signed 8-bit PCM (`i8`).
/// - `+0x04`: `num_blocks: u32` — Length in 16-byte DMA/FIFO blocks (`bytes = num_blocks * 16`).
/// - `+0x08`: `timer0_cnt_reload: u32` — Packed Timer control (`hi16 = 0x0080`) + reload (`lo16`).
/// - `+0x0C`: `volume_mask: i32` — Hardware DirectSound gain mask: `0` = 50% DirectSound gain
///   (`SOUNDCNT_H` bit cleared), `-1` (`0xFFFF_FFFF`) = 100% DirectSound gain (`SOUNDCNT_H` bit 2/3 set).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SouthpawSampleDesc {
    /// GBA ROM bus pointer (`0x08xxxxxx`) to the signed 8-bit PCM payload.
    pub sample_ptr: u32,
    /// Sample length in 16-byte DirectSound DMA/FIFO blocks.
    pub num_blocks: u32,
    /// Packed GBA Timer control (`hi16 = 0x0080`) and 16-bit reload (`lo16`).
    pub timer0_cnt_reload: u32,
    /// Hardware DirectSound gain mask (`0` = 50% gain, `-1` = 100% gain).
    pub volume_mask: i32,
}

impl SouthpawSampleDesc {
    /// Parse a 16-byte `SouthpawSampleDesc` from `data[..16]`.
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < SOUTHPAW_DESC_SIZE {
            return Err(Error::UnexpectedEof);
        }
        let sample_ptr = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let num_blocks = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let timer0_cnt_reload = u32::from_le_bytes([data[8], data[9], data[10], data[11]]);
        let volume_mask = i32::from_le_bytes([data[12], data[13], data[14], data[15]]);

        if sample_ptr < GBA_ROM_BUS_BASE {
            return Err(Error::InvalidHeader("sample_ptr below GBA ROM bus base"));
        }
        if num_blocks == 0 {
            return Err(Error::InvalidHeader("num_blocks must be non-zero"));
        }
        if (timer0_cnt_reload >> 16) != TIMER_ENABLE_HI16 {
            return Err(Error::InvalidHeader("timer control hi16 != 0x0080"));
        }

        Ok(Self {
            sample_ptr,
            num_blocks,
            timer0_cnt_reload,
            volume_mask,
        })
    }

    /// Returns the ROM file offset of the PCM payload (`sample_ptr - 0x0800_0000`).
    #[must_use]
    #[inline]
    pub const fn rom_offset(self) -> usize {
        self.sample_ptr.saturating_sub(GBA_ROM_BUS_BASE) as usize
    }

    /// Returns the PCM payload length in bytes (`num_blocks * 16`).
    #[must_use]
    #[inline]
    pub const fn byte_len(self) -> usize {
        (self.num_blocks as usize).saturating_mul(DMA_BLOCK_BYTES)
    }

    /// Returns the native playback sample rate in Hz derived from `timer0_cnt_reload`.
    #[must_use]
    #[inline]
    pub const fn sample_rate_hz(self) -> u32 {
        timer_reload_to_hz(self.timer0_cnt_reload)
    }

    /// Returns `true` if `volume_mask` selects 100% GBA DirectSound hardware gain
    /// (`(volume_mask & 8) != 0` / `volume_mask == -1`), or `false` for 50% gain.
    #[must_use]
    #[inline]
    pub const fn is_full_volume(self) -> bool {
        (self.volume_mask & 0x0C) != 0
    }

    /// Returns `true` if this descriptor passes strict ROM table scanner bounds.
    #[must_use]
    #[inline]
    pub const fn is_valid_for_rom(self, rom_len: usize) -> bool {
        let off = self.rom_offset();
        let len = self.byte_len();
        let reload = self.timer0_cnt_reload & 0xFFFF;
        self.sample_ptr >= GBA_ROM_BUS_BASE
            && self.num_blocks >= MIN_SCAN_BLOCKS
            && self.num_blocks <= MAX_SCAN_BLOCKS
            && (self.timer0_cnt_reload >> 16) == TIMER_ENABLE_HI16
            && reload >= MIN_SCAN_TIMER_RELOAD
            && reload <= MAX_SCAN_TIMER_RELOAD
            && (self.volume_mask == 0 || self.volume_mask == -1)
            && off < rom_len
            && len <= rom_len - off
    }
}

/// Zero-copy borrowed view of a Southpaw signed 8-bit PCM sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SouthpawSampleView<'a> {
    /// Sample index within the descriptor table.
    pub index: u32,
    /// Native sample rate in Hz.
    pub sample_rate_hz: u32,
    /// Optional loop start byte offset (`Some(start)` for looped playback, `None` for one-shot).
    pub loop_start: Option<u32>,
    /// Whether the descriptor enables 100% GBA DirectSound hardware gain (`false` = 50%).
    pub full_volume: bool,
    /// Raw signed 8-bit linear mono PCM bytes (`i8` reinterpreted as `u8`).
    pub pcm: &'a [u8],
}

impl<'a> SouthpawSampleView<'a> {
    /// Construct a `SouthpawSampleView` directly from raw signed 8-bit PCM bytes.
    #[must_use]
    #[inline]
    pub const fn from_pcm_bytes(
        index: u32,
        sample_rate_hz: u32,
        loop_start: Option<u32>,
        full_volume: bool,
        pcm: &'a [u8],
    ) -> Self {
        Self {
            index,
            sample_rate_hz,
            loop_start,
            full_volume,
            pcm,
        }
    }

    /// Resolve a `SouthpawSampleDesc` against a ROM slice into a zero-copy sample view.
    pub fn from_desc(index: u32, desc: SouthpawSampleDesc, rom: &'a [u8]) -> Result<Self> {
        let start = desc.rom_offset();
        let len = desc.byte_len();
        let end = start.checked_add(len).ok_or(Error::UnexpectedEof)?;
        if end > rom.len() {
            return Err(Error::UnexpectedEof);
        }
        Ok(Self {
            index,
            sample_rate_hz: desc.sample_rate_hz(),
            loop_start: None,
            full_volume: desc.is_full_volume(),
            pcm: &rom[start..end],
        })
    }

    /// Returns the sample length in PCM8 frames/bytes.
    #[must_use]
    #[inline]
    pub const fn len(&self) -> usize {
        self.pcm.len()
    }

    /// Returns `true` if the PCM payload is empty.
    #[must_use]
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.pcm.is_empty()
    }
}

/// Owned Southpaw signed 8-bit PCM sample (requires `alloc` feature).
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SouthpawSample {
    /// Sample index within the descriptor table.
    pub index: u32,
    /// Native sample rate in Hz.
    pub sample_rate_hz: u32,
    /// Optional loop start byte offset.
    pub loop_start: Option<u32>,
    /// Whether 100% hardware DirectSound gain is enabled.
    pub full_volume: bool,
    /// Owned signed 8-bit linear mono PCM bytes.
    pub pcm: Vec<u8>,
}

#[cfg(feature = "alloc")]
impl SouthpawSample {
    /// Convert a borrowed [`SouthpawSampleView`] into an owned [`SouthpawSample`].
    #[must_use]
    pub fn from_view(view: &SouthpawSampleView<'_>) -> Self {
        Self {
            index: view.index,
            sample_rate_hz: view.sample_rate_hz,
            loop_start: view.loop_start,
            full_volume: view.full_volume,
            pcm: view.pcm.to_vec(),
        }
    }

    /// Borrow this owned sample as a [`SouthpawSampleView`].
    #[must_use]
    pub fn as_view(&self) -> SouthpawSampleView<'_> {
        SouthpawSampleView {
            index: self.index,
            sample_rate_hz: self.sample_rate_hz,
            loop_start: self.loop_start,
            full_volume: self.full_volume,
            pcm: &self.pcm,
        }
    }
}

/// Locate the primary contiguous table of 16-byte `SouthpawSampleDesc` entries in a GBA ROM.
///
/// Returns `Some((table_rom_offset, entry_count))` for the longest contiguous valid run
/// with at least [`MIN_TABLE_ENTRIES`] (5) entries, or `None` if no table is found.
#[must_use]
pub fn find_sample_table(rom: &[u8]) -> Option<(usize, usize)> {
    let rlen = rom.len();
    let min_bytes = SOUTHPAW_DESC_SIZE * MIN_TABLE_ENTRIES;
    if rlen < min_bytes {
        return None;
    }

    let mut best_off = 0usize;
    let mut best_count = 0usize;
    let mut off = 0usize;

    while off + min_bytes <= rlen {
        if let Ok(desc) = SouthpawSampleDesc::parse(&rom[off..off + SOUTHPAW_DESC_SIZE]) {
            if desc.is_valid_for_rom(rlen) {
                let mut count = 1usize;
                let mut cur = off + SOUTHPAW_DESC_SIZE;
                while cur + SOUTHPAW_DESC_SIZE <= rlen {
                    match SouthpawSampleDesc::parse(&rom[cur..cur + SOUTHPAW_DESC_SIZE]) {
                        Ok(next) if next.is_valid_for_rom(rlen) => {
                            count += 1;
                            cur += SOUTHPAW_DESC_SIZE;
                        }
                        _ => break,
                    }
                }
                if count > best_count {
                    best_off = off;
                    best_count = count;
                }
                off = cur;
                continue;
            }
        }
        off += 4;
    }

    if best_count >= MIN_TABLE_ENTRIES {
        Some((best_off, best_count))
    } else {
        None
    }
}

/// Scan a GBA ROM for its primary Southpaw sample table and return zero-copy views of all entries.
#[cfg(feature = "alloc")]
#[must_use]
pub fn scan_sample_table(rom: &[u8]) -> Vec<SouthpawSampleView<'_>> {
    let Some((table_off, count)) = find_sample_table(rom) else {
        return Vec::new();
    };
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let off = table_off + i * SOUTHPAW_DESC_SIZE;
        if let Ok(desc) = SouthpawSampleDesc::parse(&rom[off..off + SOUTHPAW_DESC_SIZE]) {
            if let Ok(view) = SouthpawSampleView::from_desc(i as u32, desc, rom) {
                out.push(view);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_finds_synthetic_sample_table() {
        let mut rom = [0u8; 0x1000];
        for i in 0..6usize {
            let off = 0x100 + i * 16;
            let ptr = GBA_ROM_BUS_BASE + 0x400 + (i as u32) * 64;
            rom[off..off + 4].copy_from_slice(&ptr.to_le_bytes());
            rom[off + 4..off + 8].copy_from_slice(&4u32.to_le_bytes());
            rom[off + 8..off + 12].copy_from_slice(&0x0080_FC15u32.to_le_bytes());
            let mask: i32 = if i % 2 == 0 { 0 } else { -1 };
            rom[off + 12..off + 16].copy_from_slice(&mask.to_le_bytes());
        }
        assert_eq!(find_sample_table(&rom), Some((0x100, 6)));
        let d0 = SouthpawSampleDesc::parse(&rom[0x100..0x110]).unwrap();
        assert_eq!(d0.byte_len(), 64);
        assert_eq!(d0.sample_rate_hz(), 16_727);
        assert!(!d0.is_full_volume());
        let d1 = SouthpawSampleDesc::parse(&rom[0x110..0x120]).unwrap();
        assert!(d1.is_full_volume());
    }
}
