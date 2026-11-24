//! Voice, 5-frame echo ring-buffer, and Dual-DMA hardware register state types
//! for the Southpaw GBA Sound Engine.

use crate::version::{SouthpawVersion, SOUTHPAW_DMA_IRQ_CNT};

/// Number of frame buffers in the Southpaw Echo-Mixer ring buffer (`~83.7 ms` delay at 59.73 Hz).
pub const ECHO_RING_SLOTS: usize = 5;

/// Frame length in samples for `SouthpawVersion::EchoMixer10512` (`0xB0` = 176 samples @ 10,512 Hz).
pub const ECHO_10512_FRAME_SAMPLES: usize = 176;

/// Frame length in samples for `SouthpawVersion::EchoMixer6690` (`0x70` = 112 samples @ 6,690 Hz).
pub const ECHO_6690_FRAME_SAMPLES: usize = 112;

/// 24.8 fixed-point resampled voice state for the Southpaw Thumb Echo-Mixer
/// (`0x0300055C` in *Extreme Ghostbusters* and *Antz: Extreme Racing*).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EchoVoice {
    /// 24.8 fixed-point playback step per output sample (`0` = voice inactive/stopped).
    pub step_q8: u32,
    /// Current 24.8 fixed-point playback cursor (`byte_index = pos_q8 >> 8`).
    pub pos_q8: u32,
    /// End-of-sample 24.8 fixed-point limit (`sample_len_bytes << 8`).
    pub end_q8: u32,
    /// Loop start 24.8 fixed-point position (`0` = one-shot, non-zero = loop to `loop_start_q8 + overshoot`).
    pub loop_start_q8: u32,
}

impl EchoVoice {
    /// Create a new [`EchoVoice`] from raw 24.8 fixed-point parameters.
    #[must_use]
    #[inline]
    pub const fn new(step_q8: u32, pos_q8: u32, end_q8: u32, loop_start_q8: u32) -> Self {
        Self {
            step_q8,
            pos_q8,
            end_q8,
            loop_start_q8,
        }
    }

    /// Create an active [`EchoVoice`] from byte lengths and a 24.8 fixed-point step.
    ///
    /// Note: on Southpaw GBA hardware, `loop_start_q8 == 0` disables looping (one-shot),
    /// so passing `Some(0)` behaves identically to `None`.
    #[must_use]
    #[inline]
    pub const fn from_bytes(step_q8: u32, pcm_len: usize, loop_start_bytes: Option<usize>) -> Self {
        let loop_q8 = match loop_start_bytes {
            Some(s) => (s as u32) << 8,
            None => 0,
        };
        Self {
            step_q8,
            pos_q8: 0,
            end_q8: (pcm_len as u32) << 8,
            loop_start_q8: loop_q8,
        }
    }

    /// Returns `true` if the voice is currently active (`step_q8 != 0`).
    #[must_use]
    #[inline]
    pub const fn is_active(&self) -> bool {
        self.step_q8 != 0
    }
}

/// 5-slot ring buffer holding the Southpaw Echo-Mixer's delay line (`5 × N` PCM8 bytes).
///
/// Use `EchoRingBuffer<176>` for [`SouthpawVersion::EchoMixer10512`] and
/// `EchoRingBuffer<112>` for [`SouthpawVersion::EchoMixer6690`].
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct EchoRingBuffer<const N: usize> {
    /// Five consecutive frame buffers (`0x030000F8..0x03000478` in GBA IWRAM).
    pub slots: [[u8; N]; ECHO_RING_SLOTS],
    /// Current slot index (`0..=4`), matching `(*(u16*)0x03000558) / 4` on GBA hardware.
    pub slot_index: usize,
}

impl<const N: usize> Default for EchoRingBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> EchoRingBuffer<N> {
    /// Create a zero-initialized 5-slot echo ring buffer with `slot_index = 0`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: [[0u8; N]; ECHO_RING_SLOTS],
            slot_index: 0,
        }
    }

    /// Reset all 5 ring slots to silence and rewind `slot_index` to `0`.
    pub fn reset(&mut self) {
        for slot in &mut self.slots {
            slot.fill(0);
        }
        self.slot_index = 0;
    }

    /// Returns a slice of the most recently mixed frame slot.
    #[must_use]
    #[inline]
    pub fn current_slot(&self) -> &[u8; N] {
        &self.slots[self.slot_index % ECHO_RING_SLOTS]
    }
}

/// State of one hardware DirectSound DMA-IRQ channel (DMA1/FIFO A or DMA2/FIFO B).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct DirectSoundChannel {
    /// Source ROM pointer (`DMAxSAD`).
    pub dma_sad: u32,
    /// Destination FIFO address (`0x0400_00A0` for DMA1, `0x0400_00A4` for DMA2).
    pub dma_dad: u32,
    /// DMA control register (`0xF660_0004` when active, `0` when stopped).
    pub dma_cnt: u32,
    /// Hardware Timer control+reload word (`TM0CNT` or `TM1CNT`, `0` when stopped).
    pub timer_cnt: u32,
    /// Remaining 16-byte DMA/FIFO refill blocks before the IRQ handler stops playback.
    pub blocks_remaining: u32,
    /// Current byte offset within the sample payload (advances by 16 bytes per DMA IRQ).
    pub byte_cursor: usize,
    /// Whether this channel's DirectSound hardware gain is 100% (`true`) or 50% (`false`).
    pub full_volume: bool,
}

impl DirectSoundChannel {
    /// Returns `true` if DMA playback is currently active.
    #[must_use]
    #[inline]
    pub const fn is_active(&self) -> bool {
        self.dma_cnt == SOUTHPAW_DMA_IRQ_CNT && self.blocks_remaining > 0
    }
}

/// Complete hardware register & IRQ state for the Southpaw Dual-DMA DirectSound engine
/// (`DualDmaPsg50`, `DualDmaPsg100`, `DualDmaFixedRate`, and the DMA2 side of Echo-Mixer builds).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DualDmaHardwareState {
    /// Active Southpaw engine revision.
    pub version: SouthpawVersion,
    /// Cached DirectSound A/B volume bits (`bit 2` = DS-A 100%, `bit 3` = DS-B 100%).
    pub vol_bits: u16,
    /// Last `SOUNDCNT_H` (`0x0400_0082`) value written by the driver.
    pub soundcnt_h: u16,
    /// DirectSound A (`DMA1` + `Timer0`) hardware channel state.
    pub dma1: DirectSoundChannel,
    /// DirectSound B (`DMA2` + `Timer1`) hardware channel state.
    pub dma2: DirectSoundChannel,
}

impl DualDmaHardwareState {
    /// Create a freshly initialized [`DualDmaHardwareState`] for `version`.
    #[must_use]
    pub const fn new(version: SouthpawVersion) -> Self {
        Self {
            version,
            vol_bits: 0,
            soundcnt_h: 0,
            dma1: DirectSoundChannel {
                dma_sad: 0,
                dma_dad: 0x0400_00A0,
                dma_cnt: 0,
                timer_cnt: 0,
                blocks_remaining: 0,
                byte_cursor: 0,
                full_volume: false,
            },
            dma2: DirectSoundChannel {
                dma_sad: 0,
                dma_dad: 0x0400_00A4,
                dma_cnt: 0,
                timer_cnt: 0,
                blocks_remaining: 0,
                byte_cursor: 0,
                full_volume: false,
            },
        }
    }
}
