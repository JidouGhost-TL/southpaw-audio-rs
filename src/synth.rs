//! Dual synthesis pipelines for the Southpaw GBA Sound Engine:
//!
//! 1. [`AccuracySynth`]: 100% bit-exact emulation of all 5 Southpaw ARM7TDMI/Thumb revisions,
//!    covering both the 5-frame ring-buffer Echo-Mixer (`EchoMixer10512`, `EchoMixer6690`)
//!    and the Dual-DMA hardware DirectSound + DMA-IRQ block countdown engine
//!    (`DualDmaPsg50`, `DualDmaPsg100`, `DualDmaFixedRate`).
//! 2. [`StudioSynth`]: High-fidelity 48,000 Hz `f32` renderer with a compile-time 64-phase
//!    Lanczos-3 polyphase windowed-sinc LUT (plus general Lanczos-`a` fallback) and true
//!    equal-power stereo panning.

use crate::sample::{SouthpawSampleDesc, SouthpawSampleView, DMA_BLOCK_BYTES};
use crate::version::SOUTHPAW_DMA_IRQ_CNT;
use crate::voice::{
    DualDmaHardwareState, EchoRingBuffer, EchoVoice, ECHO_10512_FRAME_SAMPLES,
    ECHO_6690_FRAME_SAMPLES, ECHO_RING_SLOTS,
};

/// 100% bit-exact ARM7TDMI/Thumb software & hardware DirectSound emulator for all
/// 5 Southpaw GBA Sound Engine revisions.
pub struct AccuracySynth;

impl AccuracySynth {
    /// Compute the exact Southpaw Thumb 1/4-amplitude signed echo feedback byte
    /// from the ring buffer sample written 5 frames (`~83.7 ms`) earlier:
    ///
    /// ```text
    /// movs r2, #0x81
    /// adds r3, r3, r2
    /// lsrs r3, r3, #2
    /// movs r4, #0x3f
    /// ands r3, r4
    /// movs r2, #0x20
    /// subs r3, r3, r2
    /// ```
    #[must_use]
    #[inline]
    pub const fn echo_feedback_byte(prev: u8) -> i8 {
        let shifted = (((prev as u32) + 0x81) >> 2) & 0x3F;
        (shifted as i32 - 0x20) as i8
    }

    /// Mix one frame of `N` PCM8 samples through the Southpaw 5-frame ring-buffer
    /// Thumb Echo-Mixer (`EchoMixer10512` with `N = 176`, `EchoMixer6690` with `N = 112`).
    ///
    /// Matches the verbatim ARM7TDMI/Thumb routine (`AEGE 0x08001C58`, `ANUE 0x082DA670`)
    /// strictly byte-for-byte:
    /// 1. Advances `ring.slot_index` (`0 -> 1 -> 2 -> 3 -> 4 -> 0`).
    /// 2. For each sample `i in 0..N`:
    ///    - Computes `acc = echo_feedback_byte(slot[i])`.
    ///    - If `voice.step_q8 != 0`: advances `pos_q8 = pos_q8.wrapping_add(step_q8)`.
    ///      If `pos_q8 >= end_q8` (unsigned):
    ///      - If `loop_start_q8 == 0`: sets `voice.step_q8 = 0` (stops voice, skips sample add).
    ///      - Else: wraps `pos_q8 = loop_start_q8.wrapping_add(pos_q8.wrapping_sub(end_q8))`.
    ///    - Fetches signed `pcm[(pos_q8 >> 8) as usize] as i8` and adds with wrapping `i8`.
    ///    - Stores the resulting byte into `slot[i]` and `out[i]`.
    pub fn mix_echo_frame<const N: usize>(
        pcm: &[u8],
        voice: &mut EchoVoice,
        ring: &mut EchoRingBuffer<N>,
        out: &mut [i8; N],
    ) {
        // Advance ring index (matching `cmp r2, #0x10; bne +4; movs r2, #0`)
        ring.slot_index = if ring.slot_index + 1 >= ECHO_RING_SLOTS {
            0
        } else {
            ring.slot_index + 1
        };

        let slot = &mut ring.slots[ring.slot_index];
        for i in 0..N {
            let mut acc = Self::echo_feedback_byte(slot[i]);
            if voice.step_q8 != 0 {
                let mut next_pos = voice.pos_q8.wrapping_add(voice.step_q8);
                let mut active = true;
                if next_pos >= voice.end_q8 {
                    let overshoot = next_pos.wrapping_sub(voice.end_q8);
                    if voice.loop_start_q8 == 0 {
                        voice.step_q8 = 0;
                        active = false;
                    } else {
                        next_pos = voice.loop_start_q8.wrapping_add(overshoot);
                    }
                }
                if active {
                    voice.pos_q8 = next_pos;
                    let idx = (next_pos >> 8) as usize;
                    if idx < pcm.len() {
                        let sample = pcm[idx] as i8;
                        acc = acc.wrapping_add(sample);
                    }
                }
            }
            slot[i] = acc as u8;
            out[i] = acc;
        }
    }

    /// Convenience wrapper for `SouthpawVersion::EchoMixer10512` (`176` samples/frame @ 10,512 Hz).
    #[inline]
    pub fn mix_echo_10512_frame(
        pcm: &[u8],
        voice: &mut EchoVoice,
        ring: &mut EchoRingBuffer<ECHO_10512_FRAME_SAMPLES>,
        out: &mut [i8; ECHO_10512_FRAME_SAMPLES],
    ) {
        Self::mix_echo_frame(pcm, voice, ring, out);
    }

    /// Convenience wrapper for `SouthpawVersion::EchoMixer6690` (`112` samples/frame @ 6,690 Hz).
    #[inline]
    pub fn mix_echo_6690_frame(
        pcm: &[u8],
        voice: &mut EchoVoice,
        ring: &mut EchoRingBuffer<ECHO_6690_FRAME_SAMPLES>,
        out: &mut [i8; ECHO_6690_FRAME_SAMPLES],
    ) {
        Self::mix_echo_frame(pcm, voice, ring, out);
    }

    /// Trigger DirectSound A (`DMA1` + `Timer0`) using the exact Southpaw Dual-DMA
    /// register sequence (`ADME 0x0801E366`, `AGGE`, `ALLP`).
    ///
    /// Preserves the DirectSound B gain bit (`bit 3`) in `state.vol_bits`, updates
    /// the DirectSound A gain bit (`bit 2`) from `desc.volume_mask & 4`, writes
    /// `SOUNDCNT_H`, programs `DMA1SAD`/`DMA1CNT`/`TM0CNT`, and loads `blocks_remaining`.
    pub fn trigger_dma1(
        state: &mut DualDmaHardwareState,
        desc: SouthpawSampleDesc,
        note_timer_cnt: u32,
    ) {
        let Some(base_sc) = state.version.dma1_soundcnt_h_base() else {
            return;
        };
        let ds_a_bit = ((desc.volume_mask as u32) & 4) as u16;
        state.vol_bits = (state.vol_bits & 8) | ds_a_bit;
        state.soundcnt_h = base_sc.wrapping_add(state.vol_bits);

        let timer_cnt = if state.version.uses_fixed_descriptor_rate() {
            desc.timer0_cnt_reload
        } else {
            note_timer_cnt
        };

        state.dma1.dma_sad = desc.sample_ptr;
        state.dma1.dma_dad = 0x0400_00A0;
        state.dma1.dma_cnt = SOUTHPAW_DMA_IRQ_CNT;
        state.dma1.timer_cnt = timer_cnt;
        state.dma1.blocks_remaining = desc.num_blocks;
        state.dma1.byte_cursor = 0;
        state.dma1.full_volume = ds_a_bit != 0;
    }

    /// Trigger DirectSound B (`DMA2` + `Timer1`) using the exact Southpaw register sequence
    /// (shared across all 5 Southpaw revisions: Dual-DMA and Echo-Mixer Hybrid).
    ///
    /// For Dual-DMA revisions, preserves the DirectSound A gain bit (`bit 2`) in `state.vol_bits`
    /// and updates `bit 3` from `desc.volume_mask & 8`. For Echo-Mixer revisions, DirectSound A
    /// gain is fixed in `dma2_soundcnt_h_base()` and `bit 3` is added directly.
    pub fn trigger_dma2(
        state: &mut DualDmaHardwareState,
        desc: SouthpawSampleDesc,
        note_timer_cnt: u32,
    ) {
        let base_sc = state.version.dma2_soundcnt_h_base();
        let ds_b_bit = ((desc.volume_mask as u32) & 8) as u16;
        if state.version.is_echo_mixer() {
            state.vol_bits = ds_b_bit;
            state.soundcnt_h = base_sc.wrapping_add(ds_b_bit);
        } else {
            state.vol_bits = (state.vol_bits & 4) | ds_b_bit;
            state.soundcnt_h = base_sc.wrapping_add(state.vol_bits);
        }

        let timer_cnt = if state.version.uses_fixed_descriptor_rate() {
            desc.timer0_cnt_reload
        } else {
            note_timer_cnt
        };

        state.dma2.dma_sad = desc.sample_ptr;
        state.dma2.dma_dad = 0x0400_00A4;
        state.dma2.dma_cnt = SOUTHPAW_DMA_IRQ_CNT;
        state.dma2.timer_cnt = timer_cnt;
        state.dma2.blocks_remaining = desc.num_blocks;
        state.dma2.byte_cursor = 0;
        state.dma2.full_volume = ds_b_bit != 0;
    }

    /// Service one DirectSound A (`DMA1`) 16-byte block completion IRQ
    /// (verbatim match to `ADME 0x0801E808`, `AGGE 0x08002A04`, `ALLP 0x08020974`).
    ///
    /// Decrements `dma1.blocks_remaining` by 1 and advances `byte_cursor` by 16.
    /// When `blocks_remaining` reaches `0`, clears `dma1.dma_cnt` and `dma1.timer_cnt` to `0`.
    /// Returns `true` if the channel is still active after this IRQ.
    pub fn service_dma1_irq(state: &mut DualDmaHardwareState) -> bool {
        state.dma1.blocks_remaining = state.dma1.blocks_remaining.wrapping_sub(1);
        state.dma1.byte_cursor = state.dma1.byte_cursor.saturating_add(DMA_BLOCK_BYTES);
        if state.dma1.blocks_remaining == 0 {
            state.dma1.dma_cnt = 0;
            state.dma1.timer_cnt = 0;
            false
        } else {
            true
        }
    }

    /// Service one DirectSound B (`DMA2`) 16-byte block completion IRQ
    /// (verbatim match to `ADME 0x0801E828`, `AEGE 0x0800282C`, `ANUE 0x082DB244`).
    ///
    /// Decrements `dma2.blocks_remaining` by 1 and advances `byte_cursor` by 16.
    /// When `blocks_remaining` reaches `0`, clears `dma2.dma_cnt` and `dma2.timer_cnt` to `0`.
    /// Returns `true` if the channel is still active after this IRQ.
    pub fn service_dma2_irq(state: &mut DualDmaHardwareState) -> bool {
        state.dma2.blocks_remaining = state.dma2.blocks_remaining.wrapping_sub(1);
        state.dma2.byte_cursor = state.dma2.byte_cursor.saturating_add(DMA_BLOCK_BYTES);
        if state.dma2.blocks_remaining == 0 {
            state.dma2.dma_cnt = 0;
            state.dma2.timer_cnt = 0;
            false
        } else {
            true
        }
    }

    /// Apply GBA hardware DirectSound 50% vs 100% channel gain to a signed 8-bit FIFO sample
    /// (`100%` = `sample << 1` in 10-bit GBA DAC space, `50%` = `sample`, clamped to `[-512, 511]`).
    #[must_use]
    #[inline]
    pub const fn gba_directsound_dac_mix(
        fifo_a: i8,
        a_full_vol: bool,
        fifo_b: i8,
        b_full_vol: bool,
    ) -> i16 {
        let a = if a_full_vol {
            (fifo_a as i16) << 1
        } else {
            fifo_a as i16
        };
        let b = if b_full_vol {
            (fifo_b as i16) << 1
        } else {
            fifo_b as i16
        };
        let sum = a + b;
        if sum > 511 {
            511
        } else if sum < -512 {
            -512
        } else {
            sum
        }
    }
}

// =============================================================================
// StudioSynth: 48 kHz f32 Compile-Time Lanczos-3 Polyphase LUT + Equal-Power Pan
// =============================================================================

/// Number of fractional sub-sample phases in the compile-time Lanczos-3 polyphase LUT.
const LANCZOS3_PHASES: usize = 64;

/// Number of taps per phase for Lanczos-3 (`2 * 3 = 6`).
const LANCZOS3_TAPS: usize = 6;

/// Compile-time Taylor series approximation of `sin(x)` accurate to `< 1e-13` for `|x| <= 10`.
const fn sin_const(mut x: f64) -> f64 {
    const TWO_PI: f64 = 2.0 * core::f64::consts::PI;
    const PI: f64 = core::f64::consts::PI;
    while x > PI {
        x -= TWO_PI;
    }
    while x < -PI {
        x += TWO_PI;
    }
    let x2 = x * x;
    let mut term = x;
    let mut sum = x;
    let mut k = 1u32;
    while k <= 11 {
        let denom = ((2 * k) * (2 * k + 1)) as f64;
        term = -term * x2 / denom;
        sum += term;
        k += 1;
    }
    sum
}

/// Compile-time `cos(x)` using `sin(x + PI/2)`.
#[inline]
const fn cos_const(x: f64) -> f64 {
    sin_const(x + core::f64::consts::FRAC_PI_2)
}

/// Compile-time normalized Lanczos-`a` windowed-sinc kernel.
const fn lanczos_kernel_const(x: f64, a: f64) -> f64 {
    let ax = if x < 0.0 { -x } else { x };
    if ax < 1e-12 {
        return 1.0;
    }
    if ax >= a {
        return 0.0;
    }
    let px = core::f64::consts::PI * x;
    let px_a = px / a;
    (sin_const(px) / px) * (sin_const(px_a) / px_a)
}

/// Build the normalized 64-phase × 6-tap Lanczos-3 polyphase filter bank at compile time.
const fn build_lanczos3_lut() -> [[f32; LANCZOS3_TAPS]; LANCZOS3_PHASES] {
    let mut lut = [[0.0f32; LANCZOS3_TAPS]; LANCZOS3_PHASES];
    let mut p = 0usize;
    while p < LANCZOS3_PHASES {
        let frac = (p as f64) / (LANCZOS3_PHASES as f64);
        let mut weights = [0.0f64; LANCZOS3_TAPS];
        let mut sum = 0.0f64;
        let mut t = 0usize;
        while t < LANCZOS3_TAPS {
            let k = (t as isize) - 2; // taps at center - 2 ..= center + 3
            let w = lanczos_kernel_const(frac - (k as f64), 3.0);
            weights[t] = w;
            sum += w;
            t += 1;
        }
        let inv = if sum > 1e-12 { 1.0 / sum } else { 1.0 };
        let mut t = 0usize;
        while t < LANCZOS3_TAPS {
            lut[p][t] = (weights[t] * inv) as f32;
            t += 1;
        }
        p += 1;
    }
    lut
}

/// Compile-time 64-phase × 6-tap normalized Lanczos-3 polyphase filter LUT.
static LANCZOS3_LUT: [[f32; LANCZOS3_TAPS]; LANCZOS3_PHASES] = build_lanczos3_lut();

/// High-fidelity studio renderer for Southpaw GBA samples (48,000 Hz `f32`,
/// compile-time Lanczos-3 polyphase windowed-sinc interpolation, equal-power stereo panning).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StudioSynth {
    /// Target output sample rate in Hz (default: `48_000`).
    pub output_rate_hz: u32,
    /// Number of Lanczos sinc lobes (`3` uses the fast compile-time 64-phase polyphase LUT).
    pub sinc_lobes: usize,
}

impl Default for StudioSynth {
    fn default() -> Self {
        Self::new(48_000)
    }
}

impl StudioSynth {
    /// Create a new [`StudioSynth`] configured for `output_rate_hz` with Lanczos-3 interpolation.
    #[must_use]
    pub const fn new(output_rate_hz: u32) -> Self {
        Self {
            output_rate_hz,
            sinc_lobes: 3,
        }
    }

    /// Create a [`StudioSynth`] with a custom number of Lanczos lobes (`sinc_lobes >= 1`).
    #[must_use]
    pub const fn with_sinc_lobes(output_rate_hz: u32, sinc_lobes: usize) -> Self {
        Self {
            output_rate_hz,
            sinc_lobes: if sinc_lobes == 0 { 1 } else { sinc_lobes },
        }
    }

    /// Render a [`SouthpawSampleView`] additively into stereo `out_l` / `out_r` buffers
    /// at `playback_hz` with `gain` (`0.0..=1.0+`) and equal-power `pan` (`-1.0` = hard left,
    /// `0.0` = center, `+1.0` = hard right).
    #[allow(clippy::too_many_arguments)]
    pub fn render_voice_f32(
        &self,
        sample: &SouthpawSampleView<'_>,
        pos_f64: &mut f64,
        playback_hz: f64,
        gain: f32,
        pan: f32,
        out_l: &mut [f32],
        out_r: &mut [f32],
    ) {
        let pcm = sample.pcm;
        let pcm_len = pcm.len();
        if pcm_len == 0 || self.output_rate_hz == 0 || playback_hz <= 0.0 {
            return;
        }
        let frames = out_l.len().min(out_r.len());
        if frames == 0 {
            return;
        }

        let step = playback_hz / (self.output_rate_hz as f64);
        let pan_clamped = pan.clamp(-1.0, 1.0) as f64;
        let angle = (pan_clamped + 1.0) * core::f64::consts::FRAC_PI_4; // [0, pi/2]
        let norm_gain = (gain as f64) / 128.0;
        let gain_l = (norm_gain * cos_const(angle)) as f32;
        let gain_r = (norm_gain * sin_const(angle)) as f32;

        let pcm_len_f = pcm_len as f64;
        let (looped, loop_start, loop_len) = match sample.loop_start {
            Some(ls) if (ls as usize) < pcm_len => (true, ls as usize, pcm_len - (ls as usize)),
            _ => (false, 0usize, 0usize),
        };
        let loop_start_f = loop_start as f64;
        let loop_len_f = loop_len as f64;

        // Fast path: compile-time 64-phase Lanczos-3 polyphase LUT
        if self.sinc_lobes == 3 {
            let mut ph = *pos_f64;
            let mut idx = 0usize;
            while idx < frames {
                if ph >= pcm_len_f {
                    if looped {
                        ph = loop_start_f + ((ph - loop_start_f) % loop_len_f);
                    } else {
                        break;
                    }
                }
                let center = ph as isize;
                let frac = ph - (center as f64);
                let phase = ((frac * (LANCZOS3_PHASES as f64)) as usize).min(LANCZOS3_PHASES - 1);
                let w = &LANCZOS3_LUT[phase];

                let mut s = [0.0f32; LANCZOS3_TAPS];
                for (t, slot) in s.iter_mut().enumerate() {
                    let k = center + (t as isize) - 2;
                    if k >= 0 {
                        let u = k as usize;
                        if u < pcm_len {
                            *slot = (pcm[u] as i8) as f32;
                        } else if looped {
                            let wrapped = loop_start + ((u - loop_start) % loop_len);
                            *slot = (pcm[wrapped] as i8) as f32;
                        }
                    }
                }

                let val = s[0] * w[0]
                    + s[1] * w[1]
                    + s[2] * w[2]
                    + s[3] * w[3]
                    + s[4] * w[4]
                    + s[5] * w[5];

                out_l[idx] += val * gain_l;
                out_r[idx] += val * gain_r;
                ph += step;
                idx += 1;
            }
            *pos_f64 = ph;
            return;
        }

        // General fallback for custom `sinc_lobes != 3`
        let a = self.sinc_lobes.max(1) as isize;
        let a_f64 = a as f64;
        let mut ph = *pos_f64;

        for (ol, or) in out_l[..frames].iter_mut().zip(out_r[..frames].iter_mut()) {
            if ph >= pcm_len_f {
                if looped {
                    ph = loop_start_f + ((ph - loop_start_f) % loop_len_f);
                } else {
                    break;
                }
            }
            let center = ph as isize;
            let mut acc = 0.0f64;
            let mut wsum = 0.0f64;

            for k in (center - a + 1)..=(center + a) {
                let s_val = if k < 0 {
                    0.0
                } else if (k as usize) < pcm_len {
                    (pcm[k as usize] as i8) as f64
                } else if looped {
                    let wrapped = loop_start + (((k as usize) - loop_start) % loop_len);
                    (pcm[wrapped] as i8) as f64
                } else {
                    0.0
                };
                let w = lanczos_kernel_const(ph - (k as f64), a_f64);
                acc += s_val * w;
                wsum += w;
            }

            let val = if wsum.abs() > 1e-9 {
                (acc / wsum) as f32
            } else {
                0.0
            };

            *ol += val * gain_l;
            *or += val * gain_r;
            ph += step;
        }

        *pos_f64 = ph;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn studio_synth_lut_matches_direct_lanczos3() {
        let pcm: [u8; 64] = core::array::from_fn(|i| {
            let angle = (i as f64) * 0.35;
            (sin_const(angle) * 100.0) as i8 as u8
        });
        let view = SouthpawSampleView::from_pcm_bytes(0, 16_727, None, true, &pcm);
        let synth = StudioSynth::new(48_000);
        let mut left = [0.0f32; 128];
        let mut right = [0.0f32; 128];
        let mut pos = 5.25;
        synth.render_voice_f32(&view, &mut pos, 12_000.0, 1.0, 0.0, &mut left, &mut right);

        let step = 12_000.0 / 48_000.0;
        let gain_l = ((1.0 / 128.0) * cos_const(core::f64::consts::FRAC_PI_4)) as f32;
        let mut ref_pos = 5.25f64;
        for &actual in &left {
            let center = ref_pos as isize;
            let mut acc = 0.0f64;
            let mut wsum = 0.0f64;
            for k in (center - 2)..=(center + 3) {
                let s = (pcm[k as usize] as i8) as f64;
                let w = lanczos_kernel_const(ref_pos - (k as f64), 3.0);
                acc += s * w;
                wsum += w;
            }
            let expected = ((acc / wsum) as f32) * gain_l;
            assert!(
                (actual - expected).abs() < 1e-3,
                "LUT mismatch: actual={actual}, expected={expected}"
            );
            ref_pos += step;
        }
    }

    #[test]
    fn echo_feedback_matches_signed_quarter_decay() {
        assert_eq!(AccuracySynth::echo_feedback_byte(0), 0);
        // +126 -> (0x7E + 0x81) >> 2 = 0x3F -> 0x3F - 0x20 = +31
        assert_eq!(AccuracySynth::echo_feedback_byte(126), 31);
        // Hardware +0x81 bias wrap quirk: +127 -> (0x7F + 0x81) >> 2 = 0x40, & 0x3F = 0 -> -32
        assert_eq!(AccuracySynth::echo_feedback_byte(127), -32);
        assert_eq!(AccuracySynth::echo_feedback_byte((-128i8) as u8), -32);
    }
}
