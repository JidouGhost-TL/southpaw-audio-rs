//! GBA Timer0/Timer1 sample rate conversion and 24.8 fixed-point pitch utilities
//! for the Southpaw GBA Sound Engine.

/// GBA master CPU clock frequency in Hz (`2^24 = 16,777,216 Hz`).
pub const GBA_CLOCK_HZ: u32 = 16_777_216;

/// Timer0 control+reload word for `EchoMixer10512` (`10,512 Hz`, `176` samples/frame).
pub const ECHO_10512_TIMER0: u32 = 0x0080_F9C4;

/// Timer0 control+reload word for `EchoMixer6690` (`6,689 Hz`, `112` samples/frame).
pub const ECHO_6690_TIMER0: u32 = 0x0080_F634;

/// Nominal DirectSound A output rate in Hz for `EchoMixer10512`.
pub const ECHO_10512_RATE_HZ: u32 = 10_512;

/// Nominal DirectSound A output rate in Hz for `EchoMixer6690` (`16,777,216 / 2,508 ≈ 6,689.48 Hz`).
pub const ECHO_6690_RATE_HZ: u32 = 6_689;

/// Unity 24.8 fixed-point step (`1.0x` playback rate in the Southpaw echo-mixer).
pub const Q8_ONE: u32 = 0x0100;

/// Standard 24-entry (2-octave) 12-TET hardware Timer0/Timer1 control+reload table
/// used by Southpaw's note-pitched DirectSound triggers (verified from *Doom* `ADME`
/// `0x0801EA0C` and shared across Southpaw Dual-DMA and Echo-Mixer titles).
pub const SOUTHPAW_NOTE_TIMER_TABLE: [u32; 24] = [
    0x0080_F055, 0x0080_F135, 0x0080_F20A, 0x0080_F2D3, 0x0080_F390, 0x0080_F443,
    0x0080_F4EB, 0x0080_F58B, 0x0080_F621, 0x0080_F6AF, 0x0080_F735, 0x0080_F7B3,
    0x0080_F82A, 0x0080_F89B, 0x0080_F905, 0x0080_F969, 0x0080_F9C8, 0x0080_FA22,
    0x0080_FA76, 0x0080_FAC6, 0x0080_FB11, 0x0080_FB58, 0x0080_FB9B, 0x0080_FBDA,
];

/// Compute the exact rounded playback sample rate in Hz from a GBA Timer control+reload word
/// (`lo16` = 16-bit timer reload value, `hi16` = timer control bits such as `0x0080`).
#[must_use]
#[inline]
pub const fn timer_reload_to_hz(timer_cnt_reload: u32) -> u32 {
    let reload = timer_cnt_reload & 0xFFFF;
    let ticks = 65_536u32.saturating_sub(reload);
    if ticks == 0 {
        return GBA_CLOCK_HZ;
    }
    // Rounded integer division: (GBA_CLOCK_HZ + ticks / 2) / ticks
    let num = (GBA_CLOCK_HZ as u64) + ((ticks as u64) / 2);
    (num / (ticks as u64)) as u32
}

/// Convert a desired playback sample rate in Hz into a packed GBA Timer control+reload word
/// with `TMxCNT_H = 0x0080` (Timer enable, prescaler 1).
#[must_use]
#[inline]
pub const fn hz_to_timer_reload(sample_rate_hz: u32) -> u32 {
    if sample_rate_hz == 0 {
        return 0x0080_0000;
    }
    let ticks = ((GBA_CLOCK_HZ as u64) + (sample_rate_hz as u64) / 2) / (sample_rate_hz as u64);
    let clamped_ticks = if ticks == 0 {
        1
    } else if ticks > 65_536 {
        65_536
    } else {
        ticks as u32
    };
    let reload = 65_536u32.saturating_sub(clamped_ticks);
    0x0080_0000 | (reload & 0xFFFF)
}

/// Convert a source sample frequency and output mix rate into a 24.8 fixed-point step
/// for the Southpaw echo-mixer resampler.
#[must_use]
#[inline]
pub const fn freq_to_step_q8(sample_rate_hz: u32, mix_rate_hz: u32) -> u32 {
    if mix_rate_hz == 0 {
        return 0;
    }
    let num = ((sample_rate_hz as u64) << 8) + ((mix_rate_hz as u64) / 2);
    (num / (mix_rate_hz as u64)) as u32
}

/// Convert a 24.8 fixed-point step back to an effective sample playback frequency in Hz.
#[must_use]
#[inline]
pub const fn step_q8_to_freq(step_q8: u32, mix_rate_hz: u32) -> u32 {
    let num = ((step_q8 as u64) * (mix_rate_hz as u64)) + 128;
    (num >> 8) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timer_reload_conversions_are_exact() {
        assert_eq!(timer_reload_to_hz(0x0080_FC15), 16_727);
        assert_eq!(timer_reload_to_hz(0x0080_F800), 8_192);
        assert_eq!(timer_reload_to_hz(0x0080_FB1A), 13_379);
        assert_eq!(timer_reload_to_hz(ECHO_10512_TIMER0), ECHO_10512_RATE_HZ);
        assert_eq!(timer_reload_to_hz(ECHO_6690_TIMER0), ECHO_6690_RATE_HZ);
        assert_eq!(hz_to_timer_reload(10_512), ECHO_10512_TIMER0);
        assert_eq!(hz_to_timer_reload(6_689), ECHO_6690_TIMER0);
    }

    #[test]
    fn q8_step_round_trips() {
        assert_eq!(freq_to_step_q8(10_512, 10_512), Q8_ONE);
        assert_eq!(step_q8_to_freq(Q8_ONE, 10_512), 10_512);
        let step = freq_to_step_q8(16_727, 10_512);
        assert_eq!(step, 0x0197);
    }
}
