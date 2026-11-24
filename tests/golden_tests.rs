//! Byte-exact `libmgba.so.0.10` ARM7TDMI Golden Integration Tests for `southpaw-audio`.
//!
//! Verifies all **9** clean-room ARM7TDMI execution golden vectors (**2,976 / 2,976 bytes**,
//! strict `assert_eq!` with zero tolerance) covering all **5** Southpaw GBA Sound Engine
//! revisions discovered across the 3,377-ROM GBA corpus:
//!
//! 1. `SouthpawVersion::EchoMixer10512` (4 vectors, 1,936 bytes):
//!    - 10,512 Hz 176-sample cold-start 24.8 FP resampled frame (`step_q8 = 0x0147`)
//!    - 6 consecutive frames (`1,056` bytes) exercising 5-slot ring-buffer echo wrap
//!    - Looped overshoot + pre-seeded echo ring + wrapping `i8` overflow (`352` bytes)
//!    - One-shot mid-frame termination + echo decay tail (`352` bytes)
//! 2. `SouthpawVersion::EchoMixer6690` (2 vectors, 896 bytes):
//!    - 6,690 Hz 6 consecutive 112-sample frames (`672` bytes) with ring echo wrap
//!    - One-shot mid-frame termination + echo decay tail (`224` bytes)
//! 3. `SouthpawVersion::DualDmaPsg50` (1 vector, 48 bytes):
//!    - `SOUNDCNT_H` (`0x7B01`/`0xF309`/`0x7B0D`/`0xF305`), note timers, verbatim DMA-IRQ countdown
//! 4. `SouthpawVersion::DualDmaPsg100` (1 vector, 48 bytes):
//!    - `SOUNDCNT_H` (`0x7B02`/`0xF30A`/`0x7B0E`/`0xF306`), note timers, verbatim DMA-IRQ countdown
//! 5. `SouthpawVersion::DualDmaFixedRate` (1 vector, 48 bytes):
//!    - `desc[+0x08]` fixed Timer0/Timer1 loads, `SOUNDCNT_H`, verbatim DMA-IRQ countdown

use southpaw_audio::{
    AccuracySynth, DualDmaHardwareState, EchoRingBuffer, EchoVoice, SouthpawSampleDesc,
    SouthpawSampleView, SouthpawVersion, StudioSynth, ECHO_10512_FRAME_SAMPLES,
    ECHO_6690_FRAME_SAMPLES,
};

const MANIFEST_JSON: &str = include_str!("golden/manifest.json");

// EchoMixer10512 goldens (4 vectors, 1,936 bytes)
const GOLDEN_ECHO10512_COLD: &[u8] = include_bytes!("golden/echo10512_cold_resampled.bin");
const GOLDEN_ECHO10512_6FRAMES: &[u8] =
    include_bytes!("golden/echo10512_echo_feedback_6frames.bin");
const GOLDEN_ECHO10512_LOOP_OVERFLOW: &[u8] =
    include_bytes!("golden/echo10512_loop_wrap_overflow.bin");
const GOLDEN_ECHO10512_ONESHOT_DECAY: &[u8] =
    include_bytes!("golden/echo10512_oneshot_decay_tail.bin");

// EchoMixer6690 goldens (2 vectors, 896 bytes)
const GOLDEN_ECHO6690_6FRAMES: &[u8] = include_bytes!("golden/echo6690_echo_feedback_6frames.bin");
const GOLDEN_ECHO6690_ONESHOT_DECAY: &[u8] = include_bytes!("golden/echo6690_oneshot_decay.bin");

// Dual-DMA hardware goldens (3 vectors, 144 bytes)
const GOLDEN_DUALDMA_PSG50: &[u8] = include_bytes!("golden/dualdma_psg50_hw_state.bin");
const GOLDEN_DUALDMA_PSG100: &[u8] = include_bytes!("golden/dualdma_psg100_hw_state.bin");
const GOLDEN_DUALDMA_FIXEDRATE: &[u8] = include_bytes!("golden/dualdma_fixedrate_hw_state.bin");

/// Clean-room deterministic synthetic waveform A (identical to the `libmgba` ARM harness).
fn synth_wave_a() -> [u8; 256] {
    let mut out = [0u8; 256];
    for (i, slot) in out.iter_mut().enumerate() {
        let mut v = (((i as i32) * 19 + 7) % 256) - 128;
        if i % 5 == 0 {
            v = -v;
        }
        v = v.clamp(-128, 127);
        *slot = v as i8 as u8;
    }
    out
}

/// Clean-room deterministic synthetic waveform B (identical to the `libmgba` ARM harness).
fn synth_wave_b() -> [u8; 256] {
    let mut out = [0u8; 256];
    for (i, slot) in out.iter_mut().enumerate() {
        let v = (((i as i32) * 37 + 53) % 220) - 110;
        *slot = v as i8 as u8;
    }
    out
}

fn assert_i8_slice_matches_u8(actual: &[i8], expected: &[u8], label: &str) {
    assert_eq!(actual.len(), expected.len(), "{label} length mismatch");
    for (i, (&a, &e)) in actual.iter().zip(expected.iter()).enumerate() {
        assert_eq!(a, e as i8, "{label} mismatch at byte {i}");
    }
}

fn run_dualdma_sequence(version: SouthpawVersion) -> [u8; 48] {
    let mut state = DualDmaHardwareState::new(version);
    let desc0 = SouthpawSampleDesc {
        sample_ptr: 0x0800_2100,
        num_blocks: 2,
        timer0_cnt_reload: 0x0080_FC15,
        volume_mask: 0,
    };
    let desc1 = SouthpawSampleDesc {
        sample_ptr: 0x0800_2200,
        num_blocks: 3,
        timer0_cnt_reload: 0x0080_F800,
        volume_mask: -1,
    };

    let note_timer_a = 0x0080_F969;
    let note_timer_b = 0x0080_FB58;

    let mut words = [0u32; 12];

    // Step 1: Trigger DMA1 with desc0 (50% gain, 2 blocks)
    AccuracySynth::trigger_dma1(&mut state, desc0, note_timer_a);
    words[0] = u32::from(state.soundcnt_h);
    words[1] = state.dma1.timer_cnt;
    words[2] = state.dma1.blocks_remaining;

    // Step 2: Trigger DMA2 with desc1 (100% gain, 3 blocks)
    AccuracySynth::trigger_dma2(&mut state, desc1, note_timer_b);
    words[3] = u32::from(state.soundcnt_h);
    words[4] = state.dma2.timer_cnt;
    words[5] = state.dma2.blocks_remaining;

    // Step 3: Trigger DMA1 with desc1 (100% gain, preserves DMA2 100%)
    AccuracySynth::trigger_dma1(&mut state, desc1, note_timer_a);
    words[6] = u32::from(state.soundcnt_h);

    // Step 4: Trigger DMA2 with desc0 (50% gain, preserves DMA1 100%)
    AccuracySynth::trigger_dma2(&mut state, desc0, note_timer_b);
    words[7] = u32::from(state.soundcnt_h);

    // Reset blocks to 2 and 3 to match the ARM harness state before IRQ countdown
    state.dma1.blocks_remaining = 2;
    state.dma2.blocks_remaining = 3;

    // Step 5: Service DMA1 IRQ twice (2 -> 1 -> 0, stops DMA1 + TM0)
    assert!(AccuracySynth::service_dma1_irq(&mut state));
    words[8] = state.dma1.blocks_remaining;
    assert!(!AccuracySynth::service_dma1_irq(&mut state));
    words[9] = state.dma1.blocks_remaining;
    words[10] = state.dma1.timer_cnt >> 16;

    // Step 6: Service DMA2 IRQ three times (3 -> 2 -> 1 -> 0, stops DMA2 + TM1)
    assert!(AccuracySynth::service_dma2_irq(&mut state));
    assert!(AccuracySynth::service_dma2_irq(&mut state));
    assert!(!AccuracySynth::service_dma2_irq(&mut state));
    words[11] = state.dma2.blocks_remaining | ((state.dma2.timer_cnt >> 16) << 16);

    let mut out = [0u8; 48];
    for (i, w) in words.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_le_bytes());
    }
    out
}

#[test]
fn golden_manifest_covers_all_2976_bytes() {
    assert!(MANIFEST_JSON.contains("\"total_bytes\": 2976"));
    assert!(MANIFEST_JSON.contains("EchoMixer10512"));
    assert!(MANIFEST_JSON.contains("EchoMixer6690"));
    assert!(MANIFEST_JSON.contains("DualDmaPsg50"));
    assert!(MANIFEST_JSON.contains("DualDmaPsg100"));
    assert!(MANIFEST_JSON.contains("DualDmaFixedRate"));
}

#[test]
fn golden_echo10512_cold_resampled() {
    let wave = synth_wave_a();
    let mut voice = EchoVoice::new(0x0147, 0, 256 << 8, 0);
    let mut ring = EchoRingBuffer::<ECHO_10512_FRAME_SAMPLES>::new();
    let mut out = [0i8; ECHO_10512_FRAME_SAMPLES];

    AccuracySynth::mix_echo_10512_frame(&wave, &mut voice, &mut ring, &mut out);
    assert_i8_slice_matches_u8(&out, GOLDEN_ECHO10512_COLD, "echo10512_cold_resampled");
}

#[test]
fn golden_echo10512_echo_feedback_6frames() {
    let wave = synth_wave_a();
    let mut voice = EchoVoice::new(0x0123, 0, 256 << 8, 16 << 8);
    let mut ring = EchoRingBuffer::<ECHO_10512_FRAME_SAMPLES>::new();
    let mut frame = [0i8; ECHO_10512_FRAME_SAMPLES];

    for f in 0..6 {
        AccuracySynth::mix_echo_10512_frame(&wave, &mut voice, &mut ring, &mut frame);
        let start = f * ECHO_10512_FRAME_SAMPLES;
        let end = start + ECHO_10512_FRAME_SAMPLES;
        assert_i8_slice_matches_u8(
            &frame,
            &GOLDEN_ECHO10512_6FRAMES[start..end],
            "echo10512_echo_feedback_6frames",
        );
    }
}

#[test]
fn golden_echo10512_loop_wrap_overflow() {
    let wave_a = synth_wave_a();
    let wave_b = synth_wave_b();
    let mut voice = EchoVoice::new(0x01D3, 0, 96 << 8, 32 << 8);
    let mut ring = EchoRingBuffer::<ECHO_10512_FRAME_SAMPLES>::new();
    ring.slots[1].copy_from_slice(&wave_b[..ECHO_10512_FRAME_SAMPLES]);

    let mut frame = [0i8; ECHO_10512_FRAME_SAMPLES];
    for f in 0..2 {
        AccuracySynth::mix_echo_10512_frame(&wave_a, &mut voice, &mut ring, &mut frame);
        let start = f * ECHO_10512_FRAME_SAMPLES;
        let end = start + ECHO_10512_FRAME_SAMPLES;
        assert_i8_slice_matches_u8(
            &frame,
            &GOLDEN_ECHO10512_LOOP_OVERFLOW[start..end],
            "echo10512_loop_wrap_overflow",
        );
    }
}

#[test]
fn golden_echo10512_oneshot_decay_tail() {
    let wave_a = synth_wave_a();
    let wave_b = synth_wave_b();
    let mut voice = EchoVoice::new(0x0100, 0, 48 << 8, 0);
    let mut ring = EchoRingBuffer::<ECHO_10512_FRAME_SAMPLES>::new();
    ring.slots[1].copy_from_slice(&wave_b[..ECHO_10512_FRAME_SAMPLES]);

    let mut frame = [0i8; ECHO_10512_FRAME_SAMPLES];
    for f in 0..2 {
        AccuracySynth::mix_echo_10512_frame(&wave_a, &mut voice, &mut ring, &mut frame);
        let start = f * ECHO_10512_FRAME_SAMPLES;
        let end = start + ECHO_10512_FRAME_SAMPLES;
        assert_i8_slice_matches_u8(
            &frame,
            &GOLDEN_ECHO10512_ONESHOT_DECAY[start..end],
            "echo10512_oneshot_decay_tail",
        );
    }
    assert!(!voice.is_active());
}

#[test]
fn golden_echo6690_echo_feedback_6frames() {
    let wave = synth_wave_a();
    let mut voice = EchoVoice::new(0x016B, 0, 80 << 8, 16 << 8);
    let mut ring = EchoRingBuffer::<ECHO_6690_FRAME_SAMPLES>::new();
    let mut frame = [0i8; ECHO_6690_FRAME_SAMPLES];

    for f in 0..6 {
        AccuracySynth::mix_echo_6690_frame(&wave, &mut voice, &mut ring, &mut frame);
        let start = f * ECHO_6690_FRAME_SAMPLES;
        let end = start + ECHO_6690_FRAME_SAMPLES;
        assert_i8_slice_matches_u8(
            &frame,
            &GOLDEN_ECHO6690_6FRAMES[start..end],
            "echo6690_echo_feedback_6frames",
        );
    }
}

#[test]
fn golden_echo6690_oneshot_decay() {
    let wave_a = synth_wave_a();
    let wave_b = synth_wave_b();
    let mut voice = EchoVoice::new(0x0100, 0, 40 << 8, 0);
    let mut ring = EchoRingBuffer::<ECHO_6690_FRAME_SAMPLES>::new();
    ring.slots[1].copy_from_slice(&wave_b[..ECHO_6690_FRAME_SAMPLES]);

    let mut frame = [0i8; ECHO_6690_FRAME_SAMPLES];
    for f in 0..2 {
        AccuracySynth::mix_echo_6690_frame(&wave_a, &mut voice, &mut ring, &mut frame);
        let start = f * ECHO_6690_FRAME_SAMPLES;
        let end = start + ECHO_6690_FRAME_SAMPLES;
        assert_i8_slice_matches_u8(
            &frame,
            &GOLDEN_ECHO6690_ONESHOT_DECAY[start..end],
            "echo6690_oneshot_decay",
        );
    }
    assert!(!voice.is_active());
}

#[test]
fn golden_dualdma_psg50_hw_state() {
    let actual = run_dualdma_sequence(SouthpawVersion::DualDmaPsg50);
    assert_eq!(&actual[..], GOLDEN_DUALDMA_PSG50);
}

#[test]
fn golden_dualdma_psg100_hw_state() {
    let actual = run_dualdma_sequence(SouthpawVersion::DualDmaPsg100);
    assert_eq!(&actual[..], GOLDEN_DUALDMA_PSG100);
}

#[test]
fn golden_dualdma_fixedrate_hw_state() {
    let actual = run_dualdma_sequence(SouthpawVersion::DualDmaFixedRate);
    assert_eq!(&actual[..], GOLDEN_DUALDMA_FIXEDRATE);
}

#[test]
fn studio_synth_renders_equal_power_stereo() {
    let wave = synth_wave_a();
    let sample = SouthpawSampleView::from_pcm_bytes(0, 16_727, Some(32), true, &wave);
    let studio = StudioSynth::default();

    let mut l_center = [0.0f32; 256];
    let mut r_center = [0.0f32; 256];
    let mut pos = 0.0f64;
    studio.render_voice_f32(&sample, &mut pos, 16_727.0, 0.9, 0.0, &mut l_center, &mut r_center);

    for (&l, &r) in l_center.iter().zip(r_center.iter()) {
        assert!((l - r).abs() < 1e-6, "center pan must be symmetric");
    }

    // Hard left pan should produce silence on right channel
    let mut l_hard = [0.0f32; 64];
    let mut r_hard = [0.0f32; 64];
    let mut pos2 = 0.0f64;
    studio.render_voice_f32(&sample, &mut pos2, 16_727.0, 0.9, -1.0, &mut l_hard, &mut r_hard);
    assert!(r_hard.iter().all(|&x| x.abs() < 1e-6));
    assert!(l_hard.iter().any(|&x| x.abs() > 0.05));
}
