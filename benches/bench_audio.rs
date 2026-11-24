use std::hint::black_box;
use std::time::Instant;

use southpaw_audio::{
    AccuracySynth, DualDmaHardwareState, EchoRingBuffer, EchoVoice, SouthpawSampleDesc,
    SouthpawSampleView, SouthpawVersion, StudioSynth, ECHO_10512_FRAME_SAMPLES,
    ECHO_6690_FRAME_SAMPLES,
};

fn main() {
    println!("=== southpaw-audio Audio Performance Benchmarks ===");

    let pcm: Vec<u8> = (0i32..4096)
        .map(|i| (((i * 19 + 7) % 256) - 128) as i8 as u8)
        .collect();
    let sample = SouthpawSampleView::from_pcm_bytes(0, 16_727, Some(512), true, &pcm);

    let iters = 200_000usize;

    // 1. AccuracySynth EchoMixer10512 (176 samples/frame)
    let mut ring_176 = EchoRingBuffer::<ECHO_10512_FRAME_SAMPLES>::new();
    let mut out_176 = [0i8; ECHO_10512_FRAME_SAMPLES];
    let start = Instant::now();
    for _ in 0..iters {
        let mut voice = EchoVoice::new(0x0147, 0, 4096 << 8, 512 << 8);
        AccuracySynth::mix_echo_10512_frame(black_box(&pcm), &mut voice, &mut ring_176, &mut out_176);
        black_box(&out_176);
    }
    let elapsed = start.elapsed();
    println!(
        "AccuracySynth::echo_10512 (176f):                  {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / iters as f64,
        ((iters * ECHO_10512_FRAME_SAMPLES) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );

    // 2. AccuracySynth EchoMixer6690 (112 samples/frame)
    let mut ring_112 = EchoRingBuffer::<ECHO_6690_FRAME_SAMPLES>::new();
    let mut out_112 = [0i8; ECHO_6690_FRAME_SAMPLES];
    let start = Instant::now();
    for _ in 0..iters {
        let mut voice = EchoVoice::new(0x016B, 0, 4096 << 8, 512 << 8);
        AccuracySynth::mix_echo_6690_frame(black_box(&pcm), &mut voice, &mut ring_112, &mut out_112);
        black_box(&out_112);
    }
    let elapsed = start.elapsed();
    println!(
        "AccuracySynth::echo_6690 (112f):                   {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / iters as f64,
        ((iters * ECHO_6690_FRAME_SAMPLES) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );

    // 3. AccuracySynth Dual-DMA trigger + IRQ block countdown
    let desc = SouthpawSampleDesc {
        sample_ptr: 0x0800_1000,
        num_blocks: 16,
        timer0_cnt_reload: 0x0080_FC15,
        volume_mask: -1,
    };
    let mut hw = DualDmaHardwareState::new(SouthpawVersion::DualDmaPsg50);
    let start = Instant::now();
    for _ in 0..iters {
        AccuracySynth::trigger_dma1(&mut hw, black_box(desc), 0x0080_F969);
        while AccuracySynth::service_dma1_irq(&mut hw) {}
        black_box(&hw);
    }
    let elapsed = start.elapsed();
    println!(
        "AccuracySynth::dual_dma_16blocks:                  {:8.2} ns/iter",
        elapsed.as_nanos() as f64 / iters as f64
    );

    // 4. StudioSynth 48 kHz Lanczos-3 (1024 frames)
    let studio = StudioSynth::default();
    let mut out_l = vec![0.0f32; 1024];
    let mut out_r = vec![0.0f32; 1024];
    let studio_iters = 50_000usize;
    let start = Instant::now();
    for _ in 0..studio_iters {
        let mut pos = 0.0f64;
        studio.render_voice_f32(
            black_box(&sample),
            &mut pos,
            16_727.0,
            0.85,
            0.25,
            &mut out_l,
            &mut out_r,
        );
        black_box((&out_l, &out_r));
    }
    let elapsed = start.elapsed();
    println!(
        "StudioSynth::render_voice_f32 (48kHz Lanczos3, 1k): {:8.2} ns/iter ({:8.2} MSamples/sec)",
        elapsed.as_nanos() as f64 / studio_iters as f64,
        ((studio_iters * 1024) as f64 / elapsed.as_secs_f64()) / 1_000_000.0
    );
}
