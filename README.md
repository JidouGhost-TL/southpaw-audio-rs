# southpaw-audio-rs

**Zero-dependency, pure Rust parser, software synthesizer, and byte-exact golden-tested emulator for the Southpaw GBA Sound Engine (Game Boy Advance).**

[![Crates.io](https://img.shields.io/crates/v/southpaw-audio.svg)](https://crates.io/crates/southpaw-audio)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE-MIT)
[![no_std](https://img.shields.io/badge/no__std-compatible-success.svg)](#features)

The **Southpaw GBA Sound Engine** powered **20 commercial Game Boy Advance releases** across **5 distinct ARM7TDMI/Thumb revisions** spanning two architectural families:

- **Dual-DMA Hardware DirectSound (16 ROMs across 3 revisions)**: Streams raw signed 8-bit PCM directly from ROM over both DirectSound A (`DMA1` / `Timer0`) and DirectSound B (`DMA2` / `Timer1`) using DMA-IRQ 16-byte FIFO block counting (`DMAxCNT = 0xF6600004`) alongside 4 Game Boy PSG channels (*Doom*, *Lucky Luke: Wanted!*, *Gremlins: Stripe vs Gizmo*, *Hot Potato!*, *Inspector Gadget: Advance Mission*, *Planet Monsters*, *Total Soccer 2002 / Manager*, *Thunderbirds*).
- **Echo-Mixer + DMA2 Hybrid (4 ROMs across 2 revisions)**: Replaces DirectSound A with a real-time **Thumb 5-frame ring-buffer echo/reverb software mixer** (`((prev + 0x81) >> 2) & 0x3F - 0x20` signed 1/4-amplitude delay feedback over `~83.7 ms`) + 24.8 fixed-point resampled voice (`adds r3, r3, r4` wrapping `i8`), paired with hardware DMA2-IRQ DirectSound B (*Extreme Ghostbusters: Code Ecto-1*, *Antz: Extreme Racing*).

`southpaw-audio` provides a clean-room, reverse-engineered implementation designed for audio preservation, DAWs, PC source ports, and emulators.

> **Looking for a ready-to-use CLI?** You can scan GBA ROMs, extract WAV samples, and compile SoundFont 2.04 (.sf2) banks directly from the command line using our **[gbakit](https://github.com/JidouGhost-TL/gbakit)** toolkit (`gbakit audio scan`, `gbakit audio export`, `gbakit audio soundfont`).

---

## Features

- **Zero External Dependencies**: Pure Rust with no dependencies in `[dependencies]`, `#![deny(unsafe_code)]`.
- **`#![no_std]` Compatible**: Ready for embedded targets and WASM runtimes (`alloc` optional).
- **Native Sample Table Parser**: Zero-allocation parser (`SouthpawSampleDesc`, `SouthpawSampleView`, `find_sample_table`) for Southpaw's 16-byte Timer0-reloaded PCM8 sample tables.
- **Dual Synthesis Engines**:
  - **`AccuracySynth`**: 100% bit-exact emulation of all **5** Southpaw ARM7TDMI/Thumb revisions — including the 5-frame ring-buffer Echo-Mixer (`176` samples @ `10,512 Hz` and `112` samples @ `6,690 Hz`) and Dual-DMA `SOUNDCNT_H` / `Timer0` / `Timer1` / DMA-IRQ 16-byte block countdown state machine.
  - **`StudioSynth`**: Renders at 48,000 Hz, 32-bit floating-point precision with a compile-time 1,024-phase Lanczos-3 polyphase windowed-sinc LUT (plus general Lanczos-`a` fallback) and true equal-power stereo panning.
- **Byte-Exact `libmgba` ARM Golden Tested**: 2,976 / 2,976 output bytes verified strictly byte-for-byte (`assert_eq!`) across all 5 Southpaw engine revisions.

---

## Supported Southpaw Engine Revisions

Auditing all 3,377 GBA ROMs identifies **5 distinct ARM7TDMI/Thumb revisions** spanning **20 commercial titles** (100% coverage via `SouthpawVersion::detect_from_bytes`):

| Revision | Corpus Share | Representative Titles | DirectSound A (`DMA1`) | DirectSound B (`DMA2`) | `SOUNDCNT_H` / PSG |
| --- | ---: | --- | --- | --- | --- |
| `DualDmaPsg50` | 3 ROMs | *Doom*, *Total Soccer 2002* (EU/JP) | Hardware DMA1-IRQ (`0xF6600004`, note-pitched) | Hardware DMA2-IRQ (`0xF6600004`, note-pitched) | PSG 50% (`0x7B01` / `0xF301`) |
| `DualDmaPsg100` | 12 ROMs | *Gremlins: Stripe vs Gizmo*, *Hot Potato!*, *Inspector Gadget*, *Planet Monsters*, *Total Soccer Manager*, *Thunderbirds* | Hardware DMA1-IRQ (`0xF6600004`, note-pitched) | Hardware DMA2-IRQ (`0xF6600004`, note-pitched) | PSG 100% (`0x7B02` / `0xF302`) |
| `DualDmaFixedRate` | 1 ROM | *Lucky Luke: Wanted!* | Hardware DMA1-IRQ (`0xF6600004`, `desc[+0x08]` fixed Timer0) | Hardware DMA2-IRQ (`0xF6600004`, `desc[+0x08]` fixed Timer1) | PSG 100% (`0x7B02` / `0xF302`) |
| `EchoMixer10512` | 2 ROMs | *Extreme Ghostbusters: Code Ecto-1* (US/EU) | Thumb 5-frame echo-mixer (`10,512 Hz`, `176` samples/frame, `DMA1CNT = 0xB6600004`) | Hardware DMA2-IRQ (`0xF6600004`, note-pitched) | PSG 50%, DS-A 50% (`0xFB09`) |
| `EchoMixer6690` | 2 ROMs | *Antz: Extreme Racing* (US/EU) | Thumb 5-frame echo-mixer (`6,690 Hz`, `112` samples/frame, `DMA1CNT = 0xB6600004`) | Hardware DMA2-IRQ (`0xF6600004`, note-pitched) | PSG 100%, DS-A 100% (`0xF306`) |

---

## Quickstart

### Zero-Allocation Mixing (`#![no_std]`)

```rust
use southpaw_audio::{AccuracySynth, EchoRingBuffer, EchoVoice, ECHO_10512_FRAME_SAMPLES};

// Raw signed 8-bit PCM waveform (0 = silence)
let pcm: &[u8] = &[0, 32, 64, 96, 127, 96, 64, 32, 0, 224, 192, 160, 128, 160, 192, 224];

// 10,512 Hz Southpaw Echo-Mixer voice (24.8 fixed-point step = 0x0100 = 1.0x, one-shot)
let mut voice = EchoVoice::from_bytes(0x0100, pcm.len(), None);
let mut ring = EchoRingBuffer::<ECHO_10512_FRAME_SAMPLES>::new();
let mut frame = [0i8; ECHO_10512_FRAME_SAMPLES];

AccuracySynth::mix_echo_10512_frame(pcm, &mut voice, &mut ring, &mut frame);
```

---

## Golden Test Methodology

Every synthesis and hardware register path in `AccuracySynth` is verified strictly byte-for-byte (`assert_eq!`, zero tolerance) against real ARM7TDMI/Thumb execution traces captured in headless `libmgba.so.0.10` using clean-room synthetic deterministic waveforms:

- **4 `EchoMixer10512` ARM vectors (1,936 bytes)**: Cold-start 24.8 FP resampled frame (`176` bytes), 6-frame 5-slot ring-buffer echo feedback wrap (`1,056` bytes), looped overshoot + pre-seeded echo + wrapping `i8` overflow (`352` bytes), and one-shot mid-frame termination + echo decay tail (`352` bytes).
- **2 `EchoMixer6690` ARM vectors (896 bytes)**: 6-frame 112-sample ring-buffer echo wrap (`672` bytes) and one-shot termination + echo decay tail (`224` bytes).
- **3 Dual-DMA ARM vectors (144 bytes)**: `DualDmaPsg50`, `DualDmaPsg100`, and `DualDmaFixedRate` verifying exact `SOUNDCNT_H` gain-bit preservation, note-pitched vs fixed `desc[+0x08]` hardware timers, and verbatim ROM DMA1/DMA2 IRQ block-countdown exhaustion.

---

## Benchmarks

```sh
cargo bench --bench bench_audio
```

| Benchmark | Throughput |
| --- | ---: |
| `AccuracySynth::mix_echo_10512_frame` (176f) | ~1.68 GSamples/s |
| `AccuracySynth::mix_echo_6690_frame` (112f) | ~1.64 GSamples/s |
| `AccuracySynth::service_dma1_irq` (16 blocks) | ~2.22 ns/iter |
| `StudioSynth::render_voice_f32` (48 kHz Lanczos-3, 1,024f) | ~408.0 MSamples/s |

---

## License

Licensed under either of:
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))
