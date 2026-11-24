//! # southpaw-audio
//!
//! Pure Rust, zero-dependency parser, software synthesizer, and byte-exact golden-tested
//! emulator for the **Southpaw GBA Sound Engine** on the Game Boy Advance.
//!
//! Covers **100% (20 / 20)** of commercial Southpaw GBA releases across **5 distinct
//! ARM7TDMI/Thumb revisions** spanning two architectural families:
//! - **Dual-DMA Hardware DirectSound** (`16` ROMs across 3 revisions):
//!   - [`SouthpawVersion::DualDmaPsg50`] (*Doom*, *Total Soccer 2002* EU/JP)
//!   - [`SouthpawVersion::DualDmaPsg100`] (*Gremlins: Stripe vs Gizmo*, *Hot Potato!*,
//!     *Inspector Gadget: Advance Mission*, *Planet Monsters*, *Total Soccer Manager*,
//!     *Thunderbirds*)
//!   - [`SouthpawVersion::DualDmaFixedRate`] (*Lucky Luke: Wanted!*)
//! - **Echo-Mixer + DMA2 Hybrid** (`4` ROMs across 2 revisions):
//!   - [`SouthpawVersion::EchoMixer10512`] (*Extreme Ghostbusters: Code Ecto-1* US/EU)
//!   - [`SouthpawVersion::EchoMixer6690`] (*Antz: Extreme Racing* US/EU)
//!
//! ## Features
//! - **Zero dependencies** in `[dependencies]`, `#![no_std]` compatible, `#![deny(unsafe_code)]`.
//! - **Dual synthesis pipelines**:
//!   - [`AccuracySynth`]: 100% bit-exact emulation of all 5 Southpaw ARM7TDMI/Thumb revisions.
//!   - [`StudioSynth`]: 48,000 Hz `f32` band-limited Lanczos-3 polyphase windowed-sinc renderer
//!     with true equal-power stereo panning.
//! - **Byte-exact `libmgba` ARM golden tested**: 2,976 / 2,976 output bytes verified strictly
//!   byte-for-byte (`assert_eq!`) against real ARM7TDMI execution across all 5 engine revisions.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod error;
pub mod freq;
pub mod sample;
pub mod synth;
pub mod version;
pub mod voice;

pub use error::{Error, Result};
pub use freq::{
    freq_to_step_q8, hz_to_timer_reload, step_q8_to_freq, timer_reload_to_hz, ECHO_10512_RATE_HZ,
    ECHO_10512_TIMER0, ECHO_6690_RATE_HZ, ECHO_6690_TIMER0, GBA_CLOCK_HZ, Q8_ONE,
    SOUTHPAW_NOTE_TIMER_TABLE,
};
#[cfg(feature = "alloc")]
pub use sample::{scan_sample_table, SouthpawSample};
pub use sample::{
    find_sample_table, SouthpawSampleDesc, SouthpawSampleView, DMA_BLOCK_BYTES, GBA_ROM_BUS_BASE,
    MIN_TABLE_ENTRIES, SOUTHPAW_DESC_SIZE,
};
pub use synth::{AccuracySynth, StudioSynth};
pub use version::{
    SouthpawFamily, SouthpawVersion, SOUTHPAW_DMA_IRQ_CNT, SOUTHPAW_ECHO_DMA_CNT,
};
pub use voice::{
    DirectSoundChannel, DualDmaHardwareState, EchoRingBuffer, EchoVoice, ECHO_10512_FRAME_SAMPLES,
    ECHO_6690_FRAME_SAMPLES, ECHO_RING_SLOTS,
};
