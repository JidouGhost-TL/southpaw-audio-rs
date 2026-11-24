//! Error types for Southpaw GBA audio parsing and synthesis.

use core::fmt;

/// Errors returned when parsing Southpaw sample descriptors or configuring synthesis.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// Input buffer ended before a complete descriptor or PCM payload could be read.
    UnexpectedEof,
    /// Descriptor field or table header failed structural validation.
    InvalidHeader(&'static str),
    /// Loop range exceeded the PCM payload length.
    InvalidLoopRange {
        /// Loop start offset in bytes.
        loop_start: u32,
        /// Total PCM payload length in bytes.
        pcm_length: usize,
    },
    /// Sample index was out of range for the descriptor table.
    InvalidSample(u32),
    /// Invalid synthesis parameter.
    InvalidParameter(&'static str),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnexpectedEof => {
                write!(f, "Unexpected end of data while parsing Southpaw stream")
            }
            Self::InvalidHeader(msg) => write!(f, "Invalid Southpaw header: {msg}"),
            Self::InvalidLoopRange {
                loop_start,
                pcm_length,
            } => write!(
                f,
                "Invalid Southpaw loop start {loop_start} for {pcm_length}-byte sample"
            ),
            Self::InvalidSample(id) => write!(f, "Invalid Southpaw sample index {id}"),
            Self::InvalidParameter(msg) => write!(f, "Invalid Southpaw parameter: {msg}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// Convenience result alias for `southpaw-audio`.
pub type Result<T> = core::result::Result<T, Error>;
