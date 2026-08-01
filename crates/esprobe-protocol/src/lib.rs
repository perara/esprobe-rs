//! The wire contract between the ESP32-C3 SWD bridge and its host tool.
//!
//! Both ends must agree on this byte for byte, and they are built for
//! different architectures by different toolchains, so it lives in one crate
//! that each depends on rather than being copied. The firmware side is
//! `no_std`; nothing here allocates.

#![cfg_attr(not(test), no_std)]

pub mod clock;
pub mod frame;
pub mod json;
pub mod wifi;
