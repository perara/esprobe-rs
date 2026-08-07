//! Hardware-neutral contracts for the ESP32-C3 SWD bridge.
//!
//! This crate is the probe and nothing else: a debug port on two pins, a reset
//! line, and an optional UART to whatever is on the other end. It knows no
//! vendor, no chip family and no board.
//!
//! A board that carries more than a debug port — a mux, a motor, a second radio
//! — builds on top of this rather than inside it, defines its own commands in
//! the range [`esprobe_protocol::frame::EXTENSION_BASE`] upward, and keeps its
//! own pin assignment. Nothing here needs to know it exists.

pub mod safety;
pub mod swd;

/// The ESP32-C3 side of the wire: the pads, the GPSPI2 shifter, and the cycle
/// counter that times them.
///
/// In the library rather than beside the binary so a board that is more than a
/// probe can drive the same engine without forking it. Only builds for the
/// target — there is no GPSPI2 on a host.
#[cfg(target_os = "espidf")]
pub mod hardware;
#[cfg(target_os = "espidf")]
pub mod spi_wire;

/// The credential codec, shared with the host tool.
pub use esprobe_protocol::wifi as wifi_credentials;

/// The wire contract, shared with the host tool rather than copied.
pub use esprobe_protocol::clock as spi_clock;
pub use esprobe_protocol::frame as usb_bridge;

/// GPIO assignment for the debug port.
///
/// Deliberately not a runtime setting: the pins are claimed and driven during
/// start-up, so a firmware built for the wrong board drives outputs into
/// another board's outputs before anything could reconfigure it. That
/// contention is not theoretical — it heats whatever sits between them.
///
/// The defaults suit a bare devkit with flying leads. A board that puts these
/// elsewhere overrides them at build time; a board that carries more pins than
/// these declares its own map and does not extend this one.
pub mod pinmap {
    /// Parses a build-time pin override, rejecting anything the ESP32-C3 does
    /// not have. `const` so a bad value fails the build rather than the board.
    const fn pin(override_value: Option<&str>, default: i32) -> i32 {
        let Some(text) = override_value else {
            return default;
        };
        let bytes = text.as_bytes();
        let mut number = 0i32;
        let mut index = 0;
        while index < bytes.len() {
            let digit = bytes[index];
            assert!(
                digit >= b'0' && digit <= b'9',
                "pin override must be a number"
            );
            number = number * 10 + (digit - b'0') as i32;
            index += 1;
        }
        assert!(number <= 21, "the ESP32-C3 has no GPIO above 21");
        number
    }

    /// The debug port itself. These two are the whole probe.
    pub const SWDIO: i32 = pin(option_env!("PIN_SWDIO"), 1);
    pub const SWCLK: i32 = pin(option_env!("PIN_SWCLK"), 2);
    /// Driven low to hold the target in reset, released to its pull-up
    /// otherwise. Never driven high: the target's own pull-up sets the level,
    /// so a board whose target is unpowered is not back-fed through this pin.
    pub const RESET: i32 = pin(option_env!("PIN_RESET"), 21);

    /// An optional UART to the target, for a serial console or a vendor's ROM
    /// bootloader. `TX` runs to the target's receiver and `RX` from its
    /// transmitter — the direction is not a naming preference, and getting it
    /// backwards puts two transmitters on one wire, which is silent rather
    /// than noisy and is correspondingly hard to find.
    pub const UART_TX: i32 = pin(option_env!("PIN_UART_TX"), 3);
    pub const UART_RX: i32 = pin(option_env!("PIN_UART_RX"), 4);

    /// Every pin this firmware claims, for reclaiming them from their reset
    /// functions at start-up.
    pub const CLAIMED: [i32; 5] = [SWDIO, SWCLK, RESET, UART_TX, UART_RX];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_claimed_pin_is_one_the_chip_cannot_give_us() {
        // GPIO18 and GPIO19 are the USB D-/D+ pair this probe is reached over,
        // and GPIO12..=17 are the SPI flash the firmware runs from. Driving any
        // of them does not produce a signal, it produces a brick.
        for pin in pinmap::CLAIMED {
            assert!(
                !(18..=19).contains(&pin),
                "GPIO{pin} is the USB pair this board is talked to over"
            );
            assert!(
                !(12..=17).contains(&pin),
                "GPIO{pin} belongs to the SPI flash"
            );
        }
    }

    #[test]
    fn every_claimed_pin_is_distinct() {
        // Two signals on one pad means one of them is driving the other's net.
        let mut seen = pinmap::CLAIMED;
        seen.sort_unstable();
        let mut index = 1;
        while index < seen.len() {
            assert_ne!(seen[index - 1], seen[index], "a pin is claimed twice");
            index += 1;
        }
    }

    #[test]
    fn the_transmit_pin_is_the_one_wired_to_the_targets_receiver() {
        // Not a naming preference: reversed, two transmitters share one wire
        // and two receivers share the other, which reads as "alive and silent"
        // on both ends. That cost a day to find on a real harness once.
        assert_ne!(pinmap::UART_TX, pinmap::UART_RX);
    }

    #[test]
    fn the_debug_port_does_not_share_a_pad_with_the_serial_port() {
        for uart in [pinmap::UART_TX, pinmap::UART_RX] {
            assert_ne!(uart, pinmap::SWDIO);
            assert_ne!(uart, pinmap::SWCLK);
            assert_ne!(uart, pinmap::RESET);
        }
    }
}
