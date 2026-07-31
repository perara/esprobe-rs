//! Hardware-neutral contracts for the ESP32-C3 carrier board.

pub mod safety;

/// The credential codec, shared with the host tool.
pub use esprobe_protocol::wifi as wifi_credentials;
pub mod stm32g0;
pub mod swd;

/// The wire contract, shared with the host tool rather than copied.
pub use esprobe_protocol::clock as spi_clock;
pub use esprobe_protocol::frame as usb_bridge;

/// GPIO assignment for the board this firmware is built for.
///
/// The v1.0 carrier board's assignment is the default; a revision that moves the
/// connector overrides it at build time. This is deliberately not a runtime
/// setting: the pins are claimed and driven during start-up, so a firmware
/// built for the wrong board drives outputs into another board's outputs
/// before anything could reconfigure it. That contention is not theoretical —
/// it heats the analog switch that sits between them.
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

    pub const DISP_TX: i32 = pin(option_env!("PIN_DISP_TX"), 5);
    pub const DISP_RX: i32 = pin(option_env!("PIN_DISP_RX"), 6);
    pub const RESET_ALL: i32 = pin(option_env!("PIN_RESET_ALL"), 7);
    pub const PROG_SWDIO: i32 = pin(option_env!("PIN_SWDIO"), 4);
    pub const PROG_SWCLK: i32 = pin(option_env!("PIN_SWCLK"), 3);
    pub const ASW_S1: i32 = pin(option_env!("PIN_ASW_S1"), 2);
    pub const ASW_S0: i32 = pin(option_env!("PIN_ASW_S0"), 1);

    /// Every pin this firmware claims, for reclaiming them from their reset
    /// functions at start-up.
    pub const CLAIMED: [i32; 7] = [
        DISP_TX, DISP_RX, RESET_ALL, PROG_SWDIO, PROG_SWCLK, ASW_S1, ASW_S0,
    ];
}

/// analog switch programming target selected by `(ASW_S0, ASW_S1)`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProgrammingTarget {
    Stm32,
    Aux2,
    Aux0,
    Aux1,
}

impl ProgrammingTarget {
    #[must_use]
    pub const fn selector(self) -> (bool, bool) {
        match self {
            Self::Stm32 => (false, false),
            Self::Aux2 => (true, false),
            Self::Aux0 => (false, true),
            Self::Aux1 => (true, true),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_pinmap_is_the_revision_one_schematic() {
        assert_eq!(
            [
                pinmap::DISP_TX,
                pinmap::DISP_RX,
                pinmap::RESET_ALL,
                pinmap::PROG_SWDIO,
                pinmap::PROG_SWCLK,
                pinmap::ASW_S1,
                pinmap::ASW_S0,
            ],
            [5, 6, 7, 4, 3, 2, 1]
        );
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
    fn programming_mux_truth_table_matches_schematic_page_five() {
        assert_eq!(ProgrammingTarget::Stm32.selector(), (false, false));
        assert_eq!(ProgrammingTarget::Aux2.selector(), (true, false));
        assert_eq!(ProgrammingTarget::Aux0.selector(), (false, true));
        assert_eq!(ProgrammingTarget::Aux1.selector(), (true, true));
    }
}
