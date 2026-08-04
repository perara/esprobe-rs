//! Hardware-neutral contracts for the ESP32-C3 carrier board.

pub mod safety;

/// The credential codec, shared with the host tool.
pub use esprobe_protocol::wifi as wifi_credentials;
pub mod actuator;
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

    // The revision-2 assignment, which is what the boards in use actually run.
    //
    // The v1.0 schematic put these on 5/6/7/4/3/2/1 and that was the default
    // here long after the hardware moved, because the deployed firmware was
    // built with `PIN_*` overrides on the command line and nothing compared
    // the two. `esprobe pin-map` asks the board what it is really using, and
    // it answered with this — so this is now what the source says as well.
    // A v1.0 board still builds, by passing the old numbers as overrides.
    pub const DISP_TX: i32 = pin(option_env!("PIN_DISP_TX"), 4);
    pub const DISP_RX: i32 = pin(option_env!("PIN_DISP_RX"), 3);
    pub const RESET_ALL: i32 = pin(option_env!("PIN_RESET_ALL"), 21);
    pub const PROG_SWDIO: i32 = pin(option_env!("PIN_SWDIO"), 1);
    pub const PROG_SWCLK: i32 = pin(option_env!("PIN_SWCLK"), 2);
    pub const ASW_S1: i32 = pin(option_env!("PIN_ASW_S1"), 20);
    pub const ASW_S0: i32 = pin(option_env!("PIN_ASW_S0"), 10);

    // The motor driver driving the board's actuator: one H-bridge per winding.
    //
    // These four are what revision 2 left free, which is why the board uses
    // them. `AIN1`/`AIN2` are coil A and `BIN1`/`BIN2` coil B; the pair of
    // inputs on one bridge is what picks direction, so the two halves of a
    // winding must stay on the same bridge.
    pub const STEP_AIN1: i32 = pin(option_env!("PIN_STEP_AIN1"), 7);
    pub const STEP_AIN2: i32 = pin(option_env!("PIN_STEP_AIN2"), 6);
    pub const STEP_BIN1: i32 = pin(option_env!("PIN_STEP_BIN1"), 0);
    pub const STEP_BIN2: i32 = pin(option_env!("PIN_STEP_BIN2"), 5);

    /// Every pin this firmware claims, for reclaiming them from their reset
    /// functions at start-up.
    pub const CLAIMED: [i32; 11] = [
        DISP_TX,
        DISP_RX,
        RESET_ALL,
        PROG_SWDIO,
        PROG_SWCLK,
        ASW_S1,
        ASW_S0,
        STEP_AIN1,
        STEP_AIN2,
        STEP_BIN1,
        STEP_BIN2,
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

    /// The default has to be what `esprobe pin-map` reports off a board.
    ///
    /// It was not, for a while: the source said the v1.0 schematic while every
    /// deployed board ran revision 2, because the firmware was built with
    /// `PIN_*` overrides and nothing ever compared the two. Anyone reading the
    /// source to find out which pin a signal is on got the wrong answer, and
    /// the only thing that would have caught it is asking the hardware.
    #[test]
    fn the_default_pinmap_is_what_a_board_reports() {
        assert_eq!(
            [
                pinmap::PROG_SWDIO,
                pinmap::PROG_SWCLK,
                pinmap::RESET_ALL,
                pinmap::ASW_S0,
                pinmap::ASW_S1,
                pinmap::DISP_TX,
                pinmap::DISP_RX,
            ],
            [1, 2, 21, 10, 20, 4, 3],
            "does not match `esprobe pin-map` on revision 2"
        );
    }

    #[test]
    fn the_actuator_sits_on_what_revision_two_left_free() {
        assert_eq!(
            [
                pinmap::STEP_AIN1,
                pinmap::STEP_AIN2,
                pinmap::STEP_BIN1,
                pinmap::STEP_BIN2,
            ],
            [7, 6, 0, 5]
        );
    }

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
    fn programming_mux_truth_table_matches_schematic_page_five() {
        assert_eq!(ProgrammingTarget::Stm32.selector(), (false, false));
        assert_eq!(ProgrammingTarget::Aux2.selector(), (true, false));
        assert_eq!(ProgrammingTarget::Aux0.selector(), (false, true));
        assert_eq!(ProgrammingTarget::Aux1.selector(), (true, true));
    }
}
