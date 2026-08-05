//! Hardware-neutral contracts for the ESP32-C3 IR control hub.

pub mod link;

/// Where the persistent link is served.
///
/// In the library rather than beside the handler, because the page has to name
/// the same string and a test can only check that if both sides can see it.
#[must_use]
pub const fn ws_link_path() -> &'static str {
    "/ws/stepper"
}
pub mod safety;

/// The credential codec, shared with the host tool.
pub use esprobe_protocol::wifi as wifi_credentials;
pub mod stepper;
pub mod stm32g0;

/// The trackpad page and the routes it talks to.
///
/// In the library rather than beside the handlers so a host test can read it.
/// The page is the one part of this firmware that cannot be checked by running
/// it — there is no browser on the bench — so what can be checked is that it
/// stays small, stays self-contained, and only calls routes that exist.
pub mod trackpad {
    /// The page itself, served from flash.
    ///
    /// One file with no external references: the station is reached over its
    /// own access point, which has no route to a CDN, so a page that pulled a
    /// framework would render as a blank rectangle exactly when it is needed.
    pub const PAGE: &str = include_str!("trackpad.html");

    /// A validator computed at build time from the page.
    ///
    /// FNV-1a, which is a few lines and enough for the job: it only has to
    /// change when the bytes change, not resist anyone choosing bytes to
    /// collide with it.
    #[must_use]
    pub const fn fnv1a(bytes: &[u8]) -> u64 {
        let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
        let mut index = 0;
        while index < bytes.len() {
            hash ^= bytes[index] as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
            index += 1;
        }
        hash
    }

    /// The page's ETag, so a reload costs a 304 rather than nine kilobytes
    /// over the same access point that is carrying the jog commands.
    pub const ETAG_HASH: u64 = fnv1a(PAGE.as_bytes());

    /// Where the page is served.
    pub const PATH: &str = "/stepper";
    pub const STATUS: &str = "/api/v1/stepper";
    pub const JOG: &str = "/api/v1/stepper/jog";
    pub const MOVE: &str = "/api/v1/stepper/move";
    pub const STOP: &str = "/api/v1/stepper/stop";
    pub const RELEASE: &str = "/api/v1/stepper/release";
    /// Hold one excitation state so the bridges can be measured.
    pub const HOLD: &str = "/api/v1/stepper/hold";
    /// Drive each bridge pin and read the pad back.
    pub const SELFTEST: &str = "/api/v1/stepper/selftest";
    /// Retune the acceleration while the motor runs.
    pub const CONFIG: &str = "/api/v1/stepper/config";

    /// Every route the firmware answers on, for the page to be checked against.
    pub const ROUTES: [&str; 8] = [STATUS, JOG, MOVE, STOP, RELEASE, HOLD, SELFTEST, CONFIG];

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn the_page_only_calls_routes_that_exist() {
            // A mistyped path in the page is invisible until someone drags the
            // pad and the motor does not move, with a 404 nobody is looking at.
            let mut checked = 0;
            let mut rest = PAGE;
            while let Some(at) = rest.find("\"/api/") {
                let tail = &rest[at + 1..];
                let end = tail.find('"').expect("unterminated path literal");
                let path = &tail[..end];
                assert!(
                    ROUTES.contains(&path),
                    "the page calls {path}, which the firmware does not serve"
                );
                checked += 1;
                rest = &tail[end..];
            }
            assert!(checked >= 5, "only found {checked} calls; did the page change shape?");
        }

        #[test]
        fn the_page_fetches_nothing_from_outside_the_station() {
            // The station is reached over its own access point, which has no
            // route anywhere else. Anything loaded from a CDN is a spinner.
            for marker in ["http://", "https://", "//cdn", "src=\"//"] {
                assert!(
                    !PAGE.contains(marker),
                    "the page references {marker}, which will not resolve on the station's AP"
                );
            }
        }

        #[test]
        fn the_page_stays_small_enough_to_serve_from_flash() {
            // Not a hard limit, a tripwire. It fired once, correctly, when the
            // page grew a websocket client and a REST fallback beside it.
            //
            // The reason for the original figure has weakened since: commands
            // no longer share the air with the page, because they are on their
            // own socket, and the page is served with an ETag so a returning
            // browser gets a 304 and no body at all. What is left is one
            // transfer on a cold load. If this fires again the answer is
            // probably to serve it gzipped rather than to raise it a second
            // time — roughly two thirds of it is comments and whitespace.
            const BUDGET: usize = 24 * 1024;
            assert!(
                PAGE.len() <= BUDGET,
                "the page is {} bytes, over the {BUDGET}-byte budget",
                PAGE.len()
            );
        }

        #[test]
        fn the_validator_follows_the_content() {
            // If this did not change with the bytes, a cached page would
            // survive a firmware update and the trackpad would keep talking to
            // routes that had moved.
            assert_ne!(fnv1a(b"a"), fnv1a(b"b"));
            assert_ne!(fnv1a(PAGE.as_bytes()), fnv1a(b""));
            assert_eq!(ETAG_HASH, fnv1a(PAGE.as_bytes()));
        }
    }
}
pub mod swd;

/// The wire contract, shared with the host tool rather than copied.
pub use esprobe_protocol::clock as spi_clock;
pub use esprobe_protocol::frame as usb_bridge;

/// GPIO assignment for the board this firmware is built for.
///
/// The v1.0 IR board's assignment is the default; a revision that moves the
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
    // Swapped 2026-08-06 after direction-finding on the real harness: a break
    // driven from the STM32's PA10 side arrived at this chip's receiver, and
    // real UART bytes out of the STM32's PA9 (transmit-complete confirmed in
    // its status register) did not. So GPIO3 is the wire toward the STM32's
    // RX - our TX - and GPIO4 the wire from the STM32's TX - our RX. The old
    // assignment was exactly inverted: two transmitters fighting one wire,
    // two receivers listening to each other on the other, zero bytes ever
    // crossing, and register watches on both ends showing nothing but "alive
    // and silent" for a whole day.
    pub const DISP_TX: i32 = pin(option_env!("PIN_DISP_TX"), 3);
    pub const DISP_RX: i32 = pin(option_env!("PIN_DISP_RX"), 4);
    pub const RESET_ALL: i32 = pin(option_env!("PIN_RESET_ALL"), 21);
    pub const PROG_SWDIO: i32 = pin(option_env!("PIN_SWDIO"), 1);
    pub const PROG_SWCLK: i32 = pin(option_env!("PIN_SWCLK"), 2);
    pub const ASW_S1: i32 = pin(option_env!("PIN_ASW_S1"), 20);
    pub const ASW_S0: i32 = pin(option_env!("PIN_ASW_S0"), 10);

    // The DRV8833 driving the station's stepper: one H-bridge per winding.
    //
    // These four are what revision 2 left free, which is why the station uses
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

/// CD4052 programming target selected by `(ASW_S0, ASW_S1)`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProgrammingTarget {
    Stm32,
    Gps,
    Dwm0,
    Dwm1,
}

impl ProgrammingTarget {
    #[must_use]
    pub const fn selector(self) -> (bool, bool) {
        match self {
            Self::Stm32 => (false, false),
            Self::Gps => (true, false),
            Self::Dwm0 => (false, true),
            Self::Dwm1 => (true, true),
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
    fn the_stepper_sits_on_what_revision_two_left_free() {
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
        assert_eq!(ProgrammingTarget::Gps.selector(), (true, false));
        assert_eq!(ProgrammingTarget::Dwm0.selector(), (false, true));
        assert_eq!(ProgrammingTarget::Dwm1.selector(), (true, true));
    }
}
