//! GPSPI2 clock-divider arithmetic for the hardware SWD wire.
//!
//! Kept out of the register driver so the divider search can be tested on the
//! host: a wrong divider is the difference between a 4 MHz SWD wire and one
//! fast enough to violate the target's rise time.

/// GPSPI2's master clock source with `mst_clk_sel` set to PLL_F80M.
pub const SOURCE_CLOCK_HZ: u32 = 80_000_000;
/// Above this, flying bench leads stop being trustworthy. A board that puts an
/// analog switch or a long harness between the probe and the target is slower
/// still, and should ask for a lower clock rather than raise this.
pub const MAX_CLOCK_HZ: u32 = 20_000_000;
pub const MIN_CLOCK_HZ: u32 = 100_000;
/// Fast enough that the transport, not the wire, is the limit, and still inside
/// what unshielded bench leads carry reliably. Direct-wired to a devkit there is
/// headroom above this; through anything longer there may not be.
pub const DEFAULT_CLOCK_HZ: u32 = 8_000_000;

/// A programmed `SPI_CLOCK` value and the SWCLK frequency it produces.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Divider {
    pub register: u32,
    pub clock_hz: u32,
}

/// Chooses the `SPI_CLOCK` divider closest to `target_hz` without exceeding it.
///
/// The peripheral divides the source clock by `pre * n`, where `pre` is 1..=16
/// and `n` is 2..=64. `n` also sets the period in source cycles, so the high
/// phase is programmed at `n / 2` for an even mark-space ratio.
#[must_use]
pub fn divider_for(source_hz: u32, target_hz: u32) -> Divider {
    let target = target_hz.clamp(MIN_CLOCK_HZ, MAX_CLOCK_HZ);
    let mut best = (1u32, 2u32);
    let mut best_clock = 0;
    for n in 2..=64u32 {
        for pre in 1..=16u32 {
            let candidate = source_hz / (pre * n);
            // Never round up: overclocking the wire is a hardware risk, and
            // being a few percent slow costs nothing.
            if candidate <= target && candidate > best_clock {
                best_clock = candidate;
                best = (pre, n);
            }
        }
    }
    let (pre, n) = best;
    let high = n / 2;
    Divider {
        register: (pre - 1) << 18 | (n - 1) << 12 | (high - 1) << 6 | (n - 1),
        clock_hz: best_clock,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dividers_never_exceed_the_requested_swd_clock() {
        for target in [
            100_000, 250_000, 1_000_000, 4_000_000, 8_000_000, 10_000_000,
        ] {
            let divider = divider_for(SOURCE_CLOCK_HZ, target);
            assert!(
                divider.clock_hz <= target,
                "{target} Hz resolved to {} Hz",
                divider.clock_hz
            );
            let error = f64::from(target - divider.clock_hz) / f64::from(target);
            assert!(
                error < 0.1,
                "{target} Hz resolved to {} Hz",
                divider.clock_hz
            );
        }
    }

    #[test]
    fn common_clocks_divide_exactly() {
        for target in [1_000_000, 2_000_000, 4_000_000, 5_000_000, 10_000_000] {
            assert_eq!(divider_for(SOURCE_CLOCK_HZ, target).clock_hz, target);
        }
    }

    #[test]
    fn divider_fields_stay_inside_their_register_widths() {
        for target in [MIN_CLOCK_HZ, DEFAULT_CLOCK_HZ, MAX_CLOCK_HZ] {
            let register = divider_for(SOURCE_CLOCK_HZ, target).register;
            assert_eq!(register >> 31, 0, "the sysclk bypass must stay off");
            assert!(register >> 22 == 0, "no field may overflow into clkdiv_pre");
        }
    }

    #[test]
    fn requests_outside_the_supported_range_are_clamped() {
        assert_eq!(
            divider_for(SOURCE_CLOCK_HZ, 1),
            divider_for(SOURCE_CLOCK_HZ, MIN_CLOCK_HZ)
        );
        assert_eq!(
            divider_for(SOURCE_CLOCK_HZ, 80_000_000),
            divider_for(SOURCE_CLOCK_HZ, MAX_CLOCK_HZ)
        );
    }

    #[test]
    fn the_high_phase_is_half_the_period() {
        let register = divider_for(SOURCE_CLOCK_HZ, DEFAULT_CLOCK_HZ).register;
        let n = (register >> 12 & 0x3f) + 1;
        assert_eq!((register >> 6 & 0x3f) + 1, n / 2);
        assert_eq!(register & 0x3f, n - 1);
    }
}
