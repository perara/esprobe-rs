//! Fail-closed electrical policy shared by firmware and host tests.
//!
//! The question this answers is "may the probe drive a level onto these pads?",
//! and the only evidence available before driving anything is what the pads
//! read while released. An unpowered target is the case that matters: driving a
//! high level into one back-feeds its supply through the pad's protection
//! diode, which is a way to damage a board rather than debug it.

/// Levels read from the debug pads while the probe is driving neither.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReleasedSwdState {
    /// What a powered, idle ARM debug port looks like: SWDIO pulled up and
    /// SWCLK pulled down. This is the ADIv5-recommended arrangement and is what
    /// every part observed so far resets to, but it is a convention rather than
    /// a guarantee — a target that idles otherwise reads as one of the cases
    /// below and is refused, which is the safe direction to be wrong in.
    ResetDefaults,
    /// SWDIO is low, so target power and a safe high-level drive are unproven.
    SwdioLow,
    /// Target pull-up is visible, but SWCLK differs from its idle default.
    ClockUnexpectedHigh,
}

impl ReleasedSwdState {
    #[must_use]
    pub const fn from_levels(swdio: bool, swclk: bool) -> Self {
        match (swdio, swclk) {
            (false, _) => Self::SwdioLow,
            (true, false) => Self::ResetDefaults,
            (true, true) => Self::ClockUnexpectedHigh,
        }
    }

    /// Whether the target-side SWDIO pull-up provides minimum evidence that a
    /// high-driving operation will not energize an unpowered target.
    #[must_use]
    pub const fn permits_high_drive(self) -> bool {
        !matches!(self, Self::SwdioLow)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_swdio_never_permits_a_high_drive() {
        assert!(!ReleasedSwdState::from_levels(false, false).permits_high_drive());
        assert!(!ReleasedSwdState::from_levels(false, true).permits_high_drive());
    }

    #[test]
    fn reset_default_levels_are_distinguished_from_an_unexpected_clock() {
        assert_eq!(
            ReleasedSwdState::from_levels(true, false),
            ReleasedSwdState::ResetDefaults
        );
        assert_eq!(
            ReleasedSwdState::from_levels(true, true),
            ReleasedSwdState::ClockUnexpectedHigh
        );
        assert!(ReleasedSwdState::ResetDefaults.permits_high_drive());
    }
}
