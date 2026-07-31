//! Fail-closed electrical policy shared by firmware and host tests.

/// Passive levels observed through mux position `00`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReleasedSwdState {
    /// STM32G0 reset defaults: SWDIO pull-up and SWCLK pull-down are visible.
    ResetDefaults,
    /// SWDIO is low, so target power and a safe high-level drive are unproven.
    SwdioLow,
    /// Target pull-up is visible, but SWCLK differs from its reset default.
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

    /// Whether the STM32-side SWDIO pull-up provides minimum evidence that a
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
