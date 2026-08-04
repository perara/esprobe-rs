//! Driving a bipolar stepper through a DRV8833, and deciding when not to.
//!
//! Hardware-neutral on purpose: this is the part that can be wrong in a way a
//! test can catch. The ESP-IDF side owns four output pins and does what
//! [`Planner::poll`] tells it.
//!
//! # The bridge
//!
//! A DRV8833 is two H-bridges. Each takes two inputs, and the pair means:
//!
//! | xIN1 | xIN2 | bridge |
//! | --- | --- | --- |
//! | 0 | 0 | coast — outputs floating, coil de-energised |
//! | 1 | 0 | forward |
//! | 0 | 1 | reverse |
//! | 1 | 1 | brake — both outputs low |
//!
//! One bridge per winding, so `AIN1/AIN2` drive coil A and `BIN1/BIN2` coil B.
//! Coast is what releases the motor; brake shorts the winding and is not used
//! here, because a stepper that is braked still resists being turned and still
//! dissipates whatever the back-EMF drives through it.
//!
//! # Why it releases itself
//!
//! A stepper at standstill with both coils energised draws its full rated
//! current and turns all of it into heat — the motor is not moving, so none of
//! it becomes work. The DRV8833 has no current chopping, so nothing limits that
//! except the winding resistance and the supply.
//!
//! So holding is treated as a *timed* state, not a resting one: after the last
//! step the planner holds position for [`HOLD_AFTER_MOTION_US`] and then
//! coasts. A station that needs indefinite holding torque needs a driver that
//! can chop current, not a longer timeout here.
//!
//! # Why jogging expires
//!
//! The trackpad sends a velocity and expects to send another shortly. If the
//! browser tab closes, the Wi-Fi drops, or the phone locks mid-drag, the last
//! thing the firmware heard was "keep going". Every jog therefore carries a
//! deadline and the motor stops when it passes — a stepper that keeps running
//! because nobody said stop is how a station drives itself into its end stop.

/// The four bridge inputs, in the order the schematic names them.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Coils {
    pub ain1: bool,
    pub ain2: bool,
    pub bin1: bool,
    pub bin2: bool,
}

impl Coils {
    /// Both bridges coasting: no current, no holding torque.
    pub const RELEASED: Self = Self {
        ain1: false,
        ain2: false,
        bin1: false,
        bin2: false,
    };

    const fn new(a: i8, b: i8) -> Self {
        Self {
            ain1: a > 0,
            ain2: a < 0,
            bin1: b > 0,
            bin2: b < 0,
        }
    }

    /// Is any winding drawing current?
    #[must_use]
    pub const fn energised(self) -> bool {
        self.ain1 || self.ain2 || self.bin1 || self.bin2
    }
}

/// How finely the sequence divides one electrical revolution.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum StepMode {
    /// Four states, both windings always energised. Most torque, most heat,
    /// and the coarsest motion.
    Full,
    /// Eight states, alternating one winding and two. Half the step angle and
    /// noticeably smoother at low speed, for about 70% of the torque on the
    /// single-winding states.
    #[default]
    Half,
}

impl StepMode {
    #[must_use]
    pub const fn states(self) -> u8 {
        match self {
            Self::Full => 4,
            Self::Half => 8,
        }
    }
}

/// Full-step: both windings on, four states.
const FULL: [Coils; 4] = [
    Coils::new(1, 1),
    Coils::new(-1, 1),
    Coils::new(-1, -1),
    Coils::new(1, -1),
];

/// Half-step: alternates one winding and two, eight states.
const HALF: [Coils; 8] = [
    Coils::new(1, 0),
    Coils::new(1, 1),
    Coils::new(0, 1),
    Coils::new(-1, 1),
    Coils::new(-1, 0),
    Coils::new(-1, -1),
    Coils::new(0, -1),
    Coils::new(1, -1),
];

/// Where in the excitation sequence the rotor is held.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sequencer {
    index: u8,
    mode: StepMode,
}

impl Sequencer {
    #[must_use]
    pub const fn new(mode: StepMode) -> Self {
        Self { index: 0, mode }
    }

    /// The excitation for the current position.
    #[must_use]
    pub const fn coils(self) -> Coils {
        match self.mode {
            StepMode::Full => FULL[(self.index & 3) as usize],
            StepMode::Half => HALF[(self.index & 7) as usize],
        }
    }

    /// Jump to a numbered state, for driving a known pattern on purpose.
    pub const fn set_index(&mut self, index: u8) {
        self.index = index % self.mode.states();
    }

    /// Move one step. `forward` picks the direction; the index wraps.
    pub const fn advance(&mut self, forward: bool) -> Coils {
        let n = self.mode.states();
        self.index = if forward {
            (self.index + 1) % n
        } else {
            (self.index + n - 1) % n
        };
        self.coils()
    }
}

/// How long the motor keeps position after the last step before coasting.
pub const HOLD_AFTER_MOTION_US: u64 = 400_000;

/// How long a jog is honoured without a refresh.
///
/// The trackpad refreshes several times a second while a finger is down, so
/// this only expires when something stopped talking.
pub const JOG_TIMEOUT_US: u64 = 400_000;

/// The fastest we will step, whatever is asked for.
///
/// A stepper commanded past the speed its torque can follow does not go faster,
/// it slips — and a slipping motor loses position silently, which is worse than
/// being slow, because everything downstream still believes the count.
///
/// It is also half the rate the bridges are refreshed at, so every step lands
/// within half a poll of when it was due. Raising this without shortening
/// `stepper_hw::POLL_PERIOD_US` buys nothing: the steps would just bunch onto
/// the poll boundaries.
pub const MAX_STEPS_PER_S: u32 = 1_000;

/// The longest a diagnostic hold will energise a winding.
///
/// This exists to be measured with a meter, and a meter takes seconds, not
/// minutes. Without current limiting the winding is a resistor across the
/// supply for as long as it is held, so the hold ends by itself whatever the
/// caller asked for and whether or not anyone is still listening.
pub const MAX_HOLD_US: u64 = 10_000_000;

/// What the driver should be doing right now.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Output {
    pub coils: Coils,
    /// True when this poll crossed a step boundary, for callers that count.
    pub stepped: bool,
}

/// Decides when to step, how far, and when to let go.
#[derive(Clone, Copy, Debug)]
pub struct Planner {
    seq: Sequencer,
    /// Net steps since start-up. Signed, and not wrapped: at the maximum rate
    /// this takes over a hundred million years to overflow.
    position: i64,
    /// Steps left in a bounded move. `None` while jogging.
    remaining: Option<u32>,
    /// Signed step rate; zero means not moving.
    rate: i32,
    /// When the current command stops being honoured.
    expires_at_us: u64,
    /// When the next step is due.
    next_step_us: u64,
    /// When holding position stops and the windings are released.
    release_at_us: u64,
    holding: bool,
}

impl Default for Planner {
    fn default() -> Self {
        Self::new(StepMode::default())
    }
}

impl Planner {
    #[must_use]
    pub const fn new(mode: StepMode) -> Self {
        Self {
            seq: Sequencer::new(mode),
            position: 0,
            remaining: None,
            rate: 0,
            expires_at_us: 0,
            next_step_us: 0,
            release_at_us: 0,
            holding: false,
        }
    }

    #[must_use]
    pub const fn position(&self) -> i64 {
        self.position
    }

    #[must_use]
    pub const fn rate(&self) -> i32 {
        self.rate
    }

    #[must_use]
    pub const fn is_moving(&self) -> bool {
        self.rate != 0
    }

    /// Steps left in a bounded move, or `None` while jogging or stopped.
    #[must_use]
    pub const fn remaining(&self) -> Option<u32> {
        self.remaining
    }

    /// Run at a signed rate until told otherwise — or until the jog expires.
    ///
    /// Call again to refresh it. A rate of zero stops, which is what the
    /// trackpad sends when a finger lifts inside the dead zone.
    pub fn jog(&mut self, steps_per_s: i32, now_us: u64) {
        let clamped = steps_per_s.clamp(-(MAX_STEPS_PER_S as i32), MAX_STEPS_PER_S as i32);
        if clamped == 0 {
            self.stop(now_us);
            return;
        }
        self.remaining = None;
        self.begin(clamped, now_us);
        self.expires_at_us = now_us + JOG_TIMEOUT_US;
    }

    /// Move a fixed number of steps and stop. Negative goes the other way.
    ///
    /// No deadline: a bounded move ends by itself, so there is nothing for a
    /// watchdog to rescue.
    pub fn move_by(&mut self, steps: i32, steps_per_s: u32, now_us: u64) {
        if steps == 0 || steps_per_s == 0 {
            self.stop(now_us);
            return;
        }
        let rate = steps_per_s.min(MAX_STEPS_PER_S) as i32;
        self.remaining = Some(steps.unsigned_abs());
        self.begin(if steps > 0 { rate } else { -rate }, now_us);
        self.expires_at_us = u64::MAX;
    }

    fn begin(&mut self, rate: i32, now_us: u64) {
        // Restarting an already-running motion must not reset the step clock,
        // or a trackpad refreshing faster than the step interval would hold the
        // next step permanently in the future and the motor would sit still
        // while being told to move.
        if self.rate == 0 {
            self.next_step_us = now_us;
        }
        self.rate = rate;
        self.holding = true;
    }

    /// Stop stepping but keep position, until the hold time runs out.
    pub fn stop(&mut self, now_us: u64) {
        self.rate = 0;
        self.remaining = None;
        self.expires_at_us = u64::MAX;
        if self.holding {
            self.release_at_us = now_us + HOLD_AFTER_MOTION_US;
        }
    }

    /// Hold one excitation state so the bridges can be measured.
    ///
    /// Not part of driving a motor: this is for putting a meter on `AOUT`/`BOUT`
    /// and finding out whether the driver responds to its inputs at all. In
    /// half-step, state 0 energises coil A alone and state 2 coil B alone,
    /// which is what isolates one bridge from the other.
    ///
    /// Always bounded by [`MAX_HOLD_US`].
    pub fn hold_state(&mut self, state: u8, hold_us: u64, now_us: u64) {
        self.rate = 0;
        self.remaining = None;
        self.expires_at_us = u64::MAX;
        self.seq.set_index(state);
        self.holding = true;
        self.release_at_us = now_us + hold_us.min(MAX_HOLD_US);
    }

    /// Drop the windings now, without waiting for the hold to expire.
    pub fn release(&mut self, now_us: u64) {
        self.stop(now_us);
        self.holding = false;
    }

    /// Advance to `now_us` and report what the bridges should be driving.
    ///
    /// At most one step per call. The caller polls faster than the step rate,
    /// so catching up in a burst would only bunch steps together — which a
    /// stepper answers by slipping, not by moving faster.
    pub fn poll(&mut self, now_us: u64) -> Output {
        if self.rate != 0 && now_us >= self.expires_at_us {
            self.stop(now_us);
        }

        if self.rate != 0 && now_us >= self.next_step_us {
            let forward = self.rate > 0;
            let coils = self.seq.advance(forward);
            self.position += if forward { 1 } else { -1 };
            self.next_step_us = self.next_step_us.max(now_us).saturating_add(self.interval_us());

            if let Some(left) = self.remaining {
                let left = left - 1;
                self.remaining = Some(left);
                if left == 0 {
                    self.stop(now_us);
                }
            }
            return Output {
                coils,
                stepped: true,
            };
        }

        if self.holding && self.rate == 0 && now_us >= self.release_at_us {
            self.holding = false;
        }

        Output {
            coils: if self.holding {
                self.seq.coils()
            } else {
                Coils::RELEASED
            },
            stepped: false,
        }
    }

    fn interval_us(&self) -> u64 {
        let rate = self.rate.unsigned_abs().max(1) as u64;
        (1_000_000 / rate).max(1)
    }
}

/// Pull one integer field out of a small JSON object.
///
/// A hand-rolled reader rather than a parser crate: the bodies this accepts are
/// one or two integer fields written by a page this firmware serves itself, and
/// a JSON parser is a lot of flash for that. It is deliberately strict about
/// nothing except the number — anything it cannot read comes back as `None` and
/// the caller answers 400.
#[must_use]
pub fn json_i32(body: &str, field: &str) -> Option<i32> {
    let mut search = 0;
    // The field must appear as a quoted key, so a value that happens to spell
    // another field's name cannot be mistaken for one.
    let needle = alloc_key(field);
    while let Some(found) = body[search..].find(needle.as_str()) {
        let after = search + found + needle.len();
        let rest = body[after..].trim_start();
        let Some(rest) = rest.strip_prefix(':') else {
            search = after;
            continue;
        };
        let rest = rest.trim_start();
        let (negative, digits) = match rest.strip_prefix('-') {
            Some(r) => (true, r),
            None => (false, rest),
        };
        let end = digits
            .find(|c: char| !c.is_ascii_digit())
            .unwrap_or(digits.len());
        if end == 0 {
            return None;
        }
        let magnitude: i64 = digits[..end].parse().ok()?;
        let signed = if negative { -magnitude } else { magnitude };
        return i32::try_from(signed).ok();
    }
    None
}

fn alloc_key(field: &str) -> String {
    let mut key = String::with_capacity(field.len() + 2);
    key.push('"');
    key.push_str(field);
    key.push('"');
    key
}

#[cfg(test)]
mod tests {
    use super::*;

    const SEC: u64 = 1_000_000;

    #[test]
    fn the_bridge_truth_table_is_the_datasheet_one() {
        // 0,0 coasts. Anything else drives, and no state ever asserts both
        // inputs of one bridge — that is brake, which this never wants.
        assert!(!Coils::RELEASED.energised());
        for mode in [StepMode::Full, StepMode::Half] {
            let mut seq = Sequencer::new(mode);
            for _ in 0..mode.states() {
                let c = seq.advance(true);
                assert!(!(c.ain1 && c.ain2), "coil A braked in {mode:?}");
                assert!(!(c.bin1 && c.bin2), "coil B braked in {mode:?}");
                assert!(c.energised(), "a drive state with no current in {mode:?}");
            }
        }
    }

    #[test]
    fn a_full_revolution_of_the_sequence_returns_to_where_it_started() {
        for mode in [StepMode::Full, StepMode::Half] {
            let mut seq = Sequencer::new(mode);
            let start = seq.coils();
            for _ in 0..mode.states() {
                seq.advance(true);
            }
            assert_eq!(seq.coils(), start, "{mode:?} did not close the cycle");
        }
    }

    #[test]
    fn stepping_back_undoes_stepping_forward() {
        for mode in [StepMode::Full, StepMode::Half] {
            let mut seq = Sequencer::new(mode);
            let start = seq.coils();
            // Past the wrap in both directions, so the modular arithmetic is
            // exercised rather than just the middle of the table.
            for _ in 0..(mode.states() * 3 + 1) {
                seq.advance(true);
            }
            for _ in 0..(mode.states() * 3 + 1) {
                seq.advance(false);
            }
            assert_eq!(seq.coils(), start, "{mode:?} is not reversible");
        }
    }

    #[test]
    fn half_stepping_alternates_one_winding_and_two() {
        let mut seq = Sequencer::new(StepMode::Half);
        let mut energised: Vec<usize> = Vec::new();
        for _ in 0..8 {
            let c = seq.advance(true);
            energised.push(usize::from(c.ain1 || c.ain2) + usize::from(c.bin1 || c.bin2));
        }
        // That alternation is the whole point of half-stepping; a table that
        // had two windings on throughout would be full-stepping with extra
        // states and none of the smoothness.
        for pair in energised.windows(2) {
            assert_ne!(pair[0], pair[1], "half-step did not alternate: {energised:?}");
        }
    }

    #[test]
    fn a_bounded_move_stops_at_exactly_the_step_it_was_asked_for() {
        let mut p = Planner::new(StepMode::Full);
        p.move_by(10, 1_000, 0);
        let mut steps = 0;
        // Poll well past the end; the extra time must not produce extra steps.
        for t in 0..30_000u64 {
            if p.poll(t).stepped {
                steps += 1;
            }
        }
        assert_eq!(steps, 10);
        assert_eq!(p.position(), 10);
        assert!(!p.is_moving());
    }

    #[test]
    fn a_negative_move_goes_the_other_way_and_lands_where_it_started() {
        let mut p = Planner::new(StepMode::Half);
        p.move_by(25, 1_000, 0);
        for t in 0..40_000u64 {
            p.poll(t);
        }
        assert_eq!(p.position(), 25);
        p.move_by(-25, 1_000, 40_000);
        for t in 40_000..90_000u64 {
            p.poll(t);
        }
        assert_eq!(p.position(), 0, "it did not come back");
    }

    #[test]
    fn a_jog_that_is_never_refreshed_stops_itself() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(500, 0);
        // Run past the deadline without refreshing, as a closed browser tab or
        // a dropped access point would.
        let mut last_step_at = 0;
        for t in 0..(2 * JOG_TIMEOUT_US) {
            if p.poll(t).stepped {
                last_step_at = t;
            }
        }
        assert!(!p.is_moving(), "the motor was still running with nobody home");
        assert!(
            last_step_at <= JOG_TIMEOUT_US,
            "it kept stepping {}us past the deadline",
            last_step_at - JOG_TIMEOUT_US
        );
    }

    #[test]
    fn refreshing_a_jog_keeps_it_running_and_does_not_stall_it() {
        let mut p = Planner::new(StepMode::Half);
        let mut steps = 0;
        // Refresh every 5 ms, far more often than the 10 ms step interval —
        // the case that stalled an earlier version, because each refresh
        // pushed the next step further out than the poll that would have taken
        // it.
        for t in 0..SEC {
            if t % 5_000 == 0 {
                p.jog(100, t);
            }
            if p.poll(t).stepped {
                steps += 1;
            }
        }
        assert!(
            (95..=105).contains(&steps),
            "expected about 100 steps in a second at 100/s, got {steps}"
        );
    }

    #[test]
    fn it_releases_the_windings_after_holding_for_a_while() {
        let mut p = Planner::new(StepMode::Full);
        p.move_by(2, 1_000, 0);
        for t in 0..5_000u64 {
            p.poll(t);
        }
        assert!(!p.is_moving());
        // Still holding position right after the move.
        assert!(p.poll(5_000).coils.energised(), "it let go immediately");
        // And coasting once the hold time is up, because a stepper standing
        // still with its windings on is a resistor.
        let released = p.poll(5_000 + HOLD_AFTER_MOTION_US + 1);
        assert_eq!(released.coils, Coils::RELEASED, "it never let go");
    }

    #[test]
    fn release_lets_go_without_waiting() {
        let mut p = Planner::new(StepMode::Full);
        p.move_by(5, 1_000, 0);
        for t in 0..3_000u64 {
            p.poll(t);
        }
        p.release(3_000);
        assert_eq!(p.poll(3_001).coils, Coils::RELEASED);
    }

    #[test]
    fn the_rate_is_clamped_rather_than_believed() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(i32::MAX, 0);
        assert_eq!(p.rate(), MAX_STEPS_PER_S as i32);
        p.jog(i32::MIN, 0);
        assert_eq!(p.rate(), -(MAX_STEPS_PER_S as i32));
        // And an absurd bounded move is clamped the same way rather than
        // dividing by something enormous.
        p.move_by(10, u32::MAX, 0);
        assert_eq!(p.rate(), MAX_STEPS_PER_S as i32);
    }

    #[test]
    fn a_zero_rate_jog_is_a_stop() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(400, 0);
        assert!(p.is_moving());
        p.jog(0, 1_000);
        assert!(!p.is_moving(), "a dead-zone release did not stop it");
    }

    #[test]
    fn a_diagnostic_hold_energises_then_lets_go_on_its_own() {
        let mut p = Planner::new(StepMode::Half);
        // State 0 is coil A alone, which is what isolates one bridge.
        p.hold_state(0, 3 * SEC, 0);
        let held = p.poll(1_000).coils;
        assert!(held.ain1 || held.ain2, "coil A was not energised");
        assert!(!held.bin1 && !held.bin2, "coil B should be off in state 0");
        assert!(p.poll(3 * SEC - 1).coils.energised(), "it let go early");
        assert_eq!(
            p.poll(3 * SEC + 1).coils,
            Coils::RELEASED,
            "a diagnostic hold has to end by itself"
        );
    }

    #[test]
    fn a_hold_cannot_be_asked_to_last_forever() {
        let mut p = Planner::new(StepMode::Half);
        // Without current limiting the winding is a resistor across the supply,
        // so an absurd request is clamped rather than honoured.
        p.hold_state(0, u64::MAX, 0);
        assert_eq!(p.poll(MAX_HOLD_US + 1).coils, Coils::RELEASED);
    }

    #[test]
    fn json_i32_reads_the_fields_the_trackpad_sends() {
        assert_eq!(json_i32(r#"{"steps_per_s":-250}"#, "steps_per_s"), Some(-250));
        assert_eq!(
            json_i32(r#"{ "steps" : 1200 , "steps_per_s": 800 }"#, "steps"),
            Some(1200)
        );
        assert_eq!(json_i32(r#"{"steps_per_s": 0}"#, "steps_per_s"), Some(0));
    }

    #[test]
    fn json_i32_refuses_what_it_cannot_read() {
        assert_eq!(json_i32("{}", "steps"), None);
        assert_eq!(json_i32(r#"{"steps":}"#, "steps"), None);
        assert_eq!(json_i32(r#"{"steps":"12"}"#, "steps"), None);
        // Out of range comes back as None rather than wrapping into a rate that
        // was never asked for.
        assert_eq!(json_i32(r#"{"steps":99999999999}"#, "steps"), None);
        // A key that only appears as a value must not be mistaken for the key.
        assert_eq!(json_i32(r#"{"name":"steps"}"#, "steps"), None);
    }
}
