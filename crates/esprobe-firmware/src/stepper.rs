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

/// How quickly the step rate is allowed to change, in steps per second per second.
///
/// A stepper cannot be commanded to a speed, only accelerated to one. Told to
/// go from standstill straight to a few hundred steps a second, the rotor
/// cannot follow the field: it stalls, and a stalled stepper is silent or
/// buzzes while the driver happily reports everything is fine. That failure
/// looks exactly like a wiring fault, which is a bad thing for a bench to be
/// unable to tell apart.
///
/// So every rate change is ramped. The default reaches the trackpad's default
/// speed in about a tenth of a second, which is fast enough that a pad feels
/// connected to the shaft rather than to a suggestion box.
///
/// This is a *default*, not a limit: the rate a given motor can be accelerated
/// at depends on its inertia, its supply and what is bolted to it, none of
/// which the firmware can see. [`Planner::set_accel`] tunes it at runtime so
/// finding the value just short of stalling costs a slider drag rather than a
/// reflash.
///
/// It defaults to the ceiling, which is the fastest response available and
/// **not** the safest setting. At this rate the ramp is effectively a step
/// change, and a motor asked for more than its torque can follow does not go
/// faster — it slips, silently, while the position count carries on believing
/// itself. That is a deliberate choice of responsiveness over accuracy for a
/// hand-driven bench station; anything that cares where the shaft actually is
/// should turn this down until it stops slipping.
pub const ACCEL_STEPS_PER_S2: i32 = MAX_ACCEL_STEPS_PER_S2;

/// How hard the rate is brought *down*.
///
/// Deliberately far higher than the acceleration, and not a knob. Overshooting
/// on the way up makes a motor slip, because the field is asked to be somewhere
/// the rotor has not reached yet. Coming down has no such failure: the field
/// waits and the rotor arrives, helped rather than hindered by friction. Ramping
/// both at one gentle rate is what made the pad feel disconnected — releasing it
/// took as long as pushing it, and a flick from full forward to full reverse had
/// to crawl through zero twice.
pub const DECEL_STEPS_PER_S2: i32 = 20_000;

/// Bounds on the runtime acceleration.
///
/// The floor keeps a mistyped zero from wedging the ramp so it never reaches
/// its target. The ceiling is where acceleration stops meaning anything: past
/// the point where a single poll covers the whole range, the ramp is a step
/// change and asking for more is asking for nothing.
pub const MIN_ACCEL_STEPS_PER_S2: i32 = 100;
/// See [`MIN_ACCEL_STEPS_PER_S2`].
///
/// At the maximum step rate this covers the entire range in a single poll, so
/// the ramp has become a step change and asking for more changes nothing.
pub const MAX_ACCEL_STEPS_PER_S2: i32 = 200_000;

/// The rate a stepper can start from rest without ramping to it.
///
/// A ramp that begins at one step a second has a one-second first interval, so
/// a short move sits still for a second before its second step — found by a
/// test asserting a two-step move had finished. Real steppers have a start
/// speed below which no ramp is needed; this is a conservative one.
pub const START_STEPS_PER_S: i32 = 50;

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

/// How long a jog on the persistent link is honoured without a keepalive.
///
/// Much longer than [`JOG_TIMEOUT_US`], and safe to be, because on that path a
/// client that disappears takes its socket with it and the motor is stopped by
/// the close rather than by this. See [`Planner::jog_for`] for what a short one
/// costs on a congested link.
pub const LINK_JOG_TIMEOUT_US: u64 = 1_500_000;

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
    /// The rate being stepped at now, ramped towards `target_rate`.
    rate: i32,
    /// What was asked for. The rotor has to be brought to it, not thrown at it.
    target_rate: i32,
    /// When the ramp was last advanced.
    ramped_at_us: u64,
    /// When the current command stops being honoured.
    expires_at_us: u64,
    /// When the last step was taken. The next one is due an interval after it,
    /// computed from the rate *now* — latching a deadline at step time meant a
    /// step scheduled while the ramp was slow stayed scheduled that far out,
    /// however fast the ramp had since become.
    last_step_us: u64,
    /// When holding position stops and the windings are released.
    release_at_us: u64,
    holding: bool,
    /// How hard to accelerate, in steps per second squared. Tunable because the
    /// right value is a property of the machine, not of this code.
    accel: i32,
    /// The highest command sequence applied so far. See [`Planner::accept`].
    last_seq: i64,
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
            target_rate: 0,
            ramped_at_us: 0,
            expires_at_us: 0,
            last_step_us: 0,
            release_at_us: 0,
            holding: false,
            accel: ACCEL_STEPS_PER_S2,
            last_seq: i64::MIN,
        }
    }

    /// Should a motion command carrying this sequence number be obeyed?
    ///
    /// The pad posts a jog every time the rate changes, and those requests can
    /// overtake each other on a wireless link. The case that matters is a jog
    /// still in flight when the finger lifts: it lands *after* the stop and
    /// starts the motor again, for as long as the jog watchdog allows. To
    /// anyone holding the pad that is a motor which ignored being released.
    ///
    /// So motion carries a sequence and stale motion is dropped. Commands with
    /// no sequence are always obeyed — `curl` and the CLI have no session to
    /// order, and refusing them would be refusing the bench its own tools.
    ///
    /// Stopping deliberately does **not** go through here. A stop that arrives
    /// out of order must still stop; there is no reading of a dropped stop that
    /// leaves a motor safer.
    pub fn accept(&mut self, seq: Option<i64>) -> bool {
        match seq {
            None => true,
            Some(seq) if seq > self.last_seq => {
                self.last_seq = seq;
                true
            }
            Some(_) => false,
        }
    }

    /// Record a sequence without gating on it, for commands that always run.
    pub const fn observe(&mut self, seq: Option<i64>) {
        if let Some(seq) = seq
            && seq > self.last_seq
        {
            self.last_seq = seq;
        }
    }

    /// How hard the rate is currently ramped up.
    #[must_use]
    pub const fn accel(&self) -> i32 {
        self.accel
    }

    /// Retune the acceleration, clamped to something that still ramps.
    ///
    /// Takes effect on the next poll, including part-way through a move: the
    /// point is to turn the knob while the motor runs and hear where it starts
    /// to slip.
    pub const fn set_accel(&mut self, steps_per_s2: i32) {
        self.accel = if steps_per_s2 < MIN_ACCEL_STEPS_PER_S2 {
            MIN_ACCEL_STEPS_PER_S2
        } else if steps_per_s2 > MAX_ACCEL_STEPS_PER_S2 {
            MAX_ACCEL_STEPS_PER_S2
        } else {
            steps_per_s2
        };
    }

    #[must_use]
    pub const fn position(&self) -> i64 {
        self.position
    }

    /// The rate being stepped at right now.
    #[must_use]
    pub const fn rate(&self) -> i32 {
        self.rate
    }

    /// The rate being ramped towards.
    #[must_use]
    pub const fn target_rate(&self) -> i32 {
        self.target_rate
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

    /// Are the windings drawing current — moving, or holding position?
    #[must_use]
    pub const fn is_energised(&self) -> bool {
        self.holding
    }

    /// Push a jog's deadline out without changing anything about the motion.
    ///
    /// A jog stops itself if it is not renewed, because over HTTP a controller
    /// that vanished is indistinguishable from one that has not spoken lately.
    /// The page used to renew it by re-sending the whole jog several times a
    /// second; on the persistent link it costs one byte, and this is what that
    /// byte does.
    ///
    /// Only a jog has a deadline. A bounded move ends by itself and must not
    /// have one invented for it, and something already stopped must not be
    /// given a reason to start.
    pub const fn keepalive(&mut self, now_us: u64) {
        self.keepalive_for(now_us, JOG_TIMEOUT_US);
    }

    /// A keepalive that grants a stated time. See [`Planner::jog_for`].
    pub const fn keepalive_for(&mut self, now_us: u64, timeout_us: u64) {
        if self.remaining.is_none() && self.target_rate != 0 {
            self.expires_at_us = now_us + timeout_us;
        }
    }

    /// Run at a signed rate until told otherwise — or until the jog expires.
    ///
    /// Call again to refresh it. A rate of zero stops, which is what the
    /// trackpad sends when a finger lifts inside the dead zone.
    pub fn jog(&mut self, steps_per_s: i32, now_us: u64) {
        self.jog_for(steps_per_s, now_us, JOG_TIMEOUT_US);
    }

    /// A jog that is honoured for a stated time rather than the default.
    ///
    /// The default is short because over HTTP a controller that vanished and
    /// one that has merely gone quiet look identical, so the only safe reading
    /// of silence is the pessimistic one.
    ///
    /// A persistent link does not have that problem: it reports the disconnect,
    /// and the motor is stopped by the socket closing. There the deadline is a
    /// second line of defence against a client that is still connected and no
    /// longer thinking, and it costs nothing to make it tolerant — whereas
    /// keeping it short costs a motor that stalls mid-drag whenever the network
    /// hiccups for longer than the gap between keepalives. Measured on a busy
    /// 2.4 GHz channel, round trips spiked past half a second while the median
    /// stayed near 36 ms, and a 400 ms deadline stopped the motor repeatedly
    /// with a finger still on the pad.
    pub fn jog_for(&mut self, steps_per_s: i32, now_us: u64, timeout_us: u64) {
        let clamped = steps_per_s.clamp(-(MAX_STEPS_PER_S as i32), MAX_STEPS_PER_S as i32);
        if clamped == 0 {
            self.stop(now_us);
            return;
        }
        self.remaining = None;
        self.begin(clamped, now_us);
        self.expires_at_us = now_us + timeout_us;
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
            // Behind by one interval, so the first step is taken on the next
            // poll rather than an interval from now.
            self.last_step_us = now_us.saturating_sub(1_000_000 / START_STEPS_PER_S as u64);
            self.ramped_at_us = now_us;
            // Start at the rate a stepper can leave rest at, or at the target
            // if that is slower — a slow move should not be ramped down to.
            self.rate = rate.signum() * rate.abs().min(START_STEPS_PER_S);
        }
        self.target_rate = rate;
        self.holding = true;
    }

    /// Move `rate` towards `target_rate` by whatever the ramp allows.
    fn ramp(&mut self, now_us: u64) {
        if self.rate == self.target_rate {
            self.ramped_at_us = now_us;
            return;
        }
        let elapsed = now_us.saturating_sub(self.ramped_at_us);
        // Slowing down is not the reverse of speeding up, so it does not get the
        // same rate. Reversing counts as slowing until the rate reaches zero,
        // which is what stops a flick from one side of the pad to the other from
        // spending a second crawling through the middle.
        //
        // `rate == 0` is the accelerating case: `signum()` is zero there, and
        // without this guard a standing start compares 0 against 1, reads as a
        // reversal, and leaves rest under the braking rate.
        let slowing = self.rate != 0
            && (self.target_rate.signum() != self.rate.signum()
                || self.target_rate.abs() < self.rate.abs());
        let accel = if slowing { DECEL_STEPS_PER_S2 } else { self.accel };
        // Integer division: below this the change rounds to zero and the ramp
        // would stall, so leave the clock alone and let it accumulate.
        let change = ((elapsed as i64 * accel as i64) / 1_000_000) as i32;
        if change == 0 {
            return;
        }
        self.ramped_at_us = now_us;
        let gap = self.target_rate - self.rate;
        self.rate += change.min(gap.abs()) * gap.signum();
    }

    /// Stop stepping but keep position, until the hold time runs out.
    ///
    /// Immediate, not ramped. Everything that calls this is either a deadline
    /// that expired or somebody asking it to stop, and neither is a moment to
    /// keep turning through a deceleration curve.
    pub fn stop(&mut self, now_us: u64) {
        self.rate = 0;
        self.target_rate = 0;
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

        self.ramp(now_us);

        if self.rate != 0 && now_us.saturating_sub(self.last_step_us) >= self.interval_us() {
            let forward = self.rate > 0;
            let coils = self.seq.advance(forward);
            self.position += if forward { 1 } else { -1 };
            self.last_step_us = now_us;

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
    // Out-of-range is `None`, not a wrapped value: a rate that arrived too large
    // to represent is a bug at the sender, and honouring some truncation of it
    // would move a motor by an amount nobody asked for.
    json_i64(body, field).and_then(|v| i32::try_from(v).ok())
}

/// The same, for values that do not fit in 32 bits — command sequence numbers
/// are millisecond timestamps, which passed `i32` in 1970.
#[must_use]
pub fn json_i64(body: &str, field: &str) -> Option<i64> {
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
        return Some(if negative { -magnitude } else { magnitude });
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
        // Long enough to cover the ramp as well as the steps; the extra time
        // must not produce extra steps.
        for t in 0..2 * SEC {
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
        for t in 0..2 * SEC {
            p.poll(t);
        }
        assert_eq!(p.position(), 25);
        p.move_by(-25, 1_000, 2 * SEC);
        for t in 2 * SEC..4 * SEC {
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
        // Two seconds, so the quarter-second ramp to 100/s is a small part of
        // it. The point of the test is that refreshing does not *stall* it.
        for t in 0..2 * SEC {
            if t % 5_000 == 0 {
                p.jog(100, t);
            }
            if p.poll(t).stepped {
                steps += 1;
            }
        }
        assert!(
            (170..=200).contains(&steps),
            "expected close to 200 steps in two seconds at 100/s, got {steps}"
        );
    }

    #[test]
    fn it_releases_the_windings_after_holding_for_a_while() {
        let mut p = Planner::new(StepMode::Full);
        p.move_by(2, 1_000, 0);
        // Find when the move actually ends rather than assuming: with the ramp
        // starting at 50 steps/s, two steps take about forty milliseconds, and
        // a fixed wait long enough to be safe was already past the hold.
        let mut done_at = None;
        for t in 0..SEC {
            p.poll(t);
            if !p.is_moving() && done_at.is_none() {
                done_at = Some(t);
                break;
            }
        }
        let done_at = done_at.expect("the move never finished");
        // Still holding position right after the move.
        assert!(
            p.poll(done_at + 1_000).coils.energised(),
            "it let go immediately"
        );
        // And coasting once the hold time is up, because a stepper standing
        // still with its windings on is a resistor.
        let released = p.poll(done_at + HOLD_AFTER_MOTION_US + 1);
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
        // The *requested* rate is what gets clamped; the rate actually being
        // stepped at is whatever the ramp has reached.
        p.jog(i32::MAX, 0);
        assert_eq!(p.target_rate(), MAX_STEPS_PER_S as i32);
        p.jog(i32::MIN, 0);
        assert_eq!(p.target_rate(), -(MAX_STEPS_PER_S as i32));
        p.move_by(10, u32::MAX, 0);
        assert_eq!(p.target_rate(), MAX_STEPS_PER_S as i32);
        // And the ramp never overshoots what was asked for.
        p.jog(MAX_STEPS_PER_S as i32, 0);
        for t in 0..10 * SEC {
            if t % 100_000 == 0 {
                p.jog(MAX_STEPS_PER_S as i32, t);
            }
            p.poll(t);
            assert!(p.rate() <= MAX_STEPS_PER_S as i32, "ramped past the cap");
        }
        assert_eq!(p.rate(), MAX_STEPS_PER_S as i32, "never reached the cap");
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
    fn the_rate_is_ramped_rather_than_jumped_to() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(800, 0);
        // Asking for 800 must not produce 800 immediately: the rotor cannot
        // follow a field that jumps, and a stalled stepper is silent, which
        // looks exactly like a wiring fault.
        assert_eq!(
            p.rate(),
            START_STEPS_PER_S,
            "it should leave rest at the start rate, not at the target"
        );
        let mut reached = None;
        for t in 0..5 * SEC {
            // Refreshed, or the watchdog would stop it a third of the way up
            // the ramp and the test would be measuring that instead.
            if t % 100_000 == 0 {
                p.jog(800, t);
            }
            p.poll(t);
            if p.rate() == 800 && reached.is_none() {
                reached = Some(t);
            }
        }
        let at = reached.expect("never reached the requested rate");
        // Derived from the constant rather than written out, so retuning the
        // ramp does not fail a test that was only ever asserting arithmetic.
        let want = (800 - START_STEPS_PER_S) as u64 * 1_000_000 / ACCEL_STEPS_PER_S2 as u64;
        assert!(
            at.abs_diff(want) < want / 2 + 20_000,
            "reached full rate at {at}us, expected about {want}us"
        );
    }

    #[test]
    fn slowing_down_is_quicker_than_speeding_up() {
        // The pad felt disconnected because letting go took as long as pushing.
        // Braking cannot make a motor slip the way accelerating can, so it is
        // not held to the same rate.
        let mut p = Planner::new(StepMode::Half);
        // Explicitly gentle. The shipping default is the ceiling, where the ramp
        // is a step change in both directions and this comparison would pass on
        // two zeroes without testing anything.
        p.set_accel(1_000);
        let mut up = None;
        for t in 0..5 * SEC {
            if t % 100_000 == 0 {
                p.jog(800, t);
            }
            p.poll(t);
            if p.rate() == 800 {
                up = Some(t);
                break;
            }
        }
        let up = up.expect("never got up to speed");

        let mut down = None;
        for t in up..up + 5 * SEC {
            if t % 100_000 == 0 {
                p.jog(100, t);
            }
            p.poll(t);
            if p.rate() == 100 {
                down = Some(t - up);
                break;
            }
        }
        let down = down.expect("never came back down");
        assert!(
            down * 2 < up,
            "took {down}us to slow down against {up}us to speed up; braking is not\n\
             meaningfully quicker, which is the thing that made the pad feel laggy"
        );
    }

    #[test]
    fn a_flick_across_the_pad_does_not_crawl_through_the_middle() {
        // Full forward to full reverse is the worst case: it has to unwind the
        // whole rate and build it again. Under one ramp for both halves this
        // took seconds, and the motor kept going the wrong way throughout.
        let mut p = Planner::new(StepMode::Half);
        for t in 0..2 * SEC {
            if t % 100_000 == 0 {
                p.jog(800, t);
            }
            p.poll(t);
        }
        assert_eq!(p.rate(), 800, "did not reach the rate to reverse from");

        let mut crossed = None;
        for t in 2 * SEC..4 * SEC {
            if t % 100_000 == 0 {
                p.jog(-800, t);
            }
            p.poll(t);
            if p.rate() <= 0 && crossed.is_none() {
                crossed = Some(t - 2 * SEC);
            }
        }
        let crossed = crossed.expect("never stopped going forwards");
        assert!(
            crossed < 150_000,
            "took {crossed}us just to stop going forwards after a full reversal"
        );
    }

    #[test]
    fn a_stale_jog_cannot_outlive_the_stop_that_followed_it() {
        // The pad posts a jog per rate change; on a wireless link one can still
        // be in flight when the finger lifts and land after the stop. Without
        // ordering the motor restarts and runs until the jog watchdog expires,
        // which to whoever let go is a motor that ignored them.
        let mut p = Planner::new(StepMode::Half);
        assert!(p.accept(Some(10)));
        p.jog(600, 0);
        // Inside the jog watchdog: polling past it would stop the motor for a
        // reason that has nothing to do with what is being tested.
        let lifted = JOG_TIMEOUT_US / 2;
        for t in 0..lifted {
            p.poll(t);
        }
        assert!(p.is_moving());

        // The stop is not gated, but it records its place in the sequence.
        p.observe(Some(11));
        p.stop(lifted);
        assert!(!p.is_moving());

        // The overtaken jog now arrives.
        assert!(
            !p.accept(Some(10)),
            "a jog older than the stop was accepted"
        );
        for t in lifted..lifted + JOG_TIMEOUT_US / 2 {
            p.poll(t);
        }
        assert!(!p.is_moving(), "a stale jog restarted a stopped motor");
    }

    #[test]
    fn a_stop_is_obeyed_even_when_it_arrives_out_of_order() {
        // There is no reading of a dropped stop that leaves a motor safer, so
        // stopping never goes through the sequence gate.
        let mut p = Planner::new(StepMode::Half);
        assert!(p.accept(Some(100)));
        p.jog(600, 0);
        let lifted = JOG_TIMEOUT_US / 2;
        for t in 0..lifted {
            p.poll(t);
        }
        assert!(p.is_moving());
        p.observe(Some(5));
        p.stop(lifted);
        assert!(!p.is_moving(), "an out-of-order stop was ignored");
    }

    #[test]
    fn commands_without_a_sequence_are_always_obeyed() {
        // curl and the CLI have no session to order.
        let mut p = Planner::new(StepMode::Half);
        assert!(p.accept(Some(1_000)));
        assert!(p.accept(None), "an unsequenced command was refused");
        assert!(p.accept(None));
        // ...and they must not raise the bar for the page that does sequence.
        assert!(p.accept(Some(1_001)), "an unsequenced command moved the sequence");
    }

    #[test]
    fn a_sequence_number_larger_than_a_32_bit_value_survives() {
        // They are millisecond timestamps, so they left i32 behind in 1970 and
        // a narrowing parse would make every one of them collide.
        let body = r#"{"steps_per_s":120,"seq":1785000000000}"#;
        assert_eq!(json_i64(body, "seq"), Some(1_785_000_000_000));
        assert_eq!(json_i32(body, "steps_per_s"), Some(120));
        // Too large for i32 is None rather than a truncation nobody asked for.
        assert_eq!(json_i32(body, "seq"), None);
    }

    #[test]
    fn a_keepalive_renews_a_jog_without_restarting_a_stopped_motor() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(600, 0);
        // Renewed just before each deadline, it keeps going well past the point
        // an unrenewed jog would have been stopped.
        let mut t = 0;
        while t < 3 * SEC {
            t += JOG_TIMEOUT_US - 1_000;
            p.keepalive(t);
            p.poll(t);
        }
        assert!(p.is_moving(), "a renewed jog was stopped anyway");

        // ...but it is a renewal, not a start.
        p.stop(t);
        p.keepalive(t);
        for u in t..t + JOG_TIMEOUT_US {
            p.poll(u);
        }
        assert!(!p.is_moving(), "a keepalive restarted a stopped motor");
    }

    #[test]
    fn the_link_deadline_outlives_a_network_stall_the_default_would_not() {
        // The failure this fixes, reproduced: keepalives every 150 ms on a link
        // that occasionally stalls for 600 ms. Under the default the motor
        // stops with a finger still on the pad.
        let stall_at = 300_000;
        let stall_for = 600_000;
        let run = |timeout: u64| {
            let mut p = Planner::new(StepMode::Half);
            p.jog_for(200, 0, timeout);
            let mut t = 0;
            let mut next_keepalive = 150_000;
            while t < 2 * SEC {
                t += 1_000;
                // The stall: nothing arrives for 600 ms.
                let stalled = (stall_at..stall_at + stall_for).contains(&t);
                if t >= next_keepalive && !stalled {
                    p.keepalive_for(t, timeout);
                    next_keepalive = t + 150_000;
                } else if stalled {
                    next_keepalive = stall_at + stall_for;
                }
                p.poll(t);
            }
            p.is_moving()
        };
        assert!(!run(JOG_TIMEOUT_US), "the default survived a stall it should not");
        assert!(
            run(LINK_JOG_TIMEOUT_US),
            "the link deadline stopped the motor during a stall the socket would have survived"
        );
    }

    #[test]
    fn a_keepalive_does_not_give_a_bounded_move_a_deadline() {
        // A move ends when it has taken its steps. Handing it a watchdog would
        // cut a long slow one short.
        let mut p = Planner::new(StepMode::Half);
        p.set_accel(1_000);
        p.move_by(40, 20, 0);
        p.keepalive(0);
        let mut t = 0;
        while t < 4 * SEC && p.remaining().is_some() {
            t += 100;
            p.poll(t);
        }
        assert_eq!(p.position(), 40, "the move did not finish");
    }

    #[test]
    fn energised_tracks_the_windings_rather_than_the_motion() {
        let mut p = Planner::new(StepMode::Half);
        assert!(!p.is_energised());
        p.jog(200, 0);
        p.poll(0);
        assert!(p.is_energised());

        // Stopped but still holding position: not moving, still drawing current.
        p.stop(SEC);
        p.poll(SEC);
        assert!(!p.is_moving());
        assert!(p.is_energised(), "a held position should still be energised");

        // Past the hold, the windings go.
        p.poll(SEC + HOLD_AFTER_MOTION_US);
        assert!(!p.is_energised());
    }

    #[test]
    fn the_acceleration_can_be_retuned_and_is_clamped() {
        let mut p = Planner::new(StepMode::Half);
        assert_eq!(p.accel(), ACCEL_STEPS_PER_S2);

        p.set_accel(1_000);
        assert_eq!(p.accel(), 1_000);

        // A mistyped zero must not wedge the ramp below the point where integer
        // division rounds every change to nothing.
        p.set_accel(0);
        assert_eq!(p.accel(), MIN_ACCEL_STEPS_PER_S2);
        p.set_accel(-5_000);
        assert_eq!(p.accel(), MIN_ACCEL_STEPS_PER_S2);
        p.set_accel(i32::MAX);
        assert_eq!(p.accel(), MAX_ACCEL_STEPS_PER_S2);
    }

    #[test]
    fn a_gentler_acceleration_actually_takes_longer() {
        // The knob has to reach the ramp, not just be stored.
        let time_to_speed = |accel: i32| {
            let mut p = Planner::new(StepMode::Half);
            p.set_accel(accel);
            for t in 0..10 * SEC {
                if t % 100_000 == 0 {
                    p.jog(800, t);
                }
                p.poll(t);
                if p.rate() == 800 {
                    return t;
                }
            }
            panic!("never reached the rate at {accel} steps/s^2");
        };
        assert!(
            time_to_speed(500) > time_to_speed(8_000) * 4,
            "the acceleration setting is not reaching the ramp"
        );
    }

    #[test]
    fn stopping_is_immediate_and_not_ramped_down() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(600, 0);
        for t in 0..2 * SEC {
            if t % 100_000 == 0 {
                p.jog(600, t);
            }
            p.poll(t);
        }
        assert!(p.rate() > 0);
        // Everything that stops is either a deadline that expired or somebody
        // asking, and neither is a moment to keep turning through a curve.
        p.stop(2 * SEC);
        assert_eq!(p.rate(), 0);
        assert_eq!(p.target_rate(), 0);
        assert!(!p.poll(2 * SEC + 1).stepped);
    }

    #[test]
    fn reversing_passes_through_zero_instead_of_flipping() {
        let mut p = Planner::new(StepMode::Half);
        p.jog(400, 0);
        for t in 0..2 * SEC {
            if t % 100_000 == 0 {
                p.jog(400, t);
            }
            p.poll(t);
        }
        assert!(p.rate() > 0);
        let mut seen_slow = false;
        for t in 2 * SEC..5 * SEC {
            if t % 100_000 == 0 {
                p.jog(-400, t);
            }
            p.poll(t);
            if p.rate().abs() < 40 {
                seen_slow = true;
            }
        }
        assert!(seen_slow, "the rate flipped sign without slowing down");
        assert_eq!(p.rate(), -400, "it did not reach the reversed rate");
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
