//! The four bridge pins, and the timer that clocks them.
//!
//! All the deciding lives in [`esprobe_firmware::stepper`], which is
//! hardware-neutral and tested on the host. This is the part that cannot be
//! tested here: claiming the pins, and getting called often enough.
//!
//! # Why a timer and not a thread
//!
//! ESP-IDF's FreeRTOS tick is 100 Hz by default, so a sleeping thread wakes at
//! best every 10 ms — two and a half steps a second. `esp_timer` is a separate
//! microsecond timer, so this is paced by it and the tick rate is irrelevant.
//!
//! The callback fires at a fixed [`POLL_PERIOD_US`] rather than being armed for
//! each step, which is the simpler of the two and costs a little idle work. It
//! also bounds the damage from a missed deadline: a periodic timer that runs
//! late simply runs late, where a self-arming one that fails to re-arm stops
//! the motor mid-move with the windings still energised.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result};
use esp_idf_svc::hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_svc::timer::{EspTaskTimerService, EspTimer};

use esprobe_firmware::stepper::{Coils, Planner, StepMode};

/// How often the bridges are refreshed.
///
/// Two polls per step at the maximum rate, so the interval a step actually
/// lands on is within half a poll of the one asked for.
pub const POLL_PERIOD_US: u64 = 500;

/// The planner, shared with the HTTP handlers.
pub type Shared = Arc<Mutex<Planner>>;

/// A running stepper: the timer must outlive it or the motor stops.
pub struct Stepper {
    planner: Shared,
    /// Dropping this cancels the timer, so it is kept for as long as the
    /// firmware runs. Naming it `_timer` would invite someone to drop it.
    _timer: EspTimer<'static>,
}

impl Stepper {
    #[must_use]
    pub fn planner(&self) -> Shared {
        self.planner.clone()
    }
}

/// Claim the bridge pins and start clocking them.
///
/// The pins are handed over in schematic order — coil A then coil B — because
/// swapping the two inputs of one bridge reverses that winding, which turns
/// into a motor that buzzes and does not turn.
pub fn start(
    ain1: AnyIOPin<'static>,
    ain2: AnyIOPin<'static>,
    bin1: AnyIOPin<'static>,
    bin2: AnyIOPin<'static>,
) -> Result<Stepper> {
    let mut ain1 = PinDriver::output(ain1).context("STEP_AIN1")?;
    let mut ain2 = PinDriver::output(ain2).context("STEP_AIN2")?;
    let mut bin1 = PinDriver::output(bin1).context("STEP_BIN1")?;
    let mut bin2 = PinDriver::output(bin2).context("STEP_BIN2")?;

    // Coast before anything else. A DRV8833 comes out of reset with its inputs
    // undefined, and the first thing a motor should do is nothing.
    let _ = ain1.set_low();
    let _ = ain2.set_low();
    let _ = bin1.set_low();
    let _ = bin2.set_low();

    let planner: Shared = Arc::new(Mutex::new(Planner::new(StepMode::Half)));
    let shared = planner.clone();

    let service = EspTaskTimerService::new().context("esp_timer service")?;
    let mut last = Coils::RELEASED;
    let timer = service
        .timer(move || {
            // Safety: reading the monotonic microsecond counter has no
            // preconditions; it is `unsafe` only because it is a C symbol.
            let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() } as u64;

            // A poisoned lock means a handler panicked holding the planner. The
            // motor must not keep whatever it was doing, so this releases the
            // windings and stops rather than carrying on blind.
            let coils = match shared.lock() {
                Ok(mut planner) => planner.poll(now_us).coils,
                Err(_) => Coils::RELEASED,
            };

            // Only touch the pins when something changed. At two kilohertz
            // against step rates in the hundreds, most polls change nothing,
            // and four GPIO writes that set a pin to what it already is are
            // still four register writes.
            if coils != last {
                let _ = ain1.set_level(coils.ain1.into());
                let _ = ain2.set_level(coils.ain2.into());
                let _ = bin1.set_level(coils.bin1.into());
                let _ = bin2.set_level(coils.bin2.into());
                last = coils;
            }
        })
        .context("stepper timer")?;
    timer
        .every(Duration::from_micros(POLL_PERIOD_US))
        .context("starting the stepper timer")?;

    Ok(Stepper {
        planner,
        _timer: timer,
    })
}
