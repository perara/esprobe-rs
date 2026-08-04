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
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use esp_idf_svc::hal::gpio::{AnyIOPin, PinDriver, Pull};
use esp_idf_svc::timer::{EspTaskTimerService, EspTimer};

use esprobe_firmware::stepper::{Coils, Planner, StepMode};

/// How often the bridges are refreshed.
///
/// Two polls per step at the maximum rate, so the interval a step actually
/// lands on is within half a poll of the one asked for.
pub const POLL_PERIOD_US: u64 = 500;

/// The planner, shared with the HTTP handlers.
pub type Shared = Arc<Mutex<Planner>>;

/// Asked for by the HTTP handler, performed by the timer.
///
/// The pins are owned by the timer callback and nothing else can touch them —
/// which is the right arrangement, and means a self-test cannot simply reach in
/// and drive them. So it is requested with a flag and answered with one, rather
/// than by handing the pins around behind a second lock that would have to be
/// taken in the right order against the planner's.
static SELFTEST_REQUEST: AtomicBool = AtomicBool::new(false);
static SELFTEST_DONE: AtomicBool = AtomicBool::new(false);
/// Two bits per pin, in `AIN1, AIN2, BIN1, BIN2` order: bit 0 of each pair is
/// "read back high when driven high", bit 1 "read back low when driven low".
static SELFTEST_RESULT: AtomicU32 = AtomicU32::new(0);

/// Ask the timer to drive each bridge pin and read the pad back.
pub fn request_selftest() {
    SELFTEST_DONE.store(false, Ordering::Relaxed);
    SELFTEST_REQUEST.store(true, Ordering::Relaxed);
}

/// The result, once the timer has performed it.
#[must_use]
pub fn selftest_result() -> Option<u32> {
    SELFTEST_DONE
        .load(Ordering::Relaxed)
        .then(|| SELFTEST_RESULT.load(Ordering::Relaxed))
}

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
    // `input_output`, not `output`, so the pad can be read back. It costs
    // nothing to drive and it is the only way this firmware can answer "am I
    // actually driving these pins" without somebody holding a meter — which is
    // a question that came up and which I could not answer.
    let mut ain1 = PinDriver::input_output(ain1, Pull::Floating).context("STEP_AIN1")?;
    let mut ain2 = PinDriver::input_output(ain2, Pull::Floating).context("STEP_AIN2")?;
    let mut bin1 = PinDriver::input_output(bin1, Pull::Floating).context("STEP_BIN1")?;
    let mut bin2 = PinDriver::input_output(bin2, Pull::Floating).context("STEP_BIN2")?;

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

            if SELFTEST_REQUEST.swap(false, Ordering::Relaxed) {
                let mut bits = 0u32;
                // One pin at a time, so a short between two of them shows up as
                // the wrong pin failing rather than as everything failing.
                macro_rules! check {
                    ($pin:expr, $slot:expr) => {{
                        let _ = $pin.set_high();
                        esp_idf_svc::hal::delay::Ets::delay_us(50);
                        if $pin.is_high() {
                            bits |= 1 << ($slot * 2);
                        }
                        let _ = $pin.set_low();
                        esp_idf_svc::hal::delay::Ets::delay_us(50);
                        if $pin.is_low() {
                            bits |= 1 << ($slot * 2 + 1);
                        }
                    }};
                }
                check!(ain1, 0);
                check!(ain2, 1);
                check!(bin1, 2);
                check!(bin2, 3);
                SELFTEST_RESULT.store(bits, Ordering::Relaxed);
                SELFTEST_DONE.store(true, Ordering::Relaxed);
                // The pins were just driven behind the planner's back, so make
                // the cache disagree and let the next poll write them properly.
                last = Coils {
                    ain1: true,
                    ain2: true,
                    bin1: true,
                    bin2: true,
                };
            }

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
