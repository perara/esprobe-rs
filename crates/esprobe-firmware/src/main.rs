#![cfg_attr(not(target_os = "espidf"), allow(dead_code))]

#[cfg(not(target_os = "espidf"))]
fn main() {
    panic!("build this firmware for riscv32imc-esp-espidf");
}

#[cfg(target_os = "espidf")]
mod app {
    use std::sync::{Arc, Mutex};
    use std::thread;
    use std::time::Duration;

    use anyhow::{Context, Result, anyhow};
    use embedded_svc::http::Method;
    use embedded_svc::io::Write as _;
    use embedded_svc::wifi::{
        AccessPointConfiguration, AuthMethod, ClientConfiguration, Configuration, PmfConfiguration,
        ScanMethod, ScanSortMethod,
    };
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::delay::Ets;
    use esp_idf_svc::hal::delay::TickType;
    use esp_idf_svc::hal::gpio::{AnyIOPin, InputOutput, PinDriver, Pull};
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::uart::{
        UartDriver,
        config::{Config as UartConfig, Parity},
    };
    use esp_idf_svc::hal::units::Hertz;
    use esp_idf_svc::http::server::EspHttpServer;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::sys::{
        UART_PIN_NO_CHANGE, gpio_mode_t_GPIO_MODE_INPUT, gpio_mode_t_GPIO_MODE_OUTPUT,
        gpio_reset_pin, gpio_set_direction, uart_port_t_UART_NUM_1, uart_set_pin,
    };
    use esp_idf_svc::wifi::{BlockingWifi, EspWifi, WifiEvent};
    use log::{error, info, warn};

    use esprobe_firmware::safety::ReleasedSwdState;
    use esprobe_firmware::swd::{Error as SwdError, SwdLink, WordMemory};
    use esprobe_firmware::usb_bridge::{
        Command as BridgeCommand, MAX_BLOCK_WORDS, MAX_FRAME as BRIDGE_MAX_FRAME,
        Status as BridgeStatus, decode_request, encode_response,
    };
    use esprobe_firmware::{pinmap, wifi_credentials};

    use esprobe_firmware::cycles::read as cpu_cycles;
    use esprobe_firmware::hardware::{Engine, EspSwdIo};

    // Build-time credentials are a convenience for a bench that always joins
    // the same network, not a requirement: an image with none still builds,
    // still bridges over USB, and can be given a network at runtime.
    const CONTROL_AP_SSID: Option<&str> = option_env!("CONTROL_AP_SSID");
    const CONTROL_AP_PASSWORD: Option<&str> = option_env!("CONTROL_AP_PASSWORD");
    /// Where runtime credentials live, so a network change costs a command
    /// rather than a rebuild and a reflash.
    /// Regulatory domain to start from; see `set_regulatory_domain`.
    ///
    /// `scripts/build.sh` requires this, so a normal build always names one.
    /// The fallback is ESP-IDF's own world-safe domain rather than any real
    /// country: transmitting under someone else's regulator is worse than not
    /// transmitting, and picking a default here would do exactly that for
    /// everyone who is not where this was written.
    const WIFI_COUNTRY: Option<&str> = option_env!("WIFI_COUNTRY");
    const DEFAULT_WIFI_COUNTRY: &str = "01";

    const NVS_NAMESPACE: &str = "esprobe";
    /// The transmit power that last associated successfully.
    const NVS_TX_POWER: &str = "txpower";
    const NVS_SSID: &str = "sta_ssid";
    const NVS_PASSWORD: &str = "sta_pass";
    /// Set when the bench harness has SWDIO and SWCLK crossed. The bridge's
    /// `SetPinMap` command overrides this at runtime.
    const SWD_PINS_SWAPPED: bool = option_env!("SWD_PINS_SWAPPED").is_some();
    /// Comfortably more than the 46 clocks of the longest SWD transfer, so one
    /// pass always drains whatever phase the target was left in.
    const RECOVERY_CYCLES: u16 = 128;
    /// Port the bridge protocol is served on over the network.
    const BRIDGE_TCP_PORT: u16 = 3333;

    /// What the bridge knows about the radio, and what it wants changed.
    ///
    /// The radio is owned by the main loop; the bridge only ever leaves a
    /// request here and reads back the last published snapshot. Sharing the
    /// driver itself would put a Wi-Fi reconnect — seconds of it — inside a
    /// command that the host is waiting on.
    #[derive(Default)]
    struct WifiControl {
        pending: Option<(String, String)>,
        forget: bool,
        connected: bool,
        ip: [u8; 4],
        ssid: String,
    }

    struct Hub {
        reset: PinDriver<'static, InputOutput>,
        /// An optional serial link to the target, for a console or a vendor's
        /// ROM bootloader. The probe passes bytes both ways and does not read
        /// them — what they mean is the target's business, and knowing would
        /// make this firmware care which chip it is talking to.
        uart: UartDriver<'static>,
        swd: Option<SwdLink<EspSwdIo<'static>>>,
        wifi: Arc<Mutex<WifiControl>>,
        bridge_attached: bool,
        /// Whether *this* firmware is holding the shared reset line down.
        ///
        /// Tracked as intent rather than sampled from the pad: reset is
        /// bidirectional on many parts, so the target pulls it low during its
        /// own resets — including the ones probe-rs performs while flashing —
        /// and a
        /// guard that reads the pin cannot tell that apart from a command that
        /// left reset asserted.
        reset_held: bool,
    }

    impl Hub {
        /// Drives or releases the shared reset line, recording which it is.
        fn hold_reset(&mut self, held: bool) -> Result<()> {
            if held {
                self.reset.set_low()?;
            }
            Self::set_gpio_direction(
                pinmap::RESET,
                if held {
                    gpio_mode_t_GPIO_MODE_OUTPUT
                } else {
                    gpio_mode_t_GPIO_MODE_INPUT
                },
            )?;
            self.reset_held = held;
            Ok(())
        }

        fn set_gpio_direction(pin: i32, mode: u32) -> Result<()> {
            // SAFETY: the Hub owns every GPIO passed here for its complete
            // lifetime. Direction switching implements a released idle state.
            esp_idf_svc::sys::esp!(unsafe { gpio_set_direction(pin, mode) })?;
            Ok(())
        }

        /// Route UART1's transmitter onto `UART_TX` and enable the driver.
        ///
        /// Idempotent, and cheap enough to call before every write. Nothing
        /// ever releases this pad now - see the constructor for why the old
        /// release-and-redrive dance silently stopped the port transmitting.
        fn bind_uart_pins() -> Result<()> {
            // SAFETY: the pin belongs to this driver and the port is open.
            esp_idf_svc::sys::esp!(unsafe {
                // Both pins, not just the transmitter. `uart_set_pin` with
                // NO_CHANGE for the receiver is documented to leave it alone,
                // but a reply that never arrived after a send - on a board
                // whose receive path is otherwise proven - is exactly what a
                // disturbed receive routing looks like, and naming both costs
                // nothing on a call that already runs once per command.
                uart_set_pin(
                    uart_port_t_UART_NUM_1,
                    pinmap::UART_TX,
                    pinmap::UART_RX,
                    UART_PIN_NO_CHANGE,
                    UART_PIN_NO_CHANGE,
                )
            })?;
            Ok(())
        }

        fn reclaim_gpio(pin: i32) -> Result<()> {
            // SAFETY: these are valid ESP32-C3 GPIO numbers owned by Hub.
            // gpio_reset_pin selects the GPIO IO-mux function; changing only
            // direction leaves reset-default JTAG functions active.
            esp_idf_svc::sys::esp!(unsafe { gpio_reset_pin(pin) })?;
            Ok(())
        }

        /// Drop the debug link and leave both pads released.
        ///
        /// Every command that is about to drive the pads starts here, so the
        /// state they are driven from is always the same one.
        fn release_pads(&mut self) -> Result<()> {
            if let Some(swd) = self.swd.as_mut() {
                swd.disconnect();
            }
            thread::sleep(Duration::from_millis(5));
            self.bridge_attached = false;
            Ok(())
        }

        fn require_target_power(&mut self) -> Result<()> {
            self.release_pads()?;
            let (mut swdio, mut swclk) = self
                .swd
                .as_mut()
                .context("SWD engine unavailable")?
                .released_line_state();
            if !swdio && self.reset.is_high() {
                self.pulse_reset_low_release()?;
                (swdio, swclk) = self
                    .swd
                    .as_mut()
                    .context("SWD engine unavailable")?
                    .released_line_state();
            }
            anyhow::ensure!(
                ReleasedSwdState::from_levels(swdio, swclk).permits_high_drive(),
                "target power not proven: released SWDIO is low (SWCLK={swclk})"
            );
            Ok(())
        }

        fn pulse_reset_low_release(&mut self) -> Result<()> {
            self.hold_reset(true)?;
            thread::sleep(Duration::from_millis(10));
            self.hold_reset(false)?;
            thread::sleep(Duration::from_millis(10));
            Ok(())
        }

        fn bridge_command(
            &mut self,
            command: BridgeCommand,
            payload: &[u8],
            response: &mut [u8],
        ) -> (BridgeStatus, usize) {
            let touches_target_bus = matches!(
                command,
                BridgeCommand::ReadRegister
                    | BridgeCommand::Profile
                    | BridgeCommand::MemoryRead
                    | BridgeCommand::ReadRegisterBlock
                    | BridgeCommand::WriteRegister
                    | BridgeCommand::WriteRegisterBlock
                    | BridgeCommand::SwjSequence
            );
            if touches_target_bus && !self.bridge_attached {
                return (BridgeStatus::NotAttached, 0);
            }
            // A core held in reset answers its whole address space with zeros
            // while the debug ROM keeps responding, so a bulk read looks like
            // it succeeded and returns a plausible blank device. Refuse those.
            //
            // Deliberately not the raw DP/AP commands: connect-under-reset is
            // built on talking to the debug port precisely while reset is
            // asserted, and probe-rs's own `attach_under_reset` does exactly
            // that. Guarding those would break the recovery path this guard
            // was written to protect.
            let reads_target_memory =
                matches!(command, BridgeCommand::MemoryRead | BridgeCommand::Profile);
            if reads_target_memory && self.reset_held {
                return (BridgeStatus::TargetInReset, 0);
            }
            // A command belonging to whatever board this firmware was built
            // into. A bare probe is not built into one, so there is nothing to
            // dispatch to — and the honest answer is the one an unknown opcode
            // always got. Answered here rather than refused at decode time: the
            // framing was valid, so the host gets a status it can act on
            // instead of a dropped frame it has to time out.
            if matches!(command, BridgeCommand::Extension(_)) {
                return (BridgeStatus::Unsupported, 0);
            }
            match self.try_bridge_command(command, payload, response) {
                Ok(length) => (BridgeStatus::Ok, length),
                Err(SwdError::WaitTimeout) => (BridgeStatus::Wait, 0),
                Err(SwdError::Fault) => (BridgeStatus::Fault, 0),
                Err(SwdError::Parity) => (BridgeStatus::Parity, 0),
                Err(SwdError::Protocol(ack)) => {
                    response[0] = ack;
                    (BridgeStatus::Transport, 1)
                }
                Err(error) => {
                    response[0] = match error {
                        SwdError::InvalidDpId => 1,
                        SwdError::PowerTimeout => 2,
                        SwdError::Unaligned => 3,
                        SwdError::LineHeldLow => 4,
                        _ => 0xff,
                    };
                    (BridgeStatus::Transport, 1)
                }
            }
        }

        fn try_bridge_command(
            &mut self,
            command: BridgeCommand,
            payload: &[u8],
            response: &mut [u8],
        ) -> Result<usize, SwdError> {
            match command {
                BridgeCommand::Hello => {
                    response[..4].copy_from_slice(b"DAP1");
                    Ok(4)
                }
                BridgeCommand::Attach => {
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    let (mut swdio, _) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    if !swdio && self.reset.is_high() {
                        self.pulse_reset_low_release()
                            .map_err(|_| SwdError::Protocol(0))?;
                        (swdio, _) = self
                            .swd
                            .as_mut()
                            .ok_or(SwdError::Protocol(0))?
                            .released_line_state();
                    }
                    if !swdio {
                        // A target still clocking out an abandoned data phase
                        // looks identical to an unpowered one. Give it the
                        // clocks it is waiting for before refusing.
                        (swdio, _) = self
                            .swd
                            .as_mut()
                            .ok_or(SwdError::Protocol(0))?
                            .recover_line(RECOVERY_CYCLES);
                    }
                    if !swdio {
                        return Err(SwdError::LineHeldLow);
                    }
                    let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                    swd.line_reset_and_switch();
                    self.bridge_attached = true;
                    Ok(0)
                }
                BridgeCommand::AttachUnderReset => {
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    if self.swd.is_none() {
                        return Err(SwdError::Protocol(0));
                    }
                    self.reset.set_low().map_err(|_| SwdError::Protocol(0))?;
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .line_reset_and_switch();
                    self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(2));
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .line_reset_and_switch();
                    self.bridge_attached = true;
                    Ok(0)
                }
                BridgeCommand::PadSelfTest => {
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    if self.swd.is_none() {
                        return Err(SwdError::Protocol(0));
                    }
                    self.reset.set_low().map_err(|_| SwdError::Protocol(0))?;
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    let levels = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .pad_self_test();
                    self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                    response[..4].copy_from_slice(&levels.map(u8::from));
                    self.bridge_attached = false;
                    Ok(4)
                }
                BridgeCommand::RecoveryProbe => {
                    let delay_us = match payload {
                        [] => 1_000,
                        [low, high] => u16::from_le_bytes([*low, *high]),
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    if self.swd.is_none() {
                        return Err(SwdError::Protocol(0));
                    }
                    // Everything that can fail runs inside the closure, so the
                    // release below is reached on every path. An earlier
                    // version released reset only after the last `?`, and a
                    // failure part-way left the target held in reset — where
                    // every later read returns zeros and looks like a blank
                    // device rather than an error.
                    let outcome = (|| {
                        self.reset.set_low().map_err(|_| SwdError::Protocol(0))?;
                        self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                        thread::sleep(Duration::from_millis(10));
                        self.swd
                            .as_mut()
                            .ok_or(SwdError::Protocol(0))?
                            .line_reset_and_switch();
                        // Release reset only once the SWD switch sequence is on
                        // the wire. The first DP request then lands during the
                        // board's reset-RC rise, before application pin setup.
                        self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                        Ets::delay_us(u32::from(delay_us));
                        let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                        // The debug port answering inside the reset-release
                        // window is the whole result. What the part *is* takes
                        // a vendor register this firmware deliberately does not
                        // know; probe-rs reads that over the same link once the
                        // host has attached.
                        swd.initialize_prepared()
                    })();
                    let _ = self.hold_reset(false);
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    self.bridge_attached = false;
                    let dp_id = outcome?;
                    response[..4].copy_from_slice(&dp_id.to_le_bytes());
                    Ok(4)
                }
                BridgeCommand::DiagnosticSwdio => {
                    let [level] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    self.reset.set_low().map_err(|_| SwdError::Protocol(0))?;
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    let observed = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .diagnostic_drive_swdio(*level != 0);
                    response[0] = u8::from(observed);
                    Ok(1)
                }
                BridgeCommand::UartSend => {
                    // Write to the target's UART and capture whatever it
                    // answers, in one round trip. The plane is request/response,
                    // so a separate receive would race the reply.
                    //
                    // Driving TX into an unpowered target back-feeds its supply
                    // through the pad's protection diode, which is the same
                    // hazard the debug pads have and takes the same evidence to
                    // rule out. Checked only when nothing is attached: proving
                    // it means releasing the pads to sample them, and doing
                    // that mid-session would tear down a live debug link for
                    // the sake of a serial write.
                    if !self.bridge_attached {
                        self.require_target_power()
                            .map_err(|_| SwdError::LineHeldLow)?;
                    }
                    self.uart
                        .change_parity(Parity::ParityNone)
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.uart.clear_rx().map_err(|_| SwdError::Protocol(0))?;
                    Self::bind_uart_pins().map_err(|_| SwdError::Protocol(0))?;
                    let written = self.uart.write(payload);
                    let drained = self.uart.wait_tx_done(200);
                    // Release the line whatever happened above: leaving the
                    // bridge driving UART_TX would fight the target's own
                    // transmitter, which is the contention this pin map exists
                    // to avoid.
                    // UART_TX stays bound; see the constructor.
                    written.map_err(|_| SwdError::Protocol(0))?;
                    drained.map_err(|_| SwdError::Protocol(0))?;
                    // Ten bytes are reserved by the bridge envelope.
                    let capacity = response.len().min(BRIDGE_MAX_FRAME - 10);
                    Ok(self.uart.read(&mut response[..capacity], 200).unwrap_or(0))
                }
                BridgeCommand::UartReceive => {
                    // An optional little-endian u16 of milliseconds to listen
                    // for. The bridge is a passive tap on the target's transmit
                    // line, so the window is the whole measurement.
                    let window_ms: u32 = match payload {
                        [] => 1000,
                        [low, high] => u32::from(u16::from_le_bytes([*low, *high])),
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    // The host abandons a command after three seconds.
                    let window_ms = window_ms.min(2_000);
                    self.uart
                        .change_parity(Parity::ParityNone)
                        .map_err(|_| SwdError::Protocol(0))?;
                    // Deliberately no `clear_rx`: the driver's buffer is the
                    // only thing holding traffic that arrived between calls,
                    // and discarding it made a passive tap miss everything it
                    // was not lucky enough to be inside the window for.
                    // UART_TX stays bound; see the constructor.
                    // Ten bytes are reserved by the bridge envelope.
                    let capacity = response.len().min(BRIDGE_MAX_FRAME - 10);
                    // `read` returns as soon as it has any byte at all, so a
                    // single call stops partway through a frame. Keep going
                    // until the buffer fills or the window closes.
                    let deadline =
                        std::time::Instant::now() + Duration::from_millis(u64::from(window_ms));
                    let mut filled = 0usize;
                    while filled < capacity {
                        let remaining =
                            deadline.saturating_duration_since(std::time::Instant::now());
                        if remaining.is_zero() {
                            break;
                        }
                        // This argument is FreeRTOS ticks, not milliseconds.
                        let ticks = TickType::from(remaining).ticks() as u32;
                        match self
                            .uart
                            .read(&mut response[filled..capacity], ticks.max(1))
                        {
                            Ok(0) | Err(_) => break,
                            Ok(n) => filled += n,
                        }
                    }
                    Ok(filled)
                }
                BridgeCommand::UartResetCapture => {
                    let swapped = match payload {
                        [] | [0] => false,
                        [1] => true,
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    // Keep the configured TX signal unchanged and remap only
                    // the UART input. GPIO5 remains input-only for this test.
                    esp_idf_svc::sys::esp!(unsafe {
                        uart_set_pin(
                            uart_port_t_UART_NUM_1,
                            UART_PIN_NO_CHANGE,
                            if swapped {
                                pinmap::UART_TX
                            } else {
                                pinmap::UART_RX
                            },
                            UART_PIN_NO_CHANGE,
                            UART_PIN_NO_CHANGE,
                        )
                    })
                    .map_err(|_| SwdError::Protocol(0))?;
                    let capture = (|| {
                        self.uart
                            .change_parity(Parity::ParityNone)
                            .map_err(|_| SwdError::Protocol(0))?;
                        self.uart.clear_rx().map_err(|_| SwdError::Protocol(0))?;
                        // UART_TX stays bound; see the constructor.
                        self.pulse_reset_low_release()
                            .map_err(|_| SwdError::Protocol(0))?;
                        let capacity = response.len().min(BRIDGE_MAX_FRAME - 10);
                        Ok(self.uart.read(&mut response[..capacity], 200).unwrap_or(0))
                    })();
                    let restore = esp_idf_svc::sys::esp!(unsafe {
                        uart_set_pin(
                            uart_port_t_UART_NUM_1,
                            UART_PIN_NO_CHANGE,
                            pinmap::UART_RX,
                            UART_PIN_NO_CHANGE,
                            UART_PIN_NO_CHANGE,
                        )
                    })
                    .map_err(|_| SwdError::Protocol(0));
                    let length = capture?;
                    restore?;
                    Ok(length)
                }
                BridgeCommand::Detach => {
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    self.bridge_attached = false;
                    Ok(0)
                }
                BridgeCommand::ReadRegister => {
                    let [port, address] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let value = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .raw_read_register(*port != 0, *address)?;
                    response[..4].copy_from_slice(&value.to_le_bytes());
                    Ok(4)
                }
                BridgeCommand::ReadRegisterBlock => {
                    let [port, address, low, high] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let count = usize::from(u16::from_le_bytes([*low, *high]));
                    if count == 0 || count > MAX_BLOCK_WORDS || count * 4 > response.len() {
                        return Err(SwdError::Protocol(0));
                    }
                    let mut values = [0_u32; MAX_BLOCK_WORDS];
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .raw_read_register_block(*port != 0, *address, &mut values[..count])?;
                    for (chunk, value) in response[..count * 4]
                        .chunks_exact_mut(4)
                        .zip(values[..count].iter())
                    {
                        chunk.copy_from_slice(&value.to_le_bytes());
                    }
                    Ok(count * 4)
                }
                BridgeCommand::WriteRegisterBlock => {
                    let [port, address, low, high, words @ ..] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let count = usize::from(u16::from_le_bytes([*low, *high]));
                    if count == 0 || count > MAX_BLOCK_WORDS || words.len() != count * 4 {
                        return Err(SwdError::Protocol(0));
                    }
                    let mut values = [0_u32; MAX_BLOCK_WORDS];
                    for (value, chunk) in values[..count].iter_mut().zip(words.chunks_exact(4)) {
                        *value = u32::from_le_bytes(
                            chunk.try_into().map_err(|_| SwdError::Protocol(0))?,
                        );
                    }
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .raw_write_register_block(*port != 0, *address, &values[..count])?;
                    Ok(0)
                }
                BridgeCommand::SetSpeed => {
                    let [a, b, c, d] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let requested = u32::from_le_bytes([*a, *b, *c, *d]);
                    let effective = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .io_mut()
                        .set_clock_hz(requested);
                    response[..4].copy_from_slice(&effective.to_le_bytes());
                    Ok(4)
                }
                BridgeCommand::RecoverLine => {
                    let cycles = match payload {
                        [] => RECOVERY_CYCLES,
                        [low, high] => u16::from_le_bytes([*low, *high]),
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    let (swdio, swclk) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .recover_line(cycles);
                    response[..2].copy_from_slice(&[u8::from(swdio), u8::from(swclk)]);
                    self.bridge_attached = false;
                    Ok(2)
                }
                BridgeCommand::WireProbe => {
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                    let (first, second) = match payload {
                        [] | [0] => swd.wire_probe(false),
                        [1] => swd.wire_probe(true),
                        [2, after_read, before_write] => {
                            let (first, second, third, dpidr) =
                                swd.handover_probe(*after_read, *before_write);
                            (
                                u64::from(first) | u64::from(second) << 8 | u64::from(third) << 16,
                                u64::from(dpidr),
                            )
                        }
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    swd.disconnect();
                    response[..8].copy_from_slice(&first.to_le_bytes());
                    response[8..16].copy_from_slice(&second.to_le_bytes());
                    self.bridge_attached = false;
                    Ok(16)
                }
                BridgeCommand::ApWriteProbe => {
                    let (value, after_read, before_write) = match payload {
                        [] => (0x2300_0052, None, None),
                        [a, b, c, d] => (u32::from_le_bytes([*a, *b, *c, *d]), None, None),
                        [a, b, c, d, after, before] => (
                            u32::from_le_bytes([*a, *b, *c, *d]),
                            Some(*after),
                            Some(*before),
                        ),
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                    let (dpidr_ack, write_ack, before, after) =
                        swd.ap_write_handover_probe(value, after_read, before_write);
                    swd.disconnect();
                    response[0] = dpidr_ack;
                    response[1] = write_ack;
                    response[2..6].copy_from_slice(&before.to_le_bytes());
                    response[6..10].copy_from_slice(&after.to_le_bytes());
                    self.bridge_attached = false;
                    Ok(10)
                }
                BridgeCommand::Ping => {
                    // Costs one round trip and no wire time, which is what
                    // makes it a measurement of the transport alone.
                    Ok(0)
                }
                BridgeCommand::WifiStatus => {
                    let wifi = self.wifi.lock().map_err(|_| SwdError::Protocol(0))?;
                    response[0] = u8::from(wifi.connected);
                    response[1..5].copy_from_slice(&wifi.ip);
                    let ssid = wifi.ssid.as_bytes();
                    let length = ssid.len().min(response.len() - 6);
                    response[5] = length as u8;
                    response[6..6 + length].copy_from_slice(&ssid[..length]);
                    Ok(6 + length)
                }
                BridgeCommand::WifiSet => {
                    let (ssid, password) =
                        wifi_credentials::decode(payload).ok_or(SwdError::Protocol(1))?;
                    // Stored before it is tried: a credential that only lives
                    // in RAM is one power cycle from being lost, and the point
                    // is to not have to say it twice.
                    store_wifi_credentials(ssid, password).map_err(|_| SwdError::Protocol(2))?;
                    let mut wifi = self.wifi.lock().map_err(|_| SwdError::Protocol(3))?;
                    wifi.pending = Some((ssid.to_string(), password.to_string()));
                    wifi.forget = false;
                    // Published here, not from the loop: a join blocks that
                    // loop for the better part of half a minute, and for all
                    // of it `wifi status` was still answering "no network
                    // configured" about the network just handed to it.
                    wifi.ssid = ssid.to_string();
                    Ok(0)
                }
                BridgeCommand::WifiForget => {
                    forget_wifi_credentials().map_err(|_| SwdError::Protocol(2))?;
                    let mut wifi = self.wifi.lock().map_err(|_| SwdError::Protocol(3))?;
                    wifi.pending = None;
                    wifi.forget = true;
                    wifi.ssid = String::new();
                    wifi.connected = false;
                    wifi.ip = [0; 4];
                    Ok(0)
                }
                BridgeCommand::PinMap => {
                    // Which board this firmware was built for. Worth a command
                    // of its own: a mismatch is not a wrong answer, it is two
                    // sets of outputs fighting over the same nets.
                    // The debug port first, then the serial pins. A board
                    // built on this bridge appends its own after these; the
                    // host reports the extras positionally, because only that
                    // board's firmware knows what they are.
                    for (slot, pin) in response[..5].iter_mut().zip([
                        pinmap::SWDIO,
                        pinmap::SWCLK,
                        pinmap::RESET,
                        pinmap::UART_TX,
                        pinmap::UART_RX,
                    ]) {
                        *slot = pin as u8;
                    }
                    Ok(5)
                }
                BridgeCommand::Echo => {
                    // Returns a payload without touching the wire, so the
                    // transport's sustained bandwidth can be measured apart
                    // from anything SWD does.
                    let [low, high] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let length = usize::from(u16::from_le_bytes([*low, *high]));
                    if length > response.len() {
                        return Err(SwdError::Protocol(0));
                    }
                    response[..length].fill(0xa5);
                    Ok(length)
                }
                BridgeCommand::Profile => {
                    let [a, b, c, d, low, high] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let address = u32::from_le_bytes([*a, *b, *c, *d]);
                    let count = usize::from(u16::from_le_bytes([*low, *high]));
                    if count == 0 || count > MAX_BLOCK_WORDS {
                        return Err(SwdError::Protocol(0));
                    }
                    let mut words = [0_u32; MAX_BLOCK_WORDS];
                    let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                    swd.io_mut().peripheral_profile(true);
                    let started = cpu_cycles();
                    swd.read_words(address, &mut words[..count])?;
                    let elapsed = cpu_cycles().wrapping_sub(started);
                    let (run_cycles, run_count) = swd.io_mut().peripheral_profile(true);
                    response[..4].copy_from_slice(&elapsed.to_le_bytes());
                    response[4..8].copy_from_slice(&run_cycles.to_le_bytes());
                    response[8..12].copy_from_slice(&run_count.to_le_bytes());
                    response[12..16].copy_from_slice(&(count as u32).to_le_bytes());
                    Ok(16)
                }
                BridgeCommand::MemoryRead => {
                    let [a, b, c, d, low, high] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let address = u32::from_le_bytes([*a, *b, *c, *d]);
                    let count = usize::from(u16::from_le_bytes([*low, *high]));
                    if count == 0 || count > MAX_BLOCK_WORDS || count * 4 > response.len() {
                        return Err(SwdError::Protocol(0));
                    }
                    let mut words = [0_u32; MAX_BLOCK_WORDS];
                    // TAR reprogramming at each auto-increment boundary happens
                    // here rather than costing a round trip apiece.
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .read_words(address, &mut words[..count])?;
                    for (chunk, word) in response[..count * 4]
                        .chunks_exact_mut(4)
                        .zip(words[..count].iter())
                    {
                        chunk.copy_from_slice(&word.to_le_bytes());
                    }
                    Ok(count * 4)
                }
                BridgeCommand::SpiLoopback => {
                    let [count, pattern @ ..] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    if pattern.len() != 8 || *count == 0 || *count > 64 {
                        return Err(SwdError::Protocol(0));
                    }
                    let bits =
                        u64::from_le_bytes(pattern.try_into().map_err(|_| SwdError::Protocol(0))?);
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    let observed = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .io_mut()
                        .spi_loopback(bits, *count);
                    response[..8].copy_from_slice(&observed.to_le_bytes());
                    self.bridge_attached = false;
                    Ok(8)
                }
                BridgeCommand::SetEngine => {
                    let engine = match payload {
                        [0] => Engine::Hardware,
                        [1] => Engine::BitBang,
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                    swd.disconnect();
                    swd.io_mut().set_engine(engine);
                    self.bridge_attached = false;
                    response[..4].copy_from_slice(&swd.io_mut().clock_hz().to_le_bytes());
                    Ok(4)
                }
                BridgeCommand::SetPinMap => {
                    let [swapped] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                    swd.disconnect();
                    swd.io_mut().set_pin_map(*swapped != 0);
                    self.bridge_attached = false;
                    response[0] = *swapped;
                    Ok(1)
                }
                BridgeCommand::Capabilities => {
                    let clock_hz = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .io_mut()
                        .clock_hz();
                    response[..4].copy_from_slice(&(MAX_BLOCK_WORDS as u32).to_le_bytes());
                    response[4..8].copy_from_slice(&clock_hz.to_le_bytes());
                    Ok(8)
                }
                BridgeCommand::WriteRegister => {
                    let [port, address, value @ ..] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    if value.len() != 4 {
                        return Err(SwdError::Protocol(0));
                    }
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .raw_write_register(
                            *port != 0,
                            *address,
                            u32::from_le_bytes(
                                value.try_into().map_err(|_| SwdError::Protocol(0))?,
                            ),
                        )?;
                    Ok(0)
                }
                BridgeCommand::SwjSequence => {
                    let Some((&bit_len, bits)) = payload.split_first() else {
                        return Err(SwdError::Protocol(0));
                    };
                    if bits.len() != 8 {
                        return Err(SwdError::Protocol(0));
                    }
                    self.swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .swj_sequence(
                            bit_len,
                            u64::from_le_bytes(bits.try_into().map_err(|_| SwdError::Protocol(0))?),
                        )?;
                    Ok(0)
                }
                BridgeCommand::LineState => {
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    let (swdio, swclk) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    response[..3].copy_from_slice(&[
                        u8::from(swdio),
                        u8::from(swclk),
                        u8::from(self.reset.is_high()),
                    ]);
                    self.bridge_attached = false;
                    Ok(3)
                }
                BridgeCommand::ResetLineState => {
                    self.release_pads().map_err(|_| SwdError::Protocol(0))?;
                    self.reset.set_low().map_err(|_| SwdError::Protocol(0))?;
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    let (asserted_swdio, asserted_swclk) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    let asserted_reset = self.reset.is_high();
                    self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    let (released_swdio, released_swclk) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    response[..6].copy_from_slice(&[
                        u8::from(asserted_swdio),
                        u8::from(asserted_swclk),
                        u8::from(asserted_reset),
                        u8::from(released_swdio),
                        u8::from(released_swclk),
                        u8::from(self.reset.is_high()),
                    ]);
                    self.bridge_attached = false;
                    Ok(6)
                }
                // Reset drives the target's reset net and nothing else, so
                // neither of these touches the SWD link. They used to, and
                // that made them unusable for connect-under-reset: a debug
                // sequence asserts reset *after* attaching, and tearing the
                // link down underneath it fails the very next transfer.
                BridgeCommand::ResetAssert => {
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    response[0] = u8::from(self.reset.is_high());
                    Ok(1)
                }
                BridgeCommand::ResetRelease => {
                    self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    response[0] = u8::from(self.reset.is_high());
                    Ok(1)
                }
                // Answered in `bridge_command` before reaching here, because
                // "this board defines no such command" is a dispatch fact and
                // not something that happened on the SWD wire.
                BridgeCommand::Extension(_) => Err(SwdError::Protocol(0)),
            }
        }
    }

    /// The default NVS partition, taken once.
    ///
    /// `take` is a one-shot: the radio needs the partition too, so a second
    /// call fails. Handing out clones of the first one is the only way both
    /// the credential store and Wi-Fi can have it.
    static NVS_PARTITION: Mutex<Option<EspDefaultNvsPartition>> = Mutex::new(None);

    fn nvs_partition() -> Result<EspDefaultNvsPartition> {
        let mut slot = NVS_PARTITION
            .lock()
            .map_err(|_| anyhow::anyhow!("the NVS partition lock is poisoned"))?;
        if slot.is_none() {
            *slot = Some(EspDefaultNvsPartition::take()?);
        }
        Ok(slot.as_ref().expect("just populated").clone())
    }

    /// Opens the store credentials live in.
    fn credential_store() -> Result<esp_idf_svc::nvs::EspDefaultNvs> {
        Ok(esp_idf_svc::nvs::EspNvs::new(
            nvs_partition()?,
            NVS_NAMESPACE,
            true,
        )?)
    }

    fn store_wifi_credentials(ssid: &str, password: &str) -> Result<()> {
        let store = credential_store()?;
        store.set_str(NVS_SSID, ssid)?;
        store.set_str(NVS_PASSWORD, password)?;
        Ok(())
    }

    fn forget_wifi_credentials() -> Result<()> {
        let store = credential_store()?;
        // Stored empty rather than removed. An absent key means "never
        // provisioned", which falls back to whatever the image was built with;
        // an empty one means the host said forget, and has to outrank that.
        // Removing the key made `forget` report success and change nothing.
        store.set_str(NVS_SSID, "")?;
        let _ = store.remove(NVS_PASSWORD);
        Ok(())
    }

    /// The network this probe has been told to join, if any.
    ///
    /// Storage is the only source. Credentials used to be compiled in as well,
    /// which put the real passphrase into every image built and meant `forget`
    /// could not actually forget — it cleared storage and the build-time value
    /// took over again. Provisioning over the bridge replaces all of that.
    fn load_wifi_credentials() -> Option<(String, String)> {
        let store = credential_store().ok()?;
        let mut ssid = [0u8; wifi_credentials::MAX_SSID + 1];
        let mut password = [0u8; wifi_credentials::MAX_PASSWORD + 1];
        // An empty stored SSID is the host having said forget, which is not
        // the same as never provisioned; both end up here as `None`.
        let ssid = store
            .get_str(NVS_SSID, &mut ssid)
            .ok()
            .flatten()
            .filter(|ssid| !ssid.is_empty())?
            .to_string();
        let password = store
            .get_str(NVS_PASSWORD, &mut password)
            .ok()
            .flatten()
            .unwrap_or("")
            .to_string();
        Some((ssid, password))
    }

    /// Every GPIO the chip has, indexed by its number.
    ///
    /// `Pins` names each pad as a distinct type, which is exactly wrong for a
    /// map that is chosen at build time; taking each one once out of this
    /// table keeps the compiler's guarantee that a pad is claimed only once.
    fn numbered_pins(pins: esp_idf_svc::hal::gpio::Pins) -> [Option<AnyIOPin<'static>>; 22] {
        [
            Some(pins.gpio0.into()),
            Some(pins.gpio1.into()),
            Some(pins.gpio2.into()),
            Some(pins.gpio3.into()),
            Some(pins.gpio4.into()),
            Some(pins.gpio5.into()),
            Some(pins.gpio6.into()),
            Some(pins.gpio7.into()),
            Some(pins.gpio8.into()),
            Some(pins.gpio9.into()),
            Some(pins.gpio10.into()),
            Some(pins.gpio11.into()),
            Some(pins.gpio12.into()),
            Some(pins.gpio13.into()),
            Some(pins.gpio14.into()),
            Some(pins.gpio15.into()),
            Some(pins.gpio16.into()),
            Some(pins.gpio17.into()),
            Some(pins.gpio18.into()),
            Some(pins.gpio19.into()),
            Some(pins.gpio20.into()),
            Some(pins.gpio21.into()),
        ]
    }

    pub fn run() -> Result<()> {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();

        info!(
            "Pin map: SWDIO=GPIO{} SWCLK=GPIO{} RESET=GPIO{} \
             UART_TX=GPIO{} UART_RX=GPIO{}",
            pinmap::SWDIO,
            pinmap::SWCLK,
            pinmap::RESET,
            pinmap::UART_TX,
            pinmap::UART_RX
        );

        let peripherals = Peripherals::take()?;
        let sys_loop = EspSystemEventLoop::take()?;
        let nvs = nvs_partition()?;
        for pin in pinmap::CLAIMED {
            Hub::reclaim_gpio(pin)?;
        }
        let _wifi_events = sys_loop.subscribe::<WifiEvent, _>(|event| {
            if let WifiEvent::StaDisconnected(event) = event {
                warn!(
                    "Wi-Fi disconnected from {:02x?}: reason={}, rssi={}",
                    event.bssid(),
                    event.reason(),
                    event.rssi()
                );
            }
        })?;

        // Indexed rather than named, so one build serves any board revision
        // that only moves the connector.
        let mut pins = numbered_pins(peripherals.pins);
        let mut take = |number: i32| -> Result<AnyIOPin<'static>> {
            pins[number as usize]
                .take()
                .with_context(|| format!("GPIO{number} is claimed by two signals"))
        };
        let mut reset = PinDriver::input_output(take(pinmap::RESET)?, Pull::Floating)?;
        reset.set_low()?;
        // Released to an input immediately: the level is the target's pull-up
        // to set, not this firmware's. Named through `pinmap` rather than
        // written as a literal — a literal here once outlived the revision it
        // was true for and quietly became "release some other board's pin".
        Hub::set_gpio_direction(pinmap::RESET, gpio_mode_t_GPIO_MODE_INPUT)?;

        let uart = UartDriver::new(
            peripherals.uart1,
            take(pinmap::UART_TX)?,
            take(pinmap::UART_RX)?,
            Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
            Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
            &UartConfig::new().baudrate(Hertz(115_200)),
        )?;
        // UART_TX stays bound to UART1 for the life of the firmware.
        //
        // It used to be released to an input here and re-driven around every
        // write, which would be right if this pin sat on the target's
        // *transmit* line - two transmitters on one wire is real contention.
        // It does not: UART_TX is wired to the target's receive input, which
        // drives nothing, so there is nothing to contend with. The release was
        // also not free - ESP-IDF's `gpio_set_direction(_, INPUT)`
        // calls `gpio_output_disable`, which rewrites the pad's
        // `func_out_sel` to `SIG_GPIO_OUT_IDX` to guarantee no peripheral is
        // routed there. Re-enabling the output afterwards restores the driver
        // but not the routing, so the pad drove a constant level - idle high,
        // indistinguishable from a healthy idle line - and not one byte ever
        // left. The far end saw a quiet, error-free, permanently empty wire.
        let swd = EspSwdIo::new(take(pinmap::SWDIO)?, take(pinmap::SWCLK)?, SWD_PINS_SWAPPED)?;

        let wifi_control = Arc::new(Mutex::new(WifiControl::default()));
        let hub = Arc::new(Mutex::new(Hub {
            reset,
            uart,
            swd: Some(SwdLink::new(swd)),
            wifi: wifi_control.clone(),
            bridge_attached: false,
            reset_held: false,
        }));
        spawn_host_bridge(hub.clone())?;

        let mut wifi = BlockingWifi::wrap(
            EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
            sys_loop,
        )?;
        // The bridge is up on USB before the radio is touched: a board with no
        // credentials is still a working probe, and is how the first ones get
        // set. Wi-Fi failing must never take USB down with it.
        match start_wifi(&mut wifi) {
            Ok(()) => {
                if let Ok(info) = wifi.wifi().ap_netif().get_ip_info() {
                    info!("Access point online at {}", info.ip);
                }
            }
            Err(error) => warn!("Wi-Fi did not start: {error}; the USB bridge is unaffected"),
        }

        spawn_network_bridge(hub.clone())?;

        // Explicit rather than defaulted. The realistic load is a health
        // poller and the odd curl — every socket is a buffer this chip pays
        // for whether or not anything ever connects to it.
        let mut server = EspHttpServer::new(&esp_idf_svc::http::server::Configuration {
            stack_size: 8192,
            // A websocket holds its socket for as long as the page is open,
            // so this is now a count of simultaneous *users* plus whatever
            // REST traffic overlaps them — not a count of requests in flight.
            // At four, three stale tabs left the board unreachable on every
            // route including its own health check.
            max_open_sockets: 8,
            max_uri_handlers: 24,
            lru_purge_enable: true,
            ..Default::default()
        })?;
        register_handlers(&mut server, wifi_control.clone())?;

        // What the radio should be on, read from storage once. Re-reading it
        // every pass reopened the NVS handle twice a second, which is both
        // pointless work and two lines of log noise per second.
        let mut credentials = load_wifi_credentials();
        let mut attempt = 0usize;
        let mut fallback_passes = 0usize;

        loop {
            thread::sleep(Duration::from_secs(2));

            // Anything the host asked for since the last pass.
            let (pending, forget) = match wifi_control.lock() {
                Ok(mut control) => (control.pending.take(), std::mem::take(&mut control.forget)),
                Err(_) => (None, false),
            };
            if forget {
                credentials = None;
                info!("Wi-Fi credentials cleared; disconnecting");
                let _ = wifi.disconnect();
                // Back to whatever an unprovisioned probe does, which is to
                // publish its own access point. Disconnecting alone left the
                // radio configured for a network it had just been told to
                // forget, and with no way back in over the air.
                if let Err(error) = apply_configuration(&mut wifi, None) {
                    warn!("Could not fall back to the access point: {error}");
                }
            }
            if let Some((ssid, password)) = pending {
                info!("Joining Wi-Fi {ssid} on request");
                attempt = 0;
                fallback_passes = 0;
                match join_wifi(&mut wifi, &ssid, &password, attempt) {
                    // Left at zero on success, so the next reconnect starts
                    // from the power just stored rather than one rung down it.
                    Ok(true) => info!("Joined {ssid}"),
                    Ok(false) => {
                        warn!("Could not join {ssid}");
                        attempt += 1;
                    }
                    Err(error) => {
                        warn!("Joining {ssid} failed: {error}");
                        attempt += 1;
                    }
                }
                credentials = Some((ssid, password));
            } else if fallback_passes > 0 {
                // Holding the access point open. Nothing to do but count down.
                fallback_passes -= 1;
                if fallback_passes == 0 {
                    info!("Access point window over; trying the stored network again");
                }
            } else if !station_online(&wifi)
                && let Some((ssid, password)) = credentials.as_ref()
            {
                // Each retry moves down the ladder; a success rewinds it, so a
                // link that drops later retries at the power that worked
                // instead of resuming mid-ladder at one that did not.
                if join_wifi(&mut wifi, ssid, password, attempt).unwrap_or(false) {
                    info!("Joined {ssid}");
                    attempt = 0;
                } else {
                    attempt += 1;
                    // A whole ladder with nothing to show for it. Joining puts
                    // the radio in station mode, so retrying forever means a
                    // probe given a wrong passphrase, or carried out of range,
                    // has no way in but the cable the access point exists to
                    // avoid needing. Give it back for a while, then try again.
                    // Only worth pausing for if there is an access point to
                    // pause for. Built without one, `apply_configuration`
                    // programs an empty station and still succeeds, so waiting
                    // would spend a third of the retry budget on a fallback
                    // that does not exist while the log claimed otherwise.
                    if attempt.is_multiple_of(TX_POWER_LADDER.len())
                        && access_point_configuration().is_ok_and(|ap| ap.is_some())
                    {
                        warn!(
                            "{ssid} did not answer at any transmit power; \
                             publishing the access point for {AP_FALLBACK_PASSES} passes"
                        );
                        if let Err(error) = apply_configuration(&mut wifi, None) {
                            warn!("Could not publish the access point: {error}");
                        } else {
                            fallback_passes = AP_FALLBACK_PASSES;
                        }
                    }
                }
            }

            // Publish what the host will see through `wifi status`.
            if let Ok(mut control) = wifi_control.lock() {
                // `is_connected` is also true for a running access point,
                // which reported a connection with no network and no address.
                // An address on the board interface is the thing the host
                // actually cares about.
                control.connected = station_online(&wifi);
                control.ip = wifi
                    .wifi()
                    .sta_netif()
                    .get_ip_info()
                    .map(|info| info.ip.octets())
                    .unwrap_or([0; 4]);
                control.ssid = credentials
                    .as_ref()
                    .map(|(ssid, _)| ssid.clone())
                    .unwrap_or_default();
            }
        }
    }

    /// Brings up the host link and serves the bridge on it.
    ///
    /// Which link that is depends on the part: USB Serial/JTAG where there is
    /// one, and UART0 where there is not. `serve_bridge` is generic over the
    /// stream, so only the setup differs.
    fn spawn_host_bridge(hub: Arc<Mutex<Hub>>) -> Result<()> {
        #[cfg(esp32)]
        {
            // No USB peripheral on this part. UART0 is the link, and the
            // console is configured off in its sdkconfig so that log output
            // cannot interleave with framed bridge traffic — a stray line of
            // text mid-frame is a CRC failure the host has to time out on.
            spawn_uart_bridge(hub)
        }
        #[cfg(not(esp32))]
        {
            spawn_usb_bridge(hub)
        }
    }

    #[cfg(esp32)]
    fn spawn_uart_bridge(hub: Arc<Mutex<Hub>>) -> Result<()> {
        use esp_idf_svc::sys::{
            uart_config_t, uart_driver_install, uart_param_config, uart_port_t_UART_NUM_0,
        };
        // The bridge's own framing carries the integrity check, so the link
        // only has to be fast and 8N1. 921600 is what a CP2102 sustains
        // comfortably; the transport is the limit on this part either way.
        let config = uart_config_t {
            baud_rate: 921_600,
            data_bits: esp_idf_svc::sys::uart_word_length_t_UART_DATA_8_BITS,
            parity: esp_idf_svc::sys::uart_parity_t_UART_PARITY_DISABLE,
            stop_bits: esp_idf_svc::sys::uart_stop_bits_t_UART_STOP_BITS_1,
            flow_ctrl: esp_idf_svc::sys::uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
            ..Default::default()
        };
        // SAFETY: UART0 is owned by this bridge for the life of the firmware,
        // and `config` stays valid for the duration of the call.
        unsafe {
            esp_idf_svc::sys::esp!(uart_param_config(uart_port_t_UART_NUM_0, &config))?;
            esp_idf_svc::sys::esp!(uart_driver_install(
                uart_port_t_UART_NUM_0,
                4096,
                3 * (BRIDGE_MAX_FRAME as i32 + 2),
                0,
                core::ptr::null_mut(),
                0,
            ))?;
        }
        thread::Builder::new()
            .name("uart-dap-bridge".into())
            .stack_size(48 * 1024)
            .spawn(move || {
                let mut link = UartLink;
                loop {
                    serve_bridge(&hub, &mut link);
                }
            })?;
        Ok(())
    }

    /// UART0 as a byte stream, for parts with no USB Serial/JTAG.
    #[cfg(esp32)]
    struct UartLink;

    #[cfg(esp32)]
    impl std::io::Read for UartLink {
        fn read(&mut self, buffer: &mut [u8]) -> std::io::Result<usize> {
            // SAFETY: `buffer` is writable for its length and this task is the
            // driver's only reader.
            let read = unsafe {
                esp_idf_svc::sys::uart_read_bytes(
                    esp_idf_svc::sys::uart_port_t_UART_NUM_0,
                    buffer.as_mut_ptr().cast(),
                    buffer.len() as u32,
                    10,
                )
            };
            Ok(read.max(0) as usize)
        }
    }

    #[cfg(esp32)]
    impl std::io::Write for UartLink {
        fn write(&mut self, buffer: &[u8]) -> std::io::Result<usize> {
            // SAFETY: `buffer` stays readable for its length; this task is the
            // driver's only writer.
            unsafe {
                esp_idf_svc::sys::uart_write_bytes(
                    esp_idf_svc::sys::uart_port_t_UART_NUM_0,
                    buffer.as_ptr().cast(),
                    buffer.len(),
                );
            }
            Ok(buffer.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[cfg(not(esp32))]
    fn spawn_usb_bridge(hub: Arc<Mutex<Hub>>) -> Result<()> {
        let mut config = esp_idf_svc::sys::usb_serial_jtag_driver_config_t {
            // Two full responses must fit, so the wire engine can start the
            // next block while the USB hardware is still draining the last.
            tx_buffer_size: 3 * (BRIDGE_MAX_FRAME as u32 + 2),
            rx_buffer_size: 4096,
        };
        // SAFETY: `config` remains valid for the duration of the call and the
        // driver is installed once before its single reader task starts.
        let install_result =
            unsafe { esp_idf_svc::sys::usb_serial_jtag_driver_install(&mut config) };
        if install_result != esp_idf_svc::sys::ESP_OK {
            return Err(anyhow!(
                "USB Serial/JTAG driver install failed: {install_result}"
            ));
        }
        thread::Builder::new()
            .name("usb-dap-bridge".into())
            // Frame buffers, a block-read staging array, and the response
            // payload all live on this task's stack.
            .stack_size(48 * 1024)
            .spawn(move || serve_bridge(&hub, &mut UsbSerialLink))
            .context("failed to start USB DAP bridge")?;
        Ok(())
    }

    /// The bridge protocol over the ESP32-C3's USB Serial/JTAG endpoint.
    #[cfg(not(esp32))]
    struct UsbSerialLink;

    #[cfg(not(esp32))]
    impl std::io::Read for UsbSerialLink {
        fn read(&mut self, buffer: &mut [u8]) -> std::io::Result<usize> {
            // SAFETY: `buffer` is writable for its length, and the bridge task
            // is this driver's only reader.
            let read = unsafe {
                esp_idf_svc::sys::usb_serial_jtag_read_bytes(
                    buffer.as_mut_ptr().cast(),
                    buffer.len() as u32,
                    100,
                )
            };
            Ok(read.max(0) as usize)
        }
    }

    #[cfg(not(esp32))]
    impl std::io::Write for UsbSerialLink {
        fn write(&mut self, buffer: &[u8]) -> std::io::Result<usize> {
            usb_write_all(buffer);
            Ok(buffer.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    /// Serves the bridge protocol on any byte stream until it fails.
    ///
    /// The framing, dispatch and response path are identical whichever link
    /// carries them, so USB and the network share this rather than growing two
    /// copies that drift apart. Both hold the same `Hub` mutex, so a command
    /// arriving over one link cannot interleave with one over the other.
    fn serve_bridge<L: std::io::Read + std::io::Write>(hub: &Arc<Mutex<Hub>>, link: &mut L) {
        let mut encoded = vec![0u8; BRIDGE_MAX_FRAME + 2];
        let mut frame = vec![0u8; BRIDGE_MAX_FRAME + 2];
        let mut payload = vec![0u8; MAX_BLOCK_WORDS * 4];
        let mut chunk = [0u8; 256];
        let mut frame_len = 0usize;
        loop {
            let read = match link.read(&mut chunk) {
                Ok(0) => continue,
                Ok(read) => read,
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(error) if error.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(_) => return,
            };
            for &byte in &chunk[..read] {
                if byte != 0 {
                    if frame_len < frame.len() {
                        frame[frame_len] = byte;
                        frame_len += 1;
                    } else {
                        frame_len = 0;
                    }
                    continue;
                }
                if frame_len == 0 {
                    continue;
                }
                let request = match decode_request(&mut frame[..frame_len]) {
                    Ok(request) => request,
                    Err(_) => {
                        frame_len = 0;
                        continue;
                    }
                };
                let (status, payload_len) = match hub.lock() {
                    Ok(mut hub) => {
                        hub.bridge_command(request.command, request.payload, &mut payload)
                    }
                    Err(_) => (BridgeStatus::Transport, 0),
                };
                if let Ok(length) = encode_response(
                    request.sequence,
                    status,
                    &payload[..payload_len],
                    &mut encoded,
                ) {
                    // One write per reply, delimiter included: two writes let
                    // a network client see a frame boundary as two packets.
                    let mut framed = Vec::with_capacity(length + 1);
                    framed.push(0);
                    framed.extend_from_slice(&encoded[..length]);
                    if link.write_all(&framed).is_err() {
                        return;
                    }
                }
                frame_len = 0;
            }
        }
    }

    /// Serves the same bridge protocol to one network client at a time.
    ///
    /// The point is that nothing above the transport changes: identification,
    /// bulk reads and programming behave the same whether the frames arrived
    /// over USB or over Wi-Fi.
    fn spawn_network_bridge(hub: Arc<Mutex<Hub>>) -> Result<()> {
        thread::Builder::new()
            .name("net-dap-bridge".into())
            .stack_size(32 * 1024)
            .spawn(move || {
                let listener = match std::net::TcpListener::bind(("0.0.0.0", BRIDGE_TCP_PORT)) {
                    Ok(listener) => listener,
                    Err(error) => {
                        error!("DAP bridge cannot listen on {BRIDGE_TCP_PORT}: {error}");
                        return;
                    }
                };
                info!("DAP bridge listening on tcp/{BRIDGE_TCP_PORT}");
                for stream in listener.incoming() {
                    let Ok(mut stream) = stream else { continue };
                    // Nagle would hold a finished reply back waiting for more
                    // to send, which is the opposite of what a request/response
                    // protocol wants.
                    let _ = stream.set_nodelay(true);
                    let _ = stream.set_read_timeout(Some(Duration::from_millis(100)));
                    match stream.peer_addr() {
                        Ok(peer) => info!("DAP bridge client {peer} connected"),
                        Err(_) => info!("DAP bridge client connected"),
                    }
                    serve_bridge(&hub, &mut stream);
                    info!("DAP bridge client disconnected");
                }
            })
            .context("failed to start network DAP bridge")?;
        Ok(())
    }

    #[cfg(not(esp32))]
    fn usb_write_all(mut bytes: &[u8]) {
        while !bytes.is_empty() {
            // SAFETY: `bytes` remains readable for its reported length. This
            // bridge task is the USB driver's sole direct writer.
            let written = unsafe {
                esp_idf_svc::sys::usb_serial_jtag_write_bytes(
                    bytes.as_ptr().cast(),
                    bytes.len(),
                    100,
                )
            };
            if written <= 0 {
                return;
            }
            bytes = &bytes[written as usize..];
        }
    }

    /// Loop passes to hold the access point open after a failed ladder, at two
    /// seconds a pass.
    const AP_FALLBACK_PASSES: usize = 15;

    /// Whether the board half actually has a network.
    ///
    /// `is_connected` is also true for a running access point, so on its own it
    /// reports a probe publishing its own SSID as connected to something.
    fn station_online(wifi: &BlockingWifi<EspWifi<'static>>) -> bool {
        wifi.is_connected().unwrap_or(false)
            && wifi
                .wifi()
                .sta_netif()
                .get_ip_info()
                .is_ok_and(|info| !info.ip.is_unspecified())
    }

    /// Brings the radio up with whatever configuration is available.
    ///
    /// A station with no credentials is still worth starting: the access point
    /// gives a way in, and the board can be given a network later without a
    /// restart.
    fn start_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
        set_regulatory_domain();
        let credentials = load_wifi_credentials();
        apply_configuration(wifi, credentials.as_ref())?;
        wifi.start()?;
        disable_power_save();
        Ok(())
    }

    /// A debug probe wants latency, not battery life.
    ///
    /// Modem sleep parks the radio between beacons, which put 130 ms on a round
    /// trip to a bridge whose gateway answers in 0.3 ms, and every SWD command
    /// pays it.
    fn disable_power_save() {
        // SAFETY: the driver is started; this only sets a power-save mode.
        let result = unsafe {
            esp_idf_svc::sys::esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE)
        };
        if result != esp_idf_svc::sys::ESP_OK {
            warn!("Could not disable Wi-Fi power save: {result}");
        }
    }

    /// Leaves ESP-IDF's world-safe regulatory mode for a named country.
    ///
    /// Without this the radio receives and does not transmit: it scans, reports
    /// sane signal strengths, reaches `init -> auth` and times out one second
    /// later with `reason=2`, because the authentication frame it believes it
    /// sent never reaches the air. Monitoring the target's own channel during a
    /// join caught 2112 frames from 23 transmitters and not one from this
    /// radio. Naming a country is what lets it transmit at all.
    ///
    /// 802.11d stays enabled, so the country actually used is taken from the
    /// beacons around it and this is only the starting point.
    fn set_regulatory_domain() {
        let country = WIFI_COUNTRY.unwrap_or(DEFAULT_WIFI_COUNTRY);
        if WIFI_COUNTRY.is_none() {
            warn!(
                "WIFI_COUNTRY was not set, so the radio starts in world-safe mode \
                 ({DEFAULT_WIFI_COUNTRY}). In that mode this chip receives and does not \
                 transmit: scans work, association times out with reason=2. Build with \
                 WIFI_COUNTRY set to your own regulatory domain."
            );
        }
        let Ok(code) = std::ffi::CString::new(country) else {
            warn!("WIFI_COUNTRY {country:?} is not a valid country code");
            return;
        };
        // SAFETY: `code` is a valid NUL-terminated string for the call.
        let result = unsafe { esp_idf_svc::sys::esp_wifi_set_country_code(code.as_ptr(), true) };
        if result == esp_idf_svc::sys::ESP_OK {
            info!("Regulatory domain set to {country}");
        } else {
            warn!("Could not set regulatory domain to {country}: {result}");
        }
    }

    /// Programs the radio for one set of station credentials, or for none.
    ///
    /// The access point comes up only when there is no network to join. The two
    /// cannot be had at once in any useful sense: one radio means the soft AP
    /// drags the board onto its channel, and the log shows exactly that —
    /// `ap channel adjust o:1,1 n:3,1` immediately before an authentication
    /// that expires. Since provisioning now arrives over USB, the access point
    /// is a fallback for an unprovisioned probe, not a permanent fixture.
    fn apply_configuration(
        wifi: &mut BlockingWifi<EspWifi<'static>>,
        credentials: Option<&(String, String)>,
    ) -> Result<()> {
        let configuration = match credentials {
            Some((ssid, password)) => {
                Configuration::Client(client_configuration(ssid, password, None, None)?)
            }
            None => match access_point_configuration()? {
                Some(access_point) => Configuration::AccessPoint(access_point),
                None => Configuration::Client(ClientConfiguration::default()),
            },
        };
        wifi.set_configuration(&configuration)?;
        Ok(())
    }

    /// Transmit powers to try, in quarter-dBm, strongest first.
    ///
    /// Full power is not always the best power. The ESP32-C3 SuperMini's
    /// antenna is badly matched, and a mismatched antenna reflects the power
    /// amplifier's output back into it; backing the drive off can put a
    /// cleaner signal on the air than driving it flat out. Since which level
    /// wins depends on the board, the join tries them in turn rather than
    /// assuming.
    const TX_POWER_LADDER: [i8; 5] = [80, 60, 52, 40, 32];

    /// Quarter-dBm bounds `esp_wifi_set_max_tx_power` accepts.
    const MIN_TX_POWER: i8 = 8;
    const MAX_TX_POWER: i8 = 84;

    /// The transmit power that last worked, so a probe does not re-derive it.
    ///
    /// Range-checked on the way out: a stale byte left in this namespace by
    /// another build casts to anything at all, and an out-of-range request is
    /// rejected by the driver, quietly wasting the one attempt that was
    /// supposed to be the fast path.
    fn load_tx_power() -> Option<i8> {
        let stored = credential_store()
            .ok()?
            .get_u8(NVS_TX_POWER)
            .ok()
            .flatten()? as i8;
        if (MIN_TX_POWER..=MAX_TX_POWER).contains(&stored) {
            Some(stored)
        } else {
            warn!("Ignoring stored transmit power {stored}, which is out of range");
            None
        }
    }

    fn store_tx_power(power: i8) {
        if let Ok(store) = credential_store()
            && store.set_u8(NVS_TX_POWER, power as u8).is_err()
        {
            warn!("Could not remember the working transmit power");
        }
    }

    /// Which power to try on this attempt.
    ///
    /// The level that worked last time goes first, so a probe that has already
    /// found its board's sweet spot joins immediately instead of spending a
    /// minute walking back down the ladder on every boot.
    fn tx_power_for(attempt: usize, remembered: Option<i8>) -> i8 {
        match remembered {
            Some(power) if attempt == 0 => power,
            _ => TX_POWER_LADDER[attempt % TX_POWER_LADDER.len()],
        }
    }

    /// Associates with one network, leaving the access point as it was.
    fn join_wifi(
        wifi: &mut BlockingWifi<EspWifi<'static>>,
        ssid: &str,
        password: &str,
        attempt: usize,
    ) -> Result<bool> {
        let _ = wifi.disconnect();
        // Station mode first, and unconditionally. An access-point-only radio
        // cannot scan — `esp_wifi_scan_start` returns ESP_FAIL — so a probe
        // that came up unprovisioned would skip the scan below and associate
        // without pinning a BSSID, on exactly the first join where the user
        // has no other way in.
        apply_configuration(wifi, Some(&(ssid.to_string(), password.to_string())))?;
        if !wifi.is_started().unwrap_or(false) {
            wifi.start()?;
        }

        // Pick the access point to associate with, rather than letting the
        // driver choose. Asking for a complete scan sorted by signal did not
        // do it: on a mesh publishing one SSID from several nodes, this kept
        // authenticating against a node at -80 dBm while another answering to
        // the same name sat at -60. Twenty decibels is the difference between
        // a link that closes and one that times out, and a timeout at that
        // stage is indistinguishable from a wrong password.
        let strongest = wifi.scan().ok().and_then(|found| {
            found
                .into_iter()
                .filter(|point| point.ssid.as_str() == ssid)
                .max_by_key(|point| point.signal_strength)
        });
        match &strongest {
            Some(point) => info!(
                "Associating with {ssid} at {:02x?} on channel {} ({} dBm)",
                point.bssid, point.channel, point.signal_strength
            ),
            None => warn!("{ssid} did not answer a scan; trying anyway"),
        }
        let station = client_configuration(
            ssid,
            password,
            strongest.as_ref().map(|point| point.bssid),
            strongest.as_ref().map(|point| point.channel),
        )?;
        wifi.set_configuration(&Configuration::Client(station))?;

        let tx_power = tx_power_for(attempt, load_tx_power());
        // SAFETY: the radio is started and this only sets a driver limit.
        let set = unsafe { esp_idf_svc::sys::esp_wifi_set_max_tx_power(tx_power) };
        if set != esp_idf_svc::sys::ESP_OK {
            warn!("Could not set transmit power to {tx_power} quarter-dBm: {set}");
        }
        info!(
            "Attempt {attempt} at {}.{} dBm",
            tx_power / 4,
            (tx_power % 4) * 25
        );
        match wifi.connect().and_then(|()| wifi.wait_netif_up()) {
            Ok(()) => {
                // Re-asserted after every join, not only at start-up. The
                // driver restores its default power-save mode across a stop or
                // a restart, and this path can reach `wifi.start()` above, so a
                // probe that reconnected once would quietly go back to parking
                // its radio between beacons — and pay for it on every command
                // for the rest of the session.
                disable_power_save();
                // Re-applied after association: setting it once at start-up
                // did not survive `connect`, and the round trip stayed at
                // 130 ms against a gateway 0.3 ms away.
                disable_power_save();
                store_tx_power(tx_power);
                Ok(true)
            }
            Err(error) => {
                warn!("Association with {ssid} failed: {error}");
                // Which access points actually answered, so a failure to join
                // can be told apart from a failure to find.
                if let Ok(seen) = wifi.scan() {
                    for found in seen.iter().filter(|found| found.ssid.as_str() == ssid) {
                        warn!(
                            "  {ssid} at {:02x?}: channel {}, {} dBm, {:?}",
                            found.bssid, found.channel, found.signal_strength, found.auth_method
                        );
                    }
                }
                Ok(false)
            }
        }
    }

    fn client_configuration(
        ssid: &str,
        password: &str,
        bssid: Option<[u8; 6]>,
        channel: Option<u8>,
    ) -> Result<ClientConfiguration> {
        Ok(ClientConfiguration {
            ssid: ssid.try_into().map_err(|_| anyhow!("SSID too long"))?,
            bssid,
            password: password
                .try_into()
                .map_err(|_| anyhow!("Wi-Fi password too long"))?,
            channel,
            // A threshold, not a demand: naming WPA2 here refuses a WPA3-only
            // or transition-mode access point before the passphrase is ever
            // tried, which looks identical to a wrong password. `None` accepts
            // whatever the access point actually offers.
            auth_method: AuthMethod::None,
            // A fast scan associates with the first access point answering to
            // the name, which on a mesh or repeater network is whichever node
            // replies quickest rather than the one that can hold a link. Scan
            // every channel and take the strongest.
            scan_method: ScanMethod::CompleteScan(ScanSortMethod::Signal),
            pmf_cfg: PmfConfiguration::Capable { required: false },
            ..Default::default()
        })
    }

    /// The access point, when the image was built with one.
    ///
    /// Optional on purpose: an unconfigured probe should not put an open or
    /// default-credentialed network on the air just by being powered on.
    fn access_point_configuration() -> Result<Option<AccessPointConfiguration>> {
        let Some((ssid, password)) = CONTROL_AP_SSID.zip(CONTROL_AP_PASSWORD) else {
            return Ok(None);
        };
        Ok(Some(AccessPointConfiguration {
            ssid: ssid.try_into().map_err(|_| anyhow!("AP SSID too long"))?,
            password: password
                .try_into()
                .map_err(|_| anyhow!("AP password too long"))?,
            auth_method: AuthMethod::WPA2Personal,
            max_connections: 4,
            ..Default::default()
        }))
    }

    /// The probe's own HTTP surface: liveness, and nothing else.
    ///
    /// The debug port is reached over the bridge, not over REST. A board built
    /// on this firmware registers its own routes alongside these.
    fn register_handlers(
        server: &mut EspHttpServer<'static>,
        wifi: Arc<Mutex<WifiControl>>,
    ) -> Result<()> {
        server.fn_handler("/health", Method::Get, move |req| {
            // Read per request, not captured at start-up. The address is not
            // known when the server is built, and it changes when the probe
            // moves between networks — a field baked in at boot answered
            // 0.0.0.0 for the life of the process, which is worse than absent
            // for anything discovering probes by polling this.
            // A poisoned lock means a thread died holding the radio state, so
            // this answers no rather than reporting a healthy probe with an
            // unknown address — an endpoint that cannot fail is not a health
            // check.
            let (ok, ssid, ip) = match wifi.lock() {
                Ok(state) => (
                    true,
                    state.ssid.clone(),
                    std::net::Ipv4Addr::from(state.ip).to_string(),
                ),
                Err(_) => (false, String::new(), String::from("0.0.0.0")),
            };
            // The SSID is whatever the network is called, which 802.11 leaves
            // wide open and the credential codec only checks for UTF-8.
            let body = format!(
                "{{\"ok\":{ok},\"service\":\"esprobe\",\"ip\":\"{ip}\",\"ssid\":\"{}\"}}\n",
                esprobe_protocol::json::Escaped(&ssid)
            );
            let mut response = match ok {
                true => req.into_ok_response()?,
                false => req.into_status_response(503)?,
            };
            response.write_all(body.as_bytes())?;
            Ok::<(), anyhow::Error>(())
        })?;

        Ok(())
    }
}

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    app::run()
}
