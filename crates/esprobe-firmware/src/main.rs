#![cfg_attr(not(target_os = "espidf"), allow(dead_code))]

#[cfg(not(target_os = "espidf"))]
fn main() {
    panic!("build this firmware for riscv32imc-esp-espidf");
}

#[cfg(target_os = "espidf")]
mod hardware;

#[cfg(target_os = "espidf")]
mod spi_wire;

#[cfg(target_os = "espidf")]
mod app {
    use std::net::Ipv4Addr;
    use std::sync::{Arc, Mutex};
    use std::thread;
    use std::time::Duration;

    use anyhow::{Context, Result, anyhow};
    use embedded_svc::http::{Headers as _, Method};
    use embedded_svc::io::{Read as _, Write as _};
    use embedded_svc::wifi::{
        AccessPointConfiguration, AuthMethod, ClientConfiguration, Configuration, PmfConfiguration,
        ScanMethod, ScanSortMethod,
    };
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::delay::Ets;
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
        UART_PIN_NO_CHANGE, gpio_get_level, gpio_mode_t_GPIO_MODE_INPUT,
        gpio_mode_t_GPIO_MODE_OUTPUT, gpio_reset_pin, gpio_set_direction, uart_port_t_UART_NUM_1,
        uart_set_pin,
    };
    use esp_idf_svc::wifi::{BlockingWifi, EspWifi, WifiEvent};
    use log::{error, info, warn};

    use esprobe_firmware::safety::ReleasedSwdState;
    use esprobe_firmware::stm32g0::{Error as StmError, FLASH_BYTES, Stm32G030};
    use esprobe_firmware::swd::{Error as SwdError, SwdLink, WordMemory};
    use esprobe_firmware::usb_bridge::{
        Command as BridgeCommand, MAX_BLOCK_WORDS, MAX_FRAME as BRIDGE_MAX_FRAME,
        Status as BridgeStatus, decode_request, encode_response,
    };
    use esprobe_firmware::{ProgrammingTarget, pinmap, wifi_credentials};

    use crate::hardware::{Engine, EspSwdIo, cpu_cycles};

    // Build-time credentials are a convenience for a bench that always joins
    // the same network, not a requirement: an image with none still builds,
    // still bridges over USB, and can be given a network at runtime.
    const WIFI_SSID: Option<&str> = option_env!("WIFI_SSID");
    const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");
    const WIFI_SSID_FALLBACK: Option<&str> = option_env!("WIFI_SSID_FALLBACK");
    const WIFI_PASSWORD_FALLBACK: Option<&str> = option_env!("WIFI_PASSWORD_FALLBACK");
    const CONTROL_AP_SSID: Option<&str> = option_env!("CONTROL_AP_SSID");
    const CONTROL_AP_PASSWORD: Option<&str> = option_env!("CONTROL_AP_PASSWORD");
    /// Where runtime credentials live, so a network change costs a command
    /// rather than a rebuild and a reflash.
    const NVS_NAMESPACE: &str = "esprobe";
    const NVS_SSID: &str = "sta_ssid";
    const NVS_PASSWORD: &str = "sta_pass";
    const MAX_DISPLAY_FRAME: usize = 1024;
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
        asw_s0: PinDriver<'static, InputOutput>,
        asw_s1: PinDriver<'static, InputOutput>,
        reset_all: PinDriver<'static, InputOutput>,
        display: UartDriver<'static>,
        swd: Option<SwdLink<EspSwdIo<'static>>>,
        selected: ProgrammingTarget,
        wifi: Arc<Mutex<WifiControl>>,
        bridge_attached: bool,
        /// Whether *this* firmware is holding the shared reset line down.
        ///
        /// Tracked as intent rather than sampled from the pad: on the STM32G0
        /// NRST is bidirectional, so the target pulls it low during its own
        /// resets — including the ones probe-rs performs while flashing — and a
        /// guard that reads the pin cannot tell that apart from a command that
        /// left reset asserted.
        reset_held: bool,
    }

    impl Hub {
        /// Drives or releases the shared reset line, recording which it is.
        fn hold_reset(&mut self, held: bool) -> Result<()> {
            if held {
                self.reset_all.set_low()?;
            }
            Self::set_gpio_direction(
                pinmap::RESET_ALL,
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

        fn reclaim_gpio(pin: i32) -> Result<()> {
            // SAFETY: these are valid ESP32-C3 GPIO numbers owned by Hub.
            // gpio_reset_pin selects the GPIO IO-mux function; changing only
            // direction leaves reset-default JTAG functions active.
            esp_idf_svc::sys::esp!(unsafe { gpio_reset_pin(pin) })?;
            Ok(())
        }

        fn select_stm32_passively(&mut self) -> Result<()> {
            if let Some(swd) = self.swd.as_mut() {
                swd.disconnect();
            }
            // The only selector state that never drives a high level.
            self.asw_s0.set_low()?;
            self.asw_s1.set_low()?;
            thread::sleep(Duration::from_millis(5));
            self.selected = ProgrammingTarget::Stm32;
            self.bridge_attached = false;
            Ok(())
        }

        fn require_target_power(&mut self) -> Result<()> {
            self.select_stm32_passively()?;
            let (mut swdio, mut swclk) = self
                .swd
                .as_mut()
                .context("SWD engine unavailable")?
                .released_line_state();
            if !swdio && self.reset_all.is_high() {
                self.pulse_reset_all_low_release()?;
                (swdio, swclk) = self
                    .swd
                    .as_mut()
                    .context("SWD engine unavailable")?
                    .released_line_state();
            }
            anyhow::ensure!(
                ReleasedSwdState::from_levels(swdio, swclk).permits_high_drive(),
                "target power not proven: released STM32 SWDIO is low (SWCLK={swclk})"
            );
            Ok(())
        }

        fn select(&mut self, target: ProgrammingTarget) -> Result<()> {
            self.require_target_power()?;
            let (s0, s1) = target.selector();
            if s0 {
                self.asw_s0.set_high()?;
            } else {
                self.asw_s0.set_low()?;
            }
            if s1 {
                self.asw_s1.set_high()?;
            } else {
                self.asw_s1.set_low()?;
            }
            thread::sleep(Duration::from_millis(5));
            self.selected = target;
            self.bridge_attached = false;
            Ok(())
        }

        fn pulse_reset_all_low_release(&mut self) -> Result<()> {
            self.hold_reset(true)?;
            thread::sleep(Duration::from_millis(10));
            self.hold_reset(false)?;
            thread::sleep(Duration::from_millis(10));
            Ok(())
        }

        fn pulse_reset_all(&mut self) -> Result<()> {
            anyhow::ensure!(
                self.reset_all.is_high(),
                "IR-board reset pull-up is not visible"
            );
            self.pulse_reset_all_low_release()?;
            Ok(())
        }

        fn probe_stm32(&mut self) -> Result<(u32, u32)> {
            self.select(ProgrammingTarget::Stm32)?;
            let swd = self.swd.as_mut().context("SWD engine unavailable")?;
            let result = (|| {
                let dp_id = swd.initialize().map_err(|error| anyhow!("{error:?}"))?;
                let device_id = Stm32G030::new(swd)
                    .identify()
                    .map_err(|error| anyhow!("{error:?}"))?;
                Ok((dp_id, device_id))
            })();
            swd.disconnect();
            result
        }

        fn flash_stm32(&mut self, image: &[u8]) -> Result<(u32, u32)> {
            self.select(ProgrammingTarget::Stm32)?;
            let swd = self.swd.as_mut().context("SWD engine unavailable")?;
            let result = (|| {
                let dp_id = swd.initialize().map_err(|error| anyhow!("{error:?}"))?;
                let mut target = Stm32G030::new(swd);
                let device_id = target
                    .program_and_verify(image)
                    .map_err(|error| anyhow!("{error:?}"))?;
                target.reset().map_err(|error| anyhow!("{error:?}"))?;
                Ok((dp_id, device_id))
            })();
            swd.disconnect();
            result
        }

        fn write_display(&mut self, frame: &[u8]) -> Result<usize> {
            self.require_target_power()?;
            Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_OUTPUT)?;
            let result = (|| {
                let written = self.display.write(frame)?;
                self.display.wait_tx_done(100)?;
                Ok(written)
            })();
            Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_INPUT)?;
            result
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
                    let (mut swdio, _) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    if !swdio && self.reset_all.is_high() {
                        self.pulse_reset_all_low_release()
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
                    if self.swd.is_none() {
                        return Err(SwdError::Protocol(0));
                    }
                    self.reset_all
                        .set_low()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
                    if self.swd.is_none() {
                        return Err(SwdError::Protocol(0));
                    }
                    self.reset_all
                        .set_low()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                        self.reset_all
                            .set_low()
                            .map_err(|_| SwdError::Protocol(0))?;
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
                        let dp_id = swd.initialize_prepared()?;
                        let mut target = Stm32G030::new(swd);
                        target.halt().map_err(|error| match error {
                            StmError::Transport(error) => error,
                            _ => SwdError::Protocol(0),
                        })?;
                        let device_id = target.identify().map_err(|error| match error {
                            StmError::Transport(error) => error,
                            _ => SwdError::Protocol(0),
                        })?;
                        target.reset().map_err(|error| match error {
                            StmError::Transport(error) => error,
                            _ => SwdError::Protocol(0),
                        })?;
                        Ok((dp_id, device_id))
                    })();
                    let _ = self.hold_reset(false);
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    self.bridge_attached = false;
                    let (dp_id, device_id) = outcome?;
                    response[..4].copy_from_slice(&dp_id.to_le_bytes());
                    response[4..8].copy_from_slice(&device_id.to_le_bytes());
                    Ok(8)
                }
                BridgeCommand::DiagnosticSwdio => {
                    let [level] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.reset_all
                        .set_low()
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    let observed = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .diagnostic_drive_swdio(*level != 0);
                    response[0] = u8::from(observed);
                    Ok(1)
                }
                BridgeCommand::EnterRomBoot => {
                    // Existing STM32 Rust firmware CfgSet:
                    // ns=FLASH(3), sub=OTA(0), key=ENTER_ROM_BOOT(5).
                    // Frame is COBS with CRC16/CCITT-FALSE and delimiter.
                    const ENTER_ROM_BOOT_FRAME: [u8; 10] = [5, 1, 0x41, 3, 3, 4, 5, 0xa0, 0x62, 0];
                    if !payload.is_empty() {
                        return Err(SwdError::Protocol(0));
                    }
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    self.display
                        .change_parity(Parity::ParityNone)
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.display.clear_rx().map_err(|_| SwdError::Protocol(0))?;
                    Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_OUTPUT)
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.display
                        .write(&ENTER_ROM_BOOT_FRAME)
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.display
                        .wait_tx_done(100)
                        .map_err(|_| SwdError::Protocol(0))?;
                    Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_INPUT)
                        .map_err(|_| SwdError::Protocol(0))?;
                    let mut length = self.display.read(response, 100).unwrap_or(0);
                    if length == 0 {
                        // Older application firmware may not implement the
                        // software ROM jump. Try the PA14/BOOT0 reset entry;
                        // option bytes decide whether the pin is honored.
                        self.reset_all
                            .set_low()
                            .map_err(|_| SwdError::Protocol(0))?;
                        self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                        self.swd
                            .as_mut()
                            .ok_or(SwdError::Protocol(0))?
                            .diagnostic_drive_boot0(true);
                        thread::sleep(Duration::from_millis(10));
                        self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                        thread::sleep(Duration::from_millis(10));
                        self.swd.as_mut().ok_or(SwdError::Protocol(0))?.disconnect();
                    }
                    thread::sleep(Duration::from_millis(100));
                    // STM32 system-memory USART bootloader uses 8E1. The
                    // application command above is 8N1, so parity must change
                    // only after the application had a chance to jump.
                    self.display
                        .change_parity(Parity::ParityEven)
                        .map_err(|_| SwdError::Protocol(0))?;
                    let rom_sync = (|| {
                        self.display.clear_rx().map_err(|_| SwdError::Protocol(0))?;
                        Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_OUTPUT)
                            .map_err(|_| SwdError::Protocol(0))?;
                        self.display
                            .write(&[0x7f])
                            .map_err(|_| SwdError::Protocol(0))?;
                        self.display
                            .wait_tx_done(100)
                            .map_err(|_| SwdError::Protocol(0))?;
                        Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_INPUT)
                            .map_err(|_| SwdError::Protocol(0))?;
                        Ok(self.display.read(&mut response[length..], 100).unwrap_or(0))
                    })();
                    // Restore the shared display UART even if ROM sync failed.
                    let _ = Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_INPUT);
                    let parity_restore = self
                        .display
                        .change_parity(Parity::ParityNone)
                        .map_err(|_| SwdError::Protocol(0));
                    length += rom_sync?;
                    parity_restore?;
                    Ok(length)
                }
                BridgeCommand::UartReceive => {
                    if !payload.is_empty() {
                        return Err(SwdError::Protocol(0));
                    }
                    self.display
                        .change_parity(Parity::ParityNone)
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.display.clear_rx().map_err(|_| SwdError::Protocol(0))?;
                    Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_INPUT)
                        .map_err(|_| SwdError::Protocol(0))?;
                    // Ten bytes are reserved by the bridge envelope.
                    let capacity = response.len().min(BRIDGE_MAX_FRAME - 10);
                    Ok(self
                        .display
                        .read(&mut response[..capacity], 100)
                        .unwrap_or(0))
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
                                pinmap::DISP_TX
                            } else {
                                pinmap::DISP_RX
                            },
                            UART_PIN_NO_CHANGE,
                            UART_PIN_NO_CHANGE,
                        )
                    })
                    .map_err(|_| SwdError::Protocol(0))?;
                    let capture = (|| {
                        self.display
                            .change_parity(Parity::ParityNone)
                            .map_err(|_| SwdError::Protocol(0))?;
                        self.display.clear_rx().map_err(|_| SwdError::Protocol(0))?;
                        Self::set_gpio_direction(pinmap::DISP_TX, gpio_mode_t_GPIO_MODE_INPUT)
                            .map_err(|_| SwdError::Protocol(0))?;
                        self.pulse_reset_all_low_release()
                            .map_err(|_| SwdError::Protocol(0))?;
                        let capacity = response.len().min(BRIDGE_MAX_FRAME - 10);
                        Ok(self
                            .display
                            .read(&mut response[..capacity], 200)
                            .unwrap_or(0))
                    })();
                    let restore = esp_idf_svc::sys::esp!(unsafe {
                        uart_set_pin(
                            uart_port_t_UART_NUM_1,
                            UART_PIN_NO_CHANGE,
                            pinmap::DISP_RX,
                            UART_PIN_NO_CHANGE,
                            UART_PIN_NO_CHANGE,
                        )
                    })
                    .map_err(|_| SwdError::Protocol(0));
                    let length = capture?;
                    restore?;
                    Ok(length)
                }
                BridgeCommand::MuxProbe => {
                    let [target] = payload else {
                        return Err(SwdError::Protocol(0));
                    };
                    let target = match target {
                        0 => ProgrammingTarget::Stm32,
                        1 => ProgrammingTarget::Gps,
                        2 => ProgrammingTarget::Dwm0,
                        3 => ProgrammingTarget::Dwm1,
                        _ => return Err(SwdError::Protocol(0)),
                    };
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    let operation = (|| {
                        let (s0, s1) = target.selector();
                        if s0 {
                            self.asw_s0.set_high().map_err(|_| SwdError::Protocol(0))?;
                        } else {
                            self.asw_s0.set_low().map_err(|_| SwdError::Protocol(0))?;
                        }
                        if s1 {
                            self.asw_s1.set_high().map_err(|_| SwdError::Protocol(0))?;
                        } else {
                            self.asw_s1.set_low().map_err(|_| SwdError::Protocol(0))?;
                        }
                        thread::sleep(Duration::from_millis(5));
                        // SAFETY: GPIO1/GPIO2 are owned input/output pins;
                        // reading the pad verifies voltage, not just latch state.
                        response[7] = u8::from(unsafe { gpio_get_level(pinmap::ASW_S0) != 0 });
                        response[8] = u8::from(unsafe { gpio_get_level(pinmap::ASW_S1) != 0 });
                        let swd = self.swd.as_mut().ok_or(SwdError::Protocol(0))?;
                        let (swdio, swclk) = swd.released_line_state();
                        response[0] = u8::from(swdio);
                        response[1] = u8::from(swclk);
                        Ok(if target == ProgrammingTarget::Gps {
                            None
                        } else if swdio {
                            Some(swd.initialize())
                        } else {
                            Some(Err(SwdError::LineHeldLow))
                        })
                    })();
                    // Restore the passive STM32 mux state even if selector
                    // drive/readback or target probing failed.
                    if let Some(swd) = self.swd.as_mut() {
                        swd.disconnect();
                    }
                    let cleanup_s0 = self.asw_s0.set_low().map_err(|_| SwdError::Protocol(0));
                    let cleanup_s1 = self.asw_s1.set_low().map_err(|_| SwdError::Protocol(0));
                    self.selected = ProgrammingTarget::Stm32;
                    self.bridge_attached = false;
                    let result = operation?;
                    cleanup_s0?;
                    cleanup_s1?;
                    match result {
                        None => {
                            response[2] = 9;
                            response[3..7].fill(0);
                        }
                        Some(Ok(dp_id)) => {
                            response[2] = 1;
                            response[3..7].copy_from_slice(&dp_id.to_le_bytes());
                        }
                        Some(Err(error)) => {
                            response[2] = match error {
                                SwdError::Protocol(ack) => 0x80 | ack,
                                SwdError::LineHeldLow => 2,
                                SwdError::InvalidDpId => 3,
                                SwdError::PowerTimeout => 4,
                                SwdError::WaitTimeout => 5,
                                SwdError::Fault => 6,
                                SwdError::Parity => 7,
                                SwdError::Unaligned => 8,
                            };
                            response[3..7].fill(0);
                        }
                    }
                    Ok(9)
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                    let mut wifi = self.wifi.lock().map_err(|_| SwdError::Protocol(0))?;
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
                    for (slot, pin) in response[..7].iter_mut().zip([
                        pinmap::PROG_SWDIO,
                        pinmap::PROG_SWCLK,
                        pinmap::RESET_ALL,
                        pinmap::ASW_S0,
                        pinmap::ASW_S1,
                        pinmap::DISP_TX,
                        pinmap::DISP_RX,
                    ]) {
                        *slot = pin as u8;
                    }
                    Ok(7)
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
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
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
                    let (swdio, swclk) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    response[..3].copy_from_slice(&[
                        u8::from(swdio),
                        u8::from(swclk),
                        u8::from(self.reset_all.is_high()),
                    ]);
                    self.bridge_attached = false;
                    Ok(3)
                }
                BridgeCommand::ResetLineState => {
                    self.select_stm32_passively()
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.reset_all
                        .set_low()
                        .map_err(|_| SwdError::Protocol(0))?;
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    let (asserted_swdio, asserted_swclk) = self
                        .swd
                        .as_mut()
                        .ok_or(SwdError::Protocol(0))?
                        .released_line_state();
                    let asserted_reset = self.reset_all.is_high();
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
                        u8::from(self.reset_all.is_high()),
                    ]);
                    self.bridge_attached = false;
                    Ok(6)
                }
                // Reset drives the shared RESET_ALL net, which reaches every
                // target regardless of where the mux points, so neither of
                // these touches the mux or the SWD link. They used to, and
                // that made them unusable for connect-under-reset: a debug
                // sequence asserts reset *after* attaching, and tearing the
                // link down underneath it fails the very next transfer.
                BridgeCommand::ResetAssert => {
                    self.hold_reset(true).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    response[0] = u8::from(self.reset_all.is_high());
                    Ok(1)
                }
                BridgeCommand::ResetRelease => {
                    self.hold_reset(false).map_err(|_| SwdError::Protocol(0))?;
                    thread::sleep(Duration::from_millis(10));
                    response[0] = u8::from(self.reset_all.is_high());
                    Ok(1)
                }
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
        let mut store = credential_store()?;
        store.set_str(NVS_SSID, ssid)?;
        store.set_str(NVS_PASSWORD, password)?;
        Ok(())
    }

    fn forget_wifi_credentials() -> Result<()> {
        let mut store = credential_store()?;
        // Stored empty rather than removed. An absent key means "never
        // provisioned", which falls back to whatever the image was built with;
        // an empty one means the host said forget, and has to outrank that.
        // Removing the key made `forget` report success and change nothing.
        store.set_str(NVS_SSID, "")?;
        let _ = store.remove(NVS_PASSWORD);
        Ok(())
    }

    /// Credentials from storage, falling back to any compiled into the image.
    ///
    /// Storage always wins, including when it holds an empty SSID: that is the
    /// host having said forget, and a build-time credential must not undo it.
    /// Only a device that has never been provisioned uses the image's own.
    fn load_wifi_credentials() -> Option<(String, String)> {
        let mut store = credential_store().ok()?;
        let mut ssid = [0u8; wifi_credentials::MAX_SSID + 1];
        let mut password = [0u8; wifi_credentials::MAX_PASSWORD + 1];
        let stored = store
            .get_str(NVS_SSID, &mut ssid)
            .ok()
            .flatten()
            .map(|value| value.to_string());
        if let Some(ssid) = stored {
            if ssid.is_empty() {
                return None;
            }
            let password = store
                .get_str(NVS_PASSWORD, &mut password)
                .ok()
                .flatten()
                .unwrap_or("")
                .to_string();
            return Some((ssid, password));
        }
        WIFI_SSID
            .zip(WIFI_PASSWORD)
            .map(|(ssid, password)| (ssid.to_string(), password.to_string()))
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
            "Pin map: SWDIO=GPIO{} SWCLK=GPIO{} RESET_ALL=GPIO{} ASW_S0=GPIO{} \
             ASW_S1=GPIO{} DISP_TX=GPIO{} DISP_RX=GPIO{}",
            pinmap::PROG_SWDIO,
            pinmap::PROG_SWCLK,
            pinmap::RESET_ALL,
            pinmap::ASW_S0,
            pinmap::ASW_S1,
            pinmap::DISP_TX,
            pinmap::DISP_RX
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
        let mut asw_s0 = PinDriver::input_output(take(pinmap::ASW_S0)?, Pull::Floating)?;
        let mut asw_s1 = PinDriver::input_output(take(pinmap::ASW_S1)?, Pull::Floating)?;
        asw_s0.set_low()?;
        asw_s1.set_low()?;

        let mut reset_all = PinDriver::input_output(take(pinmap::RESET_ALL)?, Pull::Floating)?;
        reset_all.set_low()?;
        Hub::set_gpio_direction(7, gpio_mode_t_GPIO_MODE_INPUT)?;

        let display = UartDriver::new(
            peripherals.uart1,
            take(pinmap::DISP_TX)?,
            take(pinmap::DISP_RX)?,
            Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
            Option::<esp_idf_svc::hal::gpio::AnyIOPin>::None,
            &UartConfig::new().baudrate(Hertz(115_200)),
        )?;
        Hub::set_gpio_direction(5, gpio_mode_t_GPIO_MODE_INPUT)?;
        let swd = EspSwdIo::new(
            take(pinmap::PROG_SWDIO)?,
            take(pinmap::PROG_SWCLK)?,
            SWD_PINS_SWAPPED,
        )?;

        let wifi_control = Arc::new(Mutex::new(WifiControl::default()));
        let hub = Arc::new(Mutex::new(Hub {
            asw_s0,
            asw_s1,
            reset_all,
            display,
            swd: Some(SwdLink::new(swd)),
            selected: ProgrammingTarget::Stm32,
            wifi: wifi_control.clone(),
            bridge_attached: false,
            reset_held: false,
        }));
        spawn_usb_bridge(hub.clone())?;

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

        let mut server = EspHttpServer::new(&Default::default())?;
        register_handlers(&mut server, hub, Ipv4Addr::UNSPECIFIED)?;

        loop {
            thread::sleep(Duration::from_secs(2));

            // Anything the host asked for since the last pass.
            let (pending, forget) = match wifi_control.lock() {
                Ok(mut control) => (control.pending.take(), std::mem::take(&mut control.forget)),
                Err(_) => (None, false),
            };
            if forget {
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
                match join_wifi(&mut wifi, &ssid, &password) {
                    Ok(true) => info!("Joined {ssid}"),
                    Ok(false) => warn!("Could not join {ssid}"),
                    Err(error) => warn!("Joining {ssid} failed: {error}"),
                }
            } else if !wifi.is_connected().unwrap_or(false)
                && let Some((ssid, password)) = load_wifi_credentials()
            {
                let _ = join_wifi(&mut wifi, &ssid, &password);
            }

            // Publish what the host will see through `wifi status`.
            if let Ok(mut control) = wifi_control.lock() {
                // `is_connected` is also true for a running access point,
                // which reported a connection with no network and no address.
                // An address on the station interface is the thing the host
                // actually cares about.
                control.connected = wifi.is_connected().unwrap_or(false)
                    && wifi
                        .wifi()
                        .sta_netif()
                        .get_ip_info()
                        .is_ok_and(|info| !info.ip.is_unspecified());
                control.ip = wifi
                    .wifi()
                    .sta_netif()
                    .get_ip_info()
                    .map(|info| info.ip.octets())
                    .unwrap_or([0; 4]);
                control.ssid = load_wifi_credentials()
                    .map(|(ssid, _)| ssid)
                    .unwrap_or_default();
            }
        }
    }

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
    struct UsbSerialLink;

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

    /// Brings the radio up with whatever configuration is available.
    ///
    /// A station with no credentials is still worth starting: the access point
    /// gives a way in, and the station can be given a network later without a
    /// restart.
    fn start_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
        let credentials = load_wifi_credentials();
        apply_configuration(wifi, credentials.as_ref())?;
        wifi.start()?;
        Ok(())
    }

    /// Programs the radio for one set of station credentials, or for none.
    ///
    /// The access point comes up only when there is no network to join. The two
    /// cannot be had at once in any useful sense: one radio means the soft AP
    /// drags the station onto its channel, and the log shows exactly that —
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

    /// Associates with one network, leaving the access point as it was.
    fn join_wifi(
        wifi: &mut BlockingWifi<EspWifi<'static>>,
        ssid: &str,
        password: &str,
    ) -> Result<bool> {
        let _ = wifi.disconnect();
        apply_configuration(wifi, Some(&(ssid.to_string(), password.to_string())))?;
        if !wifi.is_started().unwrap_or(false) {
            wifi.start()?;
        }
        match wifi.connect().and_then(|()| wifi.wait_netif_up()) {
            Ok(()) => Ok(true),
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

    fn register_handlers(
        server: &mut EspHttpServer<'static>,
        hub: Arc<Mutex<Hub>>,
        ip: std::net::Ipv4Addr,
    ) -> Result<()> {
        server.fn_handler("/health", Method::Get, move |req| {
            let body = format!("{{\"ok\":true,\"service\":\"esprobe\",\"ip\":\"{ip}\"}}\n");
            req.into_ok_response()?.write_all(body.as_bytes())?;
            Ok::<(), anyhow::Error>(())
        })?;

        let state = hub.clone();
        server.fn_handler("/api/v1/stm32/probe", Method::Post, move |req| {
            match state
                .lock()
                .map_err(|_| anyhow!("hub lock poisoned"))?
                .probe_stm32()
            {
                Ok((dp, device)) => {
                    let body = format!(
                        "{{\"ok\":true,\"dp_id\":\"0x{dp:08x}\",\"device_id\":\"0x{device:08x}\"}}\n"
                    );
                    req.into_ok_response()?.write_all(body.as_bytes())?;
                }
                Err(error) => {
                    req.into_status_response(502)?
                        .write_all(format!("{{\"ok\":false,\"error\":\"{error}\"}}\n").as_bytes())?;
                }
            }
            Ok::<(), anyhow::Error>(())
        })?;

        let state = hub.clone();
        server.fn_handler("/api/v1/stm32/flash", Method::Post, move |mut req| {
            let length = req.content_len().unwrap_or(0) as usize;
            if length == 0 || length > FLASH_BYTES {
                req.into_status_response(413)?.write_all(
                    format!("binary image must be 1..={FLASH_BYTES} bytes\n").as_bytes(),
                )?;
                return Ok::<(), anyhow::Error>(());
            }
            let mut image = vec![0; length];
            req.read_exact(&mut image)?;
            match state
                .lock()
                .map_err(|_| anyhow!("hub lock poisoned"))?
                .flash_stm32(&image)
            {
                Ok((dp, device)) => {
                    let body = format!(
                        "{{\"ok\":true,\"bytes\":{length},\"verified\":true,\"dp_id\":\"0x{dp:08x}\",\"device_id\":\"0x{device:08x}\"}}\n"
                    );
                    req.into_ok_response()?.write_all(body.as_bytes())?;
                }
                Err(error) => {
                    req.into_status_response(502)?
                        .write_all(format!("{{\"ok\":false,\"error\":\"{error}\"}}\n").as_bytes())?;
                }
            }
            Ok::<(), anyhow::Error>(())
        })?;

        let state = hub.clone();
        server.fn_handler("/api/v1/display/tx", Method::Post, move |mut req| {
            let length = req.content_len().unwrap_or(0) as usize;
            if length == 0 || length > MAX_DISPLAY_FRAME {
                req.into_status_response(413)?
                    .write_all(b"display frame must be 1..=1024 bytes\n")?;
                return Ok::<(), anyhow::Error>(());
            }
            let mut frame = vec![0; length];
            req.read_exact(&mut frame)?;
            let written = state
                .lock()
                .map_err(|_| anyhow!("hub lock poisoned"))?
                .write_display(&frame)?;
            req.into_ok_response()?
                .write_all(format!("{{\"ok\":true,\"bytes\":{written}}}\n").as_bytes())?;
            Ok::<(), anyhow::Error>(())
        })?;

        let state = hub.clone();
        server.fn_handler("/api/v1/display/rx", Method::Get, move |req| {
            let mut frame = [0u8; MAX_DISPLAY_FRAME];
            let read = state
                .lock()
                .map_err(|_| anyhow!("hub lock poisoned"))?
                .display
                .read(&mut frame, 0)?;
            req.into_ok_response()?.write_all(&frame[..read])?;
            Ok::<(), anyhow::Error>(())
        })?;

        let state = hub.clone();
        server.fn_handler("/api/v1/dwm0/reset", Method::Post, move |req| {
            state
                .lock()
                .map_err(|_| anyhow!("hub lock poisoned"))?
                .pulse_reset_all()?;
            req.into_ok_response()?.write_all(b"{\"ok\":true}\n")?;
            Ok::<(), anyhow::Error>(())
        })?;

        for (path, target) in [
            ("/api/v1/mux/stm32", ProgrammingTarget::Stm32),
            ("/api/v1/mux/gps", ProgrammingTarget::Gps),
            ("/api/v1/mux/dwm0", ProgrammingTarget::Dwm0),
            ("/api/v1/mux/dwm1", ProgrammingTarget::Dwm1),
        ] {
            let state = hub.clone();
            server.fn_handler(path, Method::Post, move |req| {
                state
                    .lock()
                    .map_err(|_| anyhow!("hub lock poisoned"))?
                    .select(target)?;
                req.into_ok_response()?.write_all(b"{\"ok\":true}\n")?;
                Ok::<(), anyhow::Error>(())
            })?;
        }

        Ok(())
    }
}

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    app::run()
}
