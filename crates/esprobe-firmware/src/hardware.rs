//! ESP32-C3 pad plumbing for the SWD wire.
//!
//! Two engines share the same two pads. GPSPI2 shifts every field that belongs
//! to a transfer, because hardware-generated SWCLK is both far faster and
//! immune to scheduler jitter. The dedicated-GPIO path stays for the pad-level
//! diagnostics — continuity checks, BOOT0 forcing, released-line sampling —
//! where the point is to hold a level rather than clock a field.
//!
//! Both engines drive the pad through the GPIO matrix, so switching between
//! them is a change of output signal. Output *enable* is deliberately taken
//! away from both peripherals (`OEN_SEL = 1`) and driven from `GPIO_ENABLE`,
//! so exactly one place decides whether this board drives another board's pin.

use esp_idf_svc::hal::delay::Ets;
use esp_idf_svc::hal::gpio::{AnyIOPin, DriveStrength, InputOutput, Pin, PinDriver, Pull};
use esp_idf_svc::sys::{
    CPU_GPIO_IN0_IDX, CPU_GPIO_IN1_IDX, CPU_GPIO_OUT0_IDX, CPU_GPIO_OUT1_IDX, FSPICLK_OUT_IDX,
    FSPID_OUT_IDX, FSPIQ_IN_IDX, esp_rom_gpio_connect_in_signal,
};

use esprobe_firmware::spi_clock::DEFAULT_CLOCK_HZ;
use esprobe_firmware::swd::SwdIo;

use crate::spi_wire::{MAX_BURST_BITS, SpiWire};

const GPIO_BASE: u32 = 0x6000_4000;
const GPIO_ENABLE_W1TS: u32 = 0x24;
const GPIO_ENABLE_W1TC: u32 = 0x28;
const GPIO_FUNC0_OUT_SEL_CFG: u32 = 0x554;
const GPIO_FUNC_OEN_SEL: u32 = 1 << 9;

/// Dedicated-GPIO channel 0 carries SWDIO and channel 1 carries SWCLK,
/// whichever pads they are routed to.
const SWDIO_CHANNEL: u32 = 0b01;
const SWCLK_CHANNEL: u32 = 0b10;

const CPU_FREQ_HZ: u32 = esp_idf_svc::sys::CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1_000_000;
/// Fallback wire clock when GPSPI2 is not driving the pads.
const BIT_BANG_CLOCK_HZ: u32 = 500_000;
/// Past this the software loop's own overhead, not the delay, sets the period,
/// so the request is honoured as "as fast as the loop goes".
const BIT_BANG_MAX_CLOCK_HZ: u32 = 20_000_000;
/// Roughly what one `delay_cycles` call costs before it delays anything. Below
/// this a requested half-period is already covered by the loop itself, and
/// polling the counter would only make the clock slower than asked for.
const MIN_DELAY_CYCLES: u32 = 24;

fn gpio_write(offset: u32, value: u32) {
    // SAFETY: `offset` names a GPIO matrix register; the bits written below
    // only ever belong to the two pads this type owns.
    unsafe { core::ptr::write_volatile((GPIO_BASE + offset) as *mut u32, value) };
}

/// Points a pad at a peripheral output signal while keeping its output enable
/// under `GPIO_ENABLE` control.
///
/// Written directly rather than through `esp_rom_gpio_connect_out_signal`,
/// which clears `OEN_SEL` and asserts `GPIO_ENABLE` as a side effect — it would
/// briefly drive the target's pin on every engine changeover.
fn route_output(pin: u32, signal: u32) {
    // SAFETY: `pin` is one of the two pads this type owns.
    unsafe {
        core::ptr::write_volatile(
            (GPIO_BASE + GPIO_FUNC0_OUT_SEL_CFG + pin * 4) as *mut u32,
            signal | GPIO_FUNC_OEN_SEL,
        )
    };
}

/// Enables the CPU's cycle counter.
///
/// The ESP32-C3 core has no standard `mcycle`; it exposes Espressif's own
/// performance counter instead (`SOC_CPU_HAS_CSR_PC`), and reading `mcycle`
/// here is an illegal instruction that takes the whole firmware down.
fn enable_cycle_counter() {
    // SAFETY: 0x7e0 selects the counted event and 0x7e1 runs the counter.
    unsafe {
        core::arch::asm!(
            "csrw 0x7e0, {cycles}",
            "csrw 0x7e1, {enable}",
            cycles = in(reg) 1u32,
            enable = in(reg) 1u32,
        )
    };
}

/// Reads the CPU cycle counter for sub-microsecond bit-bang timing.
pub fn cpu_cycles() -> u32 {
    let value: u32;
    // SAFETY: 0x7e2 is the machine-mode performance counter enabled above.
    unsafe { core::arch::asm!("csrr {value}, 0x7e2", value = out(reg) value) };
    value
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum Driver {
    DedicatedGpio,
    Spi,
}

/// Which engine clocks the wire.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Engine {
    /// GPSPI2 shifts each field in hardware. Fastest, and immune to jitter.
    Hardware,
    /// The CPU drives every edge, timed off the cycle counter. Slower, but it
    /// samples where the original bring-up firmware sampled, so it is the
    /// reference the hardware engine is measured against.
    BitBang,
}

pub struct EspSwdIo<'d> {
    _swdio: PinDriver<'d, InputOutput>,
    _swclk: PinDriver<'d, InputOutput>,
    schematic_swdio_pin: u32,
    schematic_swclk_pin: u32,
    swdio_pin: u32,
    swclk_pin: u32,
    spi: SpiWire,
    driver: Driver,
    engine: Engine,
    half_cycle_cycles: u32,
}

impl<'d> EspSwdIo<'d> {
    /// Takes the SWDIO and SWCLK pads and the GPSPI2 shift register.
    ///
    /// `swapped` exchanges which pad carries which signal without moving the
    /// logical channel numbering, so a reversed bench harness costs a command
    /// rather than a rebuild.
    pub fn new(swdio: AnyIOPin<'d>, swclk: AnyIOPin<'d>, swapped: bool) -> anyhow::Result<Self> {
        let swdio_number = swdio.pin() as u32;
        let swclk_number = swclk.pin() as u32;
        let mut swdio = PinDriver::input_output(swdio, Pull::Floating)?;
        let mut swclk = PinDriver::input_output(swclk, Pull::Floating)?;
        swdio.set_drive_strength(DriveStrength::I5mA)?;
        swclk.set_drive_strength(DriveStrength::I5mA)?;

        enable_cycle_counter();
        let mut this = Self {
            _swdio: swdio,
            _swclk: swclk,
            schematic_swdio_pin: swdio_number,
            schematic_swclk_pin: swclk_number,
            swdio_pin: swdio_number,
            swclk_pin: swclk_number,
            spi: SpiWire::new(DEFAULT_CLOCK_HZ),
            driver: Driver::DedicatedGpio,
            engine: Engine::Hardware,
            half_cycle_cycles: CPU_FREQ_HZ / (2 * BIT_BANG_CLOCK_HZ),
        };
        this.set_pin_map(swapped);
        Ok(this)
    }

    /// Rebinds the logical signals to the two pads. Both lines are released
    /// first, so a mid-session remap can never step on a live transfer.
    pub fn set_pin_map(&mut self, swapped: bool) {
        self.set_output_enable(0);
        // Derived from the schematic assignment, never from the current
        // mapping: repeating the same request must be idempotent, not a toggle.
        let (swdio_pin, swclk_pin) = if swapped {
            (self.schematic_swclk_pin, self.schematic_swdio_pin)
        } else {
            (self.schematic_swdio_pin, self.schematic_swclk_pin)
        };
        self.swdio_pin = swdio_pin;
        self.swclk_pin = swclk_pin;

        // Inputs feed both engines at once: a pad's level can fan out to any
        // number of peripheral input signals.
        // SAFETY: both pins are owned by this type for its whole lifetime.
        unsafe {
            esp_rom_gpio_connect_in_signal(swdio_pin, CPU_GPIO_IN0_IDX, false);
            esp_rom_gpio_connect_in_signal(swclk_pin, CPU_GPIO_IN1_IDX, false);
            esp_rom_gpio_connect_in_signal(swdio_pin, FSPIQ_IN_IDX, false);
        }
        // Force the routing write even when the engine is already selected:
        // the pads underneath it may have just changed.
        self.driver = Driver::Spi;
        self.select_driver(Driver::DedicatedGpio);
        self.release();
    }

    pub fn set_engine(&mut self, engine: Engine) {
        self.release();
        self.engine = engine;
    }

    pub fn engine(&self) -> Engine {
        self.engine
    }

    /// Drives a pattern and samples the pad simultaneously, so what GPSPI2
    /// emits can be compared against what it was asked to emit.
    pub fn spi_loopback(&mut self, bits: u64, count: u8) -> u64 {
        self.select_driver(Driver::Spi);
        self.set_output_enable(SWDIO_CHANNEL | SWCLK_CHANNEL);
        let observed = self.spi.loopback(bits, count);
        self.release();
        observed
    }

    /// Cycles spent inside GPSPI2 transactions, and how many there were.
    pub fn peripheral_profile(&mut self, reset: bool) -> (u32, u32) {
        let sample = (self.spi.run_cycles, self.spi.run_count);
        if reset {
            self.spi.run_cycles = 0;
            self.spi.run_count = 0;
        }
        sample
    }

    pub fn set_clock_hz(&mut self, clock_hz: u32) -> u32 {
        let effective = self.spi.set_clock(clock_hz);
        // The bit-bang loop cannot reach the frequencies GPSPI2 can, so clamp
        // it rather than silently running open-loop at whatever the loop costs.
        let bit_bang = effective.clamp(50_000, BIT_BANG_MAX_CLOCK_HZ);
        self.half_cycle_cycles = CPU_FREQ_HZ / (2 * bit_bang);
        match self.engine {
            Engine::Hardware => effective,
            // Above the rate at which the delay stops being the limit, the
            // requested figure is fiction; report what the loop actually does.
            Engine::BitBang => bit_bang.min(self.measure_bit_bang_clock()),
        }
    }

    pub fn clock_hz(&self) -> u32 {
        match self.engine {
            Engine::Hardware => self.spi.clock_hz(),
            Engine::BitBang => CPU_FREQ_HZ / (2 * self.half_cycle_cycles),
        }
    }

    /// Drives both dedicated-GPIO channels in one instruction.
    ///
    /// SWDIO and SWCLK are channels 0 and 1 of the same CSR, so a whole edge —
    /// new data level and new clock level together — is one write rather than
    /// a pair of read-modify-writes. No other channel is routed to a pad, so
    /// writing the register wholesale is safe.
    #[inline(always)]
    fn drive_channels(levels: u32) {
        // SAFETY: 0x805 is CSR_GPIO_OUT_USER on ESP32-C3.
        unsafe { core::arch::asm!("csrw 0x805, {levels}", levels = in(reg) levels) };
    }

    #[inline(always)]
    fn bit_bang_clock(&mut self) {
        let half = self.half_cycle_cycles;
        Self::drive_channels(0);
        self.delay_cycles(half);
        Self::drive_channels(SWCLK_CHANNEL);
        self.delay_cycles(half);
    }

    /// Times the bit-bang loop against the cycle counter with both pads
    /// released, so the rate reported to the host is measured rather than
    /// assumed. Nothing is driven: output enable is cleared first.
    fn measure_bit_bang_clock(&mut self) -> u32 {
        const ROUNDS: u32 = 4;
        const BITS_PER_ROUND: u8 = 64;
        self.set_output_enable(0);
        let start = cpu_cycles();
        for _ in 0..ROUNDS {
            self.bit_bang_read_bits(BITS_PER_ROUND);
        }
        let elapsed = cpu_cycles().wrapping_sub(start);
        let bits = ROUNDS * u32::from(BITS_PER_ROUND);
        CPU_FREQ_HZ / (elapsed / bits).max(1)
    }

    fn bit_bang_write_bits(&mut self, bits: u64, count: u8) {
        self.select_driver(Driver::DedicatedGpio);
        let half = self.half_cycle_cycles;
        for index in 0..count {
            let data = (bits >> index) as u32 & SWDIO_CHANNEL;
            // The data level is presented with SWCLK already low, so it is
            // stable across the low phase and over the rising edge the target
            // samples on.
            Self::drive_channels(data);
            self.delay_cycles(half);
            Self::drive_channels(data | SWCLK_CHANNEL);
            self.delay_cycles(half);
        }
        // Leave the line high, which is the idle state callers expect.
        Self::drive_channels(SWDIO_CHANNEL | SWCLK_CHANNEL);
    }

    /// Samples late in the low phase, matching the engine this port replaced.
    fn bit_bang_read_bits(&mut self, count: u8) -> u64 {
        self.select_driver(Driver::DedicatedGpio);
        let half = self.half_cycle_cycles;
        let mut value = 0;
        for index in 0..count {
            Self::drive_channels(0);
            self.delay_cycles(half);
            if Self::pad_levels() & SWDIO_CHANNEL != 0 {
                value |= 1 << index;
            }
            Self::drive_channels(SWCLK_CHANNEL);
            self.delay_cycles(half);
        }
        value
    }

    /// Points both pads at one engine's output signal.
    ///
    /// The dedicated-GPIO latch is loaded with the level the pad already holds
    /// before handing it back, so the changeover produces no edge of its own.
    fn select_driver(&mut self, driver: Driver) {
        if self.driver == driver {
            return;
        }
        match driver {
            Driver::Spi => {
                // GPSPI2 idles SWCLK low; match that before the handover so the
                // pad holds one level right across it.
                Self::latch_level(SWCLK_CHANNEL, false);
                route_output(self.swdio_pin, FSPID_OUT_IDX);
                route_output(self.swclk_pin, FSPICLK_OUT_IDX);
            }
            Driver::DedicatedGpio => {
                let levels = Self::pad_levels();
                Self::latch_level(SWDIO_CHANNEL, levels & SWDIO_CHANNEL != 0);
                Self::latch_level(SWCLK_CHANNEL, levels & SWCLK_CHANNEL != 0);
                route_output(self.swdio_pin, CPU_GPIO_OUT0_IDX);
                route_output(self.swclk_pin, CPU_GPIO_OUT1_IDX);
            }
        }
        self.driver = driver;
    }

    /// Drives exactly the requested channels and releases the others.
    fn set_output_enable(&mut self, channels: u32) {
        let mut set = 0;
        let mut clear = 0;
        for (channel, pin) in [
            (SWDIO_CHANNEL, self.swdio_pin),
            (SWCLK_CHANNEL, self.swclk_pin),
        ] {
            if channels & channel != 0 {
                set |= 1 << pin;
            } else {
                clear |= 1 << pin;
            }
        }
        if clear != 0 {
            gpio_write(GPIO_ENABLE_W1TC, clear);
        }
        if set != 0 {
            gpio_write(GPIO_ENABLE_W1TS, set);
        }
    }

    fn add_output_enable(&mut self, channels: u32) {
        let mut set = 0;
        for (channel, pin) in [
            (SWDIO_CHANNEL, self.swdio_pin),
            (SWCLK_CHANNEL, self.swclk_pin),
        ] {
            if channels & channel != 0 {
                set |= 1 << pin;
            }
        }
        gpio_write(GPIO_ENABLE_W1TS, set);
    }

    fn remove_output_enable(&mut self, channels: u32) {
        let mut clear = 0;
        for (channel, pin) in [
            (SWDIO_CHANNEL, self.swdio_pin),
            (SWCLK_CHANNEL, self.swclk_pin),
        ] {
            if channels & channel != 0 {
                clear |= 1 << pin;
            }
        }
        gpio_write(GPIO_ENABLE_W1TC, clear);
    }

    fn latch_level(channel: u32, high: bool) {
        if high {
            // SAFETY: 0x805 is CSR_GPIO_OUT_USER on ESP32-C3.
            unsafe { core::arch::asm!("csrrs zero, 0x805, {mask}", mask = in(reg) channel) };
        } else {
            // SAFETY: 0x805 is CSR_GPIO_OUT_USER on ESP32-C3.
            unsafe { core::arch::asm!("csrrc zero, 0x805, {mask}", mask = in(reg) channel) };
        }
    }

    /// Samples both pads through the dedicated-GPIO input channels.
    fn pad_levels() -> u32 {
        let value: u32;
        // SAFETY: 0x804 is CSR_GPIO_IN_USER on ESP32-C3.
        unsafe { core::arch::asm!("csrr {value}, 0x804", value = out(reg) value) };
        value
    }

    #[inline(always)]
    fn delay_cycles(&self, cycles: u32) {
        if cycles <= MIN_DELAY_CYCLES {
            return;
        }
        let start = cpu_cycles();
        while cpu_cycles().wrapping_sub(start) < cycles {}
    }
}

impl SwdIo for EspSwdIo<'_> {
    fn set_swdio_output(&mut self) {
        self.add_output_enable(SWDIO_CHANNEL | SWCLK_CHANNEL);
    }

    fn set_swdio_input(&mut self) {
        self.remove_output_enable(SWDIO_CHANNEL);
    }

    fn set_swclk_output(&mut self) {
        self.add_output_enable(SWCLK_CHANNEL);
    }

    fn write_swdio(&mut self, high: bool) {
        self.select_driver(Driver::DedicatedGpio);
        Self::latch_level(SWDIO_CHANNEL, high);
    }

    fn read_swdio(&mut self) -> bool {
        Self::pad_levels() & SWDIO_CHANNEL != 0
    }

    fn write_swclk(&mut self, high: bool) {
        self.select_driver(Driver::DedicatedGpio);
        Self::latch_level(SWCLK_CHANNEL, high);
    }

    fn write_bits(&mut self, bits: u64, count: u8) {
        match self.engine {
            Engine::Hardware => {
                self.select_driver(Driver::Spi);
                self.spi.write_bits(bits, count);
            }
            Engine::BitBang => self.bit_bang_write_bits(bits, count),
        }
    }

    fn read_bits(&mut self, count: u8) -> u64 {
        match self.engine {
            Engine::Hardware => {
                self.select_driver(Driver::Spi);
                self.spi.read_bits(count)
            }
            Engine::BitBang => self.bit_bang_read_bits(count),
        }
    }

    fn leading_turnaround(&mut self) {
        // GPSPI2 samples on the falling edge, half a period earlier than the
        // bit-bang engine sampling late in the low phase. The first clock of
        // the read that follows therefore doubles as the turnaround: the
        // target starts driving on its rising edge and is sampled on its
        // falling edge. Adding a clock here shifts the whole reply by a bit,
        // which is measurable with `wire-probe`.
        if self.engine == Engine::BitBang {
            self.read_bits(1);
        }
    }

    fn write_data_phase(&mut self, data: u32, parity: bool) {
        if self.engine == Engine::BitBang {
            self.write_bits(u64::from(data) | u64::from(parity) << 32, 33);
            return;
        }
        // GPSPI2 transmits a partial byte in the second buffer word wrongly:
        // asked for 33 bits it emits `W0` bit 24 in the 33rd position and never
        // reads `W1`, which the pad loopback shows directly. Whole-byte
        // transmits are exact, `W1` included, so the field is padded to five
        // bytes. The seven extra bits are zeros, which is what SWD requires of
        // an idle line — padding them high instead would present seven start
        // bits to the target, and does in fact break the link.
        self.select_driver(Driver::Spi);
        self.spi
            .write_bits(u64::from(data) | u64::from(parity) << 32, 40);
    }

    fn read_reply(&mut self, ok: u8) -> u64 {
        if self.engine == Engine::BitBang {
            let ack = self.read_bits(3);
            if ack as u8 != ok {
                self.trailing_turnaround();
                return ack;
            }
            let data = self.read_data_phase();
            return ack | data << 3;
        }
        // ACK, data, parity and both turnaround clocks in one transaction:
        // three peripheral round trips per word become two, and on this bus
        // the sequencing costs far more than the clocks.
        self.select_driver(Driver::Spi);
        self.spi.read_bits(38)
    }

    fn end_transfer(&mut self) {
        // Staying in SPI mode saves a pair of engine changeovers per transfer,
        // and the resting level is unobservable without a clock edge. The
        // released idle state is still established by `release`.
        if self.engine == Engine::BitBang {
            self.set_swdio_output();
            self.write_swdio(true);
        }
    }

    fn read_data_phase(&mut self) -> u64 {
        match self.engine {
            // Data, parity, and both turnaround clocks in one transaction: 35
            // bits in, of which the caller wants the low 33.
            Engine::Hardware => {
                self.select_driver(Driver::Spi);
                self.spi.read_bits(35)
            }
            Engine::BitBang => {
                let sampled = self.read_bits(33);
                self.trailing_turnaround();
                sampled
            }
        }
    }

    fn trailing_turnaround(&mut self) {
        // The hardware engine repays the clock it borrowed above on top of the
        // protocol's own turnaround.
        match self.engine {
            Engine::Hardware => self.read_bits(2),
            Engine::BitBang => self.read_bits(1),
        };
    }

    fn write_sequence(&mut self, bytes: &[u8], count: usize) {
        self.select_driver(Driver::Spi);
        let mut written = 0;
        while written < count {
            let burst = (count - written).min(usize::from(MAX_BURST_BITS));
            let mut bits = 0u64;
            for index in 0..burst {
                let bit = written + index;
                bits |= u64::from(bytes[bit / 8] >> (bit % 8) & 1) << index;
            }
            self.spi.write_bits(bits, burst as u8);
            written += burst;
        }
    }

    fn delay_half_cycle(&mut self) {
        self.delay_cycles(self.half_cycle_cycles);
    }

    fn delay_ms(&mut self, milliseconds: u16) {
        Ets::delay_ms(u32::from(milliseconds));
    }

    fn release(&mut self) {
        self.set_output_enable(0);
        // Clear the dedicated-GPIO enable too, so neither path can drive a
        // line while the bridge believes the target is released.
        // SAFETY: 0x803 is CSR_GPIO_OEN_USER on ESP32-C3.
        unsafe {
            core::arch::asm!("csrrc zero, 0x803, {mask}", mask = in(reg) SWDIO_CHANNEL | SWCLK_CHANNEL)
        };
        self.select_driver(Driver::DedicatedGpio);
        // Preload high so the next enabled transition does not start from a
        // level that was only ever an artefact of the released bus.
        Self::latch_level(SWDIO_CHANNEL, true);
        Self::latch_level(SWCLK_CHANNEL, true);
    }

    fn sample_lines(&mut self) -> (bool, bool) {
        let levels = Self::pad_levels();
        (levels & SWDIO_CHANNEL != 0, levels & SWCLK_CHANNEL != 0)
    }

    fn transaction<R>(&mut self, operation: impl FnOnce(&mut Self) -> R) -> R {
        if self.engine == Engine::Hardware {
            // GPSPI2 generates every edge from its own clock domain, so an
            // interrupt between fields cannot deform the waveform. Holding a
            // critical section across a whole block read would stall Wi-Fi and
            // USB for milliseconds to buy nothing.
            operation(self)
        } else {
            critical_section::with(|_| operation(self))
        }
    }
}

impl Drop for EspSwdIo<'_> {
    fn drop(&mut self) {
        self.release();
    }
}
