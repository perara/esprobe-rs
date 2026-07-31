//! GPSPI2 shift-register backend for the SWD wire.
//!
//! Bit-banging SWD costs two pad transitions and a software delay per bit, so
//! the achievable clock is bounded by how tightly the CPU can be held in a
//! loop, and every interrupt is a potential malformed edge. GPSPI2 generates
//! SWCLK in hardware instead: the CPU writes a shift register, starts the
//! peripheral, and the whole field leaves the pad at an exact frequency no
//! matter what the scheduler does in between. That removes both the speed
//! ceiling and the reason the bit-bang path had to run inside a critical
//! section.
//!
//! SWD maps onto SPI mode 0 exactly: the host changes SWDIO on the falling
//! edge, the target samples it on the rising edge, and both ends shift the
//! least significant bit first.
//!
//! Register offsets and field positions are from the ESP32-C3 TRM, mirrored by
//! `components/soc/esp32c3/register/soc/spi_reg.h` in ESP-IDF v5.4.3.

use esp_idf_svc::sys::{periph_module_enable, periph_module_t_PERIPH_SPI2_MODULE};

use esprobe_firmware::spi_clock::{self, SOURCE_CLOCK_HZ};

const SPI2_BASE: u32 = 0x6002_4000;
const SPI_CMD: u32 = 0x00;
const SPI_CTRL: u32 = 0x08;
const SPI_CLOCK: u32 = 0x0c;
const SPI_USER: u32 = 0x10;
const SPI_USER1: u32 = 0x14;
const SPI_USER2: u32 = 0x18;
const SPI_MS_DLEN: u32 = 0x1c;
const SPI_MISC: u32 = 0x20;
const SPI_DMA_CONF: u32 = 0x30;
const SPI_W0: u32 = 0x98;
const SPI_CLK_GATE: u32 = 0xe8;

const CMD_UPDATE: u32 = 1 << 23;
const CMD_USR: u32 = 1 << 24;
const CTRL_RD_BIT_ORDER: u32 = 1 << 25;
const CTRL_WR_BIT_ORDER: u32 = 1 << 26;
const USER_USR_MOSI: u32 = 1 << 27;
const USER_USR_MISO: u32 = 1 << 28;
const USER_DOUTDIN: u32 = 1 << 0;
const USER_CK_OUT_EDGE: u32 = 1 << 9;
const MISC_CS_DIS_ALL: u32 = 0b111;
const DMA_CONF_BUF_AFIFO_RST: u32 = 1 << 30;
const DMA_CONF_RX_AFIFO_RST: u32 = 1 << 29;
const CLK_GATE_CLK_EN: u32 = 1 << 0;
const CLK_GATE_MST_CLK_ACTIVE: u32 = 1 << 1;
const CLK_GATE_MST_CLK_SEL_PLL: u32 = 1 << 2;

/// The widest field one GPSPI2 transfer moves here. The peripheral holds 512
/// bits, but SWD never needs more than a 33-bit data-plus-parity field.
pub const MAX_BURST_BITS: u8 = 64;
/// Iterations of a register poll before a transfer is abandoned. A 64-bit
/// field at the slowest supported clock takes well under a millisecond.
const SPIN_LIMIT: u32 = 2_000_000;

/// Reads the CPU cycle counter enabled by the pad driver at start-up.
fn cpu_cycles() -> u32 {
    let value: u32;
    // SAFETY: 0x7e2 is the ESP32-C3 machine performance counter.
    unsafe { core::arch::asm!("csrr {value}, 0x7e2", value = out(reg) value) };
    value
}

fn write_reg(offset: u32, value: u32) {
    // SAFETY: `offset` names a GPSPI2 register this driver owns exclusively.
    unsafe { core::ptr::write_volatile((SPI2_BASE + offset) as *mut u32, value) };
}

fn read_reg(offset: u32) -> u32 {
    // SAFETY: `offset` names a GPSPI2 register this driver owns exclusively.
    unsafe { core::ptr::read_volatile((SPI2_BASE + offset) as *const u32) }
}

/// Which direction GPSPI2 drives for one field.
#[derive(Clone, Copy, Eq, PartialEq)]
enum Phase {
    Drive,
    Sample,
}

pub struct SpiWire {
    clock_register: u32,
    clock_hz: u32,
    /// Last values latched into the peripheral, so a transfer that repeats a
    /// configuration does not pay to rewrite it.
    programmed_user: u32,
    programmed_dlen: u32,
    /// Cycles spent inside `run`, and how many times it was entered, so the
    /// per-word cost can be split between the peripheral and everything else.
    pub run_cycles: u32,
    pub run_count: u32,
}

impl SpiWire {
    /// Claims GPSPI2 and configures it for SWD framing.
    ///
    /// The caller must route FSPICLK and FSPID/FSPIQ to the SWCLK and SWDIO
    /// pads, and stays responsible for SWDIO's output enable: this driver
    /// never drives a line the caller has not enabled.
    pub fn new(clock_hz: u32) -> Self {
        // SAFETY: SPI2 is handed to no other driver in this firmware.
        unsafe { periph_module_enable(periph_module_t_PERIPH_SPI2_MODULE) };

        write_reg(
            SPI_CLK_GATE,
            CLK_GATE_CLK_EN | CLK_GATE_MST_CLK_ACTIVE | CLK_GATE_MST_CLK_SEL_PLL,
        );
        // Least significant bit first in both directions, as SWD requires.
        write_reg(
            SPI_CTRL,
            read_reg(SPI_CTRL) | CTRL_RD_BIT_ORDER | CTRL_WR_BIT_ORDER,
        );
        // No command, address, or dummy phase, and no DMA.
        write_reg(SPI_USER1, 0);
        write_reg(SPI_USER2, 0);
        write_reg(SPI_DMA_CONF, 0);
        // No CS pin is driven: SWD has no chip select and the CD4052 owns pad
        // selection. ck_idle_edge stays clear, so SWCLK rests low.
        write_reg(SPI_MISC, MISC_CS_DIS_ALL);

        let mut wire = Self {
            clock_register: 0,
            clock_hz: 0,
            programmed_user: u32::MAX,
            programmed_dlen: u32::MAX,
            run_cycles: 0,
            run_count: 0,
        };
        wire.set_clock(clock_hz);
        wire
    }

    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    /// Sets the SWCLK frequency, returning the frequency actually programmed.
    pub fn set_clock(&mut self, requested_hz: u32) -> u32 {
        let divider = spi_clock::divider_for(SOURCE_CLOCK_HZ, requested_hz);
        self.clock_register = divider.register;
        self.clock_hz = divider.clock_hz;
        write_reg(SPI_CLOCK, divider.register);
        self.programmed_user = u32::MAX;
        divider.clock_hz
    }

    /// Clocks out `count` bits, least significant first. SWDIO must already be
    /// an output.
    pub fn write_bits(&mut self, bits: u64, count: u8) {
        if count == 0 {
            return;
        }
        // One transaction per field, always. Splitting a field across two
        // transactions — even at the word boundary, even for the single
        // trailing parity bit — breaks it: a 56-bit line reset sent as 32+24
        // stops resetting the line, and a 33-bit data phase sent as 32+1 stops
        // being accepted. Whatever the pad does in the gap between two
        // transactions, the target sees it, so fields must not contain gaps.
        write_reg(SPI_W0, bits as u32);
        write_reg(SPI_W0 + 4, (bits >> 32) as u32);
        self.run(Phase::Drive, count);
    }

    /// Clocks in `count` bits, least significant first. SWDIO must already be
    /// released.
    pub fn read_bits(&mut self, count: u8) -> u64 {
        if count == 0 {
            return 0;
        }
        self.run(Phase::Sample, count);
        let value = u64::from(read_reg(SPI_W0)) | u64::from(read_reg(SPI_W0 + 4)) << 32;
        if count >= 64 {
            value
        } else {
            value & ((1 << count) - 1)
        }
    }

    /// Transmits `count` bits and samples the pad at the same time, returning
    /// what was actually seen on the wire.
    ///
    /// `FSPID` and `FSPIQ` are both routed to the SWDIO pad, so a full-duplex
    /// transfer reads back the peripheral's own output through the pad. That
    /// turns "what does GPSPI2 actually emit" from an inference off the
    /// target's pass/fail verdict into a direct measurement.
    ///
    /// The caller must have SWDIO enabled as an output; with it released this
    /// samples the pull-up instead.
    pub fn loopback(&mut self, bits: u64, count: u8) -> u64 {
        Self::reset_fifos();
        self.programmed_user = u32::MAX;
        self.programmed_dlen = u32::MAX;
        write_reg(SPI_W0, bits as u32);
        write_reg(SPI_W0 + 4, (bits >> 32) as u32);
        write_reg(SPI_MS_DLEN, u32::from(count) - 1);
        write_reg(SPI_USER, USER_USR_MOSI | USER_USR_MISO | USER_DOUTDIN);
        write_reg(SPI_CLOCK, self.clock_register);
        write_reg(SPI_CMD, CMD_UPDATE);
        if Self::wait_for(CMD_UPDATE) {
            write_reg(SPI_CMD, CMD_USR);
            Self::wait_for(CMD_USR);
        }
        let value = u64::from(read_reg(SPI_W0)) | u64::from(read_reg(SPI_W0 + 4)) << 32;
        if count >= 64 {
            value
        } else {
            value & ((1 << count) - 1)
        }
    }

    /// Spins long enough for a 64-bit field at the slowest supported clock and
    /// no longer. A peripheral bit that never clears must not be able to take
    /// the whole bridge down with it.
    fn wait_for(bit: u32) -> bool {
        for _ in 0..SPIN_LIMIT {
            if read_reg(SPI_CMD) & bit == 0 {
                return true;
            }
        }
        false
    }

    /// Clears the CPU-side buffer FIFO between transfers.
    ///
    /// Without this the peripheral does not read past the first buffer word:
    /// a 33-bit transmit emitted `W0` bit 24 in the 33rd position and ignored
    /// `W1` entirely, which is a parity error on roughly half of all values.
    /// The pad loopback shows it directly — send bit 24, watch bit 32 come
    /// back set — and IDF's own driver resets these FIFOs around every
    /// CPU-controlled transfer.
    fn reset_fifos() {
        write_reg(SPI_DMA_CONF, DMA_CONF_BUF_AFIFO_RST | DMA_CONF_RX_AFIFO_RST);
        write_reg(SPI_DMA_CONF, 0);
    }

    fn run(&mut self, phase: Phase, count: u8) {
        debug_assert!(count <= MAX_BURST_BITS);
        let started = cpu_cycles();
        let dlen = u32::from(count) - 1;
        if dlen != self.programmed_dlen {
            write_reg(SPI_MS_DLEN, dlen);
            self.programmed_dlen = dlen;
        }
        // The two directions of SWD do not share a clock phase. The host
        // changes SWDIO on the falling edge and the target samples it on the
        // rising one, which is SPI mode 0; but the target changes SWDIO on the
        // rising edge, so the host has to sample on the falling one — mode 1.
        // Driving both phases at mode 0 reads every input bit half a period
        // early, which is exactly as wrong as it sounds.
        let user = match phase {
            Phase::Drive => USER_USR_MOSI,
            Phase::Sample => USER_USR_MISO | USER_CK_OUT_EDGE,
        };
        if user != self.programmed_user {
            write_reg(SPI_USER, user);
            write_reg(SPI_CLOCK, self.clock_register);
            self.programmed_user = user;
        }
        // Configuration registers live in the APB clock domain and only reach
        // the SPI clock domain once UPDATE has self-cleared.
        write_reg(SPI_CMD, CMD_UPDATE);
        if !Self::wait_for(CMD_UPDATE) {
            return;
        }
        write_reg(SPI_CMD, CMD_USR);
        Self::wait_for(CMD_USR);
        self.run_cycles = self
            .run_cycles
            .wrapping_add(cpu_cycles().wrapping_sub(started));
        self.run_count = self.run_count.wrapping_add(1);
    }
}
