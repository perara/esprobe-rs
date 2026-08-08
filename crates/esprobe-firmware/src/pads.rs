//! Driving and sampling the two debug pads, however this part can.
//!
//! The wire engine needs three things of a pad: set both levels at once, set
//! one, and read both. What that costs varies more than anything else in this
//! firmware, so it is the one place worth a trait.
//!
//! # Why "at once" matters
//!
//! A SWD edge is a new data level and a new clock level together. On a part
//! that can write both in one instruction, an edge is one store; on one that
//! cannot, it is two, and the pads are briefly inconsistent between them. That
//! is harmless for SWD — the target samples on the clock edge, which is the
//! second of the two — but it is why [`Pads::drive`] exists as its own
//! operation rather than two [`Pads::latch`] calls.
//!
//! # The implementations
//!
//! - [`DedicatedGpio`] — the CPU's dedicated-GPIO peripheral, reached through
//!   RISC-V CSR instructions. One instruction per edge. ESP32-C3, C6 and H2.
//! - [`MatrixGpio`] — the ordinary GPIO output registers, which every part in
//!   the family has. Two stores per edge and a bus round trip to sample, but it
//!   assembles anywhere. This is what the original ESP32 needs, having no
//!   dedicated-GPIO peripheral at all, and what the Xtensa parts use until
//!   someone writes their `wur`/`rur` equivalent.
//!
//! Both are addressed in *channels*, not pins: bit 0 is SWDIO and bit 1 is
//! SWCLK, matching the dedicated-GPIO channel numbering. [`MatrixGpio`] holds
//! the pin numbers so it can translate; [`DedicatedGpio`] ignores them, because
//! the routing was done when the pads were claimed.

/// Bit 0 of every channel mask: the data pad.
pub const SWDIO: u32 = 0b01;
/// Bit 1 of every channel mask: the clock pad.
pub const SWCLK: u32 = 0b10;

/// What the wire engine needs of a pad, and nothing more.
pub trait Pads {
    /// Claims the pads. The caller has already routed them; this only records
    /// what an implementation needs to address them later.
    fn new(swdio_pin: i32, swclk_pin: i32) -> Self;

    /// Sets both pads to the levels in `levels`, ideally in one instruction.
    fn drive(&self, levels: u32);

    /// Sets or clears one channel, leaving the other alone.
    fn latch(&self, channel: u32, high: bool);

    /// Samples both pads.
    fn levels(&self) -> u32;

    /// Binds the pads to this layer.
    ///
    /// On a part with a dedicated-GPIO peripheral the pads must be routed to
    /// its channels through the GPIO matrix; on a part without, the pads are
    /// already ordinary GPIO and there is nothing to route. Called whenever the
    /// pin map changes, so it must be idempotent.
    ///
    /// `route_output` is the caller's, because selecting *which* driver owns
    /// the output at any moment is the engine's decision, not the pad layer's.
    fn bind(&self, swdio_pin: u32, swclk_pin: u32, route_output: impl Fn(u32, u32));

    /// The output signal indices this layer drives the pads with, if it has
    /// any of its own. `None` on a part whose pads are plain GPIO.
    fn output_signals(&self) -> Option<(u32, u32)>;

    /// Stops this layer driving the named channels.
    ///
    /// Distinct from clearing the GPIO output enable, which the caller does
    /// separately: on a part with a dedicated-GPIO peripheral the two are
    /// different switches, and leaving this one on means the pad is still
    /// driven from a path the caller thinks it released.
    fn release(&self, channels: u32);
}

/// The dedicated-GPIO peripheral, through RISC-V CSRs.
///
/// The fast path, and the reason the bit-bang engine can reach a few MHz at
/// all: an edge is a single `csrw`. Only exists on the RISC-V parts — the
/// instructions do not assemble on Xtensa, which is what makes this a trait
/// rather than a `cfg` around a constant.
#[cfg(any(esp32c3, esp32c6, esp32h2))]
pub struct DedicatedGpio;

#[cfg(any(esp32c3, esp32c6, esp32h2))]
impl Pads for DedicatedGpio {
    fn new(_swdio_pin: i32, _swclk_pin: i32) -> Self {
        // The pads were bound to channels 0 and 1 when they were claimed, so
        // there is nothing per-pin to remember.
        Self
    }

    #[inline(always)]
    fn drive(&self, levels: u32) {
        // SAFETY: 0x805 is CSR_GPIO_OUT_USER. No channel other than 0 and 1 is
        // routed to a pad, so writing the register wholesale is safe.
        unsafe { core::arch::asm!("csrw 0x805, {levels}", levels = in(reg) levels) };
    }

    #[inline(always)]
    fn latch(&self, channel: u32, high: bool) {
        // SAFETY: as above; these are the atomic set/clear forms.
        if high {
            unsafe { core::arch::asm!("csrrs zero, 0x805, {mask}", mask = in(reg) channel) };
        } else {
            unsafe { core::arch::asm!("csrrc zero, 0x805, {mask}", mask = in(reg) channel) };
        }
    }

    #[inline(always)]
    fn levels(&self) -> u32 {
        let value: u32;
        // SAFETY: 0x804 is CSR_GPIO_IN_USER, a read with no side effects.
        unsafe { core::arch::asm!("csrr {value}, 0x804", value = out(reg) value) };
        value
    }

    fn release(&self, channels: u32) {
        // SAFETY: 0x803 is CSR_GPIO_OEN_USER; the clear form touches only the
        // channels named.
        unsafe { core::arch::asm!("csrrc zero, 0x803, {mask}", mask = in(reg) channels) };
    }

    fn bind(&self, swdio_pin: u32, swclk_pin: u32, _route_output: impl Fn(u32, u32)) {
        use crate::chip::signals::{DEDICATED_IN0, DEDICATED_IN1};
        // Inputs fan out: a pad's level can feed any number of peripheral
        // input signals at once, so binding these does not take the pad away
        // from the SPI engine's own input.
        //
        // SAFETY: both pins are owned by the caller for its whole lifetime.
        unsafe {
            esp_idf_svc::sys::esp_rom_gpio_connect_in_signal(swdio_pin, DEDICATED_IN0, false);
            esp_idf_svc::sys::esp_rom_gpio_connect_in_signal(swclk_pin, DEDICATED_IN1, false);
        }
    }

    fn output_signals(&self) -> Option<(u32, u32)> {
        use esp_idf_svc::sys::{CPU_GPIO_OUT0_IDX, CPU_GPIO_OUT1_IDX};
        Some((CPU_GPIO_OUT0_IDX, CPU_GPIO_OUT1_IDX))
    }
}

/// The ordinary GPIO matrix, through its output registers.
///
/// Portable to every part in the family, and correspondingly slower: an edge is
/// two stores rather than one, and sampling is a bus read rather than a CSR.
/// The hardware engine does not care — GPSPI2 shifts through the matrix either
/// way — so this only sets the ceiling of the bit-bang fallback and the pace of
/// the pad-level diagnostics.
///
/// Addresses pins below 32 only. The original ESP32 carries GPIO32..=39 in a
/// second bank at different offsets, and 34..=39 are input-only; a debug pad
/// belongs in the first bank on every board worth wiring, so that bank is
/// simply not addressed here rather than half-supported.
pub struct MatrixGpio {
    swdio_mask: u32,
    swclk_mask: u32,
}

impl MatrixGpio {
    /// Offsets from the GPIO block's base. Identical across the family for the
    /// first bank, which is why these are not in `chip`.
    const OUT_W1TS: u32 = 0x08;
    const OUT_W1TC: u32 = 0x0c;
    const IN: u32 = 0x3c;

    fn mask_for(&self, channels: u32) -> u32 {
        let mut mask = 0;
        if channels & SWDIO != 0 {
            mask |= self.swdio_mask;
        }
        if channels & SWCLK != 0 {
            mask |= self.swclk_mask;
        }
        mask
    }

    #[inline(always)]
    fn write(offset: u32, value: u32) {
        // SAFETY: the GPIO block is memory-mapped at a base this crate owns for
        // the pads it has claimed, and W1TS/W1TC affect only the bits written.
        unsafe {
            core::ptr::write_volatile((crate::chip::GPIO_BASE + offset) as *mut u32, value);
        }
    }
}

impl Pads for MatrixGpio {
    fn new(swdio_pin: i32, swclk_pin: i32) -> Self {
        Self {
            swdio_mask: 1 << swdio_pin,
            swclk_mask: 1 << swclk_pin,
        }
    }

    #[inline(always)]
    fn drive(&self, levels: u32) {
        // Both pads move, so both a set and a clear are needed. Clear first:
        // between the two stores one pad already holds its new level, and a
        // clock that falls early is a shorter high phase, where a clock that
        // rises early would be an extra edge.
        let set = self.mask_for(levels);
        let clear = self.mask_for(!levels);
        if clear != 0 {
            Self::write(Self::OUT_W1TC, clear);
        }
        if set != 0 {
            Self::write(Self::OUT_W1TS, set);
        }
    }

    #[inline(always)]
    fn latch(&self, channel: u32, high: bool) {
        let mask = self.mask_for(channel);
        Self::write(if high { Self::OUT_W1TS } else { Self::OUT_W1TC }, mask);
    }

    #[inline(always)]
    fn levels(&self) -> u32 {
        // SAFETY: a read of a memory-mapped input register, no side effects.
        let raw =
            unsafe { core::ptr::read_volatile((crate::chip::GPIO_BASE + Self::IN) as *const u32) };
        let mut levels = 0;
        if raw & self.swdio_mask != 0 {
            levels |= SWDIO;
        }
        if raw & self.swclk_mask != 0 {
            levels |= SWCLK;
        }
        levels
    }

    fn release(&self, _channels: u32) {
        // Nothing to do: this layer drives through the ordinary output enable,
        // which the caller clears itself. There is no second switch to undo.
    }

    fn bind(&self, _swdio_pin: u32, _swclk_pin: u32, _route_output: impl Fn(u32, u32)) {
        // The pads are plain GPIO already. Nothing to route.
    }

    fn output_signals(&self) -> Option<(u32, u32)> {
        // No peripheral drives these pads on this path; the GPIO output
        // register does, so the matrix stays on its plain-GPIO function.
        None
    }
}

/// The implementation this part gets.
///
/// One alias so the engine never names a concrete type, and adding a part is
/// choosing here rather than editing the driver.
#[cfg(any(esp32c3, esp32c6, esp32h2))]
pub type ChipPads = DedicatedGpio;
/// Xtensa parts and anything else fall back to the matrix. The S2 and S3 do
/// have a dedicated-GPIO peripheral, reached through `wur`/`rur` rather than
/// CSRs; implementing that is a third `impl Pads` and nothing else.
#[cfg(not(any(esp32c3, esp32c6, esp32h2)))]
pub type ChipPads = MatrixGpio;

#[cfg(test)]
mod tests {
    use super::*;

    /// Host builds get `MatrixGpio`, so its translation is testable without a
    /// board — and it is the half with arithmetic in it. The CSR path has no
    /// logic to check: it is three instructions.
    #[test]
    fn channels_map_to_the_pins_they_were_given() {
        let pads = MatrixGpio::new(1, 2);
        assert_eq!(pads.mask_for(SWDIO), 1 << 1);
        assert_eq!(pads.mask_for(SWCLK), 1 << 2);
        assert_eq!(pads.mask_for(SWDIO | SWCLK), (1 << 1) | (1 << 2));
        assert_eq!(pads.mask_for(0), 0);
    }

    #[test]
    fn a_pin_map_that_moved_moves_the_masks_with_it() {
        // The pins are a build-time override, so nothing may assume 1 and 2.
        let pads = MatrixGpio::new(7, 10);
        assert_eq!(pads.mask_for(SWDIO), 1 << 7);
        assert_eq!(pads.mask_for(SWCLK), 1 << 10);
    }

    #[test]
    fn driving_a_level_sets_one_channel_and_clears_the_other() {
        // `drive` splits its argument into a set mask and a clear mask; getting
        // the complement wrong drives both pads the same way, which reads as a
        // wire that never clocks.
        let pads = MatrixGpio::new(1, 2);
        assert_eq!(pads.mask_for(SWCLK), 1 << 2, "set half");
        assert_eq!(
            pads.mask_for(!SWCLK & (SWDIO | SWCLK)),
            1 << 1,
            "clear half"
        );
    }
}
