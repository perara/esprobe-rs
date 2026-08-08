//! The CPU cycle counter, which is what times a bit-banged edge.
//!
//! The bit-bang engine needs a clock finer than a microsecond — a half period
//! at 8 MHz is 62 ns — and the only thing on these parts with that resolution
//! is the core's own cycle counter. Every part has one; no two reach it the
//! same way, and on one of them reading the wrong register is an illegal
//! instruction rather than a wrong number.
//!
//! Also used by the GPSPI2 driver to time its own transactions, which is why
//! this is a module of its own rather than a private helper in the pad driver.

/// Prepares the counter, if this part needs it prepared.
///
/// Called once during start-up, before anything times anything.
pub fn enable() {
    #[cfg(all(target_arch = "riscv32", target_os = "espidf"))]
    {
        // The ESP32-C3's `mcycle` is not the RISC-V standard one: it counts
        // only while the core is not stalled, so a delay measured with it runs
        // long. 0x7e0 selects the counted event, 0x7e1 runs the counter, and
        // 0x7e2 is the result. Reading `mcycle` instead is an illegal
        // instruction on this core and takes the firmware down with it.
        //
        // SAFETY: machine-mode CSRs this firmware owns; no side effects beyond
        // starting the counter.
        unsafe {
            core::arch::asm!(
                "csrw 0x7e0, {cycles}",
                "csrw 0x7e1, {enable}",
                cycles = in(reg) 1u32,
                enable = in(reg) 1u32,
            )
        };
    }
    // Xtensa's CCOUNT free-runs from reset and has nothing to enable. Left
    // explicit rather than absent so a part that does need setting up has an
    // obvious place to say so.
}

/// Reads the counter.
///
/// Wraps at 32 bits — about nine seconds at 480 MHz and thirty at 160 MHz —
/// so callers must compare with a wrapping subtraction, never an ordering.
#[inline(always)]
pub fn read() -> u32 {
    #[cfg(all(target_arch = "riscv32", target_os = "espidf"))]
    {
        let value: u32;
        // SAFETY: 0x7e2 is the machine performance counter enabled above.
        unsafe { core::arch::asm!("csrr {value}, 0x7e2", value = out(reg) value) };
        value
    }
    #[cfg(all(target_arch = "xtensa", target_os = "espidf"))]
    {
        let value: u32;
        // SAFETY: CCOUNT is a free-running read-only special register.
        unsafe { core::arch::asm!("rsr.ccount {value}", value = out(reg) value) };
        value
    }
    // A host build never times an edge; it exists so the hardware-neutral
    // half of the crate links.
    #[cfg(not(target_os = "espidf"))]
    {
        0
    }
}

#[cfg(test)]
mod tests {
    /// The counter wraps, so elapsed time is only ever a wrapping difference.
    /// This pins the arithmetic the callers must use rather than the counter
    /// itself, which a host build does not have.
    #[test]
    fn elapsed_is_correct_across_a_wrap() {
        let before: u32 = u32::MAX - 10;
        let after: u32 = 5;
        assert_eq!(after.wrapping_sub(before), 16);
    }
}
