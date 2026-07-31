//! Allocation-free ARM ADIv5 Serial Wire Debug host.

const TRANSFER_OK: u8 = 0b001;
const TRANSFER_WAIT: u8 = 0b010;
const TRANSFER_FAULT: u8 = 0b100;
const TRANSFER_AP: u8 = 1;
const TRANSFER_READ: u8 = 2;

const DP_ABORT: u8 = 0x00;
const DP_IDCODE: u8 = TRANSFER_READ;
const DP_CTRL_STAT_WRITE: u8 = 0x04;
const DP_CTRL_STAT_READ: u8 = 0x04 | TRANSFER_READ;
const DP_SELECT: u8 = 0x08;
const DP_RDBUFF: u8 = 0x0c | TRANSFER_READ;
const AP_CSW: u8 = TRANSFER_AP;
const AP_TAR: u8 = TRANSFER_AP | 0x04;
const AP_DRW_WRITE: u8 = TRANSFER_AP | 0x0c;
const AP_DRW_READ: u8 = TRANSFER_AP | 0x0c | TRANSFER_READ;

const ABORT_ALL: u32 = 0x1f;
const POWER_REQUEST: u32 = 0x5000_0000;
const POWER_ACK: u32 = 0xa000_0000;
const CORTEX_M_WORD_CSW: u32 = 0x2300_0052;

/// The widest field the wire engine moves in one backend call.
pub const MAX_BURST_BITS: u8 = 64;

/// Electrical operations required by the SWD wire engine.
pub trait SwdIo {
    fn set_swdio_output(&mut self);
    fn set_swdio_input(&mut self);
    fn set_swclk_output(&mut self);
    fn write_swdio(&mut self, high: bool);
    fn read_swdio(&mut self) -> bool;
    fn write_swclk(&mut self, high: bool);
    fn delay_half_cycle(&mut self);
    fn delay_ms(&mut self, milliseconds: u16);
    fn release(&mut self);
    fn sample_lines(&mut self) -> (bool, bool);

    /// Clocks out `count` bits of `bits`, least significant bit first, while
    /// SWDIO is already an output.
    ///
    /// Backends with a hardware shift register override this; the default
    /// implementation drives one pad transition per bit.
    fn write_bits(&mut self, bits: u64, count: u8)
    where
        Self: Sized,
    {
        debug_assert!(count <= MAX_BURST_BITS);
        for index in 0..count {
            write_bit(self, bits >> index & 1 != 0);
        }
    }

    /// Clocks in `count` bits, least significant bit first, while SWDIO is
    /// already released. Bits above `count` are zero.
    fn read_bits(&mut self, count: u8) -> u64
    where
        Self: Sized,
    {
        debug_assert!(count <= MAX_BURST_BITS);
        let mut value = 0;
        for index in 0..count {
            value |= u64::from(read_bit(self)) << index;
        }
        value
    }

    /// Clocks the turnaround period between the host's request and the
    /// target's first driven bit.
    ///
    /// A backend that samples late in the low phase needs a clock of its own
    /// here, because by the time it looks the target has not started driving.
    /// One that samples on the falling edge already sees the target's first bit
    /// within the first clock of the following read, and overrides this to
    /// nothing — an extra clock there would shift the whole reply by a bit.
    fn leading_turnaround(&mut self)
    where
        Self: Sized,
    {
        self.read_bits(1);
    }

    /// Clocks a write transfer's 32 data bits followed by its parity bit.
    ///
    /// Separated from `write_bits` because a backend may need to pad it. SWD
    /// lets the host hold the line high for idle clocks once the parity bit is
    /// out, so a field that is awkward for the hardware to emit at exactly 33
    /// bits may legally be sent longer.
    fn write_data_phase(&mut self, data: u32, parity: bool)
    where
        Self: Sized,
    {
        self.write_bits(u64::from(data) | u64::from(parity) << 32, 33);
    }

    /// Reads a transfer's reply: ACK in bits 0..3, data in 3..35, parity in
    /// 35, and the trailing turnaround above that.
    ///
    /// `ok` is the ACK value whose data phase is worth having. A backend where
    /// each field costs a separate peripheral transaction may clock the whole
    /// reply unconditionally and discard it: with SWDIO released the target
    /// drives nothing after a WAIT or FAULT, so the extra clocks are idle ones
    /// and cost only time. That licence belongs to reads alone — a write must
    /// never drive its data phase after a refused ACK.
    fn read_reply(&mut self, ok: u8) -> u64
    where
        Self: Sized,
    {
        let ack = self.read_bits(3);
        if ack as u8 != ok {
            // The turnaround is owed whatever the ACK said. A backend that
            // clocks the whole reply in one go has already issued it; one that
            // stops here has not, and skipping it desynchronises the next
            // transfer — which is what a WAIT on an AP read then looks like.
            self.trailing_turnaround();
            return ack;
        }
        let data = self.read_data_phase();
        ack | data << 3
    }

    /// Returns the bus to its resting state at the end of a transfer.
    ///
    /// Between two transfers there are no clock edges, so the level a backend
    /// leaves on SWDIO is not observable by the target; a backend that would
    /// have to change engines to set it may skip doing so.
    fn end_transfer(&mut self)
    where
        Self: Sized,
    {
        self.set_swdio_output();
        self.write_swdio(true);
    }

    /// Clocks a read transfer's 32 data bits, its parity bit, and the
    /// turnaround that follows, returning data in bits 0..32.
    ///
    /// One call rather than a data read followed by a turnaround read, because
    /// on a hardware backend each is a separate peripheral transaction and the
    /// sequencing costs more than the clocks do.
    fn read_data_phase(&mut self) -> u64
    where
        Self: Sized,
    {
        let sampled = self.read_bits(33);
        self.trailing_turnaround();
        sampled
    }

    /// Clocks the turnaround period that hands the bus back to the host,
    /// after a read's data phase or before a write's.
    ///
    /// A backend that borrows a clock at `leading_turnaround` repays it here.
    fn trailing_turnaround(&mut self)
    where
        Self: Sized,
    {
        self.read_bits(1);
    }

    /// Clocks out `count` bits taken from `bytes`, least significant bit of
    /// the first byte first, while SWDIO is already an output.
    fn write_sequence(&mut self, bytes: &[u8], count: usize)
    where
        Self: Sized,
    {
        let mut written = 0;
        while written < count {
            let burst = (count - written).min(usize::from(MAX_BURST_BITS));
            let mut bits = 0u64;
            for index in 0..burst {
                let bit = written + index;
                bits |= u64::from(bytes[bit / 8] >> (bit % 8) & 1) << index;
            }
            self.write_bits(bits, burst as u8);
            written += burst;
        }
    }

    /// Executes one request/ACK/data transaction without interrupt-created
    /// malformed clock edges.
    fn transaction<R>(&mut self, operation: impl FnOnce(&mut Self) -> R) -> R {
        operation(self)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidDpId,
    PowerTimeout,
    WaitTimeout,
    Fault,
    Protocol(u8),
    Parity,
    Unaligned,
    LineHeldLow,
}

pub trait WordMemory {
    fn read_word(&mut self, address: u32) -> Result<u32, Error>;
    fn write_word(&mut self, address: u32, value: u32) -> Result<(), Error>;
    fn read_words(&mut self, address: u32, values: &mut [u32]) -> Result<(), Error>;
    fn write_words(&mut self, address: u32, values: &[u32]) -> Result<(), Error>;
    fn delay_ms(&mut self, milliseconds: u16);
}

pub struct SwdLink<I> {
    io: I,
    selected: Option<u32>,
    csw_written: bool,
    retries: u16,
}

impl<I: SwdIo> SwdLink<I> {
    pub fn new(io: I) -> Self {
        Self {
            io,
            selected: None,
            csw_written: false,
            retries: 100,
        }
    }

    pub fn initialize(&mut self) -> Result<u32, Error> {
        let (swdio, _) = self.released_line_state();
        if !swdio {
            return Err(Error::LineHeldLow);
        }
        self.line_reset_and_switch();
        self.initialize_prepared()
    }

    pub fn initialize_prepared(&mut self) -> Result<u32, Error> {
        let idcode = self.dp_read(DP_IDCODE)?;
        if idcode == 0 || idcode == u32::MAX || idcode & 1 == 0 {
            self.disconnect();
            return Err(Error::InvalidDpId);
        }
        self.dp_write(DP_ABORT, ABORT_ALL)?;
        self.dp_write(DP_SELECT, 0)?;
        self.selected = Some(0);
        self.dp_write(DP_CTRL_STAT_WRITE, POWER_REQUEST)?;
        for _ in 0..100 {
            if self.dp_read(DP_CTRL_STAT_READ)? & POWER_ACK == POWER_ACK {
                return Ok(idcode);
            }
            self.io.delay_ms(1);
        }
        self.disconnect();
        Err(Error::PowerTimeout)
    }

    pub fn disconnect(&mut self) {
        self.io.release();
        self.selected = None;
        self.csw_written = false;
    }

    pub fn line_reset_and_switch(&mut self) {
        self.sequence(56, &[0xff; 7]);
        self.sequence(16, &[0x9e, 0xe7]);
        self.sequence(56, &[0xff; 7]);
        self.sequence(8, &[0x00]);
        self.selected = None;
        self.csw_written = false;
    }

    pub fn swj_sequence(&mut self, bit_len: u8, bits: u64) -> Result<(), Error> {
        if bit_len == 0 || bit_len > 64 {
            return Err(Error::Protocol(0));
        }
        self.sequence(usize::from(bit_len), &bits.to_le_bytes());
        Ok(())
    }

    /// Clocks the bus with SWDIO released, then reports the released levels.
    ///
    /// A target interrupted part-way through a read data phase keeps driving
    /// SWDIO until it has clocked out the rest of that phase, which reads as a
    /// permanently held-low line — and the held-low guard then refuses the very
    /// line reset that would clear it. Only SWCLK is driven here, so this can
    /// never contend with whatever the far end is doing.
    pub fn recover_line(&mut self, cycles: u16) -> (bool, bool) {
        self.disconnect();
        self.io.transaction(|io| {
            io.set_swdio_input();
            io.set_swclk_output();
            let mut remaining = cycles;
            while remaining > 0 {
                let burst = remaining.min(u16::from(MAX_BURST_BITS)) as u8;
                io.read_bits(burst);
                remaining -= u16::from(burst);
            }
        });
        self.released_line_state()
    }

    pub fn released_line_state(&mut self) -> (bool, bool) {
        self.disconnect();
        self.io.sample_lines()
    }

    pub fn pad_self_test(&mut self) -> [bool; 4] {
        let levels = self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_swdio(false);
            io.write_swclk(false);
            io.delay_half_cycle();
            let (swdio_low, swclk_low) = io.sample_lines();
            io.write_swdio(true);
            io.write_swclk(true);
            io.delay_half_cycle();
            let (swdio_high, swclk_high) = io.sample_lines();
            [swdio_low, swdio_high, swclk_low, swclk_high]
        });
        self.disconnect();
        levels
    }

    /// Hold SWCLK low and drive SWDIO to a known level for a slow physical
    /// continuity measurement. The caller must hold the target in reset and
    /// call `disconnect` when the measurement is complete.
    pub fn diagnostic_drive_swdio(&mut self, high: bool) -> bool {
        self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_swclk(false);
            io.write_swdio(high);
            io.delay_half_cycle();
            io.read_swdio()
        })
    }

    /// Drive PA14/SWCLK as BOOT0 while the target is held in reset.
    pub fn diagnostic_drive_boot0(&mut self, high: bool) {
        self.io.transaction(|io| {
            io.set_swdio_input();
            io.write_swclk(high);
            io.set_swclk_output();
            io.delay_half_cycle();
        });
    }

    /// Sends a DPIDR read and samples the whole reply as one undecoded field.
    ///
    /// Reading IDCODE has no side effects, so over-clocking past the data phase
    /// is safe, and it is the only way to see where a backend actually places
    /// the ACK and data bits rather than where it is assumed to.
    /// Runs read, write, read against the DP with an explicit number of
    /// turnaround clocks at each direction handover, and reports CTRL/STAT.
    ///
    /// The write under test targets CTRL/STAT rather than ABORT: ABORT is
    /// accepted without the data-parity check and clears the very error bit
    /// that would reveal a bad data phase, so it cannot detect this at all.
    /// CTRL/STAT is checked, readable, and leaves WDATAERR standing.
    pub fn handover_probe(&mut self, after_read: u8, before_write: u8) -> (u8, u8, u8, u32) {
        self.line_reset_and_switch();
        // After a line reset the DP answers nothing but a DPIDR read.
        let (first_ack, _) = self.read_with_handover(DP_IDCODE, after_read);
        let second_ack = self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_bits(u64::from(packet_request(DP_CTRL_STAT_WRITE)), 8);
            io.set_swdio_input();
            io.leading_turnaround();
            let ack = io.read_bits(3) as u8;
            for _ in 0..before_write {
                io.read_bits(1);
            }
            io.set_swdio_output();
            io.write_bits(
                u64::from(POWER_REQUEST) | u64::from(POWER_REQUEST.count_ones() & 1) << 32,
                33,
            );
            io.write_swdio(true);
            ack
        });
        let (third_ack, status) = self.read_with_handover(DP_CTRL_STAT_READ, after_read);
        (first_ack, second_ack, third_ack, status)
    }

    fn read_with_handover(&mut self, request: u8, after_read: u8) -> (u8, u32) {
        self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_bits(u64::from(packet_request(request)), 8);
            io.set_swdio_input();
            io.leading_turnaround();
            let ack = io.read_bits(3) as u8;
            let data = io.read_bits(33) as u32;
            for _ in 0..after_read {
                io.read_bits(1);
            }
            io.set_swdio_output();
            io.write_swdio(true);
            (ack, data)
        })
    }

    /// Brings the DP up, then performs exactly one AP write with no retry,
    /// reporting the raw ACK and CTRL/STAT on either side of it.
    ///
    /// Distinguishes the two ways WDATAERR can arise: a bad data phase, or a
    /// write the DP accepted and then discarded. A retry loop hides the
    /// difference, so this deliberately does not have one.
    pub fn ap_write_probe(&mut self, value: u32) -> (u8, u8, u32, u32) {
        self.ap_write_handover_probe(value, None, None)
    }

    /// As `ap_write_probe`, but with the read-to-write boundary parameterised.
    ///
    /// `after_read` replaces the clocks the engine would issue after a read's
    /// data phase, and `before_write` those between a write's ACK and its data
    /// phase. `None` leaves the engine's own choice in place.
    pub fn ap_write_handover_probe(
        &mut self,
        value: u32,
        after_read: Option<u8>,
        before_write: Option<u8>,
    ) -> (u8, u8, u32, u32) {
        self.line_reset_and_switch();
        // Bring the DP up with the engine's own framing, which is known to
        // work: only the final read-then-write pair is under test.
        let mut scratch = 0;
        let dpidr_ack = self
            .io
            .transaction(|io| wire_transfer(io, DP_IDCODE, &mut scratch))
            .unwrap_or(0);
        for (request, mut data) in [
            (DP_ABORT, ABORT_ALL),
            (DP_SELECT, 0),
            (DP_CTRL_STAT_WRITE, POWER_REQUEST),
        ] {
            let _ = self
                .io
                .transaction(|io| wire_transfer(io, request, &mut data));
        }
        for _ in 0..100 {
            let mut status = 0;
            let _ = self
                .io
                .transaction(|io| wire_transfer(io, DP_CTRL_STAT_READ, &mut status));
            if status & POWER_ACK == POWER_ACK {
                break;
            }
            self.io.delay_ms(1);
        }

        // The read whose trailing boundary is under test.
        self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_bits(u64::from(packet_request(DP_CTRL_STAT_READ)), 8);
            io.set_swdio_input();
            io.leading_turnaround();
            let _ = io.read_bits(3);
            let _ = io.read_bits(33);
            match after_read {
                Some(clocks) => {
                    io.read_bits(clocks);
                }
                None => io.trailing_turnaround(),
            }
            io.set_swdio_output();
            io.write_swdio(true);
        });

        // TAR rather than CSW: it is fully readable, so whatever the target
        // actually received can be recovered over the read path, which is
        // known to be bit-exact. A parity verdict alone cannot show that.
        let write_ack = self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_bits(u64::from(packet_request(AP_TAR)), 8);
            io.set_swdio_input();
            io.leading_turnaround();
            let ack = io.read_bits(3) as u8;
            match before_write {
                Some(clocks) => {
                    io.read_bits(clocks);
                }
                None => io.trailing_turnaround(),
            }
            io.set_swdio_output();
            io.write_bits(
                u64::from(value) | u64::from(value.count_ones() & 1) << 32,
                33,
            );
            io.write_swdio(true);
            ack
        });

        // CTRL/STAT reads still answer while a sticky error stands.
        let mut after = 0;
        let _ = self
            .io
            .transaction(|io| wire_transfer(io, DP_CTRL_STAT_READ, &mut after));
        // Clear it so the readback itself is not refused.
        let mut abort = ABORT_ALL;
        let _ = self
            .io
            .transaction(|io| wire_transfer(io, DP_ABORT, &mut abort));
        let mut posted = 0;
        let _ = self
            .io
            .transaction(|io| wire_transfer(io, AP_TAR | TRANSFER_READ, &mut posted));
        let mut readback = 0;
        let _ = self
            .io
            .transaction(|io| wire_transfer(io, DP_RDBUFF, &mut readback));
        (dpidr_ack, write_ack, readback, after)
    }

    pub fn wire_probe(&mut self, split: bool) -> (u64, u64) {
        self.line_reset_and_switch();
        self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_bits(u64::from(packet_request(DP_IDCODE)), 8);
            io.set_swdio_input();
            io.leading_turnaround();
            // `split` mirrors how a real transfer reads the reply: a short ACK
            // field first, then the data field as a second burst. A backend
            // that places short reads differently from long ones shows up as a
            // disagreement between the two forms.
            let sampled = if split {
                (io.read_bits(3), io.read_bits(33))
            } else {
                (io.read_bits(48), 0)
            };
            io.set_swdio_output();
            io.write_swdio(true);
            sampled
        })
    }

    pub fn raw_read_register(&mut self, access_port: bool, address: u8) -> Result<u32, Error> {
        if address & !0x0c != 0 {
            return Err(Error::Protocol(address));
        }
        let request =
            (if access_port { TRANSFER_AP } else { 0 }) | TRANSFER_READ | (address & 0x0c);
        let mut value = 0;
        self.transfer(request, &mut value)?;
        if access_port {
            self.dp_read(DP_RDBUFF)
        } else {
            Ok(value)
        }
    }

    /// Reads one register repeatedly without a host round trip per word.
    ///
    /// AP reads are posted: the value requested by one transfer is returned by
    /// the next, so a block of `n` words costs `n + 1` transfers instead of the
    /// `2n` a naive read-then-RDBUFF loop would need.
    pub fn raw_read_register_block(
        &mut self,
        access_port: bool,
        address: u8,
        values: &mut [u32],
    ) -> Result<(), Error> {
        if address & !0x0c != 0 {
            return Err(Error::Protocol(address));
        }
        if values.is_empty() {
            return Ok(());
        }
        let request =
            (if access_port { TRANSFER_AP } else { 0 }) | TRANSFER_READ | (address & 0x0c);
        if !access_port {
            for value in values.iter_mut() {
                self.transfer(request, value)?;
            }
            return Ok(());
        }

        let mut posted = 0;
        self.transfer(request, &mut posted)?;
        let last = values.len() - 1;
        for value in values.iter_mut().take(last) {
            self.transfer(request, value)?;
        }
        values[last] = self.dp_read(DP_RDBUFF)?;
        Ok(())
    }

    /// Writes one register repeatedly without a host round trip per word.
    ///
    /// Writes are not posted, so this is a plain loop; the saving is entirely
    /// in the transport, which is what dominates a flash download.
    pub fn raw_write_register_block(
        &mut self,
        access_port: bool,
        address: u8,
        values: &[u32],
    ) -> Result<(), Error> {
        if address & !0x0c != 0 {
            return Err(Error::Protocol(address));
        }
        let request = (if access_port { TRANSFER_AP } else { 0 }) | (address & 0x0c);
        for value in values {
            let mut value = *value;
            self.transfer(request, &mut value)?;
        }
        Ok(())
    }

    pub fn raw_write_register(
        &mut self,
        access_port: bool,
        address: u8,
        value: u32,
    ) -> Result<(), Error> {
        if address & !0x0c != 0 {
            return Err(Error::Protocol(address));
        }
        let request = (if access_port { TRANSFER_AP } else { 0 }) | (address & 0x0c);
        let mut value = value;
        self.transfer(request, &mut value)
    }

    /// Borrows the electrical backend for pad-level configuration such as wire
    /// speed or pin mapping.
    pub fn io_mut(&mut self) -> &mut I {
        &mut self.io
    }

    pub fn into_io(mut self) -> I {
        self.disconnect();
        self.io
    }

    fn sequence(&mut self, bits: usize, bytes: &[u8]) {
        self.io.transaction(|io| {
            io.set_swdio_output();
            io.write_sequence(bytes, bits);
        });
    }

    fn transfer(&mut self, request: u8, data: &mut u32) -> Result<(), Error> {
        for _ in 0..=self.retries {
            let status = self.io.transaction(|io| wire_transfer(io, request, data));
            match status {
                Ok(TRANSFER_OK) => return Ok(()),
                Ok(TRANSFER_WAIT) => continue,
                Ok(TRANSFER_FAULT) => {
                    let mut abort = ABORT_ALL;
                    let _ = self
                        .io
                        .transaction(|io| wire_transfer(io, DP_ABORT, &mut abort));
                    return Err(Error::Fault);
                }
                Ok(other) => return Err(Error::Protocol(other)),
                Err(error) => return Err(error),
            }
        }
        Err(Error::WaitTimeout)
    }

    fn dp_read(&mut self, request: u8) -> Result<u32, Error> {
        let mut value = 0;
        self.transfer(request, &mut value)?;
        Ok(value)
    }

    fn dp_write(&mut self, request: u8, value: u32) -> Result<(), Error> {
        let mut value = value;
        self.transfer(request, &mut value)
    }

    fn ap_write(&mut self, request: u8, value: u32) -> Result<(), Error> {
        self.dp_write(request, value)
    }

    fn select_bank_zero(&mut self) -> Result<(), Error> {
        if self.selected != Some(0) {
            self.dp_write(DP_SELECT, 0)?;
            self.selected = Some(0);
            self.csw_written = false;
        }
        if !self.csw_written {
            self.ap_write(AP_CSW, CORTEX_M_WORD_CSW)?;
            self.csw_written = true;
        }
        Ok(())
    }
}

impl<I: SwdIo> WordMemory for SwdLink<I> {
    fn read_word(&mut self, address: u32) -> Result<u32, Error> {
        let mut value = [0];
        self.read_words(address, &mut value)?;
        Ok(value[0])
    }

    fn write_word(&mut self, address: u32, value: u32) -> Result<(), Error> {
        self.write_words(address, &[value])
    }

    fn read_words(&mut self, address: u32, values: &mut [u32]) -> Result<(), Error> {
        if address & 3 != 0 {
            return Err(Error::Unaligned);
        }
        self.select_bank_zero()?;
        let mut offset = 0;
        while offset < values.len() {
            let start = address + (offset as u32) * 4;
            let span = auto_increment_words(start).min(values.len() - offset);
            self.ap_write(AP_TAR, start)?;
            // `raw_read_register_block` re-derives the AP and read bits, so it
            // takes the bare register address.
            self.raw_read_register_block(
                true,
                AP_DRW_READ & 0x0c,
                &mut values[offset..offset + span],
            )?;
            offset += span;
        }
        Ok(())
    }

    fn write_words(&mut self, address: u32, values: &[u32]) -> Result<(), Error> {
        if address & 3 != 0 {
            return Err(Error::Unaligned);
        }
        self.select_bank_zero()?;
        let mut offset = 0;
        while offset < values.len() {
            let start = address + (offset as u32) * 4;
            let span = auto_increment_words(start).min(values.len() - offset);
            self.ap_write(AP_TAR, start)?;
            for value in &values[offset..offset + span] {
                self.ap_write(AP_DRW_WRITE, *value)?;
            }
            offset += span;
        }
        let _ = self.dp_read(DP_RDBUFF)?;
        Ok(())
    }

    fn delay_ms(&mut self, milliseconds: u16) {
        self.io.delay_ms(milliseconds);
    }
}

/// Words addressable from `address` before the MEM-AP auto-increment wraps.
///
/// ADIv5 only guarantees auto-increment within a 1 KiB region, so every block
/// access has to re-write TAR at each boundary.
fn auto_increment_words(address: u32) -> usize {
    const AUTO_INCREMENT_BYTES: u32 = 1024;
    ((AUTO_INCREMENT_BYTES - (address % AUTO_INCREMENT_BYTES)) / 4) as usize
}

/// Packs a host packet request: start, the four request bits, parity, stop,
/// and park, in transmission order.
fn packet_request(request: u8) -> u8 {
    let payload = request & 0x0f;
    1 | (payload << 1) | ((payload.count_ones() as u8 & 1) << 5) | 0x80
}

fn wire_transfer<I: SwdIo>(io: &mut I, request: u8, data: &mut u32) -> Result<u8, Error> {
    io.set_swdio_output();
    io.write_bits(u64::from(packet_request(request)), 8);

    io.set_swdio_input();
    io.leading_turnaround();

    if request & TRANSFER_READ != 0 {
        let reply = io.read_reply(TRANSFER_OK);
        let ack = reply as u8 & 0b111;
        if ack != TRANSFER_OK {
            io.end_transfer();
            return Ok(ack);
        }
        let value = (reply >> 3) as u32;
        let observed_parity = reply >> 35 & 1 != 0;
        io.end_transfer();
        if (value.count_ones() & 1 != 0) != observed_parity {
            return Err(Error::Parity);
        }
        *data = value;
        return Ok(ack);
    }

    let ack = io.read_bits(3) as u8 & 0b111;
    if ack != TRANSFER_OK {
        io.trailing_turnaround();
        io.end_transfer();
        return Ok(ack);
    }

    {
        io.trailing_turnaround();
        io.set_swdio_output();
        io.write_data_phase(*data, data.count_ones() & 1 != 0);
        io.end_transfer();
    }
    Ok(ack)
}

fn write_bit<I: SwdIo>(io: &mut I, high: bool) {
    io.write_swdio(high);
    clock(io);
}

fn read_bit<I: SwdIo>(io: &mut I) -> bool {
    // Match the proven CMSIS-DAP ESP32-C3 GPIO backend: create the falling
    // edge first, sample near the end of the low phase, and finish high.
    io.write_swclk(false);
    io.delay_half_cycle();
    let value = io.read_swdio();
    io.write_swclk(true);
    io.delay_half_cycle();
    value
}

fn clock<I: SwdIo>(io: &mut I) {
    io.write_swclk(false);
    io.delay_half_cycle();
    io.write_swclk(true);
    io.delay_half_cycle();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[derive(Default)]
    struct MockIo {
        samples: VecDeque<bool>,
        released: bool,
        transactions: usize,
        line_state: (bool, bool),
    }

    impl SwdIo for MockIo {
        fn set_swdio_output(&mut self) {}
        fn set_swdio_input(&mut self) {}
        fn set_swclk_output(&mut self) {}
        fn write_swdio(&mut self, _high: bool) {}
        fn read_swdio(&mut self) -> bool {
            self.samples.pop_front().unwrap_or(true)
        }
        fn write_swclk(&mut self, _high: bool) {}
        fn delay_half_cycle(&mut self) {}
        fn delay_ms(&mut self, _milliseconds: u16) {}
        fn release(&mut self) {
            self.released = true;
        }
        fn sample_lines(&mut self) -> (bool, bool) {
            self.line_state
        }
        fn transaction<R>(&mut self, operation: impl FnOnce(&mut Self) -> R) -> R {
            self.transactions += 1;
            operation(self)
        }
    }

    #[derive(Default)]
    struct TimingIo {
        events: Vec<&'static str>,
    }

    impl SwdIo for TimingIo {
        fn set_swdio_output(&mut self) {}
        fn set_swdio_input(&mut self) {}
        fn set_swclk_output(&mut self) {}
        fn write_swdio(&mut self, high: bool) {
            self.events.push(if high { "dio1" } else { "dio0" });
        }
        fn read_swdio(&mut self) -> bool {
            self.events.push("read");
            true
        }
        fn write_swclk(&mut self, high: bool) {
            self.events.push(if high { "clk1" } else { "clk0" });
        }
        fn delay_half_cycle(&mut self) {
            self.events.push("delay");
        }
        fn delay_ms(&mut self, _milliseconds: u16) {}
        fn release(&mut self) {}
        fn sample_lines(&mut self) -> (bool, bool) {
            (false, false)
        }
    }

    #[test]
    fn bit_primitives_change_and_sample_data_while_clock_is_low() {
        let mut output = TimingIo::default();
        write_bit(&mut output, true);
        assert_eq!(output.events, ["dio1", "clk0", "delay", "clk1", "delay"]);

        let mut input = TimingIo::default();
        assert!(read_bit(&mut input));
        assert_eq!(input.events, ["clk0", "delay", "read", "clk1", "delay"]);
    }

    #[test]
    fn wire_read_accepts_odd_parity_and_returns_lsb_first_word() {
        let expected = 0xa55a_1234u32;
        // Turnaround period, then ACK OK least significant bit first.
        let mut samples = VecDeque::from([false, true, false, false]);
        samples.extend((0..32).map(|bit| expected >> bit & 1 != 0));
        samples.push_back(expected.count_ones() & 1 != 0);
        let mut io = MockIo {
            samples,
            ..Default::default()
        };
        let mut observed = 0;

        assert_eq!(
            wire_transfer(&mut io, DP_IDCODE, &mut observed),
            Ok(TRANSFER_OK)
        );
        assert_eq!(observed, expected);
    }

    #[test]
    fn wire_read_rejects_bad_parity() {
        let expected = 0x0000_0001u32;
        // Turnaround period, then ACK OK least significant bit first.
        let mut samples = VecDeque::from([false, true, false, false]);
        samples.extend((0..32).map(|bit| expected >> bit & 1 != 0));
        samples.push_back(false);
        let mut io = MockIo {
            samples,
            ..Default::default()
        };

        assert_eq!(
            wire_transfer(&mut io, DP_IDCODE, &mut 0),
            Err(Error::Parity)
        );
    }

    #[test]
    fn packet_request_matches_the_bit_by_bit_encoding() {
        for request in 0..16u8 {
            let expected = 1u64
                | u64::from(request & 1) << 1
                | u64::from(request >> 1 & 1) << 2
                | u64::from(request >> 2 & 1) << 3
                | u64::from(request >> 3 & 1) << 4
                | u64::from(request.count_ones() as u8 & 1) << 5
                | 1 << 7;
            assert_eq!(u64::from(packet_request(request)), expected);
        }
    }

    #[test]
    fn default_bit_bursts_are_least_significant_bit_first() {
        let mut io = TimingIo::default();
        io.write_bits(0b10, 2);
        assert_eq!(
            io.events,
            [
                "dio0", "clk0", "delay", "clk1", "delay", "dio1", "clk0", "delay", "clk1", "delay"
            ]
        );

        let mut sequence = TimingIo::default();
        sequence.write_sequence(&[0x00, 0x01], 9);
        assert_eq!(sequence.events.iter().filter(|e| **e == "dio1").count(), 1);
        assert_eq!(sequence.events.iter().filter(|e| **e == "clk1").count(), 9);
    }

    #[test]
    fn auto_increment_stops_at_each_kibibyte_boundary() {
        assert_eq!(auto_increment_words(0x0800_0000), 256);
        assert_eq!(auto_increment_words(0x0800_03fc), 1);
        assert_eq!(auto_increment_words(0x0800_0400), 256);
    }

    #[test]
    fn block_reads_of_an_access_port_use_one_posted_transfer_per_word() {
        // ACK plus 32 data bits and parity for every posted response.
        let mut samples = VecDeque::new();
        for value in [0u32, 0x1111_1111, 0x2222_2222, 0x3333_3333] {
            samples.extend([false, true, false, false]);
            samples.extend((0..32).map(|bit| value >> bit & 1 != 0));
            samples.push_back(value.count_ones() & 1 != 0);
            // Trailing turnaround period before the line is driven again.
            samples.push_back(false);
        }
        let mut link = SwdLink::new(MockIo {
            samples,
            ..Default::default()
        });

        let mut values = [0u32; 3];
        assert_eq!(
            link.raw_read_register_block(true, 0x0c, &mut values),
            Ok(())
        );
        assert_eq!(values, [0x1111_1111, 0x2222_2222, 0x3333_3333]);
        // One priming transfer, one per word, and RDBUFF for the last value.
        assert_eq!(link.into_io().transactions, 4);
    }

    #[test]
    fn disconnect_releases_programming_pins() {
        let mut link = SwdLink::new(MockIo::default());
        link.disconnect();
        assert!(link.into_io().released);
    }

    #[test]
    fn initialize_refuses_to_drive_an_externally_low_swdio_line() {
        let mut link = SwdLink::new(MockIo {
            line_state: (false, false),
            ..Default::default()
        });

        assert_eq!(link.initialize(), Err(Error::LineHeldLow));
        let io = link.into_io();
        assert!(io.released);
        assert_eq!(io.transactions, 0);
    }
}
