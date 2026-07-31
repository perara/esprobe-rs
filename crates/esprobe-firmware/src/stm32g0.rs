//! Fail-closed STM32G030K8 flash programming over ADIv5 memory access.

use crate::swd::{Error as SwdError, WordMemory};

pub const FLASH_BASE: u32 = 0x0800_0000;
pub const FLASH_BYTES: usize = 64 * 1024;
pub const PAGE_BYTES: usize = 2 * 1024;
pub const DEVICE_ID: u16 = 0x0466;

const DBGMCU_IDCODE: u32 = 0x4001_5800;
const DHCSR: u32 = 0xe000_edf0;
const AIRCR: u32 = 0xe000_ed0c;
const FLASH_REGS: u32 = 0x4002_2000;
const FLASH_KEYR: u32 = FLASH_REGS + 0x08;
const FLASH_SR: u32 = FLASH_REGS + 0x10;
const FLASH_CR: u32 = FLASH_REGS + 0x14;

const DHCSR_HALT: u32 = 0xa05f_0003;
const DHCSR_RUN: u32 = 0xa05f_0001;
const DHCSR_S_HALT: u32 = 1 << 17;
const AIRCR_SYSRESETREQ: u32 = 0x05fa_0004;
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xcdef_89ab;
const SR_EOP: u32 = 1;
const SR_ERRORS: u32 = 0x0000_c3fa;
const SR_BSY1: u32 = 1 << 16;
const CR_PG: u32 = 1;
const CR_PER: u32 = 1 << 1;
const CR_PNB_SHIFT: u32 = 3;
const CR_PNB_MASK: u32 = 0x3ff << CR_PNB_SHIFT;
const CR_STRT: u32 = 1 << 16;
const CR_LOCK: u32 = 1 << 31;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Transport(SwdError),
    Empty,
    TooLarge,
    InvalidDevice(u16),
    HaltTimeout,
    UnlockFailed,
    BusyTimeout,
    FlashStatus(u32),
    Verify {
        address: u32,
        expected: u32,
        observed: u32,
    },
}

pub struct Stm32G030<'a, M> {
    memory: &'a mut M,
}

impl<'a, M: WordMemory> Stm32G030<'a, M> {
    pub fn new(memory: &'a mut M) -> Self {
        Self { memory }
    }

    pub fn identify(&mut self) -> Result<u32, Error> {
        let idcode = self.read(DBGMCU_IDCODE)?;
        let device = (idcode & 0x0fff) as u16;
        if device != DEVICE_ID {
            return Err(Error::InvalidDevice(device));
        }
        Ok(idcode)
    }

    pub fn program_and_verify(&mut self, image: &[u8]) -> Result<u32, Error> {
        if image.is_empty() {
            return Err(Error::Empty);
        }
        if image.len() > FLASH_BYTES {
            return Err(Error::TooLarge);
        }
        self.halt()?;
        let idcode = self.identify()?;
        self.unlock()?;

        let result = self.program_unlocked(image);
        let cleanup = self.finish_flash_operation();
        match (result, cleanup) {
            (Err(error), _) => Err(error),
            (Ok(()), Err(error)) => Err(error),
            (Ok(()), Ok(())) => Ok(idcode),
        }
    }

    fn program_unlocked(&mut self, image: &[u8]) -> Result<(), Error> {
        let erase_bytes = image.len().div_ceil(PAGE_BYTES) * PAGE_BYTES;
        for page in 0..erase_bytes / PAGE_BYTES {
            self.erase_page(page as u32)?;
        }

        let mut address = FLASH_BASE;
        for chunk in image.chunks(8) {
            let mut bytes = [0xff; 8];
            bytes[..chunk.len()].copy_from_slice(chunk);
            self.program_double_word(address, u64::from_le_bytes(bytes))?;
            address += 8;
        }

        address = FLASH_BASE;
        for chunk in image.chunks(4) {
            let mut bytes = [0xff; 4];
            bytes[..chunk.len()].copy_from_slice(chunk);
            let expected = u32::from_le_bytes(bytes);
            let observed = self.read(address)?;
            if observed != expected {
                return Err(Error::Verify {
                    address,
                    expected,
                    observed,
                });
            }
            address += 4;
        }
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.write(DHCSR, DHCSR_RUN)?;
        self.write(AIRCR, AIRCR_SYSRESETREQ)
    }

    pub fn halt(&mut self) -> Result<(), Error> {
        self.write(DHCSR, DHCSR_HALT)?;
        for _ in 0..250 {
            if self.read(DHCSR)? & DHCSR_S_HALT != 0 {
                return Ok(());
            }
            self.memory.delay_ms(1);
        }
        Err(Error::HaltTimeout)
    }

    fn unlock(&mut self) -> Result<(), Error> {
        if self.read(FLASH_CR)? & CR_LOCK == 0 {
            return Ok(());
        }
        self.write(FLASH_KEYR, KEY1)?;
        self.write(FLASH_KEYR, KEY2)?;
        if self.read(FLASH_CR)? & CR_LOCK != 0 {
            return Err(Error::UnlockFailed);
        }
        Ok(())
    }

    fn finish_flash_operation(&mut self) -> Result<(), Error> {
        let control = self.read(FLASH_CR)?;
        let cleaned = control & !(CR_PG | CR_PER | CR_PNB_MASK | CR_STRT);
        self.write(FLASH_CR, cleaned | CR_LOCK)
    }

    fn erase_page(&mut self, page: u32) -> Result<(), Error> {
        self.wait_ready()?;
        self.clear_status()?;
        let mut control = self.read(FLASH_CR)? & !(CR_PG | CR_PER | CR_PNB_MASK);
        control |= CR_PER | (page << CR_PNB_SHIFT);
        self.write(FLASH_CR, control)?;
        self.write(FLASH_CR, control | CR_STRT)?;
        self.wait_ready()?;
        self.write(FLASH_CR, control & !CR_PER)?;
        Ok(())
    }

    fn program_double_word(&mut self, address: u32, value: u64) -> Result<(), Error> {
        self.wait_ready()?;
        self.clear_status()?;
        let control = (self.read(FLASH_CR)? & !(CR_PER | CR_PG | CR_PNB_MASK)) | CR_PG;
        self.write(FLASH_CR, control)?;
        self.memory
            .write_words(address, &[value as u32, (value >> 32) as u32])
            .map_err(Error::Transport)?;
        self.wait_ready()?;
        self.write(FLASH_CR, control & !CR_PG)?;
        Ok(())
    }

    fn wait_ready(&mut self) -> Result<(), Error> {
        for _ in 0..2_000 {
            let status = self.read(FLASH_SR)?;
            if status & SR_BSY1 == 0 {
                if status & SR_ERRORS != 0 {
                    return Err(Error::FlashStatus(status));
                }
                if status & SR_EOP != 0 {
                    self.write(FLASH_SR, SR_EOP)?;
                }
                return Ok(());
            }
            self.memory.delay_ms(1);
        }
        Err(Error::BusyTimeout)
    }

    fn clear_status(&mut self) -> Result<(), Error> {
        self.write(FLASH_SR, SR_EOP | SR_ERRORS)
    }

    fn read(&mut self, address: u32) -> Result<u32, Error> {
        self.memory.read_word(address).map_err(Error::Transport)
    }

    fn write(&mut self, address: u32, value: u32) -> Result<(), Error> {
        self.memory
            .write_word(address, value)
            .map_err(Error::Transport)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct MockMemory {
        flash_control: u32,
        unlocked: bool,
        fail_program_write: bool,
        writes: Vec<(u32, u32)>,
    }

    impl WordMemory for MockMemory {
        fn read_word(&mut self, address: u32) -> Result<u32, SwdError> {
            Ok(match address {
                DBGMCU_IDCODE => u32::from(DEVICE_ID),
                DHCSR => DHCSR_S_HALT,
                FLASH_CR if self.unlocked => self.flash_control & !CR_LOCK,
                FLASH_CR => self.flash_control | CR_LOCK,
                FLASH_SR => 0,
                _ => u32::MAX,
            })
        }

        fn write_word(&mut self, address: u32, value: u32) -> Result<(), SwdError> {
            self.writes.push((address, value));
            if address == FLASH_KEYR && value == KEY2 {
                self.unlocked = true;
            }
            if address == FLASH_CR {
                self.flash_control = value;
            }
            Ok(())
        }

        fn read_words(&mut self, address: u32, values: &mut [u32]) -> Result<(), SwdError> {
            for (offset, value) in values.iter_mut().enumerate() {
                *value = self.read_word(address + offset as u32 * 4)?;
            }
            Ok(())
        }

        fn write_words(&mut self, address: u32, values: &[u32]) -> Result<(), SwdError> {
            if self.fail_program_write && address >= FLASH_BASE {
                return Err(SwdError::Fault);
            }
            for (offset, value) in values.iter().enumerate() {
                self.write_word(address + offset as u32 * 4, *value)?;
            }
            Ok(())
        }

        fn delay_ms(&mut self, _milliseconds: u16) {}
    }

    #[test]
    fn rejects_images_outside_supported_flash_range_without_touching_target() {
        let mut memory = MockMemory::default();
        let mut target = Stm32G030::new(&mut memory);

        assert_eq!(target.program_and_verify(&[]), Err(Error::Empty));
        assert_eq!(
            target.program_and_verify(&vec![0; FLASH_BYTES + 1]),
            Err(Error::TooLarge)
        );
        assert!(memory.writes.is_empty());
    }

    #[test]
    fn programming_failure_clears_operation_bits_and_relocks_flash() {
        let mut memory = MockMemory {
            fail_program_write: true,
            ..Default::default()
        };
        let mut target = Stm32G030::new(&mut memory);

        assert_eq!(
            target.program_and_verify(&[0x12, 0x34]),
            Err(Error::Transport(SwdError::Fault))
        );
        let final_control = memory
            .writes
            .iter()
            .rev()
            .find_map(|(address, value)| (*address == FLASH_CR).then_some(*value))
            .expect("flash control cleanup write");
        assert_eq!(final_control & (CR_PG | CR_PER | CR_PNB_MASK), 0);
        assert_ne!(final_control & CR_LOCK, 0);
    }
}
