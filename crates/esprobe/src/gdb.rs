//! A GDB server, so a debugger can drive the target through this bridge.
//!
//! probe-rs has a GDB server of its own, but it lives in `probe-rs-tools`,
//! which publishes no library target — it is binaries only. Nothing there can
//! be reused, so this implements the protocol side with `gdbstub` and maps it
//! onto `probe_rs::Core`, which is the same core the `core` subcommand drives.
//!
//! What that buys is the part of "debug support" nothing else here provides:
//! source-level stepping, breakpoints that persist across resumes, and any
//! front end that speaks the GDB remote protocol — `gdb` itself, VS Code's
//! `cortex-debug`, CLion — attached to a target over USB or Wi-Fi.

use std::net::{TcpListener, TcpStream};

use anyhow::{Context as _, Result};
use gdbstub::common::Signal;
use gdbstub::conn::ConnectionExt;
use gdbstub::stub::run_blocking::{BlockingEventLoop, Event, WaitForStopReasonError};
use gdbstub::stub::{DisconnectReason, GdbStub, SingleThreadStopReason};
use gdbstub::target::ext::base::BaseOps;
use gdbstub::target::ext::base::single_register_access::{
    SingleRegisterAccess, SingleRegisterAccessOps,
};
use gdbstub::target::ext::base::singlethread::{
    SingleThreadBase, SingleThreadResume, SingleThreadResumeOps, SingleThreadSingleStep,
    SingleThreadSingleStepOps,
};
use gdbstub::target::ext::breakpoints::{
    Breakpoints, BreakpointsOps, HwBreakpoint, HwBreakpointOps, SwBreakpoint, SwBreakpointOps,
};
use gdbstub::target::{Target, TargetError, TargetResult};
use probe_rs::{Core, MemoryInterface as _, RegisterId, Session};

/// How long to wait for a core to actually stop when asked.
const HALT_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(500);

/// Cortex-M register numbers, which are also the order `ArmCoreRegs` uses.
const SP: u16 = 13;
const LR: u16 = 14;
const PC: u16 = 15;
const XPSR: u16 = 16;

/// Tells GDB this is a Thumb-only M-profile core.
///
/// Without it GDB assumes A-profile ARM from the register layout and
/// disassembles Thumb code as if it were 32-bit ARM instructions, which is
/// unreadable. The register order here has to match `read_registers` exactly:
/// r0 through r12, then sp, lr, pc, xpsr.
const TARGET_XML: &str = r#"<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
  <architecture>armv6-m</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32" type="uint32"/>
    <reg name="r1" bitsize="32" type="uint32"/>
    <reg name="r2" bitsize="32" type="uint32"/>
    <reg name="r3" bitsize="32" type="uint32"/>
    <reg name="r4" bitsize="32" type="uint32"/>
    <reg name="r5" bitsize="32" type="uint32"/>
    <reg name="r6" bitsize="32" type="uint32"/>
    <reg name="r7" bitsize="32" type="uint32"/>
    <reg name="r8" bitsize="32" type="uint32"/>
    <reg name="r9" bitsize="32" type="uint32"/>
    <reg name="r10" bitsize="32" type="uint32"/>
    <reg name="r11" bitsize="32" type="uint32"/>
    <reg name="r12" bitsize="32" type="uint32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32" type="uint32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" type="uint32"/>
  </feature>
</target>
"#;

/// The seventeen registers this architecture exposes to GDB.
///
/// `gdbstub_arch` ships only `Armv4t` for ARM, whose `g` packet carries the
/// legacy FPA registers as well — 168 bytes where a Cortex-M has 68. GDB
/// checks that length against the target description and refuses the session
/// outright, so the register file has to be defined here to match.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct CortexMRegs {
    pub r: [u32; 13],
    pub sp: u32,
    pub lr: u32,
    pub pc: u32,
    pub xpsr: u32,
}

impl gdbstub::arch::Registers for CortexMRegs {
    type ProgramCounter = u32;

    fn pc(&self) -> u32 {
        self.pc
    }

    fn gdb_serialize(&self, mut write_byte: impl FnMut(Option<u8>)) {
        for value in self
            .r
            .iter()
            .chain([&self.sp, &self.lr, &self.pc, &self.xpsr])
        {
            for byte in value.to_le_bytes() {
                write_byte(Some(byte));
            }
        }
    }

    fn gdb_deserialize(&mut self, bytes: &[u8]) -> Result<(), ()> {
        let (words, remainder) = bytes.as_chunks::<4>();
        if words.len() != 17 || !remainder.is_empty() {
            return Err(());
        }
        let mut values = words.iter().map(|word| u32::from_le_bytes(*word));
        for slot in self.r.iter_mut() {
            *slot = values.next().ok_or(())?;
        }
        self.sp = values.next().ok_or(())?;
        self.lr = values.next().ok_or(())?;
        self.pc = values.next().ok_or(())?;
        self.xpsr = values.next().ok_or(())?;
        Ok(())
    }
}

/// Register numbers, which are GDB's and the core's alike for r0..pc.
#[derive(Debug, Clone, Copy)]
pub struct CortexMRegId(u16);

impl gdbstub::arch::RegId for CortexMRegId {
    fn from_raw_id(id: usize) -> Option<(Self, Option<std::num::NonZeroUsize>)> {
        let register = match id {
            // r0..r12, sp, lr, pc are numbered alike by GDB and the core.
            0..=15 => id as u16,
            // GDB's seventeenth register is xpsr, which the core calls 16.
            16 => XPSR,
            _ => return None,
        };
        Some((Self(register), std::num::NonZeroUsize::new(4)))
    }
}

/// A Thumb-only M-profile core, described to GDB as exactly what it is.
#[derive(Debug)]
pub enum CortexM {}

impl gdbstub::arch::Arch for CortexM {
    type Usize = u32;
    type Registers = CortexMRegs;
    type BreakpointKind = usize;
    type RegId = CortexMRegId;

    fn target_description_xml() -> Option<&'static str> {
        Some(TARGET_XML)
    }
}

/// The target GDB drives, which is one Cortex-M core behind the bridge.
pub struct ProbeTarget<'session> {
    core: Core<'session>,
    /// Every breakpoint GDB has asked for. GDB sets and clears these around
    /// each resume, and the core has only a handful of units, so refusing
    /// clearly beats silently dropping one.
    breakpoints: Vec<u64>,
    units: usize,
}

impl<'session> ProbeTarget<'session> {
    fn new(mut core: Core<'session>) -> Result<Self> {
        let units = core.available_breakpoint_units()? as usize;
        Ok(Self {
            core,
            breakpoints: Vec::new(),
            units,
        })
    }

    /// Runs one probe operation, retrying it once.
    ///
    /// Rapidly halting and restarting a core provokes the occasional SWD
    /// protocol error — an ACK of 0b111, the target not driving a response.
    /// It is transient and a second attempt clears it, but a debug session
    /// makes thousands of transactions, so over any real session one is close
    /// to certain, and every one of them used to end the session.
    fn retrying<T>(
        &mut self,
        mut operation: impl FnMut(&mut Core<'_>) -> Result<T, probe_rs::Error>,
    ) -> Result<T, probe_rs::Error> {
        match operation(&mut self.core) {
            Ok(value) => Ok(value),
            Err(first) => {
                std::thread::sleep(std::time::Duration::from_millis(2));
                operation(&mut self.core).map_err(|second| {
                    tracing::debug!(?first, ?second, "probe operation failed twice");
                    second
                })
            }
        }
    }

    fn add_breakpoint(&mut self, address: u64) -> TargetResult<bool, Self> {
        if self.breakpoints.contains(&address) {
            return Ok(true);
        }
        if self.breakpoints.len() >= self.units {
            // Reported to GDB as a plain failure, which it renders as "cannot
            // insert breakpoint". Better than accepting it and never stopping.
            return Ok(false);
        }
        self.retrying(|core| core.set_hw_breakpoint(address))
            .map_err(TargetError::Fatal)?;
        self.breakpoints.push(address);
        Ok(true)
    }

    fn remove_breakpoint(&mut self, address: u64) -> TargetResult<bool, Self> {
        let Some(index) = self.breakpoints.iter().position(|held| *held == address) else {
            return Ok(false);
        };
        self.retrying(|core| core.clear_hw_breakpoint(address))
            .map_err(TargetError::Fatal)?;
        self.breakpoints.remove(index);
        Ok(true)
    }
}

impl Target for ProbeTarget<'_> {
    type Arch = CortexM;
    type Error = probe_rs::Error;

    fn base_ops(&mut self) -> BaseOps<'_, Self::Arch, Self::Error> {
        BaseOps::SingleThread(self)
    }

    #[inline(always)]
    fn support_breakpoints(&mut self) -> Option<BreakpointsOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadBase for ProbeTarget<'_> {
    fn read_registers(&mut self, regs: &mut CortexMRegs) -> TargetResult<(), Self> {
        for (index, slot) in regs.r.iter_mut().enumerate() {
            *slot = self
                .retrying(|core| core.read_core_reg(RegisterId(index as u16)))
                .map_err(TargetError::Fatal)?;
        }
        regs.sp = self
            .retrying(|core| core.read_core_reg(RegisterId(SP)))
            .map_err(TargetError::Fatal)?;
        regs.lr = self
            .retrying(|core| core.read_core_reg(RegisterId(LR)))
            .map_err(TargetError::Fatal)?;
        regs.pc = self
            .retrying(|core| core.read_core_reg(RegisterId(PC)))
            .map_err(TargetError::Fatal)?;
        regs.xpsr = self
            .retrying(|core| core.read_core_reg(RegisterId(XPSR)))
            .map_err(TargetError::Fatal)?;
        Ok(())
    }

    fn write_registers(&mut self, regs: &CortexMRegs) -> TargetResult<(), Self> {
        for (index, value) in regs.r.iter().enumerate() {
            let value = *value;
            self.retrying(|core| core.write_core_reg(RegisterId(index as u16), value))
                .map_err(TargetError::Fatal)?;
        }
        for (id, value) in [
            (SP, regs.sp),
            (LR, regs.lr),
            (PC, regs.pc),
            (XPSR, regs.xpsr),
        ] {
            self.retrying(|core| core.write_core_reg(RegisterId(id), value))
                .map_err(TargetError::Fatal)?;
        }
        Ok(())
    }

    fn read_addrs(&mut self, start_addr: u32, data: &mut [u8]) -> TargetResult<usize, Self> {
        // A read GDB cannot satisfy is routine — it probes around the stack
        // and past the end of mapped memory while unwinding — so a failure
        // here is reported as "no data" rather than killing the session.
        match self.retrying(|core| core.read(u64::from(start_addr), data)) {
            Ok(()) => Ok(data.len()),
            Err(_) => Ok(0),
        }
    }

    fn write_addrs(&mut self, start_addr: u32, data: &[u8]) -> TargetResult<(), Self> {
        self.retrying(|core| core.write(u64::from(start_addr), data))
            .map_err(TargetError::Fatal)
    }

    #[inline(always)]
    fn support_resume(&mut self) -> Option<SingleThreadResumeOps<'_, Self>> {
        Some(self)
    }

    #[inline(always)]
    fn support_single_register_access(&mut self) -> Option<SingleRegisterAccessOps<'_, (), Self>> {
        Some(self)
    }
}

/// One register at a time, which is what GDB asks for when evaluating `$pc`
/// or a single `info registers pc` rather than refreshing the whole file.
impl SingleRegisterAccess<()> for ProbeTarget<'_> {
    fn read_register(
        &mut self,
        _tid: (),
        reg_id: CortexMRegId,
        buf: &mut [u8],
    ) -> TargetResult<usize, Self> {
        let value: u32 = self
            .core
            .read_core_reg(RegisterId(reg_id.0))
            .map_err(TargetError::Fatal)?;
        let bytes = value.to_le_bytes();
        let written = bytes.len().min(buf.len());
        buf[..written].copy_from_slice(&bytes[..written]);
        Ok(written)
    }

    fn write_register(
        &mut self,
        _tid: (),
        reg_id: CortexMRegId,
        value: &[u8],
    ) -> TargetResult<(), Self> {
        let Ok(bytes) = <[u8; 4]>::try_from(value) else {
            return Err(TargetError::NonFatal);
        };
        self.core
            .write_core_reg(RegisterId(reg_id.0), u32::from_le_bytes(bytes))
            .map_err(TargetError::Fatal)
    }
}

impl SingleThreadResume for ProbeTarget<'_> {
    fn resume(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
        self.retrying(|core| core.run())
    }

    #[inline(always)]
    fn support_single_step(&mut self) -> Option<SingleThreadSingleStepOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadSingleStep for ProbeTarget<'_> {
    /// Steps one instruction, disarming any breakpoint on the instruction
    /// being stepped over.
    ///
    /// Stepping off an address that still has a hardware unit pointed at it
    /// makes the core halt on the breakpoint again instead of advancing, and
    /// probe-rs only handles that itself when its cached halt reason says
    /// "breakpoint" — which it does not when the halt was observed by polling
    /// rather than requested. Left alone, a source-level `step` in GDB, which
    /// is many instruction steps, failed with an ARM error and took the whole
    /// session down with it.
    fn step(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
        let pc: u64 = self.retrying(|core| core.read_core_reg(RegisterId(PC)))?;
        let armed = self.breakpoints.contains(&pc);
        if armed {
            self.retrying(|core| core.clear_hw_breakpoint(pc))?;
        }
        // Retried once. Rapidly halting and restarting a core provokes the
        // occasional SWD protocol error — an ACK of 0b111, the target not
        // driving a response — and a source-level `step` is many instruction
        // steps, so over a long session one is close to certain. Losing the
        // whole debug session to a transient that a second attempt clears is
        // the wrong trade.
        let stepped = self.retrying(|core| core.step()).map(|_| ());
        if armed {
            // Re-armed even if the step failed, so the breakpoint GDB believes
            // in still exists.
            self.retrying(|core| core.set_hw_breakpoint(pc))?;
        }
        stepped
    }
}

impl Breakpoints for ProbeTarget<'_> {
    #[inline(always)]
    fn support_hw_breakpoint(&mut self) -> Option<HwBreakpointOps<'_, Self>> {
        Some(self)
    }

    #[inline(always)]
    fn support_sw_breakpoint(&mut self) -> Option<SwBreakpointOps<'_, Self>> {
        Some(self)
    }
}

impl HwBreakpoint for ProbeTarget<'_> {
    fn add_hw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.add_breakpoint(u64::from(addr))
    }

    fn remove_hw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.remove_breakpoint(u64::from(addr))
    }
}

impl SwBreakpoint for ProbeTarget<'_> {
    /// Served from the same hardware units as a hardware breakpoint.
    ///
    /// GDB reaches for a software breakpoint first, which means writing a trap
    /// instruction into the target's memory. On a Cortex-M that memory is
    /// flash: the write silently does nothing, and the breakpoint never fires.
    /// Answering with a real unit is what makes `break main` work.
    fn add_sw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.add_breakpoint(u64::from(addr))
    }

    fn remove_sw_breakpoint(&mut self, addr: u32, _kind: usize) -> TargetResult<bool, Self> {
        self.remove_breakpoint(u64::from(addr))
    }
}

/// Drives the target between GDB packets.
///
/// Carries the session's lifetime so the target does not have to be `'static`:
/// the trait has no lifetime of its own, but the type implementing it can.
struct ProbeEventLoop<'session>(std::marker::PhantomData<&'session ()>);

impl<'session> BlockingEventLoop for ProbeEventLoop<'session> {
    type Target = ProbeTarget<'session>;
    type Connection = TcpStream;
    type StopReason = SingleThreadStopReason<u32>;

    fn wait_for_stop_reason(
        target: &mut Self::Target,
        conn: &mut Self::Connection,
    ) -> Result<
        Event<Self::StopReason>,
        WaitForStopReasonError<<Self::Target as Target>::Error, std::io::Error>,
    > {
        loop {
            // The core is polled rather than interrupt-driven: SWD has no way
            // to notify, so stopping is only ever observed by asking.
            let halted = target
                .retrying(|core| core.core_halted())
                .map_err(WaitForStopReasonError::Target)?;
            if halted {
                // Refreshes probe-rs's cached halt reason as a side effect,
                // which its own stepping logic consults.
                let reason = target
                    .retrying(|core| core.status())
                    .map_err(WaitForStopReasonError::Target)?;
                let stop = match reason {
                    probe_rs::CoreStatus::Halted(probe_rs::HaltReason::Breakpoint(_)) => {
                        SingleThreadStopReason::SwBreak(())
                    }
                    _ => SingleThreadStopReason::Signal(Signal::SIGTRAP),
                };
                return Ok(Event::TargetStopped(stop));
            }
            if conn
                .peek()
                .map_err(WaitForStopReasonError::Connection)?
                .is_some()
            {
                let byte = conn.read().map_err(WaitForStopReasonError::Connection)?;
                return Ok(Event::IncomingData(byte));
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
    }

    fn on_interrupt(
        target: &mut Self::Target,
    ) -> Result<Option<Self::StopReason>, <Self::Target as Target>::Error> {
        target.retrying(|core| core.halt(HALT_TIMEOUT))?;
        Ok(Some(SingleThreadStopReason::Signal(Signal::SIGINT)))
    }
}

/// Serves GDB sessions on `port` until interrupted.
///
/// Successive sessions rather than one: a debugger that disconnects, or a
/// session lost to a wire fault, would otherwise mean restarting the server
/// and re-attaching the probe.
pub fn serve(session: &mut Session, port: u16, halt_first: bool, slow_link: bool) -> Result<()> {
    let listener = TcpListener::bind(("0.0.0.0", port))
        .with_context(|| format!("failed to listen on port {port}"))?;
    eprintln!("gdb server on tcp/{port}; connect with: target remote :{port}");
    if slow_link {
        // Reading the register file is seventeen separate core-register
        // transactions, and each is a round trip over the bridge. Across Wi-Fi
        // that runs to several seconds, comfortably past GDB's two-second
        // default, and the session dies with a nack rather than anything that
        // names the cause.
        eprintln!("this bridge is on the network: run `set remotetimeout 30` before connecting");
    }

    loop {
        let (stream, peer) = listener.accept().context("failed to accept a debugger")?;
        stream.set_nodelay(true)?;
        eprintln!("debugger attached from {peer}");
        if let Err(error) = serve_one(session, stream, halt_first) {
            eprintln!("session ended with an error: {error:#}");
        }
        eprintln!("waiting for the next debugger on tcp/{port}");
    }
}

/// One attach, from the debugger connecting to it going away again.
fn serve_one(session: &mut Session, stream: TcpStream, halt_first: bool) -> Result<()> {
    let mut core = session.core(0)?;
    if halt_first {
        // GDB expects to be talking to a stopped program the moment it
        // attaches; letting it discover a running core produces a session
        // where the first `info registers` is a lie.
        core.halt(HALT_TIMEOUT)?;
    }
    let mut target = ProbeTarget::new(core)?;

    let stub = GdbStub::new(stream);
    match stub.run_blocking::<ProbeEventLoop<'_>>(&mut target) {
        Ok(DisconnectReason::Disconnect) => eprintln!("debugger disconnected"),
        Ok(DisconnectReason::TargetExited(code)) => eprintln!("target exited with {code}"),
        Ok(DisconnectReason::TargetTerminated(signal)) => {
            eprintln!("target terminated with {signal:?}")
        }
        Ok(DisconnectReason::Kill) => eprintln!("debugger killed the session"),
        Err(error) => eprintln!("gdb session ended: {error}"),
    }

    // Whatever GDB left armed is released, so the next attach does not find
    // units already spent.
    for address in std::mem::take(&mut target.breakpoints) {
        let _ = target.core.clear_hw_breakpoint(address);
    }
    Ok(())
}
