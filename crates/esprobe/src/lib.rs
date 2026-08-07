//! Drive SWD through an ESP32-C3 bridge, over USB or over the network.
//!
//! This is the half worth depending on: the transport, the probe-rs
//! `DebugProbe` implementation, target identification, and the [`factory`]
//! that lets probe-rs discover the bridge alongside its own drivers. The
//! `esprobe` binary is one consumer of it; anything else wanting a probe on
//! the far side of a network can be another.
//!
//! ```no_run
//! use probe_rs::probe::list::Lister;
//! let lister = Lister::with_lister(Box::new(esprobe::factory::EspBridgeLister::new()));
//! for probe in lister.list_all() {
//!     println!("{}", probe.identifier);
//! }
//! ```

pub mod factory;
pub mod gdb;

use std::fmt;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use esprobe_protocol::clock::{MAX_CLOCK_HZ, MIN_CLOCK_HZ};
use esprobe_protocol::frame::{
    Command, MAX_BLOCK_WORDS, MAX_FRAME, Status, decode_response, encode_request,
};
use probe_rs::architecture::arm::dp::DpRegisterAddress;
use probe_rs::architecture::arm::sequences::ArmDebugSequence;
use probe_rs::architecture::arm::{
    ArmCommunicationInterface, ArmDebugInterface, ArmError, DapProbe, RawDapAccess, RegisterAddress,
};
use probe_rs::probe::{DebugProbe, DebugProbeError, WireProtocol};
use probe_rs::{CoreStatus, Error as ProbeRsError};

/// Where udev exposes stable USB serial names.
pub const SERIAL_BY_ID: &str = "/dev/serial/by-id";
/// The port the firmware serves the bridge protocol on over the network.
pub const BRIDGE_TCP_PORT: u16 = 3333;

/// Bridge round trips issued so far. A transfer's cost splits between the wire
/// and the transport, and only counting them says which one to attack.
pub static ROUND_TRIPS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum Engine {
    /// GPSPI2 shifts each field in hardware.
    Hardware,
    /// The CPU drives every edge.
    BitBang,
}

/// Stopping, starting and inspecting the target's core.
///
/// Everything here is probe-rs's `Core`, reached through this bridge. The
/// point of the subcommand is that a probe you can only flash is not a debug
/// probe: halting, stepping and reading registers is what makes one.
pub fn discover_bridge_port() -> Result<PathBuf> {
    let mut found: Vec<PathBuf> = std::fs::read_dir(SERIAL_BY_ID)
        .with_context(|| format!("failed to list {SERIAL_BY_ID}"))?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| {
                    name.contains("Espressif_USB_JTAG_serial_debug_unit") && name.ends_with("-if00")
                })
        })
        .collect();
    found.sort();
    match found.as_slice() {
        [port] => Ok(port.clone()),
        [] => bail!("no Espressif USB Serial/JTAG device found; pass --port"),
        ports => bail!("{} bridges attached; pass --port to choose", ports.len()),
    }
}

/// A bank-0 Debug Port register address.
pub fn dp_address(address: u8) -> RegisterAddress {
    RegisterAddress::DpRegister(DpRegisterAddress {
        address,
        bank: Some(0),
    })
}

/// Brings the debug port up without probe-rs, which cannot be asked to attach
/// before the part is known.
pub fn power_up_debug_port(serial: &mut SerialDapProbe, under_reset: bool) -> Result<()> {
    serial.attach_under_reset = under_reset;
    let outcome = serial.attach();
    serial.attach_under_reset = false;
    outcome?;
    serial.raw_read_register(dp_address(0x00))?;
    serial.raw_write_register(dp_address(0x00), 0x1f)?;
    serial.raw_write_register(dp_address(0x08), 0)?;
    serial.raw_write_register(dp_address(0x04), 0x5000_0000)?;
    let status = serial.raw_read_register(dp_address(0x04))?;
    if status & 0xa000_0000 != 0xa000_0000 {
        bail!("debug power-up did not complete, CTRL/STAT=0x{status:08x}");
    }
    Ok(())
}

/// What the ARM debug port says it is, before any vendor register is touched.
///
/// This is the only identity a probe can report without knowing the vendor:
/// DPIDR is architectural, defined by ADIv5, and every conforming target
/// answers it. What *part* this is takes a vendor register at a vendor address,
/// which is a thing this tool deliberately does not know — name the target with
/// `--target` and probe-rs answers it from its own chip database.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DebugPortId {
    pub raw: u32,
    /// JEP106 continuation and identity code of whoever designed the DP.
    pub designer: u16,
    pub part_number: u8,
    pub revision: u8,
    /// DP architecture version: 1 for DPv1, 2 for DPv2, and so on.
    pub version: u8,
    /// Whether the DP reports minimal-DP behaviour (no TRANSACTION COUNTER,
    /// no PUSHED verify).
    pub minimal: bool,
}

impl DebugPortId {
    #[must_use]
    pub const fn decode(raw: u32) -> Self {
        Self {
            raw,
            designer: ((raw >> 1) & 0x7ff) as u16,
            part_number: ((raw >> 20) & 0xff) as u8,
            revision: ((raw >> 28) & 0xf) as u8,
            version: ((raw >> 12) & 0xf) as u8,
            minimal: (raw >> 16) & 1 != 0,
        }
    }

    #[must_use]
    pub fn describe(&self) -> String {
        format!(
            "dp_id=0x{:08x} designer=0x{:03x} part=0x{:02x} rev={} dp_version={} minimal={}",
            self.raw, self.designer, self.part_number, self.revision, self.version, self.minimal
        )
    }
}

/// Reads the debug port's own identity, falling back to connect-under-reset.
///
/// A target that resets repeatedly — a corrupt or half-written image will do it
/// — answers nothing to a plain attach, because it is back in reset before the
/// first transfer lands. Falling back is the difference between "no answer" and
/// "recoverable", so it is automatic rather than a flag to reach for.
///
/// The `bool` is whether reset had to be held.
pub fn read_debug_port_id(serial: &mut SerialDapProbe) -> Result<(DebugPortId, bool)> {
    match read_debug_port_id_once(serial, false) {
        Ok(id) => Ok((id, false)),
        Err(_) => read_debug_port_id_once(serial, true).map(|id| (id, true)),
    }
}

pub fn read_debug_port_id_once(
    serial: &mut SerialDapProbe,
    under_reset: bool,
) -> Result<DebugPortId> {
    power_up_debug_port(serial, under_reset)?;
    let raw = serial.raw_read_register(dp_address(0x00))?;
    if raw == 0 || raw == u32::MAX {
        bail!("debug port answered 0x{raw:08x}, which is not an identity");
    }
    Ok(DebugPortId::decode(raw))
}

/// Turns a bridge refusal into something that names the likely cause.
///
/// The status and its detail byte are exact, and were what this printed. They
/// are also opaque unless you have the firmware open beside you — `Transport,
/// detail=[04]` is the fail-closed power check refusing to drive a pad, which
/// is a wiring or power problem and reads as an internal error. The raw values
/// are kept on the end, because they are what a bug report needs.
fn describe_failure(status: Status, detail: &[u8]) -> String {
    let cause = match (status, detail.first()) {
        (Status::Transport, Some(1)) => {
            Some("the debug port returned an implausible IDCODE (all zeros or all ones)")
        }
        (Status::Transport, Some(2)) => Some("the debug port never reported powered-up"),
        (Status::Transport, Some(3)) => Some("a memory access was not word-aligned"),
        // Both the attach path and the fail-closed power check report this,
        // and they mean the same thing: nothing is holding SWDIO up, so there
        // is no evidence a powered target is on the other end.
        (Status::Transport, Some(4)) => Some(
            "SWDIO reads low when released, so no powered target is presenting its \
             pull-up. Check target power, ground, and the SWDIO wire",
        ),
        // A three-bit SWD acknowledgement, shifted out LSB first. 0b111 is what
        // an idle pull-up looks like when nothing is driving the line at all.
        (Status::Transport, Some(7)) => Some(
            "no response on the wire: nothing acknowledged the request. Check that a \
             target is connected and powered",
        ),
        (Status::NotAttached, _) => Some("not attached; run a command that attaches first"),
        (Status::TargetInReset, _) => {
            Some("the target is held in reset, so its bus would answer zeros")
        }
        (Status::Unsupported, _) => Some("this firmware does not implement that command"),
        _ => None,
    };
    match cause {
        Some(cause) => format!("{cause} (bridge status {status:?}, detail={detail:02x?})"),
        None => format!("bridge status {status:?}, detail={detail:02x?}"),
    }
}

/// Reads target words over the bridge's bulk path.
pub fn read_words(serial: &mut SerialDapProbe, address: u32, count: usize) -> Result<Vec<u32>> {
    let mut request = address.to_le_bytes().to_vec();
    request.extend_from_slice(&(count as u16).to_le_bytes());
    let bytes = serial.transport.command(Command::MemoryRead, &request)?;
    let (words, remainder) = bytes.as_chunks::<4>();
    if !remainder.is_empty() || words.len() != count {
        bail!(
            "bulk read returned {} bytes, wanted {}",
            bytes.len(),
            count * 4
        );
    }
    Ok(words.iter().copied().map(u32::from_le_bytes).collect())
}

pub trait Link: std::io::Read + std::io::Write + Send {}
impl<T: std::io::Read + std::io::Write + Send> Link for T {}

pub struct Transport {
    pub port: Box<dyn Link>,
    pub sequence: u16,
    /// Bytes read from the link but not yet consumed by a frame.
    ///
    /// This has to outlive a single `receive`. With a request in flight ahead
    /// of the reply being read, one read can carry the end of one reply and
    /// the beginning of the next; when that buffer was a local, the beginning
    /// of the next reply went out of scope with it, and the following
    /// `receive` waited three seconds for a frame whose first bytes had
    /// already been thrown away. Sequential commands never saw it because they
    /// never have a second reply in flight — which is why `ping` was reliable
    /// over two hundred round trips while a bulk read of the same length was
    /// not.
    pub pending: Vec<u8>,
}

impl fmt::Debug for Transport {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("Transport")
            .field("sequence", &self.sequence)
            .finish_non_exhaustive()
    }
}

impl Transport {
    /// Connects over TCP. A missing port means the bridge's default.
    pub fn connect(endpoint: &str) -> Result<Self> {
        let address = if endpoint.contains(':') {
            endpoint.to_string()
        } else {
            format!("{endpoint}:{BRIDGE_TCP_PORT}")
        };
        let stream = std::net::TcpStream::connect(&address)
            .with_context(|| format!("failed to connect to {address}"))?;
        // A request/response protocol gains nothing from Nagle and loses a
        // round trip to it.
        stream.set_nodelay(true)?;
        // Wi-Fi round trips are two orders of magnitude longer than USB's, and
        // this only bounds how long a quiet read blocks before looping.
        stream.set_read_timeout(Some(Duration::from_millis(250)))?;
        Ok(Self {
            port: Box::new(stream),
            sequence: 0,
            pending: Vec::new(),
        })
    }

    pub fn open(path: &std::path::Path) -> Result<Self> {
        // The ESP32-C3's USB Serial/JTAG is a native USB endpoint: the baud
        // rate is a formality, and the frame timeout only bounds a stall.
        let port = serialport::new(path.to_string_lossy(), 921_600)
            .timeout(Duration::from_millis(50))
            .open()
            .with_context(|| format!("failed to open {}", path.display()))?;
        Ok(Self {
            port: Box::new(port),
            sequence: 0,
            pending: Vec::new(),
        })
    }

    pub fn command(&mut self, command: Command, payload: &[u8]) -> Result<Vec<u8>> {
        let sequence = self.send(command, payload)?;
        self.receive(sequence)
    }

    /// Queues a request without waiting for its reply, so the bridge can work
    /// on it while a previous reply is still draining over USB.
    pub fn send(&mut self, command: Command, payload: &[u8]) -> Result<u16> {
        ROUND_TRIPS.fetch_add(1, Ordering::Relaxed);
        self.sequence = self.sequence.wrapping_add(1);
        let sequence = self.sequence;
        let mut request = [0u8; MAX_FRAME + 2];
        let request_len = encode_request(sequence, command, payload, &mut request)
            .map_err(|_| anyhow::anyhow!("request is too large"))?;
        self.port.write_all(&[0])?;
        self.port.write_all(&request[..request_len])?;
        tracing::trace!(
            sequence,
            ?command,
            payload = payload.len(),
            framed = request_len,
            "request"
        );
        Ok(sequence)
    }

    pub fn receive(&mut self, sequence: u16) -> Result<Vec<u8>> {
        let started = Instant::now();
        let deadline = started + Duration::from_secs(3);
        // One read syscall per frame rather than per packet. A 4 KiB reply took
        // nine calls at 512 bytes, and their latency lands squarely between the
        // reply arriving and the next request going out.
        let mut chunk = [0u8; MAX_FRAME + 2];
        loop {
            // Whatever is already buffered, before asking the link for more.
            while let Some(end) = self.pending.iter().position(|&byte| byte == 0) {
                let mut frame: Vec<u8> = self.pending.drain(..=end).collect();
                frame.pop();
                if frame.is_empty() {
                    continue;
                }
                let decoded = decode_response(&mut frame);
                let Ok(response) = decoded else {
                    tracing::trace!(bytes = frame.len(), "discarded an undecodable frame");
                    continue;
                };
                if response.sequence != sequence {
                    // Ordinary while a reply for an abandoned request drains.
                    tracing::trace!(
                        got = response.sequence,
                        want = sequence,
                        "reply for another request"
                    );
                    continue;
                }
                tracing::trace!(
                    sequence,
                    status = ?response.status,
                    payload = response.payload.len(),
                    micros = started.elapsed().as_micros(),
                    "reply"
                );
                return match response.status {
                    Status::Ok => Ok(response.payload.to_vec()),
                    status => bail!("{}", describe_failure(status, response.payload)),
                };
            }
            if Instant::now() >= deadline {
                break;
            }
            let read = match self.port.read(&mut chunk) {
                // Zero bytes from a socket is the peer having closed it, which
                // no amount of waiting undoes. Falling through to the loop
                // spun a core flat until the deadline, once per command, for
                // as long as the caller kept trying.
                Ok(0) => bail!("the bridge closed the connection"),
                Ok(read) => read,
                // A serial port reports a lapsed read timeout as `TimedOut`; a
                // socket reports it as `WouldBlock`. Only the first was
                // handled, so the network transport failed on its first quiet
                // moment with "Resource temporarily unavailable" — which never
                // showed up until a bridge was actually reachable over Wi-Fi.
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::TimedOut
                            | std::io::ErrorKind::WouldBlock
                            | std::io::ErrorKind::Interrupted
                    ) =>
                {
                    continue;
                }
                Err(error) => return Err(error.into()),
            };
            self.pending.extend_from_slice(&chunk[..read]);
            // A link emitting bytes with no delimiter in them is not going to
            // start; do not grow without bound waiting for one.
            if self.pending.len() > 4 * (MAX_FRAME + 2) {
                self.pending.clear();
            }
        }
        tracing::debug!(
            sequence,
            buffered = self.pending.len(),
            "no reply within the deadline"
        );
        bail!("bridge response timeout")
    }
}

#[derive(Debug)]
pub struct SerialDapProbe {
    pub transport: Transport,
    pub attached: bool,
    pub protocol: Option<WireProtocol>,
    pub speed_khz: u32,
    pub block_words: usize,
    pub attach_under_reset: bool,
}

impl SerialDapProbe {
    pub fn open(path: &std::path::Path) -> Result<Self> {
        Self::with_transport(Transport::open(path)?)
    }

    pub fn connect(endpoint: &str) -> Result<Self> {
        Self::with_transport(Transport::connect(endpoint)?)
    }

    pub fn with_transport(transport: Transport) -> Result<Self> {
        let mut transport = transport;
        // A frame carrying a different protocol version is rejected outright
        // by the far end, which then says nothing at all — so the first
        // command timing out is what a version mismatch looks like, and it is
        // otherwise indistinguishable from a dead board or the wrong port.
        let version = esprobe_protocol::frame::VERSION;
        let hello = transport.command(Command::Hello, &[]).with_context(|| {
            format!(
                "the bridge did not answer; if it is powered and this is the \
                 right port, its firmware probably predates protocol version \
                 {version} — rebuild and reflash it"
            )
        })?;
        if hello != b"DAP1" {
            bail!("unexpected bridge identity");
        }
        // A bridge that predates block transfers answers Unsupported here, and
        // its frame buffers are too small for the batched path either way.
        let capabilities = transport
            .command(Command::Capabilities, &[])
            .context("bridge is too old for this host; reflash the ESP32-C3 firmware")?;
        let [block, clock] = capabilities.as_chunks::<4>().0 else {
            bail!("invalid capability response");
        };
        let block_words = u32::from_le_bytes(*block) as usize;
        Ok(Self {
            transport,
            attached: false,
            protocol: None,
            speed_khz: u32::from_le_bytes(*clock) / 1_000,
            block_words: block_words.min(MAX_BLOCK_WORDS),
            attach_under_reset: false,
        })
    }

    /// Selects which engine clocks the wire.
    pub fn set_engine(&mut self, engine: Engine) -> Result<()> {
        let selector = match engine {
            Engine::Hardware => 0,
            Engine::BitBang => 1,
        };
        let response = self.command(Command::SetEngine, &[selector])?;
        let bytes: [u8; 4] = response
            .as_slice()
            .try_into()
            .map_err(|_| anyhow::anyhow!("invalid set-engine response"))?;
        self.speed_khz = u32::from_le_bytes(bytes) / 1_000;
        Ok(())
    }

    /// Tells the bridge which pad carries which signal.
    pub fn set_pin_map(&mut self, swapped: bool) -> Result<()> {
        self.command(Command::SetPinMap, &[u8::from(swapped)])?;
        Ok(())
    }

    pub fn command(
        &mut self,
        command: Command,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        self.transport
            .command(command, payload)
            .map_err(|error| DebugProbeError::Other(error.to_string()))
    }
}

impl DebugProbe for SerialDapProbe {
    fn get_name(&self) -> &str {
        "ESP32-C3 IR USB-serial DAP bridge"
    }

    fn speed_khz(&self) -> u32 {
        self.speed_khz
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let requested = speed_khz
            .saturating_mul(1_000)
            .clamp(MIN_CLOCK_HZ, MAX_CLOCK_HZ);
        let response = self.command(Command::SetSpeed, &requested.to_le_bytes())?;
        let bytes: [u8; 4] = response
            .as_slice()
            .try_into()
            .map_err(|_| DebugProbeError::Other("invalid set-speed response".to_string()))?;
        self.speed_khz = u32::from_le_bytes(bytes) / 1_000;
        Ok(self.speed_khz)
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        let command = if self.attach_under_reset {
            Command::AttachUnderReset
        } else {
            Command::Attach
        };
        self.command(command, &[])?;
        self.attached = true;
        Ok(())
    }

    fn detach(&mut self) -> Result<(), ProbeRsError> {
        self.command(Command::Detach, &[])
            .map_err(ProbeRsError::Probe)?;
        self.attached = false;
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        self.command(Command::ResetAssert, &[])?;
        std::thread::sleep(Duration::from_millis(10));
        self.command(Command::ResetRelease, &[])?;
        Ok(())
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        self.command(Command::ResetAssert, &[])?;
        Ok(())
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        self.command(Command::ResetRelease, &[])?;
        Ok(())
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        if protocol != WireProtocol::Swd {
            return Err(DebugProbeError::UnsupportedProtocol(protocol));
        }
        self.protocol = Some(protocol);
        Ok(())
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        self.protocol
    }

    fn has_arm_interface(&self) -> bool {
        true
    }

    fn try_get_arm_debug_interface<'probe>(
        self: Box<Self>,
        sequence: Arc<dyn ArmDebugSequence>,
    ) -> Result<Box<dyn ArmDebugInterface + 'probe>, (Box<dyn DebugProbe>, ArmError)> {
        Ok(ArmCommunicationInterface::create(self, sequence, false))
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn try_as_dap_probe(&mut self) -> Option<&mut dyn DapProbe> {
        Some(self)
    }
}

impl DapProbe for SerialDapProbe {}

impl RawDapAccess for SerialDapProbe {
    fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        let payload = [u8::from(address.is_ap()), address.a2_and_3()];
        let response = self.command(Command::ReadRegister, &payload)?;
        let bytes: [u8; 4] = response
            .as_slice()
            .try_into()
            .map_err(|_| ArmError::Probe(DebugProbeError::Other("invalid read response".into())))?;
        Ok(u32::from_le_bytes(bytes))
    }

    fn raw_read_block(
        &mut self,
        address: RegisterAddress,
        values: &mut [u32],
    ) -> Result<(), ArmError> {
        if self.block_words <= 1 {
            for value in values.iter_mut() {
                *value = self.raw_read_register(address)?;
            }
            return Ok(());
        }
        for chunk in values.chunks_mut(self.block_words) {
            let count = chunk.len() as u16;
            let [low, high] = count.to_le_bytes();
            let payload = [u8::from(address.is_ap()), address.a2_and_3(), low, high];
            let response = self.command(Command::ReadRegisterBlock, &payload)?;
            if response.len() != chunk.len() * 4 {
                return Err(ArmError::Probe(DebugProbeError::Other(
                    "invalid block-read response".into(),
                )));
            }
            let (words, remainder) = response.as_chunks::<4>();
            if !remainder.is_empty() {
                return Err(ArmError::Probe(DebugProbeError::Other(
                    "unaligned block-read response".into(),
                )));
            }
            for (value, bytes) in chunk.iter_mut().zip(words) {
                *value = u32::from_le_bytes(*bytes);
            }
        }
        Ok(())
    }

    fn raw_write_block(
        &mut self,
        address: RegisterAddress,
        values: &[u32],
    ) -> Result<(), ArmError> {
        if self.block_words <= 1 {
            for value in values {
                self.raw_write_register(address, *value)?;
            }
            return Ok(());
        }
        let mut payload = Vec::with_capacity(4 + self.block_words * 4);
        for chunk in values.chunks(self.block_words) {
            payload.clear();
            payload.push(u8::from(address.is_ap()));
            payload.push(address.a2_and_3());
            payload.extend_from_slice(&(chunk.len() as u16).to_le_bytes());
            for value in chunk {
                payload.extend_from_slice(&value.to_le_bytes());
            }
            self.command(Command::WriteRegisterBlock, &payload)?;
        }
        Ok(())
    }

    fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        let mut payload = [0u8; 6];
        payload[0] = u8::from(address.is_ap());
        payload[1] = address.a2_and_3();
        payload[2..].copy_from_slice(&value.to_le_bytes());
        self.command(Command::WriteRegister, &payload)?;
        Ok(())
    }

    fn jtag_sequence(&mut self, _cycles: u8, _tms: bool, _tdi: u64) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }

    fn swj_sequence(&mut self, bit_len: u8, bits: u64) -> Result<(), DebugProbeError> {
        let mut payload = [0u8; 9];
        payload[0] = bit_len;
        payload[1..].copy_from_slice(&bits.to_le_bytes());
        self.command(Command::SwjSequence, &payload)?;
        Ok(())
    }

    fn swj_pins(
        &mut self,
        pin_out: u32,
        pin_select: u32,
        _pin_wait: u32,
    ) -> Result<u32, DebugProbeError> {
        // The CMSIS-DAP pin numbering probe-rs speaks here.
        const SWCLK: u32 = 1 << 0;
        const SWDIO: u32 = 1 << 1;
        const NRESET: u32 = 1 << 7;

        // nRESET is the only line worth driving this way: SWCLK and SWDIO
        // belong to the wire engine, and letting a sequence poke them
        // individually would fight it. probe-rs's connect-under-reset only
        // needs nRESET, which is the whole reason this is implemented.
        if pin_select & NRESET != 0 {
            let asserted = pin_out & NRESET == 0;
            let command = if asserted {
                Command::ResetAssert
            } else {
                Command::ResetRelease
            };
            // Both replies carry the resulting level, so the state below is
            // measured rather than assumed.
            let state = self.command(command, &[])?;
            let high = state.first().copied().unwrap_or(0) != 0;
            let mut pins = pin_out & (SWCLK | SWDIO);
            if high {
                pins |= NRESET;
            }
            return Ok(pins);
        }

        // Nothing to change. Reporting the requested state is the convention
        // for probes that cannot sample a line without disturbing it, and
        // sampling ours would tear down an attach in progress.
        Ok(pin_out)
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn core_status_notification(&mut self, _state: CoreStatus) -> Result<(), DebugProbeError> {
        Ok(())
    }
}
