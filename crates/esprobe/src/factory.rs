//! probe-rs discovery for the ESP32-C3 SWD bridge, over USB or the network.
//!
//! Everything above the probe layer — ADIv5, the debug sequences, the chip
//! database, the CMSIS-Pack flash algorithms — is probe-rs's already. The one
//! piece it cannot supply is knowing that this bridge exists, so that is all
//! this module adds: a [`ProbeFactory`] that enumerates and opens it, and a
//! [`ProbeLister`] that offers it alongside probe-rs's own drivers.
//!
//! The same factory serves both links. Which one is used is decided by the
//! selector's serial-number field, exactly as probe-rs's Black Magic driver
//! decides: an `address:port` is a network bridge, anything else is a serial
//! device. That keeps `--probe` meaning one thing whether the bridge is on the
//! bench or across the room.

use std::net::SocketAddr;

use probe_rs::probe::list::{AllProbesLister, ProbeLister};
use probe_rs::probe::{
    DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector, Probe, ProbeCreationError,
    ProbeFactory,
};
use serialport::{SerialPortType, available_ports};

use crate::{BRIDGE_TCP_PORT, SerialDapProbe};

/// The ESP32-C3's USB Serial/JTAG controller, which is fixed in silicon.
pub const ESPRESSIF_VID: u16 = 0x303a;
pub const USB_SERIAL_JTAG_PID: u16 = 0x1001;

/// The product string the USB Serial/JTAG controller reports.
///
/// Filtering on it is what keeps this driver from claiming every Espressif
/// device on the bus — an ESP32 being *debugged* presents the same identifiers
/// as one acting as the bridge, and only the interface differs. `sifliuart`
/// resolves the same ambiguity the same way.
const USB_PRODUCT_MARKER: &str = "usb jtag/serial debug unit";

/// Set to list every serial port as a candidate, for a bridge behind an
/// adapter that reports some other product string.
const OVERRIDE_ENV: &str = "ESPROBE_ANY_SERIAL";

/// Names a network bridge that has no way to announce itself, as `host` or
/// `host:port`. Discovery over IP needs somewhere to look.
const NETWORK_ENV: &str = "ESPROBE_NETWORK";

#[derive(Debug)]
pub struct EspBridgeFactory;

impl std::fmt::Display for EspBridgeFactory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("ESP32-C3 SWD bridge")
    }
}

/// Windows names its serial ports `COM1`, `COM2`, and so on.
///
/// These are the one device name with no path separator in it, which is what
/// every other rule here leans on. Without this, `COM3` reads as a host and
/// the bridge that was just enumerated cannot be opened.
fn is_windows_com_port(name: &str) -> bool {
    let Some(number) = name
        .strip_prefix("COM")
        .or_else(|| name.strip_prefix("com"))
    else {
        return false;
    };
    !number.is_empty() && number.bytes().all(|byte| byte.is_ascii_digit())
}

/// Parses a selector's serial number as a network endpoint.
///
/// Deliberately conservative: everything not recognised here is opened as a
/// serial device, because misreading a port as a host produces a DNS failure
/// reported as "probe not found", while the reverse is a clear error. A single
/// -label host such as `probe` is therefore not guessed at — write `probe:3333`
/// or `probe.local`.
fn network_endpoint(serial: &str) -> Option<String> {
    if serial.is_empty() {
        return None;
    }
    // Device nodes, on every platform that has them.
    if serial.contains('/') || serial.contains('\\') || is_windows_com_port(serial) {
        return None;
    }
    // `1.2.3.4:3333` and `[::1]:3333`.
    if serial.parse::<SocketAddr>().is_ok() {
        return Some(serial.to_string());
    }
    // `host:port`, with a port that is really a port. A bracketless IPv6
    // address is not accepted: it is ambiguous with this form and is not
    // something `TcpStream::connect` takes anyway.
    if let Some((host, port)) = serial.rsplit_once(':')
        && !host.is_empty()
        && !host.contains(':')
        && port.parse::<u16>().is_ok()
    {
        return Some(serial.to_string());
    }
    // A bare host, given the bridge's default port. The dot is what separates
    // an address or a qualified name from a word that could be a device.
    if serial.contains('.') {
        return Some(format!("{serial}:{BRIDGE_TCP_PORT}"));
    }
    None
}

impl ProbeFactory for EspBridgeFactory {
    fn open(&self, selector: &DebugProbeSelector) -> Result<Box<dyn DebugProbe>, DebugProbeError> {
        let Some(serial) = selector.serial_number.as_deref() else {
            return Err(DebugProbeError::ProbeCouldNotBeCreated(
                ProbeCreationError::NotFound,
            ));
        };

        let probe = match network_endpoint(serial) {
            Some(endpoint) => SerialDapProbe::connect(&endpoint),
            None => SerialDapProbe::open(std::path::Path::new(serial)),
        };

        // A device that does not answer the bridge handshake is simply not one
        // of ours; report that rather than a transport error, so probe-rs can
        // carry on asking its other drivers.
        probe
            .map(|probe| Box::new(probe) as Box<dyn DebugProbe>)
            .map_err(|_| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::NotFound))
    }

    fn list_probes(&self) -> Vec<DebugProbeInfo> {
        let mut probes = Vec::new();

        if let Ok(endpoint) = std::env::var(NETWORK_ENV) {
            probes.push(DebugProbeInfo::new(
                format!("ESP32-C3 SWD bridge ({endpoint})"),
                ESPRESSIF_VID,
                USB_SERIAL_JTAG_PID,
                Some(endpoint),
                &EspBridgeFactory,
                None,
                false,
            ));
        }

        let accept_any = std::env::var(OVERRIDE_ENV).is_ok();
        let Ok(ports) = available_ports() else {
            return probes;
        };
        for port in ports {
            // macOS exposes each port twice; keep the callout device only.
            if cfg!(target_os = "macos") && !port.port_name.contains("/cu.") {
                continue;
            }
            let SerialPortType::UsbPort(usb) = port.port_type else {
                continue;
            };
            let looks_right = usb
                .product
                .as_deref()
                .is_some_and(|product| product.to_lowercase().contains(USB_PRODUCT_MARKER));
            if !accept_any && !looks_right {
                continue;
            }
            probes.push(DebugProbeInfo::new(
                format!("ESP32-C3 SWD bridge ({})", port.port_name),
                usb.vid,
                usb.pid,
                // The port path, not the USB serial: it is what `open` needs,
                // and every Espressif bridge reports its MAC as the serial.
                Some(port.port_name.clone()),
                &EspBridgeFactory,
                None,
                false,
            ));
        }
        probes
    }
}

/// Offers the bridge alongside every probe driver probe-rs ships.
///
/// Out-of-tree registration: `AllProbesLister`'s driver table is a private
/// constant, so a custom lister is the supported way to add one. Delegating to
/// it rather than replacing it means a J-Link or CMSIS-DAP plugged into the
/// same machine still works.
#[derive(Debug)]
pub struct EspBridgeLister {
    builtin: AllProbesLister,
}

impl EspBridgeLister {
    pub fn new() -> Self {
        Self {
            builtin: AllProbesLister::new(),
        }
    }
}

impl Default for EspBridgeLister {
    fn default() -> Self {
        Self::new()
    }
}

impl ProbeLister for EspBridgeLister {
    fn open(&self, selector: &DebugProbeSelector) -> Result<Probe, DebugProbeError> {
        match EspBridgeFactory.open(selector) {
            Ok(probe) => Ok(Probe::from_specific_probe(probe)),
            // Not ours; let the built-in drivers have it.
            Err(_) => self.builtin.open(selector),
        }
    }

    fn list(&self, selector: Option<&DebugProbeSelector>) -> Vec<DebugProbeInfo> {
        let mut list = self.builtin.list(selector);
        list.extend(EspBridgeFactory.list_probes_filtered(selector));
        list
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn socket_addresses_are_network_bridges() {
        assert_eq!(
            network_endpoint("192.0.2.10:3333").as_deref(),
            Some("192.0.2.10:3333")
        );
    }

    #[test]
    fn a_bare_address_gets_the_default_port() {
        assert_eq!(
            network_endpoint("192.0.2.10").as_deref(),
            Some("192.0.2.10:3333")
        );
    }

    #[test]
    fn serial_device_paths_are_not_network_bridges() {
        assert_eq!(network_endpoint("/dev/ttyACM0"), None);
        assert_eq!(network_endpoint("/dev/serial/by-id/usb-Espressif"), None);
        assert_eq!(network_endpoint(r"COM3\"), None);
    }

    #[test]
    fn windows_serial_ports_are_not_hosts() {
        // `list_probes` puts the port name in the selector, and on Windows
        // that is a bare `COM3`. Reading it as a host made every probe this
        // lister enumerated there impossible to open.
        for name in ["COM1", "COM3", "COM12", "com3", r"\\.\COM12"] {
            assert_eq!(network_endpoint(name), None, "{name} read as a host");
        }
    }

    #[test]
    fn qualified_hosts_and_explicit_ports_are_network_bridges() {
        assert_eq!(
            network_endpoint("probe.local").as_deref(),
            Some("probe.local:3333")
        );
        assert_eq!(
            network_endpoint("probe.local:4444").as_deref(),
            Some("probe.local:4444")
        );
        assert_eq!(
            network_endpoint("bench:3333").as_deref(),
            Some("bench:3333")
        );
        assert_eq!(
            network_endpoint("[::1]:3333").as_deref(),
            Some("[::1]:3333")
        );
    }

    #[test]
    fn a_word_that_could_be_a_device_is_left_alone() {
        // Nothing distinguishes a single-label host from a device name, so
        // this errs towards serial and the docs say to qualify it.
        assert_eq!(network_endpoint("probe"), None);
        assert_eq!(network_endpoint(""), None);
        // A port that is not a port does not make a host.
        assert_eq!(network_endpoint("ttyACM0:notaport"), None);
    }
}

#[cfg(test)]
mod network_transport_tests {
    use crate::{BRIDGE_TCP_PORT, SerialDapProbe};
    use esprobe_protocol::frame::{Command, MAX_FRAME, Status, decode_request, encode_response};
    use probe_rs::probe::ProbeFactory as _;
    use std::io::{Read, Write};
    use std::net::TcpListener;

    /// Answers the handshake the way the firmware does, and nothing else.
    ///
    /// The network path cannot be proven without a bridge on a network, but the
    /// half that lives here — framing over a socket, the endpoint parsing, the
    /// transport abstraction — can be, and this is the part that would
    /// otherwise only ever be exercised by hand.
    fn spawn_stub_bridge() -> u16 {
        let listener = TcpListener::bind(("127.0.0.1", 0)).expect("bind a loopback port");
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            let Ok((mut stream, _)) = listener.accept() else {
                return;
            };
            let mut frame = [0u8; MAX_FRAME + 2];
            let mut reply = [0u8; MAX_FRAME + 2];
            let mut len = 0usize;
            let mut byte = [0u8; 1];
            while stream.read(&mut byte).unwrap_or(0) == 1 {
                if byte[0] != 0 {
                    if len < frame.len() {
                        frame[len] = byte[0];
                        len += 1;
                    }
                    continue;
                }
                if len == 0 {
                    continue;
                }
                if let Ok(request) = decode_request(&mut frame[..len]) {
                    let payload: &[u8] = match request.command {
                        Command::Hello => b"DAP1",
                        // Block words then clock, as the firmware reports.
                        Command::Capabilities => &[0, 4, 0, 0, 0x00, 0x12, 0x7a, 0x00],
                        _ => &[],
                    };
                    if let Ok(n) =
                        encode_response(request.sequence, Status::Ok, payload, &mut reply)
                    {
                        let _ = stream.write_all(&[0]);
                        let _ = stream.write_all(&reply[..n]);
                    }
                }
                len = 0;
            }
        });
        port
    }

    /// Answers requests in pairs, both replies in one write.
    ///
    /// TCP delivers them together, so the read that completes the first reply
    /// also carries the second — which is what happens on a real bridge as
    /// soon as a request is kept in flight ahead of the reply being read.
    fn spawn_batching_bridge() -> u16 {
        let listener = TcpListener::bind(("127.0.0.1", 0)).expect("bind a loopback port");
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            let Ok((mut stream, _)) = listener.accept() else {
                return;
            };
            let mut frame = [0u8; MAX_FRAME + 2];
            let mut reply = [0u8; MAX_FRAME + 2];
            let mut len = 0usize;
            let mut byte = [0u8; 1];
            let mut batch: Vec<u8> = Vec::new();
            let mut held = 0usize;
            while stream.read(&mut byte).unwrap_or(0) == 1 {
                if byte[0] != 0 {
                    if len < frame.len() {
                        frame[len] = byte[0];
                        len += 1;
                    }
                    continue;
                }
                if len == 0 {
                    continue;
                }
                if let Ok(request) = decode_request(&mut frame[..len])
                    && let Ok(n) = encode_response(request.sequence, Status::Ok, &[], &mut reply)
                {
                    batch.push(0);
                    batch.extend_from_slice(&reply[..n]);
                    held += 1;
                    if held == 2 {
                        let _ = stream.write_all(&batch);
                        batch.clear();
                        held = 0;
                    }
                }
                len = 0;
            }
        });
        port
    }

    #[test]
    fn a_reply_sharing_a_read_with_the_next_is_not_lost() {
        let port = spawn_batching_bridge();
        let mut transport =
            crate::Transport::connect(&format!("127.0.0.1:{port}")).expect("connect to the stub");
        let first = transport.send(Command::Ping, &[]).expect("send the first");
        let second = transport.send(Command::Ping, &[]).expect("send the second");
        transport.receive(first).expect("the first reply");
        // The bytes of this one arrived in the read that completed the first.
        // Held in a buffer local to `receive`, they went out of scope with it
        // and this waited three seconds for a frame already thrown away.
        transport
            .receive(second)
            .expect("the second reply shared a read with the first and was lost");
    }

    #[test]
    fn the_network_transport_completes_a_handshake() {
        let port = spawn_stub_bridge();
        let probe = SerialDapProbe::connect(&format!("127.0.0.1:{port}"))
            .expect("the bridge handshake should complete over TCP");
        // Capabilities is what tells the host how much it may ask for at once.
        assert_eq!(probe.block_words, 1024);
    }

    #[test]
    fn the_factory_opens_a_network_bridge_from_a_selector() {
        let port = spawn_stub_bridge();
        let selector = probe_rs::probe::DebugProbeSelector {
            vendor_id: super::ESPRESSIF_VID,
            product_id: super::USB_SERIAL_JTAG_PID,
            interface: None,
            serial_number: Some(format!("127.0.0.1:{port}")),
        };
        assert!(
            super::EspBridgeFactory.open(&selector).is_ok(),
            "probe-rs should reach a network bridge through the factory"
        );
        let _ = BRIDGE_TCP_PORT;
    }
}
