mod factory;

use std::fmt;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use esprobe_protocol::clock::{DEFAULT_CLOCK_HZ, MAX_CLOCK_HZ, MIN_CLOCK_HZ};
use esprobe_protocol::frame::{
    Command, MAX_BLOCK_WORDS, MAX_FRAME, Status, decode_response, encode_request,
};
use probe_rs::architecture::arm::dp::DpRegisterAddress;
use probe_rs::architecture::arm::sequences::ArmDebugSequence;
use probe_rs::architecture::arm::{
    ArmCommunicationInterface, ArmDebugInterface, ArmError, DapProbe, RawDapAccess, RegisterAddress,
};
use probe_rs::flashing::{DownloadOptions, Format, download_file_with_options};
use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbe, DebugProbeError, DebugProbeSelector, WireProtocol};
use probe_rs::{CoreStatus, Error as ProbeRsError, MemoryInterface, Permissions};

/// Where udev exposes stable USB serial names.
const SERIAL_BY_ID: &str = "/dev/serial/by-id";
/// The port the firmware serves the bridge protocol on over the network.
pub(crate) const BRIDGE_TCP_PORT: u16 = 3333;

/// Bridge round trips issued so far. A transfer's cost splits between the wire
/// and the transport, and only counting them says which one to attack.
static ROUND_TRIPS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

#[derive(Parser)]
struct Args {
    /// Defaults to the only attached Espressif USB Serial/JTAG device.
    #[arg(long)]
    port: Option<PathBuf>,
    /// Reach the bridge over the network instead, as `host` or `host:port`.
    /// Everything above the transport behaves identically either way.
    #[arg(long, conflicts_with = "port")]
    url: Option<String>,
    /// probe-rs target. Detected from the chip's own identity registers when
    /// omitted, because a probe-rs attach succeeds against whatever name it is
    /// given and so cannot be used to check the answer.
    #[arg(long)]
    target: Option<String>,
    /// SWCLK frequency in kHz. The bridge clamps and reports what it programs.
    #[arg(long, default_value_t = DEFAULT_CLOCK_HZ / 1_000)]
    speed_khz: u32,
    /// Tell the bridge that SWDIO and SWCLK are crossed on the harness.
    #[arg(long)]
    swap_swd: bool,
    /// Print probe-rs's own trace; RUST_LOG overrides the level.
    #[arg(long, short)]
    verbose: bool,
    /// Issue one register transfer per word instead of batched blocks.
    #[arg(long)]
    no_blocks: bool,
    /// Which engine clocks the wire.
    #[arg(long, value_enum, default_value_t = Engine::Hardware)]
    engine: Engine,
    #[command(subcommand)]
    command: Action,
}

/// Wire engine selection, mirroring the bridge's own.
#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Engine {
    /// GPSPI2 shifts each field in hardware.
    Hardware,
    /// The CPU drives every edge.
    BitBang,
}

/// Network provisioning, done over whichever link is already connected.
#[derive(Subcommand)]
enum WifiAction {
    /// Report whether the bridge is on a network, and which.
    Status,
    /// Give the bridge a network. Stored on the device, so it survives a power
    /// cycle and does not need to be built into the image.
    Set {
        #[arg(long)]
        ssid: String,
        /// Omit for an open network; you will be prompted rather than passing
        /// it on a command line that the shell will remember.
        #[arg(long)]
        password: Option<String>,
    },
    /// Forget the stored network.
    Forget,
}

#[derive(Subcommand)]
enum Action {
    /// Read SWDIO, SWCLK, and RESET_ALL after ESP drive is released.
    Lines,
    /// Read released SWD levels while RESET_ALL is held low and after release.
    ResetLines,
    /// Hold RESET_ALL low until reset-release or an ESP reboot.
    ResetAssert,
    /// Release RESET_ALL to the IR board's pull-up.
    ResetRelease,
    /// Alternate RESET_ALL low and released at a fixed interval.
    ResetCycle {
        #[arg(long, default_value_t = 2)]
        seconds: u64,
    },
    /// Attach through probe-rs and report the Cortex-M core status.
    Probe,
    /// Assert RESET_ALL, prepare SWD, release reset, and attach immediately.
    ProbeUnderReset,
    /// Briefly drive and read back both SWD pads while RESET_ALL is low.
    PadSelfTest,
    /// Probe and identify immediately inside the reset-release critical path.
    RecoveryProbe {
        /// Delay after releasing RESET_ALL before the first DP request.
        #[arg(long, default_value_t = 1_000)]
        delay_us: u16,
    },
    /// Hold reset and alternate SWDIO low/high for a physical continuity check.
    SwdioCycle {
        #[arg(long, default_value_t = 2)]
        seconds: u64,
    },
    /// Ask the running STM32 Rust firmware to jump to its ROM UART bootloader.
    EnterRomBoot,
    /// Passively capture bytes from the STM32 display UART.
    UartReceive,
    /// Reset the STM32 and atomically capture its startup UART bytes.
    UartResetCapture {
        /// Passively listen on GPIO5 instead of the schematic-normal GPIO6.
        #[arg(long)]
        swapped: bool,
    },
    /// Read-only SWD diagnostic on STM32, DWM0, and DWM1 mux channels.
    MuxScan,
    /// Program an ELF with probe-rs's STM32 flash algorithm and reset the core.
    Flash {
        #[arg(value_name = "ELF")]
        image: PathBuf,
    },
    /// Identify, back up, program, and prove the result by reading it back.
    ///
    /// The whole sequence, with nothing to remember: the target is detected
    /// from its own registers, its current flash is saved and hashed before
    /// anything is erased, and the written image is compared against the file
    /// by a read that does not go through the flashing algorithm.
    Program {
        #[arg(value_name = "IMAGE")]
        image: PathBuf,
        /// Where to save the existing flash. Defaults to the image path with a
        /// `.backup` suffix; the backup is never skipped.
        #[arg(long)]
        backup: PathBuf,
        /// Base address for a raw `.bin`. Ignored for ELF and hex.
        #[arg(long, default_value_t = 0x0800_0000)]
        address: u64,
    },
    /// Clock the bus with SWDIO released to free a target parked mid-phase.
    Recover {
        #[arg(long, default_value_t = 128)]
        cycles: u16,
    },
    /// Drive raw DAP transfers by hand and print CTRL/STAT after each step.
    ///
    /// Isolates which access sets a sticky error without probe-rs in the way.
    DapPoke {
        #[arg(long, default_value_t = 0xe004_2004)]
        address: u32,
        #[arg(long, default_value_t = 7)]
        value: u32,
    },
    /// Bring the DP up and perform one un-retried AP write, reporting ACKs.
    ApWriteProbe {
        #[arg(long, default_value_t = 0x2300_0052)]
        value: u32,
        /// Override the clocks issued after the preceding read's data phase.
        #[arg(long)]
        after_read: Option<u8>,
        /// Override the clocks between the write's ACK and its data phase.
        #[arg(long)]
        before_write: Option<u8>,
    },
    /// List every probe probe-rs can see, this bridge included.
    ListProbes,
    /// Read or change the network the bridge joins.
    #[command(subcommand)]
    Wifi(WifiAction),
    /// Read the target's identity registers and say what it actually is.
    Identify,
    /// Report the GPIO map the running firmware was built for.
    PinMap,
    /// Report where a word's time goes inside the firmware.
    Profile {
        #[arg(long, default_value_t = 0x0800_0000)]
        address: u32,
        #[arg(long, default_value_t = 512)]
        words: u16,
    },
    /// Measure sustained transport bandwidth with no wire activity.
    Echo {
        #[arg(long, default_value_t = 2048)]
        bytes: u16,
        #[arg(long, default_value_t = 200)]
        count: u32,
    },
    /// Time empty round trips to separate transport latency from wire time.
    Ping {
        #[arg(long, default_value_t = 200)]
        count: u32,
    },
    /// Read flash over the bridge's bulk path, without probe-rs in the loop.
    FastDump {
        #[arg(value_name = "OUTPUT")]
        output: PathBuf,
        /// Requests kept in flight ahead of the reply being read.
        #[arg(long, default_value_t = 1)]
        depth: usize,
        /// Words per round trip. Larger amortises the transport's fixed cost
        /// but coarsens the overlap; the bridge's maximum is the default.
        #[arg(long)]
        block: Option<usize>,
        #[arg(long, default_value_t = 0x0800_0000)]
        address: u32,
        #[arg(long, default_value_t = 512 * 1024)]
        size: usize,
    },
    /// Transmit a pattern and sample the pad at the same time, showing what
    /// GPSPI2 actually emits.
    SpiLoopback {
        #[arg(long, default_value_t = 33)]
        bits: u8,
        #[arg(long, default_value_t = 1)]
        pattern: u64,
    },
    /// Send a DPIDR read and print the raw sampled reply bit by bit.
    WireProbe {
        /// Sample the ACK and data fields as two bursts, as a transfer does.
        #[arg(long)]
        split: bool,
        /// Instead, run read/write/read with this many turnaround clocks after
        /// a read data phase.
        #[arg(long)]
        after_read: Option<u8>,
        /// Turnaround clocks between a write's ACK and its data phase.
        #[arg(long, default_value_t = 1)]
        before_write: u8,
    },
    /// Time a read-only block transfer and report sustained throughput.
    Bench {
        #[arg(long, default_value_t = 0x0800_0000)]
        address: u64,
        #[arg(long, default_value_t = 64 * 1024)]
        size: usize,
    },
    /// Read target flash into a binary file without modifying the target.
    Dump {
        #[arg(value_name = "OUTPUT")]
        output: PathBuf,
        #[arg(long, default_value_t = 512 * 1024)]
        size: usize,
    },
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.verbose {
        tracing_subscriber::fmt()
            .with_env_filter(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| "probe_rs=debug".into()),
            )
            .with_writer(std::io::stderr)
            .init();
    }
    // Listing must not open a link, so it runs before one is chosen.
    if matches!(args.command, Action::ListProbes) {
        let lister = Lister::with_lister(Box::new(factory::EspBridgeLister::new()));
        let probes = lister.list_all();
        if probes.is_empty() {
            println!("no probes found");
        }
        for probe in probes {
            println!("{probe}");
        }
        return Ok(());
    }

    // One string names the link for both the direct protocol and probe-rs's
    // selector, so there is no second notion of "which bridge" to keep in step.
    let link_selector = match (args.url.clone(), args.port.clone()) {
        (Some(endpoint), _) => endpoint,
        (None, Some(port)) => port.display().to_string(),
        (None, None) => discover_bridge_port()?.display().to_string(),
    };
    let mut serial = match args.url.is_some() {
        true => SerialDapProbe::connect(&link_selector)?,
        false => SerialDapProbe::open(std::path::Path::new(&link_selector))?,
    };
    if args.no_blocks {
        serial.block_words = 1;
    }
    serial.set_engine(args.engine)?;
    serial.set_pin_map(args.swap_swd)?;
    serial.set_speed(args.speed_khz)?;
    match &args.command {
        Action::ListProbes => unreachable!("probe listing returned before a link was opened"),
        Action::Lines => {
            let lines = serial.command(Command::LineState, &[])?;
            let [swdio, swclk, reset_all] = lines.as_slice() else {
                bail!("invalid line-state response");
            };
            println!("released swdio={swdio} swclk={swclk} reset_all={reset_all}");
            return Ok(());
        }
        Action::ResetLines => {
            let lines = serial.command(Command::ResetLineState, &[])?;
            let [
                asserted_swdio,
                asserted_swclk,
                asserted_reset_all,
                released_swdio,
                released_swclk,
                released_reset_all,
            ] = lines.as_slice()
            else {
                bail!("invalid reset-line-state response");
            };
            println!(
                "asserted swdio={asserted_swdio} swclk={asserted_swclk} \
                 reset_all={asserted_reset_all}; released swdio={released_swdio} \
                 swclk={released_swclk} reset_all={released_reset_all}"
            );
            return Ok(());
        }
        Action::ResetAssert => {
            let state = serial.command(Command::ResetAssert, &[])?;
            let [reset_all] = state.as_slice() else {
                bail!("invalid reset-assert response");
            };
            println!("reset_all={reset_all} held=true");
            return Ok(());
        }
        Action::ResetRelease => {
            let state = serial.command(Command::ResetRelease, &[])?;
            let [reset_all] = state.as_slice() else {
                bail!("invalid reset-release response");
            };
            println!("reset_all={reset_all} held=false");
            return Ok(());
        }
        Action::ResetCycle { seconds } => {
            let stop = Arc::new(AtomicBool::new(false));
            let handler_stop = Arc::clone(&stop);
            ctrlc::set_handler(move || handler_stop.store(true, Ordering::Release))
                .context("failed to install reset-cycle interrupt handler")?;
            let interval = Duration::from_secs(*seconds);
            println!("cycling RESET_ALL every {seconds}s; press Ctrl-C to stop and release reset");
            while !stop.load(Ordering::Acquire) {
                let state = serial.command(Command::ResetAssert, &[])?;
                println!(
                    "reset_all={} held=true",
                    state.first().copied().unwrap_or(1)
                );
                wait_or_stop(interval, &stop);
                let state = serial.command(Command::ResetRelease, &[])?;
                println!(
                    "reset_all={} held=false",
                    state.first().copied().unwrap_or(0)
                );
                wait_or_stop(interval, &stop);
            }
            let state = serial
                .command(Command::ResetRelease, &[])
                .context("failed to release RESET_ALL while stopping reset-cycle")?;
            let [reset_all] = state.as_slice() else {
                bail!("invalid final reset-release response");
            };
            println!("reset_all={reset_all} held=false stopped=true");
            return Ok(());
        }
        Action::PadSelfTest => {
            let levels = serial.command(Command::PadSelfTest, &[])?;
            let [swdio_low, swdio_high, swclk_low, swclk_high] = levels.as_slice() else {
                bail!("invalid pad-self-test response");
            };
            println!(
                "swdio_low={swdio_low} swdio_high={swdio_high} \
                 swclk_low={swclk_low} swclk_high={swclk_high}"
            );
            return Ok(());
        }
        Action::RecoveryProbe { delay_us } => {
            let identity = serial.command(Command::RecoveryProbe, &delay_us.to_le_bytes())?;
            if identity.len() != 8 {
                bail!("invalid recovery-probe response");
            }
            let dp_id = u32::from_le_bytes(identity[..4].try_into()?);
            let device_id = u32::from_le_bytes(identity[4..].try_into()?);
            println!("dp_id=0x{dp_id:08x} device_id=0x{device_id:08x}");
            return Ok(());
        }
        Action::SwdioCycle { seconds } => {
            let stop = Arc::new(AtomicBool::new(false));
            let handler_stop = Arc::clone(&stop);
            ctrlc::set_handler(move || handler_stop.store(true, Ordering::Release))
                .context("failed to install SWDIO-cycle interrupt handler")?;
            let interval = Duration::from_secs(*seconds);
            println!(
                "holding reset and cycling SWDIO every {seconds}s; press Ctrl-C to release all lines"
            );
            while !stop.load(Ordering::Acquire) {
                for level in [0_u8, 1] {
                    if stop.load(Ordering::Acquire) {
                        break;
                    }
                    let observed = serial.command(Command::DiagnosticSwdio, &[level])?;
                    println!(
                        "driven_swdio={level} observed_swdio={}",
                        observed.first().copied().unwrap_or(0xff)
                    );
                    wait_or_stop(interval, &stop);
                }
            }
            serial.command(Command::Detach, &[])?;
            serial.command(Command::ResetRelease, &[])?;
            println!("swdio=released swclk=released reset_all=released");
            return Ok(());
        }
        Action::EnterRomBoot => {
            let response = serial.command(Command::EnterRomBoot, &[])?;
            print!("stm32_uart_response=");
            for byte in response {
                print!("{byte:02x}");
            }
            println!();
            return Ok(());
        }
        Action::UartReceive => {
            let response = serial.command(Command::UartReceive, &[])?;
            print!("stm32_uart_rx=");
            for byte in response {
                print!("{byte:02x}");
            }
            println!();
            return Ok(());
        }
        Action::UartResetCapture { swapped } => {
            let response = serial.command(Command::UartResetCapture, &[u8::from(*swapped)])?;
            print!("stm32_uart_after_reset=");
            for byte in response {
                print!("{byte:02x}");
            }
            println!();
            return Ok(());
        }
        Action::Recover { cycles } => {
            let levels = serial.command(Command::RecoverLine, &cycles.to_le_bytes())?;
            let [swdio, swclk] = levels.as_slice() else {
                bail!("invalid recover response");
            };
            println!("released swdio={swdio} swclk={swclk} cycles={cycles}");
            return Ok(());
        }
        Action::Identify => {
            let (identity, under_reset) = identify(&mut serial)?;
            println!("{}", identity.describe());
            if under_reset {
                println!("attach=under-reset (the target does not answer a plain attach)");
            }
            match identity.target() {
                Some(target) => println!("probe_rs_target={target}"),
                None => println!("probe_rs_target=unknown; pass --target explicitly"),
            }
            serial.detach()?;
            return Ok(());
        }
        Action::Wifi(action) => {
            match action {
                WifiAction::Status => {
                    let reply = serial.command(Command::WifiStatus, &[])?;
                    let [connected, a, b, c, d, ssid_len, ssid @ ..] = reply.as_slice() else {
                        bail!("invalid wifi-status response");
                    };
                    let ssid = std::str::from_utf8(&ssid[..usize::from(*ssid_len).min(ssid.len())])
                        .unwrap_or("<invalid utf-8>");
                    if *connected != 0 {
                        println!("connected ssid={ssid} ip={a}.{b}.{c}.{d}");
                    } else if ssid.is_empty() {
                        println!("no network configured");
                    } else {
                        println!("configured ssid={ssid} but not connected");
                    }
                }
                WifiAction::Set { ssid, password } => {
                    let password = match password {
                        Some(password) => password.clone(),
                        None => rpassword::prompt_password("Wi-Fi password (empty for open): ")
                            .context("failed to read the password")?,
                    };
                    let mut payload = vec![0u8; 2 + ssid.len() + password.len()];
                    let length = esprobe_protocol::wifi::encode(ssid, &password, &mut payload)
                        .context("SSID or password is too long")?;
                    serial.command(Command::WifiSet, &payload[..length])?;
                    println!("stored ssid={ssid}; the bridge will join within a few seconds");
                }
                WifiAction::Forget => {
                    serial.command(Command::WifiForget, &[])?;
                    println!("forgotten");
                }
            }
            return Ok(());
        }
        Action::PinMap => {
            let map = serial.command(Command::PinMap, &[])?;
            let [swdio, swclk, reset, s0, s1, tx, rx] = map.as_slice() else {
                bail!("invalid pin-map response");
            };
            println!(
                "SWDIO=GPIO{swdio} SWCLK=GPIO{swclk} RESET_ALL=GPIO{reset} \
                 ASW_S0=GPIO{s0} ASW_S1=GPIO{s1} DISP_TX=GPIO{tx} DISP_RX=GPIO{rx}"
            );
            return Ok(());
        }
        Action::Echo { bytes, count } => {
            let started = Instant::now();
            for _ in 0..*count {
                let payload = serial.command(Command::Echo, &bytes.to_le_bytes())?;
                if payload.len() != usize::from(*bytes) {
                    bail!("echo returned {} bytes, wanted {bytes}", payload.len());
                }
            }
            let elapsed = started.elapsed();
            let total = f64::from(*count) * f64::from(*bytes);
            println!(
                "bytes={bytes} round_trips={count} seconds={:.3} kib_per_second={:.1} \
                 us_per_trip={:.0}",
                elapsed.as_secs_f64(),
                total / 1024.0 / elapsed.as_secs_f64(),
                elapsed.as_secs_f64() * 1e6 / f64::from(*count)
            );
            return Ok(());
        }
        Action::Profile { address, words } => {
            serial.attach()?;
            serial.raw_read_register(dp_address(0x00))?;
            serial.raw_write_register(dp_address(0x00), 0x1f)?;
            serial.raw_write_register(dp_address(0x08), 0)?;
            serial.raw_write_register(dp_address(0x04), 0x5000_0000)?;
            let mut request = address.to_le_bytes().to_vec();
            request.extend_from_slice(&words.to_le_bytes());
            let response = serial.command(Command::Profile, &request)?;
            let (fields, []) = response.as_chunks::<4>() else {
                bail!("invalid profile response");
            };
            let [total, run, runs, count] = fields else {
                bail!("invalid profile response");
            };
            let (total, run, runs, count) = (
                f64::from(u32::from_le_bytes(*total)),
                f64::from(u32::from_le_bytes(*run)),
                f64::from(u32::from_le_bytes(*runs)),
                f64::from(u32::from_le_bytes(*count)),
            );
            const CPU_MHZ: f64 = 160.0;
            println!(
                "words={count:.0} cycles_per_word={:.0} us_per_word={:.2}",
                total / count,
                total / count / CPU_MHZ
            );
            println!(
                "  peripheral: transactions_per_word={:.2} cycles_per_word={:.0} us={:.2}",
                runs / count,
                run / count,
                run / count / CPU_MHZ
            );
            println!(
                "  elsewhere:  cycles_per_word={:.0} us={:.2} ({:.0}% of total)",
                (total - run) / count,
                (total - run) / count / CPU_MHZ,
                (total - run) / total * 100.0
            );
            return Ok(());
        }
        Action::Ping { count } => {
            let started = Instant::now();
            for _ in 0..*count {
                serial.command(Command::Ping, &[])?;
            }
            let elapsed = started.elapsed();
            println!(
                "round_trips={count} seconds={:.3} us_per_trip={:.0}",
                elapsed.as_secs_f64(),
                elapsed.as_secs_f64() * 1e6 / f64::from(*count)
            );
            return Ok(());
        }
        Action::FastDump {
            output,
            depth,
            block,
            address,
            size,
        } => {
            let words = size / 4;
            serial.attach()?;
            // Bring the debug port up by hand; probe-rs is deliberately not in
            // this path, because its per-chunk round trips are the cost being
            // removed.
            serial.raw_read_register(dp_address(0x00))?;
            serial.raw_write_register(dp_address(0x00), 0x1f)?;
            serial.raw_write_register(dp_address(0x08), 0)?;
            serial.raw_write_register(dp_address(0x04), 0x5000_0000)?;
            let status = serial.raw_read_register(dp_address(0x04))?;
            if status & 0xa000_0000 != 0xa000_0000 {
                bail!("debug power-up did not complete, CTRL/STAT=0x{status:08x}");
            }
            let mut contents = Vec::with_capacity(*size);
            let before_trips = ROUND_TRIPS.load(Ordering::Relaxed);
            let started = Instant::now();
            // One request is always in flight ahead of the reply being read.
            // The wire and the transport each take milliseconds per block and
            // are otherwise strictly serialised; overlapping them costs the
            // larger of the two rather than their sum.
            let block = block
                .unwrap_or(serial.block_words)
                .clamp(1, serial.block_words);
            let spans: Vec<usize> = (0..words)
                .step_by(block)
                .map(|offset| (words - offset).min(block))
                .collect();
            let request_for = |index: usize, span: usize| {
                let offset: usize = spans[..index].iter().sum();
                let mut request = (address + (offset * 4) as u32).to_le_bytes().to_vec();
                request.extend_from_slice(&(span as u16).to_le_bytes());
                request
            };
            let depth = (*depth).max(1);
            let mut inflight: std::collections::VecDeque<u16> = std::collections::VecDeque::new();
            for (index, &span) in spans.iter().enumerate().take(depth) {
                inflight.push_back(
                    serial
                        .transport
                        .send(Command::MemoryRead, &request_for(index, span))?,
                );
            }
            for (index, &span) in spans.iter().enumerate() {
                let sequence = inflight.pop_front().expect("a request is always in flight");
                if let Some((next, &next_span)) = spans.iter().enumerate().nth(index + depth) {
                    inflight.push_back(
                        serial
                            .transport
                            .send(Command::MemoryRead, &request_for(next, next_span))?,
                    );
                }
                let chunk = serial.transport.receive(sequence)?;
                if chunk.len() != span * 4 {
                    bail!(
                        "short bulk read: wanted {} bytes, got {}",
                        span * 4,
                        chunk.len()
                    );
                }
                contents.extend_from_slice(&chunk);
            }
            let elapsed = started.elapsed();
            let round_trips = ROUND_TRIPS.load(Ordering::Relaxed) - before_trips;
            serial.detach()?;
            std::fs::write(output, &contents)
                .with_context(|| format!("failed to write {}", output.display()))?;
            println!(
                "dumped={} address=0x{address:08x} bytes={size} seconds={:.3} \
                 kib_per_second={:.1} round_trips={round_trips} words_per_trip={:.0}",
                output.display(),
                elapsed.as_secs_f64(),
                *size as f64 / 1024.0 / elapsed.as_secs_f64(),
                words as f64 / round_trips as f64,
            );
            return Ok(());
        }
        Action::SpiLoopback { bits, pattern } => {
            let (bits, pattern) = (*bits, *pattern);
            let mut request = vec![bits];
            request.extend_from_slice(&pattern.to_le_bytes());
            let response = serial.command(Command::SpiLoopback, &request)?;
            let observed = u64::from_le_bytes(
                response
                    .as_slice()
                    .try_into()
                    .map_err(|_| anyhow::anyhow!("invalid loopback response"))?,
            );
            let render = |value: u64| -> String {
                (0..u32::from(bits))
                    .map(|index| if value >> index & 1 != 0 { '1' } else { '0' })
                    .collect()
            };
            let mask = u64::MAX >> (64 - u32::from(bits));
            println!("sent     ={}", render(pattern));
            println!("observed ={}", render(observed));
            if observed == pattern & mask {
                println!("match=yes");
            } else {
                println!("match=no differing_bits={:#x}", (observed ^ pattern) & mask);
            }
            return Ok(());
        }
        Action::ApWriteProbe {
            value,
            after_read,
            before_write,
        } => {
            let mut request = value.to_le_bytes().to_vec();
            if let (Some(after), Some(before)) = (after_read, before_write) {
                request.push(*after);
                request.push(*before);
            }
            let response = serial.command(Command::ApWriteProbe, &request)?;
            let [dpidr_ack, write_ack, rest @ ..] = response.as_slice() else {
                bail!("invalid ap-write-probe response");
            };
            let ([before, after], []) = rest.as_chunks::<4>() else {
                bail!("invalid ap-write-probe status fields");
            };
            let before = u32::from_le_bytes(*before);
            let after = u32::from_le_bytes(*after);
            let name = |ack: u8| match ack {
                0b001 => "OK",
                0b010 => "WAIT",
                0b100 => "FAULT",
                _ => "NONE",
            };
            println!(
                "wrote=0x{value:08x} read_back=0x{before:08x} write_ack={} \
                 ctrl_stat=0x{after:08x} wdata_err={} sticky_err={} dpidr_ack={}",
                name(*write_ack),
                after >> 7 & 1,
                after >> 5 & 1,
                name(*dpidr_ack)
            );
            return Ok(());
        }
        Action::DapPoke { address, value } => {
            serial.attach()?;
            let report = |serial: &mut SerialDapProbe, step: &str| -> Result<()> {
                let status = serial.raw_read_register(dp_address(0x04))?;
                println!(
                    "{step:<22} ctrl_stat=0x{status:08x} sticky_err={} wdata_err={} sticky_orun={}",
                    status >> 5 & 1,
                    status >> 7 & 1,
                    status >> 1 & 1,
                );
                Ok(())
            };
            // Power the debug domain up by hand: probe-rs is not in this path.
            // After a line reset the DP answers nothing but a DPIDR read.
            let dpidr = serial.raw_read_register(dp_address(0x00))?;
            println!("dpidr                 0x{dpidr:08x}");
            serial.raw_write_register(dp_address(0x00), 0x1f)?;
            serial.raw_write_register(dp_address(0x08), 0)?;
            serial.raw_write_register(dp_address(0x04), 0x5000_0000)?;
            report(&mut serial, "after power-up")?;
            serial.raw_write_register(RegisterAddress::ApRegister(0x00), 0x2300_0052)?;
            report(&mut serial, "after csw write")?;
            serial.raw_write_register(RegisterAddress::ApRegister(0x04), *address)?;
            report(&mut serial, "after tar write")?;
            let read = serial.raw_read_register(RegisterAddress::ApRegister(0x0c))?;
            println!("memory read           0x{address:08x} -> 0x{read:08x}");
            report(&mut serial, "after drw read")?;
            serial.raw_write_register(RegisterAddress::ApRegister(0x0c), *value)?;
            report(&mut serial, "after drw write")?;
            serial.raw_write_register(RegisterAddress::ApRegister(0x04), *address)?;
            let read = serial.raw_read_register(RegisterAddress::ApRegister(0x0c))?;
            println!("memory readback       0x{address:08x} -> 0x{read:08x}");
            report(&mut serial, "after readback")?;
            serial.detach()?;
            return Ok(());
        }
        Action::WireProbe {
            split,
            after_read,
            before_write,
        } => {
            let request = match after_read {
                Some(clocks) => vec![2, *clocks, *before_write],
                None => vec![u8::from(*split)],
            };
            let sampled = serial.command(Command::WireProbe, &request)?;
            let ([first, second], []) = sampled.as_chunks::<8>() else {
                bail!("invalid wire-probe response");
            };
            let ack_field = u64::from_le_bytes(*first);
            let data_field = u64::from_le_bytes(*second);
            let render = |value: u64, width: u32| -> String {
                (0..width)
                    .map(|index| if value >> index & 1 != 0 { '1' } else { '0' })
                    .collect()
            };
            if let Some(clocks) = after_read {
                let status = data_field as u32;
                println!(
                    "after_read={clocks} before_write={before_write} \
                     acks={:03b},{:03b},{:03b} ctrl_stat=0x{status:08x} wdata_err={}",
                    ack_field & 0b111,
                    ack_field >> 8 & 0b111,
                    ack_field >> 16 & 0b111,
                    status >> 7 & 1
                );
                return Ok(());
            }
            if *split {
                println!(
                    "ack_burst={} data_burst={}",
                    render(ack_field, 8),
                    render(data_field, 40)
                );
                for offset in 0..8 {
                    println!(
                        "offset={offset} ack={:03b} word=0x{:08x}",
                        ack_field >> offset & 0b111,
                        (data_field >> offset) as u32
                    );
                }
            } else {
                println!("sampled_first_to_last={}", render(ack_field, 48));
                for offset in 0..8 {
                    println!(
                        "offset={offset} ack={:03b} word=0x{:08x}",
                        ack_field >> offset & 0b111,
                        (ack_field >> (offset + 3)) as u32
                    );
                }
            }
            return Ok(());
        }
        Action::MuxScan => {
            for (target, name) in [(0_u8, "stm32"), (1, "gps"), (2, "dwm0"), (3, "dwm1")] {
                let response = serial.command(Command::MuxProbe, &[target])?;
                let [swdio, swclk, status, dp0, dp1, dp2, dp3, s0, s1] = response.as_slice() else {
                    bail!("invalid mux-probe response for {name}");
                };
                let dp_id = u32::from_le_bytes([*dp0, *dp1, *dp2, *dp3]);
                println!(
                    "target={name} s0={s0} s1={s1} swdio={swdio} swclk={swclk} \
                     status=0x{status:02x} dp_id=0x{dp_id:08x}"
                );
            }
            return Ok(());
        }
        Action::Probe
        | Action::ProbeUnderReset
        | Action::Flash { .. }
        | Action::Program { .. }
        | Action::Dump { .. }
        | Action::Bench { .. } => {}
    }
    // Always ask the target what it is, even when a name was supplied: the
    // flash size is needed to back the part up before erasing it, and a
    // mismatch between what was asked for and what is present is worth seeing.
    let detection = identify(&mut serial).ok();
    if detection.is_some() {
        serial.detach()?;
    }
    let identity = detection.as_ref().map(|(identity, _)| identity);
    if let Some((found, under_reset)) = detection.as_ref() {
        eprintln!(
            "detected {}{}",
            found.describe(),
            if *under_reset {
                " (attached under reset)"
            } else {
                ""
            }
        );
    }
    // Carry the finding into the probe-rs session: a target that needed reset
    // held to be identified needs it held to be attached.
    if detection
        .as_ref()
        .is_some_and(|(_, under_reset)| *under_reset)
    {
        serial.attach_under_reset = true;
    }
    let target = match args.target.clone() {
        Some(target) => target,
        None => identity
            .and_then(|found| found.target())
            .with_context(|| {
                identity.map_or_else(
                    || "could not identify the target; pass --target".to_string(),
                    |found| {
                        format!(
                            "{} is not in the family table; pass --target",
                            found.describe()
                        )
                    },
                )
            })?
            .to_string(),
    };
    let detected_flash_kib = identity.map_or(0, |found| found.flash_kib);
    let under_reset = matches!(&args.command, Action::ProbeUnderReset)
        || detection.as_ref().is_some_and(|(_, needed)| *needed);
    let serial_speed_khz = serial.speed_khz;
    // Open through the lister rather than wrapping the transport directly.
    // It costs a reconnect, and buys the real discovery path: the same
    // selector semantics, the same factory, the same code a probe-rs release
    // would run. Testing the shortcut would prove nothing about the driver.
    let selector: DebugProbeSelector = DebugProbeSelector {
        vendor_id: factory::ESPRESSIF_VID,
        product_id: factory::USB_SERIAL_JTAG_PID,
        interface: None,
        serial_number: Some(link_selector.clone()),
    };
    drop(serial);
    let lister = Lister::with_lister(Box::new(factory::EspBridgeLister::new()));
    let mut probe = lister
        .open(selector)
        .with_context(|| format!("failed to open the bridge at {link_selector}"))?;
    probe.select_protocol(WireProtocol::Swd)?;
    // probe-rs's own connect-under-reset, driven through the standard
    // `target_reset_assert`/`_deassert` this probe now implements, rather than
    // a bespoke flag of ours doing the same thing worse.
    let mut session = if under_reset {
        probe.attach_under_reset(target.as_str(), Permissions::default())
    } else {
        probe.attach(target.as_str(), Permissions::default())
    }
    .with_context(|| format!("failed to attach to {target}"))?;

    match args.command {
        Action::Lines => unreachable!("line-state command returned before probe attachment"),
        Action::UartReceive => {
            unreachable!("UART receive command returned before probe attachment")
        }
        Action::UartResetCapture { .. } => {
            unreachable!("UART reset-capture command returned before probe attachment")
        }
        Action::MuxScan => {
            unreachable!("mux scan command returned before probe attachment")
        }
        Action::Recover { .. } => {
            unreachable!("line recovery returned before probe attachment")
        }
        Action::WireProbe { .. } => {
            unreachable!("wire probe returned before probe attachment")
        }
        Action::DapPoke { .. } => {
            unreachable!("DAP poke returned before probe attachment")
        }
        Action::ApWriteProbe { .. } => {
            unreachable!("AP write probe returned before probe attachment")
        }
        Action::SpiLoopback { .. } => {
            unreachable!("SPI loopback returned before probe attachment")
        }
        Action::Ping { .. } => unreachable!("ping returned before probe attachment"),
        Action::Echo { .. } => unreachable!("echo returned before probe attachment"),
        Action::PinMap => unreachable!("pin map returned before probe attachment"),
        Action::Identify => unreachable!("identify returned before probe attachment"),
        Action::ListProbes => unreachable!("probe listing returned before a link was opened"),
        Action::Wifi(_) => unreachable!("wifi provisioning returned before probe attachment"),
        Action::Profile { .. } => unreachable!("profile returned before probe attachment"),
        Action::FastDump { .. } => unreachable!("fast dump returned before probe attachment"),
        Action::ResetLines => {
            unreachable!("reset-line-state command returned before probe attachment")
        }
        Action::ResetAssert | Action::ResetRelease | Action::ResetCycle { .. } => {
            unreachable!("reset command returned before probe attachment")
        }
        Action::PadSelfTest => {
            unreachable!("pad self-test returned before probe attachment")
        }
        Action::RecoveryProbe { .. } => {
            unreachable!("recovery probe returned before probe attachment")
        }
        Action::SwdioCycle { .. } => {
            unreachable!("SWDIO cycle returned before probe attachment")
        }
        Action::EnterRomBoot => {
            unreachable!("ROM boot command returned before probe attachment")
        }
        Action::Probe => {
            let mut core = session.core(0)?;
            println!("target={target} status={:?}", core.status()?);
        }
        Action::ProbeUnderReset => {
            let mut core = session.core(0)?;
            println!(
                "target={target} status={:?} attached_under_reset=true",
                core.status()?
            );
        }
        Action::Flash { image } => {
            let mut options = DownloadOptions::default();
            options.verify = true;
            download_file_with_options(&mut session, &image, Format::default(), options)
                .with_context(|| format!("failed to flash {}", image.display()))?;
            session.core(0)?.reset()?;
            println!("flashed={} target={target} verified=true", image.display());
        }
        Action::Bench { address, size } => {
            let mut core = session.core(0)?;
            let mut contents = vec![0_u8; size];
            let before_trips = ROUND_TRIPS.load(Ordering::Relaxed);
            let started = Instant::now();
            core.read_8(address, &mut contents)?;
            let elapsed = started.elapsed();
            let round_trips = ROUND_TRIPS.load(Ordering::Relaxed) - before_trips;
            let kib_per_second = size as f64 / 1024.0 / elapsed.as_secs_f64();
            println!(
                "read bytes={size} address=0x{address:08x} seconds={:.3} kib_per_second={kib_per_second:.1} \
                 words_per_second={:.0} swd_khz={} round_trips={round_trips} words_per_trip={:.1} \
                 us_per_trip={:.0}",
                elapsed.as_secs_f64(),
                size as f64 / 4.0 / elapsed.as_secs_f64(),
                serial_speed_khz,
                size as f64 / 4.0 / round_trips as f64,
                elapsed.as_secs_f64() * 1e6 / round_trips as f64,
            );
        }
        Action::Program {
            image,
            backup,
            address,
        } => {
            let wanted = std::fs::read(&image)
                .with_context(|| format!("failed to read {}", image.display()))?;
            let format = match image.extension().and_then(|value| value.to_str()) {
                Some("bin") => Format::Bin(probe_rs::flashing::BinOptions {
                    base_address: Some(address),
                    skip: 0,
                }),
                Some("hex") => Format::Hex,
                _ => Format::default(),
            };

            // Back up before erasing, always. A device that turns out to be
            // the wrong one, or an image that turns out to be wrong, is
            // recoverable only if this happened first.
            let flash_bytes = usize::from(detected_flash_kib) * 1024;
            if flash_bytes == 0 {
                bail!("flash size unknown, so the part cannot be backed up before erasing");
            }
            {
                let mut core = session.core(0)?;
                let mut existing = vec![0_u8; flash_bytes];
                core.read_8(0x0800_0000, &mut existing)?;
                std::fs::write(&backup, &existing)
                    .with_context(|| format!("failed to write {}", backup.display()))?;
                println!(
                    "backed_up={} bytes={flash_bytes} sha256={}",
                    backup.display(),
                    hex_digest(&existing)
                );
            }

            let mut options = DownloadOptions::default();
            options.verify = true;
            download_file_with_options(&mut session, &image, format, options)
                .with_context(|| format!("failed to program {}", image.display()))?;

            // probe-rs verified through its own flash algorithm; read it back
            // over plain memory access as an independent check.
            let mut written = vec![0_u8; wanted.len()];
            {
                let mut core = session.core(0)?;
                core.read_8(address, &mut written)?;
            }
            if written != wanted {
                let differing = written
                    .iter()
                    .zip(&wanted)
                    .position(|(left, right)| left != right)
                    .unwrap_or(0);
                bail!(
                    "read-back differs from {} at offset {differing}",
                    image.display()
                );
            }
            session.core(0)?.reset()?;
            println!(
                "programmed={} target={target} bytes={} sha256={} verified=read-back",
                image.display(),
                wanted.len(),
                hex_digest(&wanted)
            );
        }
        Action::Dump { output, size } => {
            let mut core = session.core(0)?;
            let mut contents = vec![0_u8; size];
            let started = Instant::now();
            core.read_8(0x0800_0000, &mut contents)?;
            let elapsed = started.elapsed();
            std::fs::write(&output, &contents)
                .with_context(|| format!("failed to write {}", output.display()))?;
            println!(
                "dumped={} address=0x08000000 bytes={size} target={target} seconds={:.3} \
                 kib_per_second={:.1}",
                output.display(),
                elapsed.as_secs_f64(),
                size as f64 / 1024.0 / elapsed.as_secs_f64(),
            );
        }
    }
    Ok(())
}

/// Finds the attached ESP32-C3 bridge without pinning its MAC address, so a
/// replacement board does not need a rebuild.
fn discover_bridge_port() -> Result<PathBuf> {
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

/// A known STM32 part, keyed by the `DEV_ID` field of its DBGMCU IDCODE.
struct Family {
    dev_id: u16,
    name: &'static str,
    /// Where DBGMCU lives, since it is not at one address across families.
    dbgmcu: u32,
    /// Where the factory-programmed flash size in kibibytes lives.
    flash_size: u32,
    /// Where the 96-bit unique id lives.
    uid: u32,
    /// probe-rs target, chosen by flash size where a family spans several.
    target: fn(u16) -> &'static str,
}

/// DBGMCU addresses worth trying, most likely first. A family cannot be
/// identified before it is known, so detection reads each in turn and keeps
/// the first that decodes to something recognised.
const DBGMCU_CANDIDATES: [u32; 2] = [0x4001_5800, 0xE004_2000];

const FAMILIES: &[Family] = &[
    Family {
        dev_id: 0x466,
        name: "STM32G03x/G04x",
        dbgmcu: 0x4001_5800,
        flash_size: 0x1FFF_75E0,
        uid: 0x1FFF_7590,
        target: |kib| {
            if kib <= 32 {
                "STM32G030K6Tx"
            } else {
                "STM32G030K8Tx"
            }
        },
    },
    Family {
        dev_id: 0x460,
        name: "STM32G07x/G08x",
        dbgmcu: 0x4001_5800,
        flash_size: 0x1FFF_75E0,
        uid: 0x1FFF_7590,
        target: |kib| {
            if kib <= 64 {
                "STM32G071CBTx"
            } else {
                "STM32G071RBTx"
            }
        },
    },
    Family {
        dev_id: 0x467,
        name: "STM32G0B1/G0C1",
        dbgmcu: 0x4001_5800,
        flash_size: 0x1FFF_75E0,
        uid: 0x1FFF_7590,
        target: |_| "STM32G0B1RETx",
    },
    Family {
        dev_id: 0x456,
        name: "STM32G05x/G06x",
        dbgmcu: 0x4001_5800,
        flash_size: 0x1FFF_75E0,
        uid: 0x1FFF_7590,
        target: |_| "STM32G051K8Tx",
    },
    Family {
        dev_id: 0x413,
        name: "STM32F405/F407/F415/F417",
        dbgmcu: 0xE004_2000,
        flash_size: 0x1FFF_7A22,
        uid: 0x1FFF_7A10,
        target: |_| "STM32F407VETx",
    },
];

/// What the target turned out to be.
struct Identity {
    dev_id: u16,
    rev_id: u16,
    family: Option<&'static Family>,
    flash_kib: u16,
    uid: [u32; 3],
}

impl Identity {
    fn target(&self) -> Option<&'static str> {
        self.family.map(|family| (family.target)(self.flash_kib))
    }

    fn describe(&self) -> String {
        let name = self.family.map_or("unrecognised", |family| family.name);
        format!(
            "dev_id=0x{:03x} rev_id=0x{:04x} family={name} flash={} KiB \
             uid={:08x}{:08x}{:08x}",
            self.dev_id, self.rev_id, self.flash_kib, self.uid[2], self.uid[1], self.uid[0]
        )
    }
}

/// A bank-0 Debug Port register address.
fn dp_address(address: u8) -> RegisterAddress {
    RegisterAddress::DpRegister(DpRegisterAddress {
        address,
        bank: Some(0),
    })
}

/// Brings the debug port up without probe-rs, which cannot be asked to attach
/// before the part is known.
fn power_up_debug_port(serial: &mut SerialDapProbe, under_reset: bool) -> Result<()> {
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

/// Reads target words over the bridge's bulk path.
fn read_words(serial: &mut SerialDapProbe, address: u32, count: usize) -> Result<Vec<u32>> {
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

/// Works out what is on the wire, rather than trusting what was asked for.
///
/// A probe-rs attach succeeds against whatever target name it is handed, so it
/// cannot answer this; only DBGMCU can.
fn identify(serial: &mut SerialDapProbe) -> Result<(Identity, bool)> {
    // A target that resets repeatedly — a corrupt or half-written image will do
    // it — answers nothing to a plain attach, because it is back in reset
    // before the first transfer lands. Falling back to connect-under-reset is
    // the difference between "unidentifiable" and "recoverable", so it is
    // automatic rather than a flag the operator has to know to reach for.
    match identify_once(serial, false) {
        Ok(identity) => Ok((identity, false)),
        Err(_) => identify_once(serial, true).map(|identity| (identity, true)),
    }
}

fn identify_once(serial: &mut SerialDapProbe, under_reset: bool) -> Result<Identity> {
    power_up_debug_port(serial, under_reset)?;
    for candidate in DBGMCU_CANDIDATES {
        let Ok(words) = read_words(serial, candidate, 1) else {
            continue;
        };
        let idcode = words[0];
        let dev_id = (idcode & 0xfff) as u16;
        if dev_id == 0 || dev_id == 0xfff {
            continue;
        }
        let family = FAMILIES
            .iter()
            .find(|family| family.dev_id == dev_id && family.dbgmcu == candidate);
        let (flash_kib, uid) = match family {
            Some(family) => {
                let flash = read_words(serial, family.flash_size, 1)
                    .map(|words| (words[0] & 0xffff) as u16)
                    .unwrap_or(0);
                let uid = read_words(serial, family.uid, 3)
                    .map(|words| [words[0], words[1], words[2]])
                    .unwrap_or([0; 3]);
                (flash, uid)
            }
            None => (0, [0; 3]),
        };
        return Ok(Identity {
            dev_id,
            rev_id: (idcode >> 16) as u16,
            family,
            flash_kib,
            uid,
        });
    }
    bail!("no DBGMCU identity register responded; the target may be held in reset")
}

/// Lowercase hex of a SHA-256 digest, so a backup can be quoted and compared.
fn hex_digest(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn wait_or_stop(duration: Duration, stop: &AtomicBool) {
    let deadline = Instant::now() + duration;
    while !stop.load(Ordering::Acquire) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Either link the bridge protocol runs over.
///
/// The frames, the sequencing and every command are the same; only the bytes'
/// route differs, so nothing above this needs to know which one is in use.
trait Link: std::io::Read + std::io::Write + Send {}
impl<T: std::io::Read + std::io::Write + Send> Link for T {}

struct Transport {
    port: Box<dyn Link>,
    sequence: u16,
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
    fn connect(endpoint: &str) -> Result<Self> {
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
        })
    }

    fn open(path: &std::path::Path) -> Result<Self> {
        // The ESP32-C3's USB Serial/JTAG is a native USB endpoint: the baud
        // rate is a formality, and the frame timeout only bounds a stall.
        let port = serialport::new(path.to_string_lossy(), 921_600)
            .timeout(Duration::from_millis(50))
            .open()
            .with_context(|| format!("failed to open {}", path.display()))?;
        Ok(Self {
            port: Box::new(port),
            sequence: 0,
        })
    }

    fn command(&mut self, command: Command, payload: &[u8]) -> Result<Vec<u8>> {
        let sequence = self.send(command, payload)?;
        self.receive(sequence)
    }

    /// Queues a request without waiting for its reply, so the bridge can work
    /// on it while a previous reply is still draining over USB.
    fn send(&mut self, command: Command, payload: &[u8]) -> Result<u16> {
        ROUND_TRIPS.fetch_add(1, Ordering::Relaxed);
        self.sequence = self.sequence.wrapping_add(1);
        let sequence = self.sequence;
        let mut request = [0u8; MAX_FRAME + 2];
        let request_len = encode_request(sequence, command, payload, &mut request)
            .map_err(|_| anyhow::anyhow!("request is too large"))?;
        self.port.write_all(&[0])?;
        self.port.write_all(&request[..request_len])?;
        Ok(sequence)
    }

    fn receive(&mut self, sequence: u16) -> Result<Vec<u8>> {
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut frame = [0u8; MAX_FRAME + 2];
        let mut frame_len = 0usize;
        // One read syscall per frame rather than per packet. A 4 KiB reply took
        // nine calls at 512 bytes, and their latency lands squarely between the
        // reply arriving and the next request going out.
        let mut chunk = [0u8; MAX_FRAME + 2];
        while Instant::now() < deadline {
            let read = match self.port.read(&mut chunk) {
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
                if let Ok(response) = decode_response(&mut frame[..frame_len])
                    && response.sequence == sequence
                {
                    return match response.status {
                        Status::Ok => Ok(response.payload.to_vec()),
                        status => {
                            bail!("bridge status {status:?}, detail={:02x?}", response.payload)
                        }
                    };
                }
                frame_len = 0;
            }
        }
        bail!("USB bridge response timeout")
    }
}

#[derive(Debug)]
struct SerialDapProbe {
    transport: Transport,
    attached: bool,
    protocol: Option<WireProtocol>,
    speed_khz: u32,
    block_words: usize,
    attach_under_reset: bool,
}

impl SerialDapProbe {
    fn open(path: &std::path::Path) -> Result<Self> {
        Self::with_transport(Transport::open(path)?)
    }

    fn connect(endpoint: &str) -> Result<Self> {
        Self::with_transport(Transport::connect(endpoint)?)
    }

    fn with_transport(transport: Transport) -> Result<Self> {
        let mut transport = transport;
        let hello = transport.command(Command::Hello, &[])?;
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
    fn set_engine(&mut self, engine: Engine) -> Result<()> {
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
    fn set_pin_map(&mut self, swapped: bool) -> Result<()> {
        self.command(Command::SetPinMap, &[u8::from(swapped)])?;
        Ok(())
    }

    fn command(&mut self, command: Command, payload: &[u8]) -> Result<Vec<u8>, DebugProbeError> {
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
