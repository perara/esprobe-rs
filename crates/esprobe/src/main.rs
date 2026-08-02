use esprobe::{
    Engine, ROUND_TRIPS, SerialDapProbe, discover_bridge_port, dp_address, gdb, identify,
};

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use esprobe_protocol::clock::DEFAULT_CLOCK_HZ;
use esprobe_protocol::frame::Command;
use probe_rs::architecture::arm::{RawDapAccess, RegisterAddress};
use probe_rs::flashing::{DownloadOptions, Format, download_file_with_options};
use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbe, DebugProbeSelector, WireProtocol};
use probe_rs::{MemoryInterface, Permissions};

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
    /// Trace probe-rs and this bridge. `RUST_LOG=esprobe=trace` adds every
    /// protocol frame; `RUST_LOG` overrides the level entirely.
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
#[derive(Subcommand)]
enum CoreAction {
    /// Report whether the core is running, halted, or sleeping, and why.
    Status,
    /// Halt the core and report where it stopped.
    Halt,
    /// Resume a halted core.
    Run,
    /// Execute instructions one at a time, reporting the address after each.
    Step {
        #[arg(long, default_value_t = 1)]
        count: usize,
    },
    /// Reset the core and catch it before it runs a single instruction.
    ResetHalt,
    /// Let a halted core run from a reset vector.
    Reset,
    /// Dump every core register the architecture defines.
    Registers,
    /// Read words from the target's address space.
    Read {
        #[arg(value_parser = parse_address)]
        address: u64,
        #[arg(long, default_value_t = 1)]
        words: usize,
    },
    /// Write one or more words to the target's address space.
    Write {
        #[arg(value_parser = parse_address)]
        address: u64,
        #[arg(value_parser = parse_address, num_args = 1.., required = true)]
        values: Vec<u64>,
    },
    /// Resume until the core reaches an address, then report where it stopped.
    ///
    /// The breakpoint is set, the core resumed and the halt awaited inside one
    /// session, which is the only way it can work: every invocation of this
    /// tool attaches and detaches, so a breakpoint set by one command is gone
    /// before the next can run to it.
    RunTo {
        #[arg(value_parser = parse_address)]
        address: u64,
        #[arg(long, default_value_t = 5000)]
        timeout_ms: u64,
    },
    /// Set a hardware breakpoint. See `run-to` for one that can be reached.
    BreakSet {
        #[arg(value_parser = parse_address)]
        address: u64,
    },
    /// Clear a hardware breakpoint.
    BreakClear {
        #[arg(value_parser = parse_address)]
        address: u64,
    },
    /// Report how many hardware breakpoint units the core has.
    BreakInfo,
}

/// Accepts `0x`-prefixed hex as well as decimal, because addresses are written
/// in hex everywhere else a person meets them.
fn parse_address(text: &str) -> Result<u64, String> {
    let trimmed = text.trim();
    let parsed = match trimmed
        .strip_prefix("0x")
        .or_else(|| trimmed.strip_prefix("0X"))
    {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => trimmed.parse(),
    };
    parsed.map_err(|error| format!("{text:?} is not an address: {error}"))
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
        /// Omit to be prompted, rather than passing the passphrase on a
        /// command line that the shell will remember. For an open network,
        /// omit it and submit the prompt empty.
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
    /// Release RESET_ALL to the carrier board's pull-up.
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
    UartReceive {
        /// How long to listen, in milliseconds. The bridge clears the receive
        /// buffer first, so this is the whole capture window.
        #[arg(long, default_value_t = 100)]
        ms: u16,
    },
    /// Send bytes to the STM32 display UART and print whatever it answers.
    UartSend {
        /// The bytes to send, as hex. Whitespace and a leading `0x` are ignored.
        hex: String,
    },
    /// Reset the STM32 and atomically capture its startup UART bytes.
    UartResetCapture {
        /// Passively listen on GPIO5 instead of the schematic-normal GPIO6.
        #[arg(long)]
        swapped: bool,
    },
    /// Read-only SWD diagnostic on STM32, AUX0, and AUX1 mux channels.
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
    /// Halt, step and inspect the target's core.
    #[command(subcommand)]
    Core(CoreAction),
    /// Serve the GDB remote protocol, so a debugger can drive the target.
    Gdb {
        #[arg(long, default_value_t = 1234)]
        port: u16,
        /// Leave the core running instead of halting it when GDB attaches.
        #[arg(long)]
        no_halt: bool,
    },
    /// Stream the target's RTT output.
    Rtt {
        /// Look for the control block at this exact address instead of
        /// scanning RAM, which is faster and avoids a false match.
        #[arg(long, value_parser = parse_address)]
        control_block: Option<u64>,
        /// Stop after this long with no new data. Omit to stream until Ctrl-C.
        #[arg(long)]
        idle_ms: Option<u64>,
        /// Read only this channel; by default every up channel is streamed.
        #[arg(long)]
        channel: Option<usize>,
    },
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
                    // Both halves by default. The bridge's own frames are at
                    // trace: `RUST_LOG=esprobe=trace` shows every request and
                    // reply, which is what tells a wire fault apart from a
                    // transport one.
                    .unwrap_or_else(|_| "probe_rs=debug,esprobe=debug".into()),
            )
            .with_writer(std::io::stderr)
            .init();
    }
    // Listing must not open a link, so it runs before one is chosen.
    if matches!(args.command, Action::ListProbes) {
        let lister = Lister::with_lister(Box::new(esprobe::factory::EspBridgeLister::new()));
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
    // Before any SWD configuration is pushed to the bridge. Provisioning the
    // radio has nothing to do with the wire, and `set_pin_map` below would
    // quietly revert a bridge built with its pins swapped — the same class of
    // mistake `pin-map` exists to catch.
    if let Action::Wifi(action) = &args.command {
        return run_wifi(&mut serial, action);
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
        Action::UartSend { hex } => {
            let payload = parse_hex(hex)?;
            let response = serial.command(Command::UartSend, &payload)?;
            print!("stm32_uart_rx=");
            for byte in response {
                print!("{byte:02x}");
            }
            println!();
            return Ok(());
        }
        Action::UartReceive { ms } => {
            let payload = ms.to_le_bytes();
            let response = serial.command(Command::UartReceive, &payload)?;
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
        Action::Wifi(_) => unreachable!("wifi runs before any SWD configuration"),
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
            for (target, name) in [(0_u8, "stm32"), (1, "aux2"), (2, "aux0"), (3, "aux1")] {
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
        | Action::Core(_)
        | Action::Gdb { .. }
        | Action::Rtt { .. }
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
        vendor_id: esprobe::factory::ESPRESSIF_VID,
        product_id: esprobe::factory::USB_SERIAL_JTAG_PID,
        interface: None,
        serial_number: Some(link_selector.clone()),
    };
    drop(serial);
    let lister = Lister::with_lister(Box::new(esprobe::factory::EspBridgeLister::new()));
    let mut probe = lister
        .open(selector.clone())
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
        Action::UartReceive { .. } => {
            unreachable!("UART receive command returned before probe attachment")
        }
        Action::UartSend { .. } => {
            unreachable!("UART send command returned before probe attachment")
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
        Action::Core(action) => {
            run_core(&mut session, &action)?;
        }
        Action::Gdb { port, no_halt } => {
            // A closure rather than the session itself: a wire fault needs
            // the whole attach redone, and only here is what it takes to do
            // that still in scope.
            let attach_target = target.clone();
            let attach_selector = selector;
            let attach_under_reset = under_reset;
            gdb::serve(
                session,
                move || {
                    let lister =
                        Lister::with_lister(Box::new(esprobe::factory::EspBridgeLister::new()));
                    let mut probe = lister.open(attach_selector.clone())?;
                    probe.select_protocol(WireProtocol::Swd)?;
                    let session = if attach_under_reset {
                        probe.attach_under_reset(attach_target.as_str(), Permissions::default())
                    } else {
                        probe.attach(attach_target.as_str(), Permissions::default())
                    }?;
                    Ok(session)
                },
                port,
                !no_halt,
                args.url.is_some(),
            )?;
        }
        Action::Rtt {
            control_block,
            idle_ms,
            channel,
        } => {
            run_rtt(&mut session, control_block, idle_ms, channel)?;
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

/// Streams whatever the target is writing to its RTT up channels.
///
/// RTT is a ring buffer in the target's own RAM with a signature the host
/// scans for, so it needs no extra wires and no cooperation from the core
/// beyond firmware that uses it. This is what makes a debug probe useful for
/// printf-style work rather than only for stopping and starting.
fn run_rtt(
    session: &mut probe_rs::Session,
    control_block: Option<u64>,
    idle_ms: Option<u64>,
    channel: Option<usize>,
) -> Result<()> {
    let memory_map = session.target().memory_map.clone();
    let mut core = session.core(0)?;
    let region = match control_block {
        Some(address) => probe_rs::rtt::ScanRegion::Exact(address),
        None => probe_rs::rtt::ScanRegion::Ram,
    };
    let mut rtt = probe_rs::rtt::Rtt::attach_region(&mut core, &region).with_context(|| {
        match control_block {
            Some(address) => format!("no RTT control block at {address:#010x}"),
            None => format!(
                "no RTT control block found in the {} RAM regions probe-rs knows for this target; \
                 pass --control-block if the firmware places it somewhere unusual",
                memory_map.len()
            ),
        }
    })?;

    let channels: Vec<usize> = rtt
        .up_channels()
        .iter()
        .map(|up| up.number())
        .filter(|number| channel.is_none_or(|wanted| wanted == *number))
        .collect();
    if channels.is_empty() {
        bail!("the control block declares no matching up channel");
    }
    for up in rtt.up_channels().iter() {
        eprintln!(
            "channel {} {:?} buffer={} bytes{}",
            up.number(),
            up.name().unwrap_or("(unnamed)"),
            up.buffer_size(),
            if channels.contains(&up.number()) {
                ""
            } else {
                " (skipped)"
            }
        );
    }

    let mut buffer = [0u8; 1024];
    let mut idle_since = Instant::now();
    let mut total = 0usize;
    loop {
        let mut moved = false;
        for up in rtt.up_channels().iter_mut() {
            if !channels.contains(&up.number()) {
                continue;
            }
            let read = up.read(&mut core, &mut buffer)?;
            if read == 0 {
                continue;
            }
            moved = true;
            total += read;
            // Straight through, bytes as they came: RTT carries whatever the
            // firmware wrote, which is not always valid UTF-8 mid-buffer.
            use std::io::Write as _;
            let mut out = std::io::stdout();
            out.write_all(&buffer[..read])?;
            out.flush()?;
        }
        if moved {
            idle_since = Instant::now();
        } else if let Some(idle_ms) = idle_ms
            && idle_since.elapsed() >= Duration::from_millis(idle_ms)
        {
            eprintln!("\nidle for {idle_ms} ms; {total} bytes read");
            return Ok(());
        }
        if !moved {
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

/// Stops, starts and inspects the target's core.
fn run_core(session: &mut probe_rs::Session, action: &CoreAction) -> Result<()> {
    let mut core = session.core(0)?;
    // Long enough for a core that is asleep or waiting on a slow bus, short
    // enough that a target which will never halt says so rather than hanging.
    let timeout = Duration::from_millis(500);
    match action {
        CoreAction::Status => println!("status={:?}", core.status()?),
        CoreAction::Halt => {
            let information = core.halt(timeout)?;
            println!("halted pc={:#010x}", information.pc);
        }
        CoreAction::Run => {
            core.run()?;
            println!("running");
        }
        CoreAction::Step { count } => {
            if !core.core_halted()? {
                core.halt(timeout)?;
            }
            for index in 0..*count {
                let information = core.step()?;
                println!("step={} pc={:#010x}", index + 1, information.pc);
            }
        }
        CoreAction::ResetHalt => {
            let information = core.reset_and_halt(timeout)?;
            println!("reset and halted pc={:#010x}", information.pc);
        }
        CoreAction::Reset => {
            core.reset()?;
            println!("reset and running");
        }
        CoreAction::Registers => {
            // Halted first: reading a register of a running core reports
            // whatever it happened to contain mid-instruction, which is worse
            // than refusing.
            let was_running = !core.core_halted()?;
            if was_running {
                core.halt(timeout)?;
            }
            for register in core.registers().all_registers() {
                match core.read_core_reg::<u64>(register.id()) {
                    Ok(value) => println!("{:<12} {value:#018x}", register.name()),
                    Err(error) => println!("{:<12} unavailable ({error})", register.name()),
                }
            }
            if was_running {
                core.run()?;
            }
        }
        CoreAction::Read { address, words } => {
            let mut buffer = vec![0u32; *words];
            core.read_32(*address, &mut buffer)?;
            for (index, word) in buffer.iter().enumerate() {
                println!("{:#010x} {word:#010x}", address + (index * 4) as u64);
            }
        }
        CoreAction::Write { address, values } => {
            let words: Vec<u32> = values.iter().map(|value| *value as u32).collect();
            core.write_32(*address, &words)?;
            // Read back rather than trust the write: a word that lands in
            // unmapped or write-protected space fails silently on this bus.
            let mut read_back = vec![0u32; words.len()];
            core.read_32(*address, &mut read_back)?;
            for (index, (wrote, read)) in words.iter().zip(read_back.iter()).enumerate() {
                let at = address + (index * 4) as u64;
                let note = if wrote == read { "" } else { "  MISMATCH" };
                println!("{at:#010x} wrote {wrote:#010x} reads {read:#010x}{note}");
            }
            if words != read_back {
                bail!("the target did not take every word");
            }
        }
        CoreAction::RunTo {
            address,
            timeout_ms,
        } => {
            if !core.core_halted()? {
                core.halt(timeout)?;
            }
            core.set_hw_breakpoint(*address)?;
            core.run()?;
            let deadline = Instant::now() + Duration::from_millis(*timeout_ms);
            let reached = loop {
                if core.core_halted()? {
                    break true;
                }
                if Instant::now() >= deadline {
                    break false;
                }
                std::thread::sleep(Duration::from_millis(2));
            };
            // Cleared either way: leaving a unit armed in the core's flash
            // patch block outlives this process and surprises the next one.
            let cleared = core.clear_hw_breakpoint(*address);
            if !reached {
                cleared?;
                bail!("the core did not reach {address:#010x} within {timeout_ms} ms");
            }
            let program_counter = core.program_counter().id();
            let pc = core.read_core_reg::<u64>(program_counter)?;
            cleared?;
            println!("halted at breakpoint pc={pc:#010x}");
        }
        CoreAction::BreakSet { address } => {
            core.set_hw_breakpoint(*address)?;
            println!("breakpoint set at {address:#010x}");
        }
        CoreAction::BreakClear { address } => {
            core.clear_hw_breakpoint(*address)?;
            println!("breakpoint cleared at {address:#010x}");
        }
        CoreAction::BreakInfo => {
            println!(
                "hardware breakpoint units={}",
                core.available_breakpoint_units()?
            );
        }
    }
    Ok(())
}

/// Reads or changes the network the bridge joins.
fn run_wifi(serial: &mut SerialDapProbe, action: &WifiAction) -> Result<()> {
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
    Ok(())
}

/// Parses a hex byte string, tolerating whitespace, `0x`, and `:` separators.
fn parse_hex(text: &str) -> anyhow::Result<Vec<u8>> {
    let cleaned: String = text
        .trim()
        .trim_start_matches("0x")
        .chars()
        .filter(|c| !c.is_whitespace() && *c != ':' && *c != '_')
        .collect();
    if cleaned.len() % 2 != 0 {
        anyhow::bail!("a hex payload needs an even number of digits, got {}", cleaned.len());
    }
    (0..cleaned.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&cleaned[i..i + 2], 16)
                .with_context(|| format!("`{}` is not a hex byte", &cleaned[i..i + 2]))
        })
        .collect()
}
