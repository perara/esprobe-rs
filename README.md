<div align="center">

# esprobe-rs

**Turn a €3 ESP32-C3 into a real SWD debug probe — over USB, or over Wi-Fi.**

[![CI](https://github.com/perara/esprobe-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/perara/esprobe-rs/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/esprobe.svg)](https://crates.io/crates/esprobe)
[![docs.rs](https://img.shields.io/docsrs/esprobe)](https://docs.rs/esprobe)
[![licence](https://img.shields.io/crates/l/esprobe.svg)](#licence)
[![MSRV](https://img.shields.io/badge/MSRV-1.88-blue.svg)](#installing)

*Flash it. Debug it. From across the room.*

</div>

---

```console
$ esprobe --url 192.168.0.93 identify
dev_id=0x460 rev_id=0x2001 family=STM32G07x/G08x flash=128 KiB uid=323538353035510e006e0039
probe_rs_target=STM32G071RBTx

$ esprobe --url 192.168.0.93 gdb --port 1234
gdb server on tcp/1234; connect with: target remote :1234
```

```console
(gdb) break work
Breakpoint 1, g0test::work (iteration=129) at src/main.rs:101
(gdb) step
g0test::accumulate (seed=129) at src/main.rs:46
(gdb) backtrace
#0  g0test::accumulate (seed=129) at src/main.rs:46
#1  0x0800042a in g0test::work (iteration=129) at src/main.rs:101
#2  0x080002ac in g0test::main_loop () at src/main.rs:112
#3  0x080002ec in g0test::reset_handler () at src/main.rs:164
```

Three wires, a devkit you already own, and a target that no longer has to sit
next to you.

## Why

A commodity ESP32-C3 costs a few euros and has Wi-Fi. Nothing in probe-rs
drives SWD over a network link, so a board on a bench across the room — or
sealed in a test fixture, or on a robot — needs a probe cabled to a host. This
gives it one that does not.

`esprobe` is a [probe-rs](https://github.com/probe-rs/probe-rs) *backend*, not
a reimplementation. ADIv5, the vendor debug sequences, the chip database and
the CMSIS-Pack flash algorithms are all probe-rs's, reached through the
standard `DebugProbe` / `RawDapAccess` traits. Programming an STM32G071 needed
no G0 flash-controller code here at all.

## What works

Verified on an STM32G071 and an STM32F407, over both transports.

| | |
| --- | ---: |
| Identify, attach, connect-under-reset | ✅ |
| Program, with backup and read-back verification | ✅ |
| Halt, step, breakpoints, registers, memory | ✅ |
| Source-level stepping and backtraces in GDB | ✅ |
| RTT streaming | ✅ |
| Bulk read, 8 MHz default wire clock | **309 KiB/s** |
| Bulk read, `--speed-khz 16000` | **408 KiB/s** |

The wire is the limit, not the transport: `--depth` moves the bulk figure by
about one percent, the clock moves it proportionally. 8 MHz is the default
because it is what a mux path and unshielded bench leads carry reliably;
direct-wired to a devkit, 16 MHz is fine. Dumps at both clocks are
byte-identical, over USB and Wi-Fi alike.

## Quick start

```bash
cargo install esprobe
```

Wire three pins — SWDIO, SWCLK, ground — with reset optional but worth having.
Build and flash the firmware from
[`crates/esprobe-firmware`](crates/esprobe-firmware), then:

```bash
esprobe identify                       # what is actually on the other end
esprobe program firmware.elf           # backs up and verifies, always
esprobe fast-dump flash.bin --size 131072
```

### Going wireless

Point the probe at a network without rebuilding anything:

```bash
esprobe wifi set --ssid my-network     # prompts for the passphrase
esprobe wifi status
esprobe --url 192.168.0.93 identify    # everything above, now over the air
```

Credentials live in NVS on the device, so they survive a power cycle and a
reflash, and no image carries the passphrase of a network you join. The
fallback access point's own password is the one thing compiled in, so treat a
built image as carrying that.

## Debugging

Everything here is probe-rs's `Core`, reached through the bridge, and works
over either transport.

```bash
esprobe core status                    # running, halted, sleeping, and why
esprobe core halt / run / step --count 5
esprobe core reset-halt                # catch it before the first instruction
esprobe core registers
esprobe core read  0x08000000 --words 4
esprobe core write 0x20000000 0xcafebabe 0xdeadbeef
esprobe core run-to 0x08000123         # breakpoint, resume, wait, release
```

`run-to` exists because `break-set` followed by `run` cannot work: each
invocation attaches and detaches, so a breakpoint set by one command is gone
before the next runs. `run-to` does all of it in one session.

### GDB

```bash
esprobe gdb --port 1234                # then, in gdb:  target remote :1234
```

A real GDB server, so `gdb`, VS Code's `cortex-debug` or CLion drive the target
through the bridge: registers, memory, Thumb disassembly, single-stepping and
hardware breakpoints that GDB manages itself. Software breakpoints are served
from hardware units too, because a Cortex-M's code lives in flash and GDB's
default trap-instruction write would go nowhere.

Over Wi-Fi, run `set remotetimeout 30` first. Reading the register file is
seventeen round trips, which takes several seconds across a network link and
overruns GDB's two-second default; the server says so when it starts.

### RTT

```bash
esprobe rtt                            # scan RAM for the control block
esprobe rtt --control-block 0x20000000 --idle-ms 500
```

### Seeing what the bridge is doing

`--verbose` traces probe-rs and this bridge together; `RUST_LOG=esprobe=trace`
adds every protocol frame with its sequence, command, size and round-trip
time, which is what separates a wire fault from a transport one. It is how the
one genuinely nasty bug in this project was found.

## Using it as a library

The probe-rs backend is the point, so this is a library as well as a binary.
Anything that wants a probe on the far side of a network can offer the bridge
alongside probe-rs's own drivers:

```rust
use probe_rs::probe::list::Lister;

let lister = Lister::with_lister(Box::new(esprobe::factory::EspBridgeLister::new()));
for probe in lister.list_all() {
    println!("{}", probe.identifier);
}
```

`--port` selects a serial bridge, `--url host[:port]` a network one; as a
probe-rs backend the same choice is made from the selector. An address, an
`address:port`, or a dotted name is a network bridge; anything else is opened
as a serial device. A single-label host is not guessed at, since nothing
distinguishes `probe` from a device name — write `probe:3333`.

## Identify, don't assume

`probe-rs attach` succeeds against whatever target name you hand it — the same
board will attach happily as an `STM32G030K8`, an `STM32G071RBTx` and an
`STM32G081RBTx`. It answers "can I talk to this", never "what is this". So
`--target` is optional: omit it and the part is identified from its own DBGMCU
registers first.

## Layout

| | |
| --- | --- |
| [`crates/esprobe`](crates/esprobe) | the CLI and the probe-rs backend |
| [`crates/esprobe-protocol`](crates/esprobe-protocol) | the wire contract, shared by both halves |
| [`crates/esprobe-firmware`](crates/esprobe-firmware) | what runs on the ESP32-C3 |

Both ends depend on the same protocol crate, so they cannot drift, and golden
fixtures pin the actual bytes so they cannot drift in meaning either. The
firmware is excluded from the workspace because it cross-compiles for
`riscv32imc-esp-espidf` with its own toolchain; build it from its own
directory.

## Installing

```bash
cargo install esprobe
```

or from source:

```bash
git clone https://github.com/perara/esprobe-rs
cd esprobe-rs
cargo install --path crates/esprobe     # or: cargo build --release
```

Requires Rust 1.88 or newer. The firmware is a separate cross-compiled build;
see [the firmware README](crates/esprobe-firmware/README.md).

## Honest limits

- **One chip family has been exercised hard.** An STM32G071 for the debug and
  bulk-transfer work, an STM32F407 earlier. The detection table claims more
  than has been proven.
- **Wi-Fi is slower, and depends on your antenna.** Round trips run 60–160 ms
  against a gateway answering in 0.3 ms on an ESP32-C3 SuperMini, whose antenna
  is [known to be poor](https://hackaday.com/2025/04/07/simple-antenna-makes-for-better-esp32-c3-wifi/).
  USB stays the faster transport by a wide margin; the network one is for a
  board on a bench you are not sitting at.
- **The firmware's own logic is not host-testable.** Its dependencies only
  build for the target, so the shared protocol crate carries the tests that
  can run anywhere.

## Licence

MIT or Apache-2.0, at your option.
