# esprobe-rs

Turn an ESP32-C3 into an SWD debug probe — over USB, or over Wi-Fi.

`esprobe` is a [probe-rs](https://github.com/probe-rs/probe-rs) probe backend.
It is not a reimplementation of anything above the wire: ADIv5, the vendor
debug sequences, the chip database and the CMSIS-Pack flash algorithms are all
probe-rs's, reached through the standard `DebugProbe`/`RawDapAccess` traits.
Programming an STM32G071 through it needed no G0 flash-controller code here.

## Why

A commodity ESP32-C3 devkit costs a few euros and has Wi-Fi. Nothing in
probe-rs currently drives SWD over a network link, so a board on a bench across
the room — or sealed in a test fixture, or on a robot — needs a probe cabled to
a host. This gives it one that does not.

## Status

Verified against an STM32G071 and an STM32F407 over USB:

| | |
| --- | ---: |
| Identify, attach, connect-under-reset | works |
| Program, with backup and read-back verification | works |
| Source-level stepping, breakpoints, backtraces | works |
| Bulk read, 8 MHz default wire clock | 309 KiB/s |
| Bulk read, `--speed-khz 16000` | 408 KiB/s |

The wire is the limit, not the transport: raising `--depth` moves the bulk
figure by about one percent, and raising the clock moves it proportionally.
8 MHz is the default because it is what a mux path and unshielded bench leads
carry reliably; direct-wired to a devkit, 16 MHz is fine. Both dumps above are
byte-identical.

### Provisioning

Which network a probe joins is set at runtime, over whichever link is already
working:

```bash
esprobe wifi set --ssid my-network   # prompts for the passphrase
esprobe wifi status
esprobe wifi forget                  # back to the probe's own access point
```

Credentials live in NVS on the device, so they survive a power cycle and a
reflash, and no image carries the passphrase of a network you join. The
fallback access point's own password is the one thing compiled in, so treat a
built image as carrying that.

### Status of the network transport

Working end to end. On an ESP32-C3 SuperMini joined to infrastructure Wi-Fi:

```
$ esprobe --url 192.168.0.93 identify
dev_id=0x460 rev_id=0x2001 family=STM32G07x/G08x flash=128 KiB uid=323538353035510e006e0039
probe_rs_target=STM32G071RBTx
```

Same target, same UID, as over USB. Two things had to be fixed to get there,
both in `crates/esprobe-firmware/README.md`: ESP-IDF's world-safe regulatory
mode stops the radio transmitting at all, and the SuperMini's antenna needs the
transmit power backed off before an access point will answer it.

Round trips over Wi-Fi run 60–160 ms against a gateway that answers in 0.3 ms,
which is that board's marginal link retrying, not the bridge. USB stays the
faster transport by a wide margin; the network one is for a board on a bench
you are not sitting at.

## Installing

```bash
git clone https://github.com/perara/esprobe-rs
cd esprobe-rs
cargo install --path crates/esprobe     # or: cargo build --release
```

`cargo build --release` leaves the binary at `target/release/esprobe`. The
firmware is a separate cross-compiled build; see
[the firmware README](crates/esprobe-firmware/README.md).

## Using it as a library

The probe-rs backend is the point of this crate, so it is a library as well as
a binary. Anything that wants a probe on the far side of a network can offer
the bridge alongside probe-rs's own drivers:

```rust
use probe_rs::probe::list::Lister;

let lister = Lister::with_lister(Box::new(esprobe::factory::EspBridgeLister::new()));
for probe in lister.list_all() {
    println!("{}", probe.identifier);
}
```

## Layout

- `crates/esprobe` — the CLI and the probe-rs probe backend
- `crates/esprobe-protocol` — the wire contract, shared by both halves
- `crates/esprobe-firmware` — what runs on the ESP32-C3

Both ends depend on the same protocol crate, so they cannot drift. The firmware
is excluded from the workspace because it cross-compiles for
`riscv32imc-esp-espidf` with its own toolchain; build it from its own directory.

## Hardware

An ESP32-C3 devkit and three wires — SWDIO, SWCLK and ground — with reset
optional but worth having. The pin map is a build-time constant; see the
firmware README, including which pins to avoid and why.

## Using it

```bash
esprobe list-probes          # alongside every probe probe-rs can see
esprobe identify             # read the target's own identity registers
esprobe program fw.elf --backup before.bin
esprobe fast-dump flash.bin --size 524288
```

`--port` selects a serial bridge, `--url host[:port]` a network one. Nothing
above the transport changes between them.

As a probe-rs backend the same choice is made from the selector: an address,
an `address:port`, or a dotted name is a network bridge, and anything else is
opened as a serial device. A single-label host is not guessed at, since
nothing distinguishes `probe` from a device name — write `probe:3333`.

## Debugging

A probe you can only flash is not a debug probe. Everything below is
probe-rs's `Core` reached through the bridge, and works over USB or Wi-Fi.

```bash
esprobe core status                  # running, halted, sleeping, and why
esprobe core halt / run / step --count 5
esprobe core reset-halt              # catch it before the first instruction
esprobe core registers               # the whole file
esprobe core read  0x08000000 --words 4
esprobe core write 0x20000000 0xcafebabe 0xdeadbeef
esprobe core run-to 0x08000123       # breakpoint, resume, wait, release
```

`run-to` exists because `break-set` followed by `run` cannot work: each
invocation attaches and detaches, so a breakpoint set by one command is gone
before the next runs. `run-to` does all of it in one session.

### GDB

```bash
esprobe gdb --port 1234              # then, in gdb:  target remote :1234
```

A real GDB server, so `gdb`, VS Code's `cortex-debug` or CLion drive the target
through the bridge: registers, memory, Thumb disassembly, single-stepping, and
hardware breakpoints that GDB sets and clears itself. Software breakpoints are
served from hardware units too, because a Cortex-M's code lives in flash and
GDB's default trap-instruction write would go nowhere.

Over Wi-Fi, run `set remotetimeout 30` first. Reading the register file is
seventeen round trips, which takes several seconds across a network link and
overruns GDB's two-second default; the server says so when it starts.

### RTT

```bash
esprobe rtt                          # scan RAM for the control block
esprobe rtt --control-block 0x20000000 --idle-ms 500
```

Streams the target's RTT channels, found by scanning RAM or from an address you
give it.

### Seeing what the bridge is doing

`--verbose` traces probe-rs and this bridge together;
`RUST_LOG=esprobe=trace` adds every protocol frame with its sequence, command,
size and round-trip time, which is what separates a wire fault from a
transport one.

### Identify, don't assume

`probe-rs attach` succeeds against whatever target name you hand it — the same
board will attach happily as an `STM32G030K8`, an `STM32G071RBTx` and an
`STM32G081RBTx`. It answers "can I talk to this", never "what is this". So
`--target` is optional: omit it and the part is identified from its own DBGMCU
registers first.

## Licence

MIT or Apache-2.0, at your option.
