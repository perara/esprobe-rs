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
| Bulk read | 453 KiB/s |

The Wi-Fi transport is implemented, and its host half is tested against a
loopback stub bridge — framing over a socket, endpoint parsing, the handshake
and factory selection. It has **not yet run against a bridge on a real
network**, because the Wi-Fi it was to be tested on would not associate. Treat
the end-to-end path as unproven until it has.

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

### Identify, don't assume

`probe-rs attach` succeeds against whatever target name you hand it — the same
board will attach happily as an `STM32G030K8`, an `STM32G071RBTx` and an
`STM32G081RBTx`. It answers "can I talk to this", never "what is this". So
`--target` is optional: omit it and the part is identified from its own DBGMCU
registers first.

## Licence

MIT or Apache-2.0, at your option.
