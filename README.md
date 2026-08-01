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

### Provisioning

Which network a probe joins is set at runtime, over whichever link is already
working:

```bash
esprobe wifi set --ssid my-network   # prompts for the passphrase
esprobe wifi status
esprobe wifi forget                  # falls back to the probe's own access point
```

Credentials live in NVS on the device, so they survive a power cycle and a
reflash, and no image ever carries a passphrase.

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
