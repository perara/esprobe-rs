# esprobe-protocol

The wire contract between the [esprobe](https://github.com/perara/esprobe-rs)
ESP32-C3 SWD bridge and its host.

Both ends must agree on this byte for byte, and they are built for different
architectures by different toolchains — the firmware for
`riscv32imc-esp-espidf`, the host for whatever you run `esprobe` on. So it
lives in one crate that each depends on, rather than being copied and left to
drift.

`no_std`, and nothing here allocates.

## What is in it

| Module | |
| --- | --- |
| `frame` | COBS framing, CRC-16/CCITT, the command set, request and response encoding |
| `clock` | GPSPI2 divider arithmetic, so the SWD clock is chosen on the host's terms |
| `wifi` | Length-prefixed credentials, for provisioning a probe over the link it already has |
| `json` | Escaping for the small documents the bridge publishes over HTTP |

## Pinned, not just shared

Sharing a crate stops the two ends drifting apart in source. It does not stop
them drifting in *meaning* — a field reordered, a command renumbered, a CRC
seed changed. `wire_format_fixtures` pins the actual bytes:

```rust
assert_eq!(
    &frame[..length],
    &[11, b'E', b'S', b'P', b'B', 4, 0x34, 0x12, 1, 0x9e, 0x3e, 0],
    "the Hello request encoding moved"
);
```

Change the encoding and a test fails, rather than a probe in a drawer
somewhere quietly failing to answer.

## Licence

MIT or Apache-2.0, at your option.
