# esprobe firmware

The ESP32-C3 half of [esprobe-rs](../..): it drives SWD and serves the wire
protocol over USB and, when configured, over Wi-Fi.

The SWD engine is hardware-clocked through GPSPI2, so SWCLK comes from the
peripheral rather than a software loop, and a bulk read of a target's flash
runs at around 450 KiB/s. A bit-bang engine remains available and is what the
hardware engine was validated against.

## Building

```bash
cp wifi.env.example .env.local     # then fill it in; never commit it
./scripts/build.sh
espflash flash --monitor target/riscv32imc-esp-espidf/release/esprobe-firmware
```

`.env.local` needs `WIFI_COUNTRY` — the two-letter regulatory domain this probe
will be used in — plus the fallback access point's name and password. The build
refuses to run without them. `WIFI_COUNTRY` is deliberately not defaulted: see
the diagnostics section below for what world-safe mode does to this chip.

Wi-Fi is optional. Without a board link the bridge still works over USB.

The access point is published whenever the probe has no network to join, and
again for about thirty seconds after a full ladder of join attempts has failed
— station and access point share one radio, so joining necessarily takes it
down, and a probe given a wrong passphrase would otherwise be reachable only
over the cable the access point exists to replace.

## Wiring

Any four pins will do; the map is a build-time constant, defaulting to the
assignment below.

| Signal | Default |
| --- | ---: |
| `PROG_SWDIO` | GPIO1 |
| `PROG_SWCLK` | GPIO2 |
| `RESET_ALL` | GPIO21 |

```bash
PIN_SWDIO=1 PIN_SWCLK=2 PIN_RESET_ALL=21 ./scripts/build.sh
```

**Avoid GPIO2, GPIO8 and GPIO9 for SWCLK.** They are ESP32-C3 strapping pins,
sampled at reset. A target that holds SWCLK low at power-up — held in reset,
unpowered, or with a pull-down — then stops the ESP32-C3 booting at all. This
is not hypothetical: it cost an afternoon, and the symptom is a programmer that
has silently dropped into ROM download mode.

The map cannot be a runtime setting. Pins are claimed and driven during
start-up, so a firmware built for the wrong board drives its outputs into
another board's outputs before any command could correct it — which on the
board this grew up on heated an analog switch until it had to be replaced.
`esprobe pin-map` asks the running firmware what it was built for, so the map
can be checked rather than assumed.

## Board-specific extras

This firmware began as the control hub for one particular board, and still
carries parts that only make sense there: a analog switch mux for selecting between
four SWD targets, a display UART bridge, and the HTTP endpoints that drive
them. They are harmless if the pins are unused, and will be feature-gated
rather than left as permanent furniture. The rest of the HTTP API — liveness,
identify, program — works on any board; see [HTTP control API](#http-control-api).

## Board-specific pin map

The defaults are revision 2, which is what `esprobe pin-map` reports off a
board. They were the v1.0 schematic for a long time after the hardware moved,
because every deployed image was built with `PIN_*` overrides and nothing
compared the two — so reading the source to find which pin a signal was on gave
the wrong answer. `the_default_pinmap_is_what_a_board_reports` now fails if they
drift apart again.

| Signal | ESP32-C3 | v1.0 was |
| --- | ---: | ---: |
| `STEP_BIN1` | GPIO0 | — |
| `PROG_SWDIO` | GPIO1 | GPIO4 |
| `PROG_SWCLK` | GPIO2 | GPIO3 |
| `DISP_RX` | GPIO3 | GPIO6 |
| `DISP_TX` | GPIO4 | GPIO5 |
| `STEP_BIN2` | GPIO5 | — |
| `STEP_AIN2` | GPIO6 | — |
| `STEP_AIN1` | GPIO7 | — |
| `ASW_S0` | GPIO10 | GPIO1 |
| `ASW_S1` | GPIO20 | GPIO2 |
| `RESET_ALL` (originally supplied as `AUX_0_RST`) | GPIO21 | GPIO7 |

A v1.0 board still builds:

```bash
PIN_SWDIO=4 PIN_SWCLK=3 PIN_RESET_ALL=7 PIN_ASW_S0=1 PIN_ASW_S1=2 \
  PIN_DISP_TX=5 PIN_DISP_RX=6 ./scripts/build.sh
```

GPIO12-17 are the SPI flash and GPIO18/19 are the USB pair the probe is reached
over; a test rejects any claimed pin in either range, because driving one of
those does not produce a signal, it produces a brick.

## The actuator station

Four of the pins revision 2 left free drive a motor driver, one H-bridge per winding:
`AIN1`/`AIN2` on GPIO7/GPIO6 for coil A, `BIN1`/`BIN2` on GPIO0/GPIO5 for coil
B. The two halves of a winding must stay on the same bridge — swapping the two
inputs of one bridge reverses that winding, and the motor buzzes instead of
turning.

Open `http://<probe>/actuator` for a control page: drag left or right to jog, let go
to stop. Or drive it directly:

| Route | Method | Body |
| --- | --- | --- |
| `/api/v1/actuator` | GET | — |
| `/api/v1/actuator/jog` | POST | `{"steps_per_s":-250}` |
| `/api/v1/actuator/move` | POST | `{"steps":200,"steps_per_s":400}` |
| `/api/v1/actuator/stop` | POST | — |
| `/api/v1/actuator/release` | POST | — |

Two things it does on its own, both because a motor driver has no current chopping
and a actuator standing still with its windings energised is a resistor:

- **It lets go.** After the last step it holds position for 400 ms and then
  coasts. A station needing indefinite holding torque needs a driver that can
  limit current, not a longer timeout.
- **A jog expires.** `jog` is honoured for 400 ms and must be refreshed. If the
  browser tab closes or the Wi-Fi drops mid-drag, the last thing the firmware
  heard was "keep going" — and a actuator that keeps going because nobody said
  stop is how a board drives itself into its end stop.

GPIO3 and GPIO4 are not ESP32-C3 strapping pins. GPIO4 is also a pad-JTAG
signal at reset; firmware explicitly claims it as GPIO SWDIO while the
control-hub host connection continues to use the separate USB Serial/JTAG
controller.

## Build and flash

Install the Rust-on-ESP prerequisites (`ldproxy`, `espflash`, and a recent
Clang), then:

```bash
cp wifi.env.example .env.local
# Set the local password in .env.local. Never commit it.
./scripts/build.sh
espflash flash --monitor \
  target/riscv32imc-esp-espidf/release/esprobe-firmware
```

`.env.local` is ignored by Git and holds only the fallback access point's name
and password. The network the probe joins is not built in — provision it at
runtime with `esprobe wifi set`, which stores it in NVS. That way no image
carries a real passphrase, and repointing a probe does not mean rebuilding it.

The access point comes up only when there is no network to join, and uses the
ESP-IDF default address `192.168.71.1`. Station and access point share one
radio, so running both leaves the board being dragged onto the access point's
channel mid-authentication.

## Telling a radio fault from a firmware fault

`src/bin/radio-check.rs` is a control image: an open access point and a scan
loop, and nothing else — no GPIO claims, no GPSPI2, no USB driver, no servers.
Flash it, then look for `esprobe-radio-check` from another machine.

```bash
cargo build --release --bin radio-check
espflash flash --monitor target/riscv32imc-esp-espidf/release/radio-check
```

It logs the negotiated transmit power and what the scan hears every ten
seconds.

Two things this found, in the order they had to be found:

1. **The radio would not transmit at all** until the regulatory domain was set.
   ESP-IDF starts in world-safe mode, and in that state this chip received
   normally — scanning, sane signal strengths — while emitting nothing. It
   looked exactly like a dead transmitter. `WIFI_COUNTRY` at build time (see
   `set_regulatory_domain`) is what fixes it, and nothing else can be measured
   until it is set.
2. **Full transmit power does not associate on an ESP32-C3 SuperMini.** That
   board's antenna is poorly matched, and a mismatched antenna reflects the
   power amplifier's output back into it. At 20 dBm and 15 dBm the access
   point never answers; at 13 dBm it associates immediately, reproducibly,
   from the same position. `TX_POWER_LADDER` walks 20, 15, 13, 10 and 8 dBm on
   successive attempts and stores whichever one worked in NVS, so the next
   boot joins in eight seconds rather than sixty.

   The link stays marginal — round trips of 60 to 160 ms to a gateway that
   answers in 0.3 ms, which is 802.11 retrying. Soldering a 28-30 mm wire
   antenna to that board is the physical fix; the ladder is what makes it work
   without one. If the scan reports access points while `esprobe-radio-check` is
invisible from a machine in the same room, the radio receives but does not
transmit, and that is not something firmware can fix. Erasing the flash
entirely (`espflash erase-flash`) first is worth doing once: it forces a full
RF calibration instead of reusing stored calibration data, which is the one
storage-side cause of exactly that symptom.

## esprobe: a probe-rs probe

The host tool is [`crates/esprobe`](../esprobe). It is a probe-rs *backend*, not a
reimplementation: ADIv5, the vendor debug sequences, the chip database and the
CMSIS-Pack flash algorithms are all probe-rs's, reached through the standard
`DebugProbe`/`RawDapAccess` traits. Programming an STM32G071 needed no
G0 flash-controller code here at all.

The one thing probe-rs cannot supply is knowing this bridge exists, so
`src/factory.rs` adds a `ProbeFactory` and a `ProbeLister`, registered with
`Lister::with_lister`. That is the supported out-of-tree extension point —
`AllProbesLister`'s driver table is a private constant — and the lister
*delegates* to the built-in one, so a J-Link or CMSIS-DAP on the same machine
still enumerates:

```bash
esprobe list-probes
# Atmel-ICE CMSIS-DAP -- 03eb:2141-0:J42700032306 (CMSIS-DAP)
# STLink V2 -- 0483:3748:30 (ST-LINK)
# ESP32-C3 SWD bridge (/dev/ttyACM0) -- 303a:1001:/dev/ttyACM0 (ESP32-C3 SWD bridge)
```

**One factory, both links.** The selector's serial-number field decides: an
`address:port` is a network bridge, anything else is a serial device — the same
rule probe-rs's Black Magic driver uses, so `--probe` means one thing whether
the bridge is on the bench or across the room. `ESPROBE_NETWORK=host:3333` adds
a network bridge to the listing, since nothing on IP announces itself.

USB discovery filters on the product string `USB JTAG/serial debug unit`, with
`ESPROBE_ANY_SERIAL` to override. That is necessary rather than decorative: an
ESP32 being *debugged* presents the same `303a:1001` as one acting as the
bridge, and probe-rs's own `espusbjtag` driver claims it too. `sifliuart`
resolves the identical ambiguity the identical way.

### Reset belongs to probe-rs

`target_reset_assert`, `target_reset_deassert` and `swj_pins` are implemented
against the bridge's reset commands, so `probe.attach_under_reset()` — probe-rs's
own connect-under-reset — works instead of a bespoke flag doing the same job
worse. Two things had to be true for that:

- `ResetAssert` must not touch the mux or the SWD link. A debug sequence
  asserts reset *after* attaching, so tearing the link down underneath it fails
  the very next transfer. `RESET_ALL` reaches every target regardless of where
  the mux points, so selecting one was never needed.
- The reset guard covers bulk memory reads only, never raw DP/AP access.
  Connect-under-reset is *built* on talking to the debug port while reset is
  asserted; guarding that would break the recovery path the guard exists to
  protect.

Commands that need the ecosystem — `probe`, `program`, `flash`, `dump`, `bench`
— open through the lister, which costs a reconnect and buys the real discovery
path. Commands that are about the bridge itself, or that want the bandwidth,
talk the native protocol directly: `fast-dump` reads at 453 KiB/s where the same
read through probe-rs's chunking manages 77 KiB/s.

## Which board, and which chip

Two things are established by measurement rather than assumption, because
getting either wrong is expensive.

**The GPIO map is a build-time constant**, defaulting to the v1.0 schematic and
overridden per board:

```bash
PIN_SWDIO=1 PIN_SWCLK=2 PIN_RESET_ALL=21 \
  PIN_ASW_S0=10 PIN_ASW_S1=20 PIN_DISP_TX=3 PIN_DISP_RX=4 ./scripts/build.sh
```

It cannot be a runtime setting: the pins are claimed and driven during
start-up, so a firmware built for the wrong board drives its outputs into
another board's outputs before any command could correct it. That contention is
not theoretical — it heated the analog switch on a v1.0 board until the part
had to be replaced. `build.rs` tracks the `PIN_*` variables, without which
cargo silently reuses the previous binary; and `esprobe pin-map` asks the
*running* firmware what it was built for, so the map can be checked rather than
assumed.

**The target is identified from its own registers.** `probe-rs attach` succeeds
against whatever target name it is handed — the same board attached happily as
`STM32G030K8`, `STM32G071RBTx` and `STM32G081RBTx` — so it cannot answer what a
chip is. Only DBGMCU can:

```bash
cargo run -- identify
# dev_id=0x460 rev_id=0x2001 family=STM32G07x/G08x flash=128 KiB uid=...
# probe_rs_target=STM32G071RBTx
```

`--target` is optional and detection runs when it is omitted. DBGMCU is read at
`0x40015800` then `0xE0042000`, since its address depends on the family and the
family is what is being determined; `DEV_ID` then selects the flash-size and UID
locations. The table covers `0x466` G03x/G04x, `0x460` G07x/G08x, `0x467`
G0B1/G0C1, `0x456` G05x/G06x and `0x413` F405/F407, and lives host-side so a new
part costs no reflash. An unrecognised `DEV_ID` is an error asking for
`--target`, not a guess.

### Programming

One command does the whole sequence, with nothing to remember:

```bash
cargo run -- program firmware.elf --backup before.bin
```

It detects the part from its own registers, **saves and hashes the existing
flash before anything is erased**, programs through probe-rs's flash algorithm
with its verification enabled, and then reads the written range back over plain
memory access and compares it against the file. That last step matters: the
algorithm verifying its own work is not the same as an independent read
agreeing with the source. `.bin`, `.hex` and ELF are all accepted; `--address`
places a raw binary.

The backup is never skipped, including for a part that reads blank — "blank" is
a claim about a device that has just been identified, and the cost of being
wrong about it is unrecoverable.

If the target does not answer a plain attach, detection retries with
connect-under-reset and carries that decision into the flashing session. A part
that resets repeatedly — which a corrupt or half-written image will cause —
answers nothing otherwise, and that is exactly the state from which it most
needs reprogramming.

### A read that returns zeros is not a read

A core held in reset answers its entire address space with zeros while the
CoreSight ROM table keeps responding, so a bulk read completes, reports success,
and returns a plausible blank device. Two changes make that impossible to
mistake:

- Any command touching the target bus is refused with `TargetInReset` while the
  shared reset line is asserted.
- `recovery-probe` performs its fallible work inside a closure and releases
  reset afterwards, so no failure path can leave it asserted. An earlier version
  released only after the last `?`, and a mid-sequence failure left a target
  held in reset that then read as blank.

Hashes alone do not establish that a read worked, either: four reads of a blank
part at four clock rates agree perfectly. Check the bytes.

## probe-rs over ESP32-C3 USB serial

The included host adapter implements probe-rs's raw ARM DAP interface over a
versioned COBS/CRC16 serial protocol:

```bash
cd ../esprobe
cargo run -- lines
cargo run -- reset-lines
cargo run -- reset-assert
cargo run -- reset-release
cargo run -- reset-cycle --seconds 2
cargo run -- probe
cargo run -- probe-under-reset
cargo run -- recovery-probe
cargo run -- recovery-probe --delay-us 1000
cargo run -- mux-scan
cargo run -- uart-receive
cargo run -- uart-reset-capture
cargo run -- uart-reset-capture --swapped
cargo run -- enter-rom-boot
cargo run -- flash /absolute/path/to/ir-led-embassy
cargo run -- dump /absolute/path/to/backup.bin --size 524288
cargo run -- bench --size 65536
cargo run -- recover
cargo run -- wire-probe
cargo run -- wire-probe --split
cargo run -- wire-probe --after-read 2 --before-write 2
cargo run -- dap-poke
cargo run -- ap-write-probe --value 1
cargo run -- spi-loopback --bits 33 --pattern 1
cargo run -- ping --count 200
cargo run -- fast-dump backup.bin --size 524288
```

`fast-dump` brings the debug port up itself and then reads through
`MemoryRead`, bypassing probe-rs entirely. It is the fastest way to take a
backup; `dump` remains the probe-rs path and is the one to use when its target
support matters.

Global flags: `--engine {hardware,bit-bang}`, `--speed-khz`, `--swap-swd` when
the harness has SWDIO and SWCLK crossed, `--no-blocks` to fall back to one
register transfer per word, and `-v` for probe-rs's own trace.

`recover` clocks the bus with SWDIO released. A target interrupted part-way
through a read data phase keeps driving SWDIO until it has clocked out the rest
of that phase, which is indistinguishable from an unpowered target — and the
held-low guard then refuses the line reset that would clear it. Only SWCLK is
driven, so this can never contend with the far end. `attach` now runs the same
recovery before reporting a held-low line.

`dap-poke` drives raw DAP transfers by hand and prints CTRL/STAT after each
one, which isolates which access sets a sticky error without probe-rs in the
way. `wire-probe` sends a DPIDR read and returns the reply undecoded, so a
backend's actual bit placement can be read off rather than assumed.

`lines` releases both ESP pins to inputs and reports their physical idle
levels. A powered/reset STM32 normally presents SWDIO high and SWCLK low;
`reset-lines` performs the same passive sampling while `RESET_ALL` is held
low and again after release, without producing any SWD clock;
`reset-assert` holds the shared reset net low for meter measurements until
`reset-release` or an ESP reboot returns GPIO7 to high impedance. This resets
the STM32 and both AUX modules. `reset-cycle` alternates between those states
and releases RESET_ALL when Ctrl-C stops it;
`swdio=0` is a fail-closed indication to check target/mux power and wiring. The firmware
refuses to start an SWD transaction when released SWDIO is low (bridge
diagnostic detail `04`), limits both programming pads to the ESP32-C3's 5 mA
drive-strength setting, and releases both pads before preloading their output
latches. This prevents the bridge from deliberately driving into a detected
low line; it does not make a short, overvoltage, missing ground, or back-power
path electrically safe.

`probe-under-reset` is an explicit recovery path for a powered target that was
previously programmed successfully but leaves SWDIO passive-low. It asserts
RESET_ALL, prepares the SWD pin state, releases reset to the board pull-up, and
attaches immediately before application firmware can reclaim PA13/PA14. It
never enables STM32 erase or programming by itself.

`enter-rom-boot` first sends the current Rust firmware's COBS/CRC command at
115200 8N1. It then changes UART1 to the STM32 ROM bootloader's required
115200 8E1 format before sending sync byte `0x7f`. If the application command
does not answer, it also tries PA14/BOOT0 during reset. `uart-receive` is a
passive capture, while `uart-reset-capture` resets and captures startup bytes
atomically so host reconnection cannot miss them. An empty response is not a
successful bootloader handshake; ROM flashing must require ACK `0x79`.

`mux-scan` verifies the physical GPIO1/GPIO2 selector levels, passively
samples all four U14 channels, and performs read-only DP identification only
on STM32/AUX channels whose released SWDIO is high. It always restores the
all-low STM32 selection. `recovery-probe --delay-us` permits a bounded sweep
across the reset-RC/application-start window without rebuilding firmware.
`uart-reset-capture --swapped` is passive and exists only to exclude reversed
display-net direction during bring-up.

## Wire speed

Two engines can clock the wire, selected with `--engine`.

`hardware` (default) shifts every field through GPSPI2, so SWCLK is generated
by the peripheral rather than a software loop. That removes the frequency
ceiling, and it removes the reason transfers ever needed a critical section:
an interrupt between fields cannot deform a waveform the hardware is emitting.

`bit-bang` drives every edge from the CPU. Both pads are channels of one
dedicated-GPIO CSR, so a whole edge — new data level and new clock level
together — is a single instruction, and the half-period is timed against the
CPU's performance counter. Above the rate where polling that counter costs more
than the delay it asks for, the loop free-runs and the firmware **measures**
what it achieved instead of repeating the request back. It remains the
reference the hardware engine was validated against.

Reading 512 KiB of STM32F407VET6 flash on flying leads:

| Path | Time | Throughput |
| --- | ---: | ---: |
| `dump` — through probe-rs | 6.7 s | 76.6 KiB/s |
| `fast-dump` — bulk bridge command | **1.02 s** | **503 KiB/s** |

Every image produced along the way, on both engines and at clocks from 2 MHz to
20 MHz, hashes identically. None of this speed is bought with silent corruption.

### Where the time goes

Two commands bound the problem, and neither answer is SWD.

`profile` reports the firmware's own cost per word: **5.9 µs**, of which 4.0 µs
is inside GPSPI2 transactions — 2.04 of them per word, and 2.3 µs of that is the
wire itself at 20 MHz — leaving 1.9 µs everywhere else. `echo` returns a payload
without touching the wire, measuring the transport alone: **612 KiB/s** at 4 KiB
frames, a marginal 1.4 µs per byte.

So a 1024-word block costs about 6.0 ms on the wire and 6.3 ms in the transport.
Those two were originally strictly serialised, and the arithmetic said so before
any change did: their sum times the block count was exactly the runtime. What
followed, in order of what each was worth:

- **Overlap the two.** `fast-dump` keeps a request in flight while the previous
  reply drains, so a block costs the larger of the two rather than their sum.
  1.95 s to 1.37 s. Queueing more than one changes nothing — the bridge is
  single-threaded, so one is enough to keep it busy.
- **A table-driven CRC.** At kibibyte frames the bit-at-a-time form ran between
  the wire read and the USB write and delayed both. 1.37 s to 1.12 s, and it
  raised the transport ceiling itself from 468 to 599 KiB/s.
- **Build the firmware for speed, not size.** The transfer loop runs once per
  word of every read and was compiled at `opt-level = "s"`. Switching to `3`
  cut the non-peripheral cost from 2.7 to 1.9 µs per word: 1.12 s to 1.02 s,
  for four bytes of manifest and 15% more flash.
- **One round trip per 1024 words instead of 51.** `MemoryRead` walks the MEM-AP
  on the ESP, reprogramming TAR at each auto-increment boundary. probe-rs spends
  4.1 ms per chunk of its own accounting on top of a 280 µs round trip.
- **Two peripheral transactions per word instead of four.** A read's ACK, data,
  parity and turnaround are clocked as one 38-bit transfer. Sound for reads and
  *only* for reads: with SWDIO released the target drives nothing after a WAIT,
  so the surplus clocks are idle ones. A write must still withhold its data
  phase after a refused ACK; doing otherwise desynchronises the next transfer.

Each side can be timed on its own, which accounts for the result exactly.
`echo` with the wire idle costs 6.32 ms per 4 KiB block; `profile` with the USB
idle costs 5.97 ms. Their sum is 12.3 ms and a perfect overlap would be 6.3 ms.
The measured figure is 7.94 ms, so **73% of the wire time is already hidden
behind the transport**.

The 1.6 ms that is not hidden is contention for the one CPU the chip has. The
SWD engine is a tight register-polling loop, and moving a 4 KiB reply takes
sixty-four interrupts to hand 64-byte packets to the endpoint; the two compete,
and neither can be made to run during the other. Three attempts to close it all
measured worse or made no difference, which is worth recording so they are not
repeated:

- **A dedicated USB writer task**, so the wire could run while the transport
  drained. The ESP32-C3 is single-core: this creates no parallelism, because the
  copy into the driver's ring buffer costs the same cycles on the same CPU, and
  it adds context switches. 1.02 s became 1.05 s. The overlap it was meant to
  buy already exists — the ring buffer is what decouples the write from the
  host's reads.
- **A deeper transmit ring buffer**, from three frames to eight. No change. In
  steady state the wire produces slightly faster than USB drains, so the buffer
  fills whatever its depth; depth smooths bursts and cannot beat a drain rate.
- **Whole-frame reads on the host** instead of 512-byte chunks. Throughput
  unchanged, though run-to-run variance disappeared, so it was kept.

Closing the remainder means not spending CPU on polling — DMA for the SPI
transfers — which a 38-bit field with a direction change in the middle of every
word does not lend itself to. The other lever, dropping COBS for length-prefixed
framing, saves roughly 200 µs of the 7.94 ms and costs the property that lets
the parser resynchronise on an endpoint it shares with the console log.

Raising the SWD clock does not help; past about 8 MHz the wire is no longer the
limit.

### The partial-byte transmit

GPSPI2 does not transmit a partial byte in its second buffer word. Asked for
33 bits it emits `W0` bit 24 in the 33rd position and never reads `W1` at all,
which corrupts the parity bit of every SWD write whose bit 24 differs from its
parity — about half of them. Whole-byte transmits are exact, `W1` included.

The SWD write data phase is therefore padded from 33 bits to five whole bytes.
The seven added bits are **zeros**, which is what SWD requires of an idle line;
padding them high instead presents seven start bits to the target and breaks
the link, which it duly did.

`spi-loopback` is what found this. `FSPID` and `FSPIQ` are both routed to the
SWDIO pad, so a full-duplex transfer reads back the peripheral's own output
through the pad — a direct measurement of what GPSPI2 emits, with no target
involved and no inference from a pass/fail verdict:

```bash
cargo run -- spi-loopback --bits 33 --pattern 16777216   # bit 24 in, bit 32 out
cargo run -- spi-loopback --bits 40 --pattern 16777216   # exact
```

Both engines also agree on a framing detail that took measurement to pin down.
GPSPI2 samples on the falling edge, half a period earlier than the bit-bang
loop, so the first clock of a read doubles as the turnaround and the borrowed
clock is repaid at the trailing one. `leading_turnaround` and
`trailing_turnaround` exist so each backend can state that for itself.

Raise the clock only after repeated identity reads succeed on the assembled
board; through the analog switch the mux settling time, not the engine, sets the
ceiling.

On the original board, at boot GPIO5 display TX, GPIO7 `RESET_ALL`, GPIO4
SWDIO, and GPIO3 SWCLK remain inputs. The mux selectors use the all-low STM32 position. Any operation
that would drive a high level first reselects STM32 and requires its released
SWDIO pull-up to be visible. Display TX is enabled only for the duration of a
power-gated transmission, and GPIO7 asserts reset low before returning to an
input so the carrier board's 10 kΩ pull-up performs the release.

`--port` defaults to the only attached Espressif USB Serial/JTAG device, so a
replacement board needs no rebuild; pass it explicitly when more than one is
connected. `program` identifies the target from its own DBGMCU registers and
uses probe-rs's flash algorithm for whatever it turns out to be, enables full
readback verification, and resets the core only after a successful download.

This is a probe-rs library adapter, not a CMSIS-DAP device. The stock
`probe-rs` CLI cannot dynamically load an out-of-tree serial probe backend, so
use `crates/esprobe` for this transport. Logs and framed bridge traffic share
the ESP32-C3 USB Serial/JTAG endpoint; the framing parser discards unrelated
console output.

## HTTP control API

A convenience layer over the same hardware, for scripting a probe that is
already on the network. The bridge protocol on tcp/3333 is the real interface;
nothing here is needed to use `esprobe`.

```bash
curl http://PROBE_IP/health                        # address, SSID, liveness
curl -X POST http://PROBE_IP/api/v1/stm32/probe    # identify the attached target
curl --data-binary @firmware.bin \
  http://PROBE_IP/api/v1/stm32/flash               # program it
```

### Endpoints specific to the board this came from

This firmware grew on an carrier board that puts a analog switch analog
multiplexer between the ESP32-C3's SWD pins and four targets, and a UART to a
display. Those endpoints are still served, and are inert on a plain devkit —
`RESET_ALL` is the only pin they touch that a three-wire setup has.

```bash
curl -X POST http://PROBE_IP/api/v1/mux/stm32      # point the mux at a target
curl -X POST http://PROBE_IP/api/v1/mux/aux2        # ...or the others
curl -X POST http://PROBE_IP/api/v1/aux0/reset     # compatibility path: RESET_ALL
curl --data-binary @frame.bin http://PROBE_IP/api/v1/display/tx
curl http://PROBE_IP/api/v1/display/rx --output received.bin
```

The flash endpoint accepts a raw binary linked at `0x08000000`, rejects empty
or oversized images, requires exact STM32G03x/04x device ID `0x466`, halts the
core, page-erases only the image range, writes 64-bit flash units, compares the
complete uploaded range, locks flash, requests a core reset, and releases
SWDIO/SWCLK to high impedance.

## Validation boundary

Host tests prove pin constants, mux truth table, framing/CRC behavior, and
hardware-neutral SWD/flash logic. A successful USB handshake proves only that
the ESP bridge is alive. A probe-rs attach proves target-side SWD transactions;
programming plus enabled readback verification proves the downloaded image.
None of these replace signal-integrity, strap-voltage, timing, or IR waveform
measurements with suitable instruments.

If either MCU becomes unexpectedly warm, remove both the ESP USB supply and IR
board supply immediately. Do not probe, flash, or reconnect until the board has
cooled and unpowered resistance plus powered rail voltages have been checked.
