//! The binary command protocol for the persistent link.
//!
//! # Why this exists
//!
//! The trackpad used to drive the motor over `POST /api/v1/stepper/jog`. Every
//! rate change was a full HTTP request: request line, headers, a JSON body, a
//! response with its own headers — several hundred bytes to carry a number that
//! fits in two, and a fresh TCP connection whenever keep-alive lapsed, which
//! costs a round trip before the command is even sent. On top of that the page
//! polled for status twice a second whether or not anything had changed.
//!
//! One socket, held open, carrying three bytes per command, with status pushed
//! only when it differs. What is left on the wire is close to the information
//! actually being communicated.
//!
//! # What the ordering guarantee buys
//!
//! The REST path needs sequence numbers because two requests can overtake each
//! other and a jog can land after the stop that was meant to end it. A single
//! TCP connection delivers in order by construction, so on this path that race
//! cannot happen and there is nothing to stamp. The sequence field stays on the
//! REST routes, which still need it.
//!
//! # Layout
//!
//! Little-endian, matching both the RISC-V core and every browser this will
//! meet, so neither end byte-swaps. Fixed-width frames: the length *is* the
//! validation, and a truncated frame is rejected rather than half-applied.

/// Run at a signed rate until told otherwise. `i16` covers the rate limit
/// several times over and keeps the hot frame at three bytes.
pub const CMD_JOG: u8 = 0x01;
/// Stop now.
pub const CMD_STOP: u8 = 0x02;
/// Stop and drop the windings.
pub const CMD_RELEASE: u8 = 0x03;
/// Move a bounded number of steps.
pub const CMD_MOVE: u8 = 0x04;
/// Retune the acceleration.
pub const CMD_ACCEL: u8 = 0x05;
/// Round-trip probe. Answered with [`MSG_PONG`] and nothing else.
pub const CMD_PING: u8 = 0x06;

/// Pushed when the state changes.
pub const MSG_STATUS: u8 = 0x81;
/// The answer to [`CMD_PING`].
pub const MSG_PONG: u8 = 0x82;

/// The longest frame a client can send. Anything larger is refused without
/// being read into memory.
pub const MAX_COMMAND_LEN: usize = 7;

/// A decoded client command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Command {
    Jog(i32),
    Stop,
    Release,
    Move { steps: i32, steps_per_s: u32 },
    Accel(i32),
    Ping,
}

/// What the station reports back.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Status {
    pub position: i32,
    pub rate: i16,
    pub moving: bool,
    pub energised: bool,
    pub remaining: Option<u32>,
}

/// Wire length of an encoded [`Status`].
pub const STATUS_LEN: usize = 12;

const FLAG_MOVING: u8 = 1 << 0;
const FLAG_ENERGISED: u8 = 1 << 1;
const FLAG_HAS_REMAINING: u8 = 1 << 2;

/// Decode one client frame.
///
/// `None` for anything not understood — wrong opcode, wrong length, trailing
/// bytes. A frame that is not exactly what it claims to be is discarded whole:
/// there is no partially-applied command that leaves a motor in a state anyone
/// asked for.
#[must_use]
pub fn decode(frame: &[u8]) -> Option<Command> {
    let (&opcode, body) = frame.split_first()?;
    let exact = |want: usize| (body.len() == want).then_some(());
    match opcode {
        CMD_JOG => {
            exact(2)?;
            Some(Command::Jog(i32::from(i16::from_le_bytes([
                body[0], body[1],
            ]))))
        }
        CMD_STOP => exact(0).map(|()| Command::Stop),
        CMD_RELEASE => exact(0).map(|()| Command::Release),
        CMD_PING => exact(0).map(|()| Command::Ping),
        CMD_MOVE => {
            exact(6)?;
            Some(Command::Move {
                steps: i32::from_le_bytes([body[0], body[1], body[2], body[3]]),
                steps_per_s: u32::from(u16::from_le_bytes([body[4], body[5]])),
            })
        }
        CMD_ACCEL => {
            exact(4)?;
            Some(Command::Accel(i32::from_le_bytes([
                body[0], body[1], body[2], body[3],
            ])))
        }
        _ => None,
    }
}

/// Encode a status push.
#[must_use]
pub fn encode_status(status: &Status) -> [u8; STATUS_LEN] {
    let mut out = [0u8; STATUS_LEN];
    out[0] = MSG_STATUS;
    out[1..5].copy_from_slice(&status.position.to_le_bytes());
    out[5..7].copy_from_slice(&status.rate.to_le_bytes());
    let mut flags = 0;
    if status.moving {
        flags |= FLAG_MOVING;
    }
    if status.energised {
        flags |= FLAG_ENERGISED;
    }
    if let Some(remaining) = status.remaining {
        flags |= FLAG_HAS_REMAINING;
        out[8..12].copy_from_slice(&remaining.to_le_bytes());
    }
    out[7] = flags;
    out
}

/// Decode a status push. Only the page needs this; it exists so the encoder can
/// be tested against something other than a hand-written byte array.
#[must_use]
pub fn decode_status(frame: &[u8]) -> Option<Status> {
    if frame.len() != STATUS_LEN || frame[0] != MSG_STATUS {
        return None;
    }
    let flags = frame[7];
    Some(Status {
        position: i32::from_le_bytes([frame[1], frame[2], frame[3], frame[4]]),
        rate: i16::from_le_bytes([frame[5], frame[6]]),
        moving: flags & FLAG_MOVING != 0,
        energised: flags & FLAG_ENERGISED != 0,
        remaining: (flags & FLAG_HAS_REMAINING != 0)
            .then(|| u32::from_le_bytes([frame[8], frame[9], frame[10], frame[11]])),
    })
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;

    #[test]
    fn a_jog_is_three_bytes_on_the_wire() {
        // The hot frame. An HTTP POST carrying the same number was several
        // hundred bytes plus a response, and this is the whole point of the
        // exercise, so it is worth a test rather than a comment.
        let frame = [CMD_JOG, 0x2c, 0x01];
        assert_eq!(frame.len(), 3);
        assert_eq!(decode(&frame), Some(Command::Jog(300)));
    }

    #[test]
    fn a_negative_rate_survives_the_round_trip() {
        let frame = [CMD_JOG, 0xd4, 0xfe];
        assert_eq!(decode(&frame), Some(Command::Jog(-300)));
    }

    #[test]
    fn the_rate_field_covers_the_whole_range_the_planner_allows() {
        // i16 was chosen to keep the frame small; if the step-rate ceiling ever
        // outgrows it the frame has to widen, and this is where that is caught.
        let limit = crate::stepper::MAX_STEPS_PER_S as i32;
        assert!(
            limit <= i16::MAX as i32,
            "the rate no longer fits the wire format"
        );
        let frame = [CMD_JOG, (limit as i16).to_le_bytes()[0], (limit as i16).to_le_bytes()[1]];
        assert_eq!(decode(&frame), Some(Command::Jog(limit)));
    }

    #[test]
    fn a_truncated_frame_is_discarded_rather_than_half_applied() {
        // A jog missing its second rate byte must not read as a jog at some
        // other rate. There is no partial command that anyone asked for.
        assert_eq!(decode(&[CMD_JOG, 0x2c]), None);
        assert_eq!(decode(&[CMD_JOG]), None);
        assert_eq!(decode(&[CMD_MOVE, 1, 0, 0, 0, 1]), None);
        assert_eq!(decode(&[CMD_ACCEL, 0, 0, 0]), None);
        assert_eq!(decode(&[]), None);
    }

    #[test]
    fn trailing_bytes_are_refused_too() {
        // A frame longer than its opcode implies is not a frame this protocol
        // produced, so it is not one to guess at.
        assert_eq!(decode(&[CMD_STOP, 0]), None);
        assert_eq!(decode(&[CMD_JOG, 1, 0, 0]), None);
        assert_eq!(decode(&[CMD_RELEASE, 9, 9]), None);
    }

    #[test]
    fn an_unknown_opcode_is_refused() {
        assert_eq!(decode(&[0x00]), None);
        assert_eq!(decode(&[0x7f, 1, 2]), None);
        // Server-to-client opcodes must not be accepted as commands.
        assert_eq!(decode(&[MSG_STATUS]), None);
        assert_eq!(decode(&[MSG_PONG]), None);
    }

    #[test]
    fn the_one_byte_commands_decode() {
        assert_eq!(decode(&[CMD_STOP]), Some(Command::Stop));
        assert_eq!(decode(&[CMD_RELEASE]), Some(Command::Release));
        assert_eq!(decode(&[CMD_PING]), Some(Command::Ping));
    }

    #[test]
    fn a_move_carries_its_distance_and_rate() {
        let mut frame = [CMD_MOVE, 0, 0, 0, 0, 0, 0];
        frame[1..5].copy_from_slice(&(-1234i32).to_le_bytes());
        frame[5..7].copy_from_slice(&600u16.to_le_bytes());
        assert_eq!(
            decode(&frame),
            Some(Command::Move {
                steps: -1234,
                steps_per_s: 600
            })
        );
        assert!(frame.len() <= MAX_COMMAND_LEN);
    }

    #[test]
    fn status_survives_the_round_trip() {
        for status in [
            Status::default(),
            Status {
                position: -2_000_000,
                rate: -1000,
                moving: true,
                energised: true,
                remaining: None,
            },
            Status {
                position: i32::MAX,
                rate: i16::MAX,
                moving: true,
                energised: false,
                remaining: Some(0),
            },
            Status {
                position: i32::MIN,
                rate: i16::MIN,
                moving: false,
                energised: true,
                remaining: Some(u32::MAX),
            },
        ] {
            let encoded = encode_status(&status);
            assert_eq!(encoded.len(), STATUS_LEN);
            assert_eq!(decode_status(&encoded), Some(status), "round trip failed");
        }
    }

    #[test]
    fn no_remaining_is_distinct_from_a_remaining_of_zero() {
        // The page shows a dash for one and a number for the other, and a
        // sentinel value would have made a bounded move about to finish look
        // like a jog.
        let none = encode_status(&Status {
            remaining: None,
            ..Status::default()
        });
        let zero = encode_status(&Status {
            remaining: Some(0),
            ..Status::default()
        });
        assert_ne!(none, zero);
        assert_eq!(decode_status(&none).unwrap().remaining, None);
        assert_eq!(decode_status(&zero).unwrap().remaining, Some(0));
    }

    #[test]
    fn a_malformed_status_is_rejected() {
        let good = encode_status(&Status::default());
        assert_eq!(decode_status(&good[..STATUS_LEN - 1]), None);
        let mut wrong_opcode = good;
        wrong_opcode[0] = CMD_JOG;
        assert_eq!(decode_status(&wrong_opcode), None);
    }

    #[test]
    fn the_page_agrees_with_this_file_about_every_opcode() {
        // The protocol is defined here and re-declared in JavaScript, which is
        // the kind of duplication that goes wrong silently: a mismatched opcode
        // is not a compile error, not a runtime error, and not visibly anything
        // except a pad that no longer moves a motor.
        for (name, value) in [
            ("CMD_JOG", CMD_JOG),
            ("CMD_STOP", CMD_STOP),
            ("CMD_RELEASE", CMD_RELEASE),
            ("CMD_MOVE", CMD_MOVE),
            ("CMD_ACCEL", CMD_ACCEL),
            ("CMD_PING", CMD_PING),
            ("MSG_STATUS", MSG_STATUS),
            ("MSG_PONG", MSG_PONG),
        ] {
            let needle = alloc::format!("{name} = 0x");
            let at = crate::trackpad::PAGE
                .find(&needle)
                .unwrap_or_else(|| panic!("the page never declares {name}"));
            let hex = &crate::trackpad::PAGE[at + needle.len()..at + needle.len() + 2];
            let declared = u8::from_str_radix(hex, 16)
                .unwrap_or_else(|_| panic!("{name} in the page is not two hex digits: {hex:?}"));
            assert_eq!(
                declared, value,
                "the page sends {name} as 0x{declared:02x}, the firmware reads 0x{value:02x}"
            );
        }
    }

    #[test]
    fn the_page_connects_to_the_path_the_firmware_serves() {
        // Same failure mode as the opcodes, one level up: a mistyped path is a
        // socket that never opens and a pad that silently falls back to the
        // slow route it was built to replace.
        assert!(
            crate::trackpad::PAGE.contains(crate::ws_link_path()),
            "the page does not connect to {}",
            crate::ws_link_path()
        );
    }

    #[test]
    fn every_command_fits_the_receive_buffer() {
        // The server reads into a fixed buffer sized by this constant, so a
        // frame the protocol can express but the buffer cannot hold would be
        // dropped at runtime rather than caught here.
        for frame in [
            vec![CMD_JOG, 0, 0],
            vec![CMD_STOP],
            vec![CMD_RELEASE],
            vec![CMD_PING],
            vec![CMD_MOVE, 0, 0, 0, 0, 0, 0],
            vec![CMD_ACCEL, 0, 0, 0, 0],
        ] {
            assert!(decode(&frame).is_some(), "{frame:?} did not decode");
            assert!(
                frame.len() <= MAX_COMMAND_LEN,
                "{frame:?} is longer than MAX_COMMAND_LEN"
            );
        }
    }
}
