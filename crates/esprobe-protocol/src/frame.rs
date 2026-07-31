//! Framing contract for the USB-serial ARM DAP bridge.

pub const VERSION: u8 = 3;
/// Large enough for a 256-word block read plus the envelope and worst-case
/// COBS overhead. One USB round trip per kibibyte, rather than per word, is
/// what keeps the wire — not the transport — the limiting factor.
pub const MAX_FRAME: usize = 4136;
/// Words a single `ReadRegisterBlock` may carry. Matches the ADIv5 auto-
/// increment window, so probe-rs's own 1 KiB chunking maps to one frame.
pub const MAX_BLOCK_WORDS: usize = 1024;
const REQUEST_MAGIC: &[u8; 4] = b"ESPB";
const RESPONSE_MAGIC: &[u8; 4] = b"ESPR";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameError;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Command {
    Hello = 0x01,
    Attach = 0x02,
    Detach = 0x03,
    ReadRegister = 0x10,
    WriteRegister = 0x11,
    SwjSequence = 0x12,
    LineState = 0x13,
    ResetLineState = 0x14,
    ResetAssert = 0x15,
    ResetRelease = 0x16,
    AttachUnderReset = 0x17,
    PadSelfTest = 0x18,
    RecoveryProbe = 0x19,
    DiagnosticSwdio = 0x1a,
    EnterRomBoot = 0x1b,
    UartReceive = 0x1c,
    UartResetCapture = 0x1d,
    MuxProbe = 0x1e,
    ReadRegisterBlock = 0x1f,
    SetSpeed = 0x20,
    SetPinMap = 0x21,
    Capabilities = 0x22,
    WriteRegisterBlock = 0x23,
    RecoverLine = 0x24,
    WireProbe = 0x25,
    SetEngine = 0x26,
    ApWriteProbe = 0x27,
    SpiLoopback = 0x28,
    MemoryRead = 0x29,
    Ping = 0x2a,
    Profile = 0x2b,
    Echo = 0x2c,
    PinMap = 0x2d,
    WifiStatus = 0x2e,
    WifiSet = 0x2f,
    WifiForget = 0x30,
}

impl TryFrom<u8> for Command {
    type Error = FrameError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Self::Hello),
            0x02 => Ok(Self::Attach),
            0x03 => Ok(Self::Detach),
            0x10 => Ok(Self::ReadRegister),
            0x11 => Ok(Self::WriteRegister),
            0x12 => Ok(Self::SwjSequence),
            0x13 => Ok(Self::LineState),
            0x14 => Ok(Self::ResetLineState),
            0x15 => Ok(Self::ResetAssert),
            0x16 => Ok(Self::ResetRelease),
            0x17 => Ok(Self::AttachUnderReset),
            0x18 => Ok(Self::PadSelfTest),
            0x19 => Ok(Self::RecoveryProbe),
            0x1a => Ok(Self::DiagnosticSwdio),
            0x1b => Ok(Self::EnterRomBoot),
            0x1c => Ok(Self::UartReceive),
            0x1d => Ok(Self::UartResetCapture),
            0x1e => Ok(Self::MuxProbe),
            0x1f => Ok(Self::ReadRegisterBlock),
            0x20 => Ok(Self::SetSpeed),
            0x21 => Ok(Self::SetPinMap),
            0x22 => Ok(Self::Capabilities),
            0x23 => Ok(Self::WriteRegisterBlock),
            0x24 => Ok(Self::RecoverLine),
            0x25 => Ok(Self::WireProbe),
            0x26 => Ok(Self::SetEngine),
            0x27 => Ok(Self::ApWriteProbe),
            0x28 => Ok(Self::SpiLoopback),
            0x29 => Ok(Self::MemoryRead),
            0x2a => Ok(Self::Ping),
            0x2b => Ok(Self::Profile),
            0x2c => Ok(Self::Echo),
            0x2d => Ok(Self::PinMap),
            0x2e => Ok(Self::WifiStatus),
            0x2f => Ok(Self::WifiSet),
            0x30 => Ok(Self::WifiForget),
            _ => Err(FrameError),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Status {
    Ok = 0,
    BadFrame = 1,
    Unsupported = 2,
    NotAttached = 3,
    Wait = 4,
    Fault = 5,
    Parity = 6,
    Transport = 7,
    /// The shared reset line is asserted, so the target's bus would answer
    /// zeros rather than data.
    TargetInReset = 8,
}

pub struct Request<'a> {
    pub sequence: u16,
    pub command: Command,
    pub payload: &'a [u8],
}

pub struct Response<'a> {
    pub sequence: u16,
    pub status: Status,
    pub payload: &'a [u8],
}

pub fn encode_request(
    sequence: u16,
    command: Command,
    payload: &[u8],
    output: &mut [u8],
) -> Result<usize, FrameError> {
    encode(REQUEST_MAGIC, sequence, command as u8, payload, output)
}

pub fn encode_response(
    sequence: u16,
    status: Status,
    payload: &[u8],
    output: &mut [u8],
) -> Result<usize, FrameError> {
    encode(RESPONSE_MAGIC, sequence, status as u8, payload, output)
}

pub fn decode_request(frame: &mut [u8]) -> Result<Request<'_>, FrameError> {
    let decoded = cobs_decode(frame)?;
    let (sequence, kind, payload) = decode_packet(decoded, REQUEST_MAGIC)?;
    Ok(Request {
        sequence,
        command: kind.try_into()?,
        payload,
    })
}

pub fn decode_response(frame: &mut [u8]) -> Result<Response<'_>, FrameError> {
    let decoded = cobs_decode(frame)?;
    let (sequence, kind, payload) = decode_packet(decoded, RESPONSE_MAGIC)?;
    let status = match kind {
        0 => Status::Ok,
        1 => Status::BadFrame,
        2 => Status::Unsupported,
        3 => Status::NotAttached,
        4 => Status::Wait,
        5 => Status::Fault,
        6 => Status::Parity,
        7 => Status::Transport,
        8 => Status::TargetInReset,
        _ => return Err(FrameError),
    };
    Ok(Response {
        sequence,
        status,
        payload,
    })
}

fn encode(
    magic: &[u8; 4],
    sequence: u16,
    kind: u8,
    payload: &[u8],
    output: &mut [u8],
) -> Result<usize, FrameError> {
    let mut packet = [0u8; MAX_FRAME];
    let packet_len = 8 + payload.len() + 2;
    if packet_len > packet.len() {
        return Err(FrameError);
    }
    packet[..4].copy_from_slice(magic);
    packet[4] = VERSION;
    packet[5..7].copy_from_slice(&sequence.to_le_bytes());
    packet[7] = kind;
    packet[8..8 + payload.len()].copy_from_slice(payload);
    let crc = crc16(&packet[..packet_len - 2]);
    packet[packet_len - 2..packet_len].copy_from_slice(&crc.to_le_bytes());
    cobs_encode(&packet[..packet_len], output)
}

fn decode_packet<'a>(packet: &'a [u8], magic: &[u8; 4]) -> Result<(u16, u8, &'a [u8]), FrameError> {
    if packet.len() < 10 || &packet[..4] != magic || packet[4] != VERSION {
        return Err(FrameError);
    }
    let expected = u16::from_le_bytes([packet[packet.len() - 2], packet[packet.len() - 1]]);
    if crc16(&packet[..packet.len() - 2]) != expected {
        return Err(FrameError);
    }
    Ok((
        u16::from_le_bytes([packet[5], packet[6]]),
        packet[7],
        &packet[8..packet.len() - 2],
    ))
}

fn cobs_encode(input: &[u8], output: &mut [u8]) -> Result<usize, FrameError> {
    // COBS spends one code byte per run of up to 254 non-zero bytes, so the
    // overhead is not constant once a frame outgrows a single run.
    if output.len() < input.len() + input.len() / 254 + 2 {
        return Err(FrameError);
    }
    let mut code_index = 0;
    let mut write_index = 1;
    let mut code = 1u8;
    for &byte in input {
        if byte == 0 {
            output[code_index] = code;
            code_index = write_index;
            write_index += 1;
            code = 1;
        } else {
            output[write_index] = byte;
            write_index += 1;
            code = code.wrapping_add(1);
            if code == u8::MAX {
                output[code_index] = code;
                code_index = write_index;
                write_index += 1;
                code = 1;
            }
        }
    }
    output[code_index] = code;
    output[write_index] = 0;
    Ok(write_index + 1)
}

fn cobs_decode(frame: &mut [u8]) -> Result<&mut [u8], FrameError> {
    let mut read_index = 0;
    let mut write_index = 0;
    while read_index < frame.len() {
        let code = frame[read_index];
        if code == 0 {
            return Err(FrameError);
        }
        read_index += 1;
        let copy = usize::from(code - 1);
        if read_index + copy > frame.len() {
            return Err(FrameError);
        }
        for _ in 0..copy {
            frame[write_index] = frame[read_index];
            write_index += 1;
            read_index += 1;
        }
        if code != u8::MAX && read_index < frame.len() {
            frame[write_index] = 0;
            write_index += 1;
        }
    }
    Ok(&mut frame[..write_index])
}

/// CRC-16/CCITT-FALSE, one table lookup per byte.
///
/// A frame now carries kibibytes rather than a handful of bytes, and the
/// bit-at-a-time form cost eight iterations for each of them — squarely
/// between the wire read and the USB write, where it delayed both.
const CRC_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut index = 0;
    while index < 256 {
        let mut value = (index as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            value = if value & 0x8000 != 0 {
                (value << 1) ^ 0x1021
            } else {
                value << 1
            };
            bit += 1;
        }
        table[index] = value;
        index += 1;
    }
    table
};

fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &byte in bytes {
        crc = (crc << 8) ^ CRC_TABLE[usize::from((crc >> 8) as u8 ^ byte)];
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Byte-exact encodings the firmware and the host must both produce.
    ///
    /// This is the whole reason two copies of this file can coexist: the ends
    /// are built by different toolchains for different architectures, and a
    /// silent divergence here would show up as a bridge that connects and then
    /// misbehaves. A changed command number, a changed magic, a changed CRC or
    /// a changed COBS boundary all fail this instead.
    #[test]
    fn wire_format_fixtures() {
        let mut frame = [0u8; MAX_FRAME + 2];

        let length = encode_request(0x1234, Command::Hello, &[], &mut frame).unwrap();
        assert_eq!(
            &frame[..length],
            &[11, b'E', b'S', b'P', b'B', 3, 0x34, 0x12, 1, 0xb3, 0x6f, 0],
            "the Hello request encoding moved"
        );

        // The status byte is zero for Ok, so COBS relocates it: this fixture
        // pins the framing as much as the fields.
        let length = encode_response(0x1234, Status::Ok, &[0xde, 0xad], &mut frame).unwrap();
        assert_eq!(
            &frame[..length],
            &[
                8, b'E', b'S', b'P', b'R', 3, 0x34, 0x12, 5, 0xde, 0xad, 0x88, 0xde, 0
            ],
            "the response encoding moved"
        );

        // Command numbers are part of the contract; renumbering one silently
        // repoints every host that has not been rebuilt.
        assert_eq!(Command::Hello as u8, 0x01);
        assert_eq!(Command::ReadRegister as u8, 0x10);
        assert_eq!(Command::MemoryRead as u8, 0x29);
        assert_eq!(Status::TargetInReset as u8, 8);
        assert_eq!(VERSION, 3);
    }

    #[test]
    fn request_round_trip_preserves_zero_bytes() {
        let mut frame = [0u8; MAX_FRAME + 2];
        let length =
            encode_request(0x1234, Command::WriteRegister, &[0, 1, 0, 2], &mut frame).unwrap();
        let request = decode_request(&mut frame[..length - 1]).unwrap();
        assert_eq!(request.sequence, 0x1234);
        assert_eq!(request.command, Command::WriteRegister);
        assert_eq!(request.payload, [0, 1, 0, 2]);
    }

    #[test]
    fn reset_line_state_has_a_stable_wire_command() {
        assert_eq!(Command::try_from(0x14), Ok(Command::ResetLineState));
        assert_eq!(Command::ResetLineState as u8, 0x14);
    }

    #[test]
    fn reset_hold_commands_have_stable_wire_values() {
        assert_eq!(Command::try_from(0x15), Ok(Command::ResetAssert));
        assert_eq!(Command::try_from(0x16), Ok(Command::ResetRelease));
        assert_eq!(Command::try_from(0x17), Ok(Command::AttachUnderReset));
        assert_eq!(Command::try_from(0x18), Ok(Command::PadSelfTest));
        assert_eq!(Command::try_from(0x19), Ok(Command::RecoveryProbe));
        assert_eq!(Command::try_from(0x1a), Ok(Command::DiagnosticSwdio));
        assert_eq!(Command::try_from(0x1b), Ok(Command::EnterRomBoot));
        assert_eq!(Command::try_from(0x1c), Ok(Command::UartReceive));
        assert_eq!(Command::try_from(0x1d), Ok(Command::UartResetCapture));
        assert_eq!(Command::try_from(0x1e), Ok(Command::MuxProbe));
        assert_eq!(Command::try_from(0x1f), Ok(Command::ReadRegisterBlock));
        assert_eq!(Command::try_from(0x20), Ok(Command::SetSpeed));
        assert_eq!(Command::try_from(0x21), Ok(Command::SetPinMap));
        assert_eq!(Command::try_from(0x22), Ok(Command::Capabilities));
        assert_eq!(Command::try_from(0x23), Ok(Command::WriteRegisterBlock));
        assert_eq!(Command::try_from(0x24), Ok(Command::RecoverLine));
        assert_eq!(Command::try_from(0x25), Ok(Command::WireProbe));
        assert_eq!(Command::try_from(0x26), Ok(Command::SetEngine));
        assert_eq!(Command::try_from(0x27), Ok(Command::ApWriteProbe));
        assert_eq!(Command::try_from(0x28), Ok(Command::SpiLoopback));
        assert_eq!(Command::try_from(0x29), Ok(Command::MemoryRead));
        assert_eq!(Command::try_from(0x2a), Ok(Command::Ping));
        assert_eq!(Command::try_from(0x2b), Ok(Command::Profile));
        assert_eq!(Command::try_from(0x2c), Ok(Command::Echo));
        assert_eq!(Command::try_from(0x2d), Ok(Command::PinMap));
        assert_eq!(Command::try_from(0x2e), Ok(Command::WifiStatus));
        assert_eq!(Command::try_from(0x2f), Ok(Command::WifiSet));
        assert_eq!(Command::try_from(0x30), Ok(Command::WifiForget));
    }

    #[test]
    fn a_full_block_read_survives_the_round_trip() {
        let payload: Vec<u8> = (0..MAX_BLOCK_WORDS * 4).map(|index| index as u8).collect();
        let mut frame = vec![0u8; MAX_FRAME + 2];
        let length = encode_response(9, Status::Ok, &payload, &mut frame).unwrap();
        assert!(length <= frame.len());
        let response = decode_response(&mut frame[..length - 1]).unwrap();
        assert_eq!(response.sequence, 9);
        assert_eq!(response.payload, payload.as_slice());
    }

    #[test]
    fn the_table_driven_crc_matches_the_bit_at_a_time_definition() {
        fn reference(bytes: &[u8]) -> u16 {
            let mut crc = 0xffffu16;
            for &byte in bytes {
                crc ^= u16::from(byte) << 8;
                for _ in 0..8 {
                    crc = if crc & 0x8000 != 0 {
                        (crc << 1) ^ 0x1021
                    } else {
                        crc << 1
                    };
                }
            }
            crc
        }

        let sample: Vec<u8> = (0..1024).map(|index| (index * 31 % 251) as u8).collect();
        for length in [0, 1, 2, 7, 8, 255, 256, 1024] {
            assert_eq!(
                crc16(&sample[..length]),
                reference(&sample[..length]),
                "{length}"
            );
        }
    }

    #[test]
    fn response_rejects_corrupt_crc() {
        let mut frame = [0u8; MAX_FRAME + 2];
        let length = encode_response(7, Status::Ok, &[1, 2, 3], &mut frame).unwrap();
        frame[3] ^= 0x40;
        assert!(decode_response(&mut frame[..length - 1]).is_err());
    }
}
