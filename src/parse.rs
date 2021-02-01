use std::fmt::{self, Display, Formatter};

/// Data Link Escape.
const DLE: u8 = 0x10;
/// Start of TeXt.
const STX: u8 = 0x02;
/// End of TeXt.
const ETX: u8 = 0x03;
/// A control character that is use for negative acknowledgement.
const NAK: u8 = 0x15;

/// Header of messages.
const HEADER: [u8; 2] = [DLE, STX];
/// Footer of messages.
const FOOTER: [u8; 2] = [DLE, ETX];
/// Represents a communication error.
const NEGATIVE_ACKNOWLEDGEMENT: [u8; 2] = [DLE, NAK];

const HEADER_LENGTH: usize = HEADER.len();
const FOOTER_LENGTH: usize = FOOTER.len();
/// The length of BCC, Block Check Character.
const BCC_LENGTH: usize = 1;
/// The length of the shortest valid response.
const LENGTH_MIN: usize = HEADER_LENGTH + FOOTER_LENGTH + BCC_LENGTH;

/// Constructs a message to send a sensor.
pub(crate) fn parse_command(command: &[u8]) -> Vec<u8> {
    let mut buf = HEADER.to_vec();

    let mut bcc = 0;

    // Append data part.
    // If data has DLE, we need to append DLE twice in a row.
    for byte in command.iter().copied() {
        if byte == DLE {
            buf.push(byte);
        }
        buf.push(byte);
        bcc ^= byte;
    }

    for byte in FOOTER.iter().copied() {
        buf.push(byte);
        if byte != DLE {
            bcc ^= byte;
        }
    }

    buf.push(bcc);

    buf
}

/// Extract data part of a received message from a sensor.
/// # Returns
/// `Ok(data)` if succeeds, `Err(reason)` if an invalid message is specified.
pub(crate) fn parse_reception(message: &[u8]) -> Result<Vec<u8>, ParseError> {
    if message == &NEGATIVE_ACKNOWLEDGEMENT[..] {
        return Err(ParseError::NAK);
    }

    let len = message.len();
    if len < LENGTH_MIN {
        return Err(ParseError::TooShort(len));
    }

    // Message is long enough here, so indexing and unwrap() never panic below.

    // Integrity of header and footer.

    // Extract data part.
    let mut buf = vec![];
    let mut is_previous_dle = false;
    let mut expected_bcc = 0;

    for &byte in &message[HEADER_LENGTH..message.len() - FOOTER_LENGTH - BCC_LENGTH] {
        if is_previous_dle {
            if byte != DLE {
                return Err(ParseError::WrongDLE);
            }
            is_previous_dle = false;
        } else {
            buf.push(byte);
            expected_bcc ^= byte;
            is_previous_dle = byte == DLE;
        }
    }

    FOOTER
        .iter()
        .filter(|&&byte| byte != DLE)
        .for_each(|&byte| expected_bcc ^= byte);

    // Integrity of BCC.
    let actual_bcc = *message.last().unwrap();
    if expected_bcc != actual_bcc {
        return Err(ParseError::WrongBCC);
    }

    Ok(buf)
}

/// Represents an error that occurred during parsing a received message from a sensor.
#[derive(Debug)]
pub enum ParseError {
    /// Received an negative acknowledgement.
    NAK,
    /// Received a too short message.
    TooShort(usize),
    /// DLE must be contained twice or more in a row, but was not.
    WrongDLE,
    /// An communication error was detected.
    WrongBCC,
}

impl Display for ParseError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::NAK => write!(f, "Received an negative acknowledgement."),
            ParseError::TooShort(l) => write!(f, "Received too short ({} bytes) message,", l),
            ParseError::WrongDLE => write!(f, "DLE must be contained twice or more in a row, but was not."),
            ParseError::WrongBCC => write!(f, "Wrong BCC."),
        }
    }
}

impl std::error::Error for ParseError {}
