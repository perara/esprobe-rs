//! How network credentials travel between the bridge and its host.
//!
//! Credentials belong in storage on the device, not compiled into an image: a
//! probe that needs rebuilding and reflashing to join a different network is a
//! probe that stays on the bench. The format is length-prefixed so a
//! passphrase containing anything at all survives the trip.

/// Longest values 802.11 and WPA2 allow.
pub const MAX_SSID: usize = 32;
pub const MAX_PASSWORD: usize = 64;

/// Packs an SSID and passphrase for `WifiSet`.
pub fn encode(ssid: &str, password: &str, out: &mut [u8]) -> Option<usize> {
    if ssid.len() > MAX_SSID || password.len() > MAX_PASSWORD {
        return None;
    }
    let length = 2 + ssid.len() + password.len();
    if out.len() < length {
        return None;
    }
    out[0] = ssid.len() as u8;
    out[1] = password.len() as u8;
    out[2..2 + ssid.len()].copy_from_slice(ssid.as_bytes());
    out[2 + ssid.len()..length].copy_from_slice(password.as_bytes());
    Some(length)
}

/// Unpacks what `encode` produced, rejecting anything malformed.
pub fn decode(payload: &[u8]) -> Option<(&str, &str)> {
    let [ssid_len, password_len, rest @ ..] = payload else {
        return None;
    };
    let ssid_len = usize::from(*ssid_len);
    let password_len = usize::from(*password_len);
    if ssid_len > MAX_SSID || password_len > MAX_PASSWORD || rest.len() != ssid_len + password_len
    {
        return None;
    }
    let (ssid, password) = rest.split_at(ssid_len);
    Some((
        core::str::from_utf8(ssid).ok()?,
        core::str::from_utf8(password).ok()?,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn credentials_survive_the_round_trip() {
        let mut buffer = [0u8; 128];
        let length = encode("an ssid with spaces", "p@ss:w0rd#with,punctuation", &mut buffer)
            .expect("encodes");
        let (ssid, password) = decode(&buffer[..length]).expect("decodes");
        assert_eq!(ssid, "an ssid with spaces");
        assert_eq!(password, "p@ss:w0rd#with,punctuation");
    }

    #[test]
    fn an_empty_password_is_a_valid_open_network() {
        let mut buffer = [0u8; 64];
        let length = encode("open", "", &mut buffer).expect("encodes");
        assert_eq!(decode(&buffer[..length]), Some(("open", "")));
    }

    #[test]
    fn malformed_payloads_are_refused() {
        assert_eq!(decode(&[]), None);
        // Lengths that disagree with the bytes that follow.
        assert_eq!(decode(&[4, 0, b'a', b'b']), None);
        assert_eq!(decode(&[1, 1, b'a']), None);
    }
}

