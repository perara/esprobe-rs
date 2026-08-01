//! Escaping for the small JSON documents the bridge publishes over HTTP.
//!
//! Lives here rather than in the firmware because the firmware crate cannot be
//! tested on the host — its dependencies only build for the target — and an
//! escaper nobody can run tests against is how the unescaped version shipped in
//! the first place.
//!
//! A `Display` adapter rather than a function returning a `String`, so this
//! crate keeps its promise that nothing in it allocates.

use core::fmt::{self, Write as _};

/// Writes a string as the contents of a JSON string literal.
///
/// An SSID is arbitrary bytes as far as 802.11 is concerned, and the credential
/// codec only checks length and UTF-8, so a network called `guest"net` reaches
/// the HTTP layer intact and interpolating it raw produces a document no parser
/// will accept.
pub struct Escaped<'a>(pub &'a str);

impl fmt::Display for Escaped<'_> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for character in self.0.chars() {
            match character {
                '"' => formatter.write_str("\\\"")?,
                '\\' => formatter.write_str("\\\\")?,
                '\n' => formatter.write_str("\\n")?,
                '\r' => formatter.write_str("\\r")?,
                '\t' => formatter.write_str("\\t")?,
                // Everything below a space has to be escaped, and \u is the
                // only form JSON offers for the ones without a short name.
                control if (control as u32) < 0x20 => {
                    write!(formatter, "\\u{:04x}", control as u32)?
                }
                other => formatter.write_char(other)?,
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn escape(input: &str) -> String {
        Escaped(input).to_string()
    }

    #[test]
    fn ordinary_text_is_unchanged() {
        assert_eq!(escape("Spikkelia-28"), "Spikkelia-28");
        // Non-ASCII is valid unescaped in JSON and should stay readable.
        assert_eq!(escape("kjøkkenet"), "kjøkkenet");
    }

    #[test]
    fn quotes_and_backslashes_cannot_end_the_string() {
        assert_eq!(escape(r#"guest"net"#), r#"guest\"net"#);
        assert_eq!(escape(r"back\slash"), r"back\\slash");
        // A trailing backslash is the case that escapes the closing quote.
        assert_eq!(escape(r"trailing\"), r"trailing\\");
    }

    #[test]
    fn control_characters_are_escaped() {
        assert_eq!(escape("two\nlines"), "two\\nlines");
        assert_eq!(escape("tab\there"), "tab\\there");
        assert_eq!(escape("bell\u{7}"), "bell\\u0007");
        assert_eq!(escape("\u{0}"), "\\u0000");
    }

    #[test]
    fn a_hostile_ssid_still_produces_one_json_string() {
        // The whole point: whatever the network is called, the document has
        // exactly one string here and it terminates where it should.
        let document = format!(r#"{{"ssid":"{}"}}"#, Escaped(r#"a","ok":false,"x":"b"#));
        assert_eq!(
            document, r#"{"ssid":"a\",\"ok\":false,\"x\":\"b"}"#,
            "an SSID escaped out of its own field"
        );
    }
}
