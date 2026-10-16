//! Hardware-independent shell completion, parsing and formatting.

pub(super) const LINE_CAP: usize = 64;

const ROOT_CMDS: &[&str] = &[
    "help", "?", "hello", "clear", "version", "echo", "led", "system", "enc", "adc", "cal", "foc",
];
const LED_SUB: &[&str] = &["on", "off", "toggle"];
const SYSTEM_SUB: &[&str] = &["info"];
const CAL_SUB: &[&str] = &["current"];
const FOC_SUB: &[&str] = &[
    "status", "start", "stop", "align", "id", "iq", "poles", "pwm", "offset", "zero", "save",
    "openloop", "rpm", "kp", "ki", "skp", "ski", "isr", "motor",
];

pub(super) fn completion_words(line: &str) -> Option<(&str, &'static [&'static str])> {
    let ends_space = line.ends_with(' ');
    let mut parts = line.split_whitespace();
    let first = parts.next();
    let second = parts.next();
    let extra = parts.next();

    if extra.is_some() {
        return None;
    }

    match (first, second, ends_space) {
        (None, _, _) => Some(("", ROOT_CMDS)),
        (Some(a), None, false) => Some((a, ROOT_CMDS)),
        (Some("foc"), None, true) => Some(("", FOC_SUB)),
        (Some("foc"), Some(b), false) => Some((b, FOC_SUB)),
        (Some("led"), None, true) => Some(("", LED_SUB)),
        (Some("led"), Some(b), false) => Some((b, LED_SUB)),
        (Some("system"), None, true) => Some(("", SYSTEM_SUB)),
        (Some("system"), Some(b), false) => Some((b, SYSTEM_SUB)),
        (Some("cal"), None, true) => Some(("", CAL_SUB)),
        (Some("cal"), Some(b), false) => Some((b, CAL_SUB)),
        _ => None,
    }
}

pub(super) fn longest_common_prefix<'a>(words: &[&'a str]) -> &'a str {
    let first = words[0];
    let mut end = first.len();
    for w in words.iter().skip(1) {
        end = first
            .as_bytes()
            .iter()
            .zip(w.as_bytes())
            .take_while(|(a, b)| a == b)
            .count()
            .min(end);
    }
    &first[..end]
}

pub(super) fn apply_completion(
    line: &mut [u8; LINE_CAP],
    len: &mut usize,
    prefix: &str,
    filled: &str,
) -> bool {
    if filled.len() < prefix.len() {
        return false;
    }
    let add = &filled.as_bytes()[prefix.len()..];
    if *len + add.len() > LINE_CAP {
        return false;
    }
    line[*len..*len + add.len()].copy_from_slice(add);
    *len += add.len();
    true
}

pub(super) fn maybe_space_after(line: &mut [u8; LINE_CAP], len: &mut usize, word: &str) {
    if matches!(
        word,
        "foc" | "led" | "system" | "cal" | "hello" | "echo" | "id" | "iq" | "poles" | "pwm"
    ) && *len < LINE_CAP
        && (*len == 0 || line[*len - 1] != b' ')
    {
        line[*len] = b' ';
        *len += 1;
    }
}

pub(super) fn drop_crlf_lf(skip_lf: &mut bool, b: u8) -> bool {
    if *skip_lf {
        *skip_lf = false;
        return b == b'\n';
    }
    false
}

pub(super) fn note_cr(skip_lf: &mut bool) {
    *skip_lf = true;
}

pub(super) fn parse_i32(s: Option<&str>) -> Option<i32> {
    s.and_then(|t| t.parse().ok())
}

/// Omission selects the default current; malformed or extra arguments must not
/// reach the alignment start path.
pub(super) fn parse_align_args(
    args: &mut core::str::SplitWhitespace<'_>,
) -> Result<Option<i32>, ()> {
    match (args.next(), args.next()) {
        (None, None) => Ok(None),
        (Some(value), None) => value.parse().map(Some).map_err(|_| ()),
        _ => Err(()),
    }
}

pub(super) fn parse_u8(s: Option<&str>) -> Option<u8> {
    s.and_then(|t| t.parse().ok())
}

pub(super) fn parse_f32(s: Option<&str>) -> Option<f32> {
    s?.parse::<f32>().ok().filter(|v| v.is_finite())
}

pub(super) fn fmt_i32(v: i32, out: &mut [u8; 12]) -> usize {
    if v == 0 {
        out[0] = b'0';
        return 1;
    }
    let neg = v < 0;
    let mut n = if neg {
        (v as i64).unsigned_abs()
    } else {
        v as u64
    };
    let mut tmp = [0u8; 11];
    let mut i = 0;
    while n > 0 {
        tmp[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut o = 0;
    if neg {
        out[0] = b'-';
        o = 1;
    }
    while i > 0 {
        i -= 1;
        out[o] = tmp[i];
        o += 1;
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alignment_accepts_omitted_or_one_integer_argument() {
        for (text, expected) in [
            ("", None),
            (" \t", None),
            ("500", Some(500)),
            (" +500 ", Some(500)),
            ("0", Some(0)),
            ("-1", Some(-1)),
            ("2147483647", Some(i32::MAX)),
            ("-2147483648", Some(i32::MIN)),
        ] {
            assert_eq!(
                parse_align_args(&mut text.split_whitespace()),
                Ok(expected),
                "{text:?}"
            );
        }
    }

    #[test]
    fn alignment_rejects_malformed_overflowing_or_extra_arguments() {
        for text in [
            "abc",
            "500mA",
            "0.5",
            "1e3",
            "+",
            "2147483648",
            "-2147483649",
            "500 extra",
            "500 600",
            "500 600 700",
        ] {
            assert_eq!(
                parse_align_args(&mut text.split_whitespace()),
                Err(()),
                "{text:?}"
            );
        }
    }

    #[test]
    fn tuning_values_accept_exponents_and_long_decimals_but_not_nonfinite() {
        for (text, expected) in [
            ("2e-3", 0.002),
            ("+5100", 5100.0),
            (".000000000123", 1.23e-10),
        ] {
            assert_eq!(parse_f32(Some(text)), Some(expected));
        }
        for text in ["", ".", "+", "NaN", "inf", "-inf", "1e1000", "1.2.3"] {
            assert_eq!(parse_f32(Some(text)), None, "{text}");
        }
        assert_eq!(parse_f32(None), None);
    }

    #[test]
    fn crlf_submits_once() {
        let mut skip = false;
        assert!(!drop_crlf_lf(&mut skip, b'\r'));
        note_cr(&mut skip);
        assert!(drop_crlf_lf(&mut skip, b'\n'));
        assert!(!drop_crlf_lf(&mut skip, b'a'));
    }

    #[test]
    fn lf_only_is_not_dropped() {
        let mut skip = false;
        assert!(!drop_crlf_lf(&mut skip, b'\n'));
    }

    #[test]
    fn complete_root_prefix() {
        let (p, list) = completion_words("fo").unwrap();
        assert_eq!(p, "fo");
        assert!(list.contains(&"foc"));
    }

    #[test]
    fn complete_foc_sub() {
        let (p, list) = completion_words("foc s").unwrap();
        assert_eq!(p, "s");
        assert!(list.contains(&"start"));
        assert!(list.contains(&"status"));
        assert!(list.contains(&"stop"));
    }

    #[test]
    fn complete_after_space() {
        let (p, list) = completion_words("led ").unwrap();
        assert_eq!(p, "");
        assert_eq!(list, LED_SUB);
    }

    #[test]
    fn complete_cal() {
        let (p, list) = completion_words("cal ").unwrap();
        assert_eq!(p, "");
        assert_eq!(list, CAL_SUB);
    }

    #[test]
    fn complete_third_token_none() {
        assert!(completion_words("foc id 100").is_none());
    }

    #[test]
    fn apply_and_space() {
        let mut line = [0u8; LINE_CAP];
        line[..2].copy_from_slice(b"fo");
        let mut len = 2;
        assert!(apply_completion(&mut line, &mut len, "fo", "foc"));
        maybe_space_after(&mut line, &mut len, "foc");
        assert_eq!(&line[..len], b"foc ");
    }

    #[test]
    fn lcp_help_hello() {
        assert_eq!(longest_common_prefix(&["help", "hello"]), "hel");
    }

    #[test]
    fn fmt_ints() {
        let mut buf = [0u8; 12];
        let n = fmt_i32(0, &mut buf);
        assert_eq!(&buf[..n], b"0");
        let n = fmt_i32(-42, &mut buf);
        assert_eq!(&buf[..n], b"-42");
        let n = fmt_i32(i32::MIN, &mut buf);
        assert_eq!(&buf[..n], b"-2147483648");
    }
}
