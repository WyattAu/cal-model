//! `COMPU_TAB` / `COMPU_VTAB` resolution.
//!
//! `a2l-parse` (L1) retains a `TABLE` COMPU_METHOD's *reference* — the
//! `COMPU_TAB_REF` name — and skips the table body itself, because the
//! tabulated points are data rather than description structure. This module
//! closes that half of the gap: a total, hand-rolled scan of the A2L text
//! that lifts every `COMPU_TAB` / `COMPU_VTAB` block out of the source and
//! makes it evaluable.
//!
//! The scan is a plain token walk over the same keyword-driven grammar the
//! L1 parser consumes (`/begin NAME … /end NAME`), so it inherits the
//! skip-tolerance of the substrate: anything it does not recognise is
//! skipped, and a malformed table simply yields fewer points. It never
//! panics and never fails — a missing table is an
//! [`CalError::UnknownComputationTable`](crate::CalError::UnknownComputationTable)
//! at conversion time, not a parse failure.
//!
//! # Evaluation semantics
//!
//! * [`TableKind::Intp`] — piecewise-linear interpolation between the two
//!   bracketing points.
//! * [`TableKind::NoIntp`] / [`TableKind::Verb`] — step lookup: the value of
//!   the largest tabulated raw not greater than the query.
//!
//! Outside the tabulated raw span both forms fall back to
//! `DEFAULT_VALUE_NUMERIC` when the table declares one, and otherwise clamp
//! to the nearest endpoint — the ASAP2 rule, and the behaviour a
//! calibration engineer expects from an extrapolated lookup.

use std::collections::BTreeMap;

use crate::error::CalError;

/// The interpolation kind of a `COMPU_TAB` / `COMPU_VTAB` block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TableKind {
    /// `TAB_INTP` — linear interpolation between points.
    Intp,
    /// `TAB_NOINTP` — stepwise, no interpolation.
    NoIntp,
    /// `TAB_VERB` — enumerated (verb) table, resolved stepwise; numeric
    /// interpretation requires numeric point labels.
    Verb,
}

impl TableKind {
    /// Parse the table-type keyword.
    #[must_use]
    pub fn from_keyword(keyword: &str) -> Option<Self> {
        Some(match keyword {
            "TAB_INTP" => Self::Intp,
            "TAB_NOINTP" => Self::NoIntp,
            "TAB_VERB" => Self::Verb,
            _ => return None,
        })
    }

    /// The ASAP2 keyword.
    #[must_use]
    pub const fn keyword(self) -> &'static str {
        match self {
            Self::Intp => "TAB_INTP",
            Self::NoIntp => "TAB_NOINTP",
            Self::Verb => "TAB_VERB",
        }
    }

    /// `true` when queries interpolate rather than step.
    #[must_use]
    pub const fn interpolates(self) -> bool {
        matches!(self, Self::Intp)
    }
}

/// A resolved `COMPU_TAB` / `COMPU_VTAB`: the `(raw, physical)` points of a
/// tabular conversion.
#[derive(Debug, Clone, PartialEq)]
pub struct CompuTable {
    name: String,
    kind: TableKind,
    points: Vec<(f64, f64)>,
    default: Option<(f64, f64)>,
}

impl CompuTable {
    /// Build a table from its points. Points are kept in file order; a
    /// query evaluates against the bracketing pair regardless of order, so
    /// an unsorted table still interpolates.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        kind: TableKind,
        points: Vec<(f64, f64)>,
        default: Option<(f64, f64)>,
    ) -> Self {
        Self {
            name: name.into(),
            kind,
            points,
            default,
        }
    }

    /// The table name (what a `COMPU_TAB_REF` refers to).
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The interpolation kind.
    #[must_use]
    pub const fn kind(&self) -> TableKind {
        self.kind
    }

    /// The tabulated `(raw, physical)` points, in file order.
    #[must_use]
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// The `DEFAULT_VALUE_NUMERIC` point, when declared.
    #[must_use]
    pub fn default_point(&self) -> Option<(f64, f64)> {
        self.default
    }

    /// `true` when the table has no usable numeric points, in which case a
    /// `TABLE` COMPU_METHOD referencing it cannot be evaluated.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// `true` when the tabulated physical values are non-decreasing in the
    /// raw value — the monotonicity a calibration lookup wants.
    #[must_use]
    pub fn is_monotone(&self) -> bool {
        let mut iter = self.points.iter();
        let Some(first) = iter.next() else {
            return true;
        };
        let mut previous = first.1;
        for point in iter {
            if point.1 < previous {
                return false;
            }
            previous = point.1;
        }
        true
    }

    /// The raw span the table covers: `(first, last)` in file order.
    #[must_use]
    pub fn raw_span(&self) -> Option<(f64, f64)> {
        let first = self.points.first()?.0;
        let last = self.points.last()?.0;
        Some((first, last))
    }

    /// Evaluate raw → physical.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownComputationTable`] when the table carries no
    /// usable points, [`CalError::NonFiniteValue`] for a non-finite query.
    pub fn to_physical(&self, raw: f64) -> Result<f64, CalError> {
        if !raw.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.name.clone(),
                value: raw,
            });
        }
        if self.points.is_empty() {
            return Err(CalError::UnknownComputationTable(self.name.clone()));
        }
        // Outside the tabulated span: DEFAULT_VALUE_NUMERIC, else clamp.
        let (first_raw, first_phys) = self.points[0];
        if raw < first_raw {
            return Ok(self.outside(first_phys));
        }
        let (last_raw, last_phys) = self.points[self.points.len() - 1];
        if raw > last_raw {
            return Ok(self.outside(last_phys));
        }
        if self.points.len() == 1 {
            return Ok(first_phys);
        }
        // Locate the bracketing pair. Tabulated points are ascending in the
        // raw domain by construction; a non-monotone (descending) file is
        // walked in reverse so the walk always terminates.
        let ascending = last_raw >= first_raw;
        for window in self.points.windows(2) {
            let (lo, hi) = (window[0], window[1]);
            let (in_window, flip) = if ascending {
                (raw >= lo.0 && raw <= hi.0, false)
            } else {
                (raw <= lo.0 && raw >= hi.0, true)
            };
            if in_window {
                if !self.kind.interpolates() {
                    return Ok(if flip { hi.1 } else { lo.1 });
                }
                let span = hi.0 - lo.0;
                if span == 0.0 {
                    // Degenerate duplicate raw: step to the later value.
                    return Ok(hi.1);
                }
                let t = (raw - lo.0) / span;
                return Ok(lo.1 + t * (hi.1 - lo.1));
            }
        }
        // Unreachable for a table whose span brackets the query, but the
        // function is total: fall back to the nearest endpoint.
        Ok(self.outside(last_phys))
    }

    /// Evaluate physical → raw (the inverse of [`CompuTable::to_physical`]
    /// within the tabulated span).
    ///
    /// Step tables invert to the tabulated raw that produces the physical
    /// value; flat segments resolve to their lower raw. Physical values
    /// outside the tabulated range clamp to the raw span unless the table
    /// declares a default point, which wins.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownComputationTable`] when the table carries no
    /// usable points, [`CalError::NonFiniteValue`] for a non-finite query.
    pub fn to_raw(&self, physical: f64) -> Result<u64, CalError> {
        if !physical.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: self.name.clone(),
                value: physical,
            });
        }
        if self.points.is_empty() {
            return Err(CalError::UnknownComputationTable(self.name.clone()));
        }
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        // The raw span, ordered — a descending table inverts to the same
        // span.
        let span_low = first.0.min(last.0);
        let span_high = first.0.max(last.0);
        // The physical envelope: below it the table extrapolates down, above
        // it up. Outside the envelope ASAP2 defers to DEFAULT_VALUE_NUMERIC,
        // and without one the raw span clamps.
        let phys_low = self
            .points
            .iter()
            .map(|point| point.1)
            .fold(f64::INFINITY, f64::min);
        let phys_high = self
            .points
            .iter()
            .map(|point| point.1)
            .fold(f64::NEG_INFINITY, f64::max);
        if physical < phys_low || physical > phys_high {
            return Self::as_raw(
                match self.default {
                    Some((raw, _)) => raw,
                    None if physical < phys_low => span_low,
                    None => span_high,
                },
                &self.name,
            );
        }
        if self.points.len() == 1 {
            return Self::as_raw(first.0, &self.name);
        }
        for window in self.points.windows(2) {
            let (lo, hi) = (window[0], window[1]);
            let ascending_pair = hi.1 >= lo.1;
            let (low, high) = if ascending_pair {
                (lo, hi)
            } else {
                (hi, lo)
            };
            if physical < low.1 || physical > high.1 {
                continue;
            }
            if !self.kind.interpolates() {
                // Step table: the raw that produces this physical value.
                return Self::as_raw(if low.1 == physical { low.0 } else { high.0 }, &self.name);
            }
            let span = high.1 - low.1;
            if span == 0.0 {
                return Self::as_raw(low.0, &self.name);
            }
            let t = (physical - low.1) / span;
            return Self::as_raw(low.0 + t * (high.0 - low.0), &self.name);
        }
        Self::as_raw(span_high, &self.name)
    }

    /// Round a raw table coordinate to an integral deposit, half away from
    /// zero, rejecting anything outside the unsigned 64-bit range.
    fn as_raw(value: f64, name: &str) -> Result<u64, CalError> {
        if !value.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: name.to_owned(),
                value,
            });
        }
        let rounded = value.round();
        if rounded < 0.0 || rounded > 18_446_744_073_709_551_616.0 {
            return Err(CalError::OutOfBounds {
                name: name.to_owned(),
                value,
                lower: 0.0,
                upper: 18_446_744_073_709_551_616.0,
            });
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Ok(rounded as u64)
    }

    /// The value used outside the tabulated span.
    fn outside(&self, endpoint: f64) -> f64 {
        self.default.map_or(endpoint, |(_, phys)| phys)
    }
}

// ---------------------------------------------------------------------------
// Scanning
// ---------------------------------------------------------------------------

/// Table index: `(module, table name)`. Tables declared at project scope
/// (outside any MODULE, per ASAP2 1.7) are keyed under
/// [`PROJECT_SCOPE`].
pub type TableIndex = BTreeMap<(String, String), CompuTable>;

/// The module key used for tables declared outside any MODULE block.
pub const PROJECT_SCOPE: &str = "$project";

/// Nesting limit — a guard against pathological input, mirroring the L1
/// substrate's own depth limit.
const MAX_DEPTH: usize = 64;

/// Lift every `COMPU_TAB` / `COMPU_VTAB` block out of `text`, indexed by
/// `(module, table name)`. Total: unparsable input yields an empty or
/// partial index, never an error and never a panic.
#[must_use]
pub fn scan_tables(text: &str) -> TableIndex {
    let tokens = lex(text);
    let mut index = TableIndex::new();
    let mut stack: Vec<&str> = Vec::new();
    let mut module = String::from(PROJECT_SCOPE);
    let mut cursor = 0usize;
    while let Some(tok) = tokens.get(cursor) {
        cursor += 1;
        match tok {
            Token::Slash => {
                let Some(Token::Ident(keyword)) = tokens.get(cursor) else {
                    continue;
                };
                cursor += 1;
                match *keyword {
                    "begin" => {
                        if stack.len() >= MAX_DEPTH {
                            // Pathological nesting: stop scanning rather
                            // than grow the stack without bound.
                            break;
                        }
                        let Some(Token::Ident(name)) = tokens.get(cursor).copied() else {
                            continue;
                        };
                        cursor += 1;
                        if name == "MODULE" {
                            if let Some(Token::Ident(module_name)) = tokens.get(cursor).copied() {
                                cursor += 1;
                                module = module_name.to_owned();
                            }
                        }
                        if matches!(name, "COMPU_TAB" | "COMPU_VTAB") {
                            let (table, next) = parse_table(&tokens, cursor, name, &module);
                            index.insert((module.clone(), table.name.clone()), table);
                            cursor = next;
                        }
                        stack.push(name);
                    }
                    "end" => {
                        if let Some(Token::Ident(name)) = tokens.get(cursor).copied() {
                            cursor += 1;
                            if stack.last() == Some(name) {
                                stack.pop();
                            }
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }
    index
}

/// Parse one table body, starting just past its `/begin NAME` tokens.
/// Returns the table and the token index of the terminating `/`.
fn parse_table<'t>(
    tokens: &[Token<'t>],
    mut cursor: usize,
    block: &'t str,
    module: &str,
) -> (CompuTable, usize) {
    let name = match tokens.get(cursor) {
        Some(Token::Ident(name)) => {
            cursor += 1;
            name.to_owned()
        }
        _ => String::new(),
    };
    // Description string, when present.
    if matches!(tokens.get(cursor), Some(Token::Str(_))) {
        cursor += 1;
    }
    let kind = match tokens.get(cursor) {
        Some(Token::Ident(keyword)) => TableKind::from_keyword(keyword).unwrap_or(TableKind::Intp),
        _ => TableKind::Intp,
    };
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut default: Option<(f64, f64)> = None;
    while let Some(tok) = tokens.get(cursor) {
        match tok {
            Token::Slash => break,
            Token::Ident(keyword) => match *keyword {
                "DEFAULT_VALUE_NUMERIC" => {
                    cursor += 1;
                    let raw = match tokens.get(cursor) {
                        Some(Token::Num(value)) => *value,
                        _ => break,
                    };
                    let phys = match tokens.get(cursor + 1) {
                        Some(Token::Num(value)) => *value,
                        _ => break,
                    };
                    cursor += 2;
                    default = Some((raw, phys));
                }
                "DEFAULT_VALUE" => {
                    // `DEFAULT_VALUE <raw> <label>` — numeric only when the
                    // label itself parses as a number.
                    cursor += 1;
                    let raw = match tokens.get(cursor) {
                        Some(Token::Num(value)) => *value,
                        _ => break,
                    };
                    let phys = match tokens.get(cursor + 1) {
                        Some(Token::Num(value)) => *value,
                        Some(Token::Str(text)) => text.parse::<f64>().unwrap_or(f64::NAN),
                        _ => break,
                    };
                    cursor += 2;
                    if phys.is_finite() {
                        default = Some((raw, phys));
                    }
                }
                "TAB_INTP" | "TAB_NOINTP" | "TAB_VERB" => cursor += 1,
                _ => cursor += 1,
            },
            Token::Num(raw) => {
                let raw = *raw;
                cursor += 1;
                match tokens.get(cursor) {
                    Some(Token::Num(phys)) => {
                        points.push((raw, *phys));
                        cursor += 1;
                    }
                    Some(Token::Str(label)) => {
                        // TAB_VERB point with a numeric label; a
                        // non-numeric label ends the usable point run.
                        let parsed = label.parse::<f64>().ok().filter(|v| v.is_finite());
                        cursor += 1;
                        match parsed {
                            Some(phys) => points.push((raw, phys)),
                            None => break,
                        }
                    }
                    _ => break,
                }
            }
            _ => cursor += 1,
        }
    }
    let _ = module;
    (
        CompuTable {
            name,
            kind,
            points,
            default,
        },
        cursor,
    )
}

/// One A2L token. Strings carry their unescaped content; numbers carry
/// their `f64` value; everything else is an identifier, a `/`, or stray
/// punctuation the walker skips.
#[derive(Debug, Clone, PartialEq)]
enum Token<'t> {
    /// `/` — the block keyword introducer.
    Slash,
    /// A bare word (block keyword, name, table-type keyword).
    Ident(&'t str),
    /// A numeric literal.
    Num(f64),
    /// A quoted string, unescaped.
    Str(String),
    /// Any other single character (`,` `;` `{` `}` …) — skipped.
    Other,
}

/// Total A2L token scan: whitespace, `/* … */` comments, and quoted strings
/// are handled; malformed constructs (an unterminated comment or string)
/// consume the rest of the input rather than failing.
fn lex(text: &str) -> Vec<Token<'_>> {
    let bytes = text.as_bytes();
    let mut tokens = Vec::new();
    let mut index = 0usize;
    while index < bytes.len() {
        let byte = bytes[index];
        match byte {
            b'/' if bytes.get(index + 1) == Some(&b'*') => {
                // Comment to the closing `*/`, or to end of input.
                index += 2;
                let mut closed = false;
                while index < bytes.len() {
                    if bytes[index] == b'*' && bytes.get(index + 1) == Some(&b'/') {
                        index += 2;
                        closed = true;
                        break;
                    }
                    index += 1;
                }
                if !closed {
                    break;
                }
            }
            b'/' => {
                tokens.push(Token::Slash);
                index += 1;
            }
            b'"' => {
                index += 1;
                let mut text_buffer = String::new();
                let mut closed = false;
                while index < bytes.len() {
                    let byte = bytes[index];
                    match byte {
                        b'"' => {
                            index += 1;
                            closed = true;
                            break;
                        }
                        b'\\' => {
                            // Escape: copy the escaped character verbatim.
                            if let Some(escaped) = bytes.get(index + 1) {
                                if let Ok(ch) = core::str::from_utf8(&[*escaped]) {
                                    if let Ok(ch) = ch.chars().next() {
                                        text_buffer.push(ch);
                                    }
                                }
                                index += 2;
                            } else {
                                index += 1;
                            }
                        }
                        _ => {
                            let start = index;
                            let width = utf8_width(byte);
                            index += width;
                            if let Some(chunk) = text.get(start..index) {
                                text_buffer.push_str(chunk);
                            }
                        }
                    }
                }
                if !closed {
                    break;
                }
                tokens.push(Token::Str(text_buffer));
            }
            b'0'..=b'9' | b'-' | b'+' | b'.' => {
                let start = index;
                if matches!(byte, b'-' | b'+') {
                    index += 1;
                }
                let mut hex = false;
                if byte == b'0' && matches!(bytes.get(index), Some(b'x' | b'X')) {
                    hex = true;
                    index += 2;
                }
                while let Some(&next) = bytes.get(index) {
                    let acceptable = if hex {
                        next.is_ascii_hexdigit()
                    } else {
                        next.is_ascii_digit() || matches!(next, b'.' | b'e' | b'E')
                    };
                    if !acceptable {
                        break;
                    }
                    index += 1;
                }
                let value = text[start..index].parse::<f64>().unwrap_or(0.0);
                tokens.push(Token::Num(value));
            }
            b'A'..=b'Z' | b'a'..=b'z' | b'_' => {
                let start = index;
                while let Some(&next) = bytes.get(index) {
                    if next.is_ascii_alphanumeric() || matches!(next, b'_' | b'.' | b'[' | b']') {
                        index += 1;
                    } else {
                        break;
                    }
                }
                tokens.push(Token::Ident(&text[start..index]));
            }
            _ => {
                tokens.push(Token::Other);
                index += 1;
            }
        }
    }
    tokens
}

/// UTF-8 sequence length of the lead byte — total, never reads past the
/// slice (invalid bytes advance one).
fn utf8_width(byte: u8) -> usize {
    match byte {
        0x00..=0x7F => 1,
        0xC0..=0xDF => 2,
        0xE0..=0xEF => 3,
        0xF0..=0xF7 => 4,
        _ => 1,
    }
}