//! `COMPU_TAB` / `COMPU_VTAB` points, and the scanner that recovers them.
//!
//! `a2l-parse` keeps only the subset of ASAP2 the calibration stack
//! consumes. For a `TABLE` COMPU_METHOD it records the `COMPU_TAB_REF`
//! *name* and skips the referenced block entirely — which is right, because
//! the block is a lookup table and not part of the declaration graph. It
//! leaves a hole, though: a `TABLE` conversion cannot be evaluated without
//! its points, and silently treating an unknown tab as identity would write
//! wrong numbers into an ECU.
//!
//! This module closes that hole with a small, **total** scanner over the
//! description text. It is not a second A2L parser: it tokenizes just enough
//! (`/begin`, `/end`, identifiers, numbers, strings) to pull out
//!
//! - `COMPU_TAB` / `COMPU_VTAB` — the interpolation/verb kind, the
//!   `DEFAULT_VALUE_NUMERIC` fallback, and the `(input, value)` point list,
//! - `CHARACTERISTIC` / `AXIS_DESCR` axis point counts, so a block deposit's
//!   byte length is derivable,
//!
//! and to ignore everything else. Every failure mode is a typed
//! [`CalError`]; no input can panic.

use std::collections::BTreeMap;

use crate::error::CalError;

/// How a `COMPU_TAB` interpolates between its points.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[allow(clippy::exhaustive_enums)]
pub enum CompuTabKind {
    /// `TAB_INTP` — linear interpolation between neighbouring points.
    Intp,
    /// `TAB_VERB` — stepwise: the value of the nearest point below the
    /// input (a *verb* table maps a code to a meaning, so it must not
    /// invent intermediate values).
    Verb,
}

/// A `COMPU_TAB` or `COMPU_VTAB`: an input → value lookup.
///
/// Points are kept sorted by input and de-duplicated, so a lookup is a
/// binary search and evaluation is a pure function of the set rather than
/// of file order.
#[derive(Debug, Clone, PartialEq)]
pub struct CompuTable {
    /// The table name referenced by `COMPU_TAB_REF`.
    pub name: String,
    /// Interpolation kind.
    pub kind: CompuTabKind,
    /// `(input, value)` points, ascending by input.
    pub points: Vec<(f64, f64)>,
    /// Value outside the tabulated range, from `DEFAULT_VALUE_NUMERIC`
    /// (the second number, the *numeric* one). Falls back to the nearest
    /// endpoint when the clause is absent.
    pub default: f64,
}

impl CompuTable {
    /// Build a table from a raw point list.
    ///
    /// Points are sorted ascending by input; duplicate inputs keep the
    /// **last** occurrence, matching the "later entry wins" reading of a
    /// duplicate table row. Non-finite points are dropped, because a NaN
    /// input would poison every subsequent comparison.
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        kind: CompuTabKind,
        points: Vec<(f64, f64)>,
        default: Option<f64>,
    ) -> Self {
        let mut points: Vec<(f64, f64)> = points
            .into_iter()
            .filter(|(input, value)| input.is_finite() && value.is_finite())
            .collect();
        points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal));
        // Stable sort keeps the last-written duplicate last within a run of
        // equal inputs; fold forward so the newest row wins.
        points.dedup_by(|later, earlier| {
            if later.0 == earlier.0 {
                earlier.1 = later.1;
                true
            } else {
                false
            }
        });
        let fallback = points
            .first()
            .map(|(_, value)| *value)
            .or_else(|| points.last().map(|(_, value)| *value));
        Self {
            name: name.into(),
            kind,
            points,
            default: default.filter(|d| d.is_finite()).or(fallback).unwrap_or(0.0),
        }
    }

    /// `true` when the table carries at least one usable point.
    #[must_use]
    pub fn is_usable(&self) -> bool {
        !self.points.is_empty()
    }

    /// Evaluate the table at a raw input count.
    ///
    /// `TAB_INTP` interpolates linearly between the bracketing points and
    /// clamps outside the tabulated range to `default`. `TAB_VERB` steps:
    /// it returns the greatest tabulated input ≤ `input`, or `default`
    /// below the first point.
    ///
    /// A non-finite `input` yields `default` rather than a NaN, so one
    /// corrupt reading cannot propagate through a whole calibration.
    #[must_use]
    pub fn evaluate(&self, input: f64) -> f64 {
        if !input.is_finite() || self.points.is_empty() {
            return self.default;
        }
        let Some((first_input, _)) = self.points.first() else {
            return self.default;
        };
        if input < *first_input {
            return self.default;
        }
        let Some((last_input, _)) = self.points.last() else {
            return self.default;
        };
        if input > *last_input {
            return self.default;
        }
        if self.kind == CompuTabKind::Verb {
            // Greatest point at or below `input`: `partition_point` gives
            // the first index whose input is strictly greater.
            let idx = self.points.partition_point(|(x, _)| *x <= input);
            return match idx.checked_sub(1) {
                Some(i) => self.points.get(i).map_or(self.default, |(_, v)| *v),
                None => self.default,
            };
        }
        // Linear interpolation between the bracketing points. `input ==
        // last_input` is handled above only for `>`, so here it falls into
        // the final segment; an exact endpoint hit still returns the
        // tabulated value because the weight evaluates to exactly 1.
        let upper = self.points.partition_point(|(x, _)| *x <= input);
        let (Some(lower), Some(upper)) = (
            upper.checked_sub(1).and_then(|i| self.points.get(i)),
            self.points.get(upper),
        ) else {
            return self.points
                .last()
                .map_or(self.default, |(_, v)| *v);
        };
        if upper.0 == lower.0 {
            return upper.1;
        }
        let weight = (input - lower.0) / (upper.0 - lower.0);
        lower.1 + weight * (upper.1 - lower.1)
    }

    /// The input at or below `input` for a `TAB_VERB` table — the *code* a
    /// stepped table maps, which is what an inverse conversion needs.
    ///
    /// `None` when `input` is below the first tabulated input.
    #[must_use]
    pub fn invert_verb(&self, physical: f64) -> Option<f64> {
        if !physical.is_finite() {
            return None;
        }
        // The largest tabulated input whose value does not exceed the
        // target: walk ascending and keep the last candidate that fits.
        let mut best = None;
        for (input, value) in &self.points {
            if *value <= physical {
                best = Some(*input);
            } else if best.is_some() {
                break;
            }
        }
        best
    }

    /// Invert a `TAB_INTP` table: the input whose value is nearest
    /// `physical`.
    ///
    /// Only meaningful for a *monotonically increasing* table, which is the
    /// case for every real conversion table; a table that folds back on
    /// itself has no single-valued inverse, and `None` says so rather than
    /// picking an arbitrary branch.
    #[must_use]
    pub fn invert_intp(&self, physical: f64) -> Option<f64> {
        if !physical.is_finite() || self.points.is_empty() {
            return None;
        }
        let increasing = self
            .points
            .windows(2)
            .all(|w| w[1].1 >= w[0].1 && w[1].0 > w[0].0);
        if !increasing {
            return None;
        }
        // Binary search the bracketing segment on the value axis.
        let idx = self
            .points
            .partition_point(|(_, value)| *value < physical)
            .min(self.points.len() - 1);
        if idx == 0 {
            return Some(self.points.first().map_or(0.0, |(input, _)| *input));
        }
        let lower = self.points.get(idx.checked_sub(1)?)?;
        let upper = self.points.get(idx)?;
        if upper.1 == lower.1 {
            return Some(lower.0);
        }
        let weight = (physical - lower.1) / (upper.1 - lower.1);
        Some(lower.0 + weight.clamp(0.0, 1.0) * (upper.0 - lower.0))
    }
}

/// Every table recovered from one description, indexed by name.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TableSet {
    tables: BTreeMap<String, CompuTable>,
    /// `CHARACTERISTIC` name → axis point counts, in declaration order.
    axis_points: BTreeMap<String, usize>,
}

impl TableSet {
    /// The table named `name`.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&CompuTable> {
        self.tables.get(name)
    }

    /// Insert or replace a table by name.
    pub fn insert(&mut self, table: CompuTable) {
        self.tables.insert(table.name.clone(), table);
    }

    /// The axis point count declared for `characteristic`.
    #[must_use]
    pub fn axis_points(&self, characteristic: &str) -> Option<usize> {
        self.axis_points.get(characteristic).copied()
    }

    /// Record an axis point count for a characteristic.
    pub fn set_axis_points(&mut self, characteristic: impl Into<String>, points: usize) {
        self.axis_points.insert(characteristic.into(), points);
    }

    /// Number of tables held.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tables.len()
    }

    /// `true` when no tables were recovered.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tables.is_empty()
    }

    /// The tables, in name order.
    pub fn iter(&self) -> impl Iterator<Item = &CompuTable> {
        self.tables.values()
    }
}

// ---------------------------------------------------------------------------
// The scanner
// ---------------------------------------------------------------------------

/// One lexical token, kept deliberately small.
#[derive(Debug, Clone, PartialEq)]
enum Tok {
    /// A bare identifier or keyword.
    Word(String),
    /// A number, already parsed to `f64`.
    Number(f64),
    /// A quoted string; contents only.
    Str(String),
    /// A slash-prefixed keyword: `/begin`, `/end`, `/include`.
    Slash(String),
}

/// Scan `text` for every `COMPU_TAB`/`COMPU_VTAB` block and for the axis
/// point counts of block-shaped characteristics.
///
/// The scanner is total: a truncated block, an unterminated string, or a
/// number that does not parse yields a typed [`CalError::A2l`]-family
/// failure or a partial result — never a panic and never an infinite loop
/// (the tokenizer always advances).
///
/// # Errors
///
/// [`CalError::UnterminatedA2lBlock`] when a `/begin` has no matching
/// `/end`, [`CalError::MalformedA2l`] when a token stream is not valid
/// ASAP2 at all.
pub fn scan(text: &str) -> Result<TableSet, CalError> {
    let tokens = tokenize(text)?;
    let mut set = TableSet::default();
    let mut i = 0usize;
    while i < tokens.len() {
        let Tok::Slash(word) = &tokens[i] else {
            i += 1;
            continue;
        };
        if word != "begin" {
            i += 1;
            continue;
        }
        let Some(Tok::Word(block)) = tokens.get(i + 1) else {
            return Err(CalError::MalformedA2l(
                "/begin must be followed by a block keyword".to_owned(),
            ));
        };
        let block = block.clone();
        // Locate the matching `/end`, tracking nesting so a vendor block
        // containing a nested one cannot truncate our slice.
        let body_start = i + 2;
        let Some(body_end) = matching_end(&tokens, i + 1, &block)? else {
            return Err(CalError::UnterminatedA2lBlock(block));
        };
        match block.as_str() {
            "COMPU_TAB" | "COMPU_VTAB" => {
                if let Some(table) = parse_table(&block, &tokens[body_start..body_end])? {
                    set.insert(table);
                }
            }
            "CHARACTERISTIC" => {
                if let Some((name, points)) = parse_axis_points(&tokens[body_start..body_end]) {
                    set.set_axis_points(name, points);
                }
            }
            _ => {}
        }
        i = body_end + 2; // skip past `/end BLOCK`
    }
    Ok(set)
}

/// Index of the token *after* the `/end` that closes `open` at `open_idx`.
fn matching_end(tokens: &[Tok], open_idx: usize, open: &str) -> Result<Option<usize>, CalError> {
    // `open_idx` is the block *name*, i.e. we are already one `/begin` deep.
    let mut depth = 1usize;
    let mut i = open_idx;
    while i < tokens.len() {
        if let Tok::Slash(word) = &tokens[i] {
            if word == "begin" {
                depth += 1;
            } else if word == "end" {
                depth -= 1;
                if depth == 0 {
                    let closed = tokens.get(i + 1).and_then(|t| match t {
                        Tok::Word(name) => Some(name.clone()),
                        _ => None,
                    });
                    return match closed {
                        Some(name) if name == open => Ok(Some(i)),
                        Some(name) => Err(CalError::MismatchedA2lBlock {
                            opened: open.to_owned(),
                            closed: name,
                        }),
                        None => Err(CalError::MalformedA2l(format!(
                            "/end must be followed by a block keyword, closing {open}"
                        ))),
                    };
                }
            }
        }
        i += 1;
    }
    Ok(None)
}

/// Build a `CompuTable` from a `COMPU_TAB`/`COMPU_VTAB` body.
fn parse_table(block: &str, body: &[Tok]) -> Result<Option<CompuTable>, CalError> {
    let Some(Tok::Word(name)) = body.first() else {
        return Err(CalError::MalformedA2l(format!(
            "{block} must be followed by a table name"
        )));
    };
    let name = name.clone();
    // An optional quoted description.
    let mut i = 1usize;
    if matches!(body.get(i), Some(Tok::Str(_))) {
        i += 1;
    }
    let kind = match body.get(i) {
        Some(Tok::Word(word)) if word == "TAB_INTP" => CompuTabKind::Intp,
        Some(Tok::Word(word)) if word == "TAB_VERB" => CompuTabKind::Verb,
        // An unrecognised kind is not an error: the points are still a valid
        // table, and treating them as interpolated is the safer of the two
        // wrong guesses (a verb table's caller gets a value in range).
        _ => CompuTabKind::Intp,
    };
    i += 1;

    let mut default = None;
    let mut points = Vec::new();
    while i < body.len() {
        match &body[i] {
            // `DEFAULT_VALUE_NUMERIC <text-col> <numeric-col>` — a
            // numeric conversion wants the *second* column. The keyword
            // lexes as one word, so both spellings are matched explicitly.
            Tok::Word(keyword) if keyword == "DEFAULT_VALUE_NUMERIC" => {
                match (body.get(i + 1), body.get(i + 2)) {
                    (Some(Tok::Number(_)), Some(Tok::Number(numeric))) => {
                        default = Some(*numeric);
                        i += 3;
                    }
                    _ => i += 1,
                }
            }
            // `DEFAULT_VALUE <value>` — a single number, either column.
            Tok::Word(keyword) if keyword == "DEFAULT_VALUE" => {
                match body.get(i + 1) {
                    Some(Tok::Number(value)) => {
                        default = Some(*value);
                        i += 2;
                    }
                    _ => i += 1,
                }
            }
            // Anything else numeric is a `(input, value)` point pair.
            Tok::Number(input) => match body.get(i + 1) {
                Some(Tok::Number(value)) => {
                    points.push((*input, *value));
                    i += 2;
                }
                _ => i += 1,
            },
            _ => i += 1,
        }
    }
    if points.is_empty() && default.is_none() {
        // A table block with neither points nor a default carries nothing
        // usable; dropping it makes the reference resolve to
        // `UnknownComputationTable` at use, which is the honest outcome.
        return Ok(None);
    }
    Ok(Some(CompuTable::new(
        name,
        kind,
        points,
        default,
    )))
}

/// Extract `(characteristic name, axis point count)` from a `CHARACTERISTIC`
/// body, if it declares `NO_AXIS_PTS`.
fn parse_axis_points(body: &[Tok]) -> Option<(String, usize)> {
    let name = match body.first() {
        Some(Tok::Word(name)) => name.clone(),
        _ => return None,
    };
    let mut points: Option<usize> = None;
    let mut i = 0usize;
    while i < body.len() {
        if let Tok::Word(keyword) = &body[i] {
            if keyword == "NO_AXIS_PTS" || keyword == "NO_AXIS_PTS_X" || keyword == "NO_AXIS_PTS_Y"
            {
                if let Some(Tok::Number(value)) = body.get(i + 1) {
                    let count = if value.is_finite() && *value > 0.0 {
                        Some(*value as usize)
                    } else {
                        None
                    };
                    // X and Y both present: the block is a MAP and its
                    // length is the product. Record the product when both
                    // are seen, otherwise the single count.
                    points = Some(match (points, count) {
                        (Some(prev), Some(next)) => prev.saturating_mul(next).max(1),
                        (Some(prev), None) => prev,
                        (None, Some(next)) => next,
                        (None, None) => return None,
                    });
                    i += 2;
                    continue;
                }
            }
        }
        i += 1;
    }
    points.map(|count| (name, count))
}

/// Tokenize `text` into the small [`Tok`] vocabulary.
///
/// Every iteration consumes at least one character, so the loop is
/// guaranteed to terminate on any input, including one with unterminated
/// strings and comments.
///
/// # Errors
///
/// [`CalError::MalformedA2l`] when a `/`-prefixed token is not a known
/// keyword form or a string is unterminated.
fn tokenize(text: &str) -> Result<Vec<Tok>, CalError> {
    let bytes = text.as_bytes();
    let mut tokens = Vec::new();
    let mut i = 0usize;
    while i < bytes.len() {
        let c = bytes[i];
        match c {
            b' ' | b'\t' | b'\r' | b'\n' => i += 1,
            b'/' if bytes.get(i + 1) == Some(&b'*') => {
                // Block comment: skip to the closing `*/`, or to EOF.
                let mut j = i + 2;
                while j + 1 < bytes.len() && !(bytes[j] == b'*' && bytes[j + 1] == b'/') {
                    j += 1;
                }
                i = if j + 1 < bytes.len() { j + 2 } else { bytes.len() };
            }
            b'/' if bytes.get(i + 1) == Some(&b'/') => {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
            }
            b'/' => {
                let start = i;
                i += 1;
                while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
                    i += 1;
                }
                let word = &text[start + 1..i];
                if word.is_empty() {
                    return Err(CalError::MalformedA2l(
                        "stray `/` is not a valid A2L token".to_owned(),
                    ));
                }
                tokens.push(Tok::Slash(word.to_owned()));
            }
            b'"' => {
                let start = i + 1;
                let mut j = start;
                while j < bytes.len() && bytes[j] != b'"' {
                    // ASAP2 strings do not escape; a backslash is literal.
                    j += 1;
                }
                if j >= bytes.len() {
                    return Err(CalError::MalformedA2l(
                        "unterminated quoted string".to_owned(),
                    ));
                }
                tokens.push(Tok::Str(text[start..j].to_owned()));
                i = j + 1;
            }
            b'-' | b'+' | b'.' | b'0'..=b'9' => {
                let start = i;
                i += 1;
                while i < bytes.len()
                    && (bytes[i].is_ascii_digit()
                        || matches!(bytes[i], b'.' | b'e' | b'E')
                        || (matches!(bytes[i], b'-' | b'+')
                            && matches!(bytes[i - 1], b'e' | b'E')))
                {
                    i += 1;
                }
                let raw = &text[start..i];
                match parse_number(raw) {
                    Some(value) => tokens.push(Tok::Number(value)),
                    None if is_hex_or_word(raw) => {
                        // Not a number after all (e.g. a vendor keyword
                        // starting with `-`): treat as an identifier.
                        tokens.push(Tok::Word(raw.to_owned()));
                    }
                    None => {
                        return Err(CalError::MalformedA2l(format!(
                            "`{raw}` is not a valid number"
                        )))
                    }
                }
            }
            _ if c.is_ascii_alphanumeric() || c == b'_' || c == b'%' => {
                let start = i;
                i += 1;
                while i < bytes.len()
                    && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_')
                {
                    i += 1;
                }
                tokens.push(Tok::Word(text[start..i].to_owned()));
            }
            // Any other punctuation (braces, commas, colons) is structural
            // noise for our purposes.
            _ => i += 1,
        }
    }
    Ok(tokens)
}

/// Whether `raw` looks like an identifier rather than a malformed number
/// (so the tokenizer can degrade it to a word instead of failing).
fn is_hex_or_word(raw: &str) -> bool {
    raw.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '-')
}

/// Parse an ASAP2 number: decimal, hexadecimal, or scientific.
///
/// The `0x` form is handled by hand because `f64::from_str` rejects it, and
/// because the estate's grammar has no digit separators in hex — a
/// documented `a2l-parse` gotcha this scanner must not reintroduce.
fn parse_number(raw: &str) -> Option<f64> {
    if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
        return u64::from_str_radix(hex, 16).ok().map(|v| v as f64);
    }
    // Reject the forms `f64` accepts but ASAP2 does not, so a stray word is
    // not silently read as a number: `inf`, `NaN`, and `1_000`.
    if raw.is_empty() || raw.contains('_') {
        return None;
    }
    let lowered = raw.to_ascii_lowercase();
    if lowered.contains("inf") || lowered.contains("nan") {
        return None;
    }
    raw.parse::<f64>().ok()
}
