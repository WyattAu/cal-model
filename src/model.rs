//! The calibration data model: modules, characteristics, measurements, and
//! the CAN signal bindings a DBC database attaches to them.
//!
//! Every type here is a *resolved* view of an [`a2l_parse`] declaration —
//! the COMPU_METHOD a CHARACTERISTIC references has already been turned
//! into a [`CompuMethod`], and the RECORD_LAYOUT it deposits through has
//! already yielded a datatype and a byte offset. Nothing in this module
//! parses; it is the vocabulary [`CalibrationProject`](crate::CalibrationProject),
//! [`CalibrationSession`](crate::CalibrationSession), and
//! [`calibrate_curve`](crate::calibrate_curve) speak.

use crate::conversion::CompuMethod;
use a2l_parse::{CharType, DataType};

/// The ASAP2 CHARACTERISTIC type keywords, as the calibration layer
/// consumes them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CharKind {
    /// `VALUE` — a single scalar.
    Value,
    /// `CURVE` — one-dimensional table.
    Curve,
    /// `MAP` — two-dimensional table.
    Map,
    /// `VAL_BLK` — multi-dimensional value block without axes.
    ValBlk,
    /// `ASCII` — a string.
    Ascii,
}

impl CharKind {
    /// Re-tag an [`a2l_parse::CharType`] for the calibration layer.
    #[must_use]
    pub const fn from_char_type(kind: CharType) -> Self {
        match kind {
            CharType::Value => Self::Value,
            CharType::Curve => Self::Curve,
            CharType::Map => Self::Map,
            CharType::ValBlk => Self::ValBlk,
            CharType::Ascii => Self::Ascii,
        }
    }

    /// The keyword this kind came from.
    #[must_use]
    pub const fn keyword(self) -> &'static str {
        match self {
            Self::Value => "VALUE",
            Self::Curve => "CURVE",
            Self::Map => "MAP",
            Self::ValBlk => "VAL_BLK",
            Self::Ascii => "ASCII",
        }
    }

    /// `true` when the characteristic is an array of values rather than a
    /// single scalar.
    #[must_use]
    pub const fn is_multi_element(self) -> bool {
        matches!(self, Self::Curve | Self::Map | Self::ValBlk)
    }
}

/// A CHARACTERISTIC: one calibration parameter, resolved and ready to read,
/// write, and validate.
#[derive(Debug, Clone, PartialEq)]
pub struct Characteristic {
    /// CHARACTERISTIC name.
    pub name: String,
    /// `VALUE`/`CURVE`/`MAP`/`VAL_BLK`/`ASCII`.
    pub kind: CharKind,
    /// ECU address of the deposit's first byte.
    pub address: u32,
    /// RECORD_LAYOUT the values deposit through.
    pub deposit: String,
    /// Byte offset of the value array inside the deposit record
    /// (`FNC_VALUES POSITION`).
    pub deposit_position: u8,
    /// Element datatype, from the deposit's `FNC_VALUES DATATYPE`.
    pub datatype: DataType,
    /// Number of elements in the deposit. `a2l_parse` does not retain the
    /// ASAP2 `NUMBER` keyword, so this is `1` until declared with
    /// [`CalibrationProject::declare_elements`](crate::CalibrationProject::declare_elements)
    /// — see that method's documentation for why it is an explicit
    /// declaration rather than a guess.
    pub elements: usize,
    /// The resolved conversion.
    pub conversion: CompuMethod,
    /// `LOWER_LIMIT` — physical.
    pub lower_limit: f64,
    /// `UPPER_LIMIT` — physical.
    pub upper_limit: f64,
    /// `MAX_DIFF` — the largest step a single adjustment may take.
    pub max_diff: f64,
}

impl Characteristic {
    /// Element width in bits.
    #[must_use]
    pub fn element_bits(&self) -> u32 {
        // size_bytes() is 1..=8 for every DataType, so the multiplication
        // cannot overflow and the conversion to u32 is exact.
        u32::try_from(self.datatype.size_bytes() * 8).unwrap_or(32)
    }

    /// `true` when the deposit datatype is two's-complement signed.
    #[must_use]
    pub const fn is_signed(&self) -> bool {
        matches!(
            self.datatype,
            DataType::Sbyte | DataType::Sword | DataType::Slong | DataType::Int64
        )
    }

    /// Bit mask covering one element's bits.
    #[must_use]
    pub fn element_mask(&self) -> u64 {
        let bits = self.element_bits();
        if bits >= 64 {
            u64::MAX
        } else {
            (1_u64 << bits) - 1
        }
    }

    /// Total deposit size in bytes (all elements).
    #[must_use]
    pub fn size_bytes(&self) -> usize {
        self.elements * self.datatype.size_bytes()
    }

    /// Interpret a raw *bit pattern* as the numeric value the conversion
    /// sees: signed datatypes are sign-extended, unsigned ones masked.
    #[must_use]
    pub fn raw_value(&self, raw: u64) -> f64 {
        let masked = raw & self.element_mask();
        if self.is_signed() {
            #[allow(clippy::cast_precision_loss)]
            {
                sign_extend(masked, self.element_bits()) as f64
            }
        } else {
            #[allow(clippy::cast_precision_loss)]
            {
                masked as f64
            }
        }
    }

    /// Encode a numeric raw value back into its memory bit pattern.
    #[must_use]
    pub fn raw_bits(&self, value: f64) -> u64 {
        if self.is_signed() {
            #[allow(clippy::cast_possible_wrap)]
            let bits = (value as i64 as u64) & self.element_mask();
            bits
        } else {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let bits = (value as u64) & self.element_mask();
            bits
        }
    }

    /// The raw-value bounds of the deposit, in *numeric* terms (negative
    /// for signed datatypes).
    #[must_use]
    pub fn value_bounds(&self) -> (f64, f64) {
        let bits = self.element_bits();
        let exponent = i32::try_from(bits).unwrap_or(32);
        if self.is_signed() {
            let half = 2_f64.powi(exponent - 1);
            (-half, half - 1.0)
        } else if bits >= 64 {
            #[allow(clippy::cast_precision_loss)]
            {
                (0.0, u64::MAX as f64)
            }
        } else {
            (0.0, 2_f64.powi(exponent) - 1.0)
        }
    }

    /// `true` when the A2L declaration constrains the physical value at
    /// all. A degenerate range (`lower >= upper`, as generators emit for
    /// ASCII identifiers) imposes no constraint — the same convention
    /// `dbc-parse` applies to an empty `[min|max]`.
    #[must_use]
    pub fn has_limits(&self) -> bool {
        self.lower_limit.is_finite()
            && self.upper_limit.is_finite()
            && self.lower_limit < self.upper_limit
    }

    /// `true` when `physical` is inside the declared limits (or no limits
    /// are declared).
    #[must_use]
    pub fn within_limits(&self, physical: f64) -> bool {
        if !physical.is_finite() {
            return false;
        }
        if !self.has_limits() {
            return true;
        }
        physical >= self.lower_limit && physical <= self.upper_limit
    }

    /// Return a copy with the element count set (the builder form of
    /// [`CalibrationProject::declare_elements`](crate::CalibrationProject::declare_elements)).
    #[must_use]
    pub fn with_elements(mut self, elements: usize) -> Self {
        self.elements = elements;
        self
    }
}

/// A MEASUREMENT: an ECU-internal quantity, resolved for measurement.
#[derive(Debug, Clone, PartialEq)]
pub struct Measurement {
    /// MEASUREMENT name.
    pub name: String,
    /// `ECU_ADDRESS` (`0` when the declaration omitted it).
    pub address: u32,
    /// The resolved conversion.
    pub conversion: CompuMethod,
    /// Raw datatype in ECU memory.
    pub datatype: DataType,
}

impl Measurement {
    /// Element width in bytes.
    #[must_use]
    pub fn size_bytes(&self) -> usize {
        self.datatype.size_bytes()
    }

    /// `true` when the datatype is two's-complement signed.
    #[must_use]
    pub const fn is_signed(&self) -> bool {
        matches!(
            self.datatype,
            DataType::Sbyte | DataType::Sword | DataType::Slong | DataType::Int64
        )
    }

    /// Interpret a raw bit pattern as the numeric value the conversion
    /// sees.
    #[must_use]
    pub fn raw_value(&self, raw: u64) -> f64 {
        let bits = u32::try_from(self.datatype.size_bytes() * 8).unwrap_or(32);
        let mask = if bits >= 64 {
            u64::MAX
        } else {
            (1_u64 << bits) - 1
        };
        let masked = raw & mask;
        #[allow(clippy::cast_precision_loss)]
        let value = if self.is_signed() {
            sign_extend(masked, bits) as f64
        } else {
            masked as f64
        };
        value
    }
}

/// One `/begin MODULE` resolved into the calibration vocabulary.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Module {
    /// MODULE name.
    pub name: String,
    /// Characteristics, in declaration order.
    pub characteristics: Vec<Characteristic>,
    /// Measurements, in declaration order.
    pub measurements: Vec<Measurement>,
}

impl Module {
    /// The CHARACTERISTIC named `name`.
    #[must_use]
    pub fn characteristic(&self, name: &str) -> Option<&Characteristic> {
        self.characteristics.iter().find(|c| c.name == name)
    }

    /// The MEASUREMENT named `name`.
    #[must_use]
    pub fn measurement(&self, name: &str) -> Option<&Measurement> {
        self.measurements.iter().find(|m| m.name == name)
    }

    /// Characteristic names, in declaration order.
    pub fn characteristic_names(&self) -> impl Iterator<Item = &str> {
        self.characteristics.iter().map(|c| c.name.as_str())
    }

    /// Measurement names, in declaration order.
    pub fn measurement_names(&self) -> impl Iterator<Item = &str> {
        self.measurements.iter().map(|m| m.name.as_str())
    }
}

/// Sign-extend the low `bits` bits of `raw` to an `i64`.
fn sign_extend(raw: u64, bits: u32) -> i64 {
    if bits == 0 || bits >= 64 {
        #[allow(clippy::cast_possible_wrap)]
        {
            return raw as i64;
        }
    }
    let sign_bit = 1_u64 << (bits - 1);
    #[allow(clippy::cast_possible_wrap)]
    if raw & sign_bit != 0 {
        (raw | (u64::MAX << bits)) as i64
    } else {
        #[allow(clippy::cast_possible_wrap)]
        {
            raw as i64
        }
    }
}
