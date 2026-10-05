//! The resolved calibration model: modules, characteristics, measurements.
//!
//! [`a2l_parse`] hands back a *declaration* graph in which every
//! conversion is still an unresolvable string reference. The types here are
//! the *resolved* view a calibration engineer actually drives: the
//! `COMPU_METHOD` has been looked up and attached to its characteristic, the
//! deposit datatype is known, and the A2L type keyword is presented as
//! [`CharKind`].
//!
//! Resolution happens once, at [`CalibrationProject`](crate::CalibrationProject)
//! construction, so every later access is a field read rather than a search.

use a2l_parse::{CharType, Characteristic as A2lCharacteristic, DataType};

use crate::conversion::CompuMethod;
use crate::error::CalError;

/// One ECU's calibration surface.
#[derive(Debug, Clone, PartialEq)]
pub struct Module {
    /// MODULE name — the key every lookup takes.
    pub name: String,
    /// CHARACTERISTIC blocks: the adjustable parameters, in file order.
    pub characteristics: Vec<Characteristic>,
    /// MEASUREMENT blocks: the ECU-internal quantities, in file order.
    pub measurements: Vec<Measurement>,
}

impl Module {
    /// Resolve every reference in `module`, attaching each characteristic's
    /// `COMPU_METHOD` and deposit datatype.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownCompuMethod`] when a `COMPU_METHOD` reference is
    /// not declared in the same module — A2L scopes method names per module,
    /// so a missing declaration is a real defect, not a lookup miss.
    pub(crate) fn resolve(
        module: &a2l_parse::A2lModule,
        tables: &crate::tables::TableSet,
    ) -> Result<Self, CalError> {
        let mut characteristics = Vec::with_capacity(module.characteristics.len());
        for raw in &module.characteristics {
            characteristics.push(Characteristic::resolve(raw, module, tables)?);
        }
        let mut measurements = Vec::with_capacity(module.measurements.len());
        for raw in &module.measurements {
            measurements.push(Measurement::resolve(raw, module, tables)?);
        }
        Ok(Self {
            name: module.name.clone(),
            characteristics,
            measurements,
        })
    }

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
}

/// A CHARACTERISTIC: one adjustable calibration parameter, fully resolved.
#[derive(Debug, Clone, PartialEq)]
pub struct Characteristic {
    /// CHARACTERISTIC name.
    pub name: String,
    /// ECU address of the parameter's deposit.
    pub address: u32,
    /// RECORD_LAYOUT reference (the deposit's memory layout).
    pub deposit: String,
    /// Calibration value type.
    pub kind: CharKind,
    /// The resolved `COMPU_METHOD` for this parameter.
    pub conversion: CompuMethod,
    /// Declared lower limit, on the **physical** scale.
    pub lower_limit: f64,
    /// Declared upper limit, on the **physical** scale.
    pub upper_limit: f64,
    /// Maximum difference between two adjustment steps (physical units).
    pub max_diff: f64,
    /// The deposit element datatype, resolved from the `RECORD_LAYOUT`'s
    /// `FNC_VALUES` clause.
    ///
    /// `None` when the record layout is undeclared or carries no
    /// `FNC_VALUES` — legal for a `CURVE`/`MAP`/`VAL_BLK` block, and the
    /// signal that this characteristic's byte length is not derivable from
    /// the description.
    pub datatype: Option<DataType>,
}

impl Characteristic {
    /// Resolve one A2L CHARACTERISTIC against its module's declarations.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownCompuMethod`] if the conversion reference is
    /// undeclared.
    pub(crate) fn resolve(
        raw: &A2lCharacteristic,
        module: &a2l_parse::A2lModule,
        tables: &crate::tables::TableSet,
    ) -> Result<Self, CalError> {
        let method = module
            .compu_method_by_name(&raw.conversion)
            .ok_or_else(|| CalError::UnknownCompuMethod(raw.conversion.clone()))?;
        let datatype = module
            .record_layout_by_name(&raw.deposit)
            .and_then(|layout| layout.fnc_values)
            .map(|fnc| fnc.datatype);
        Ok(Self {
            name: raw.name.clone(),
            address: raw.address,
            deposit: raw.deposit.clone(),
            kind: CharKind::from(raw.r#type),
            conversion: CompuMethod::new(method.clone(), tables),
            lower_limit: raw.lower_limit,
            upper_limit: raw.upper_limit,
            max_diff: raw.max_diff,
            datatype,
        })
    }

    /// Interpret the raw memory bit pattern on the physical scale.
    ///
    /// Sign-extends for the signed ASAP2 datatypes, so a `SBYTE` holding
    /// `0xFF` converts as −1, not 255.
    #[must_use]
    pub fn to_physical(&self, raw: u64) -> f64 {
        self.conversion
            .to_physical(self.signed_value(raw) as f64)
    }

    /// The signed count a raw bit pattern denotes.
    ///
    /// With no resolved datatype the pattern is taken as unsigned, which is
    /// the only reading a bare block deposit supports.
    #[must_use]
    pub fn signed_value(&self, raw: u64) -> i64 {
        let Some(datatype) = self.datatype else {
            return raw.min(i64::MAX as u64) as i64;
        };
        let width = u32::try_from(datatype.size_bytes() * 8).unwrap_or(64).min(64);
        if !is_signed_datatype(datatype) {
            return raw.min(i64::MAX as u64) as i64;
        }
        // Sign-extend from `width` bits: shift out the sign, shift back in.
        let shift = 64 - width;
        ((raw << shift) as i64) >> shift
    }

    /// The deposit element width in bits, or `None` when the datatype is
    /// unresolved or the characteristic deposits a block.
    #[must_use]
    pub fn deposit_width(&self) -> Option<u32> {
        match self.datatype {
            Some(datatype) if !self.kind.is_block() => {
                Some(u32::try_from(datatype.size_bytes() * 8).unwrap_or(64))
            }
            _ => None,
        }
    }

    /// The deposit element size in bytes, or `None` when the datatype is
    /// unresolved. For a block deposit this is the size of *one element*.
    #[must_use]
    pub fn deposit_size(&self) -> Option<usize> {
        self.datatype.map(DataType::size_bytes)
    }

    /// The raw count range this characteristic's deposit can hold, as
    /// `(min, max)` on the signed scale.
    ///
    /// `None` when the datatype is unresolved.
    #[must_use]
    pub fn raw_range(&self) -> Option<(f64, f64)> {
        self.datatype.map(raw_range)
    }
}

/// A MEASUREMENT: one ECU-internal quantity available for measurement.
#[derive(Debug, Clone, PartialEq)]
pub struct Measurement {
    /// MEASUREMENT name.
    pub name: String,
    /// ECU address of the raw value, `0` when the A2L declares none.
    pub address: u32,
    /// The resolved `COMPU_METHOD`.
    pub conversion: CompuMethod,
    /// The ASAP2 datatype of the raw value in ECU memory.
    pub datatype: DataType,
}

impl Measurement {
    /// Resolve one A2L MEASUREMENT against its module's declarations.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownCompuMethod`] if the conversion reference is
    /// undeclared.
    pub(crate) fn resolve(
        raw: &a2l_parse::Measurement,
        module: &a2l_parse::A2lModule,
        tables: &crate::tables::TableSet,
    ) -> Result<Self, CalError> {
        let method = module
            .compu_method_by_name(&raw.conversion)
            .ok_or_else(|| CalError::UnknownCompuMethod(raw.conversion.clone()))?;
        Ok(Self {
            name: raw.name.clone(),
            address: raw.address,
            conversion: CompuMethod::new(method.clone(), tables),
            datatype: raw.datatype,
        })
    }

    /// Size of the raw value in ECU memory, in bytes.
    #[must_use]
    pub fn size_bytes(&self) -> usize {
        self.datatype.size_bytes()
    }

    /// Interpret the raw memory bit pattern on the physical scale,
    /// sign-extending for the signed ASAP2 datatypes.
    #[must_use]
    pub fn to_physical(&self, raw: u64) -> f64 {
        let width = u32::try_from(self.datatype.size_bytes() * 8).unwrap_or(64);
        let shift = 64 - width.min(64);
        let signed = is_signed_datatype(self.datatype);
        let value = if signed {
            ((raw << shift) as i64) >> shift
        } else {
            raw.min(i64::MAX as u64) as i64
        };
        self.conversion.to_physical(value as f64)
    }

    /// The physical raw span this datatype can hold, as `(min, max)`.
    #[must_use]
    pub fn raw_range(&self) -> (f64, f64) {
        raw_range(self.datatype)
    }
}

/// Whether an ASAP2 datatype is two's-complement signed.
#[must_use]
pub fn is_signed_datatype(datatype: DataType) -> bool {
    matches!(
        datatype,
        DataType::Sbyte | DataType::Sword | DataType::Slong | DataType::Int64
    )
}

/// The raw count range a datatype's bit pattern can denote, as `(min, max)`
/// on the **signed** scale — negative for the signed types.
///
/// This is the bracket [`CalibrationProject::to_raw`] checks against: it is
/// what the deposit physically holds, independent of any conversion.
#[must_use]
pub fn raw_range(datatype: DataType) -> (f64, f64) {
    let bits = u32::try_from(datatype.size_bytes() * 8).unwrap_or(64);
    if is_signed_datatype(datatype) {
        let half = 2f64.powi(i32::try_from(bits - 1).unwrap_or(31));
        (-half, half - 1.0)
    } else {
        let top = 2f64.powi(i32::try_from(bits).unwrap_or(64));
        (0.0, top - 1.0)
    }
}

/// The A2L CHARACTERISTIC type keywords, as presented by this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[allow(clippy::exhaustive_enums)]
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
    /// Map an A2L `CharType` onto this crate's kind enum.
    #[must_use]
    pub const fn from(kind: CharType) -> Self {
        match kind {
            CharType::Value => Self::Value,
            CharType::Curve => Self::Curve,
            CharType::Map => Self::Map,
            CharType::ValBlk => Self::ValBlk,
            CharType::Ascii => Self::Ascii,
        }
    }

    /// The ASAP2 keyword.
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

    /// `true` for the kinds that deposit a *block* of elements rather than
    /// one scalar.
    ///
    /// A block deposit has no single element width, so its byte length comes
    /// from the axis point counts rather than the record layout's datatype.
    #[must_use]
    pub const fn is_block(self) -> bool {
        matches!(self, Self::Curve | Self::Map | Self::ValBlk)
    }
}

impl From<CharType> for CharKind {
    fn from(kind: CharType) -> Self {
        Self::from(kind)
    }
}
