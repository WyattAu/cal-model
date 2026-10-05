//! The calibration project: a resolved A2L description, plus the optional
//! DBC that binds its characteristics to CAN signals.

use std::collections::BTreeMap;
use std::path::Path;

use dbc_parse::{Dbc, Signal as DbcSignal};

use crate::error::CalError;
use crate::model::{Characteristic, Measurement, Module};
use crate::signal::SignalBinding;
use crate::tables::{self, TableSet};

/// A fully resolved A2L calibration project.
///
/// Construction resolves **every** reference once — each characteristic's
/// `COMPU_METHOD` and record-layout datatype are looked up, and the
/// `COMPU_TAB` points of any `TABLE` conversion are attached — so a
/// successfully built project can be searched freely without any later
/// lookup being able to fail.
///
/// The DBC is optional and attaches separately, because a description
/// usually exists before the network database does.
#[derive(Debug, Clone)]
pub struct CalibrationProject {
    /// The substrate's declaration graph, kept for callers that need the
    /// raw ASAP2 (record layouts, axis descriptions, vendor blocks).
    pub a2l: a2l_parse::A2lProject,
    /// The attached CAN database, when one has been bound.
    dbc: Option<Dbc>,
    /// Resolved modules, in file order.
    modules: Vec<Module>,
    /// `module` → `characteristic` → binding.
    bindings: BTreeMap<(String, String), SignalBinding>,
    /// The tables recovered from the description text.
    tables: TableSet,
}

impl CalibrationProject {
    /// Parse and resolve an A2L description.
    ///
    /// # Errors
    ///
    /// [`CalError::A2l`] when the description does not parse,
    /// [`CalError::UnterminatedA2lBlock`] / [`CalError::MalformedA2l`] /
    /// [`CalError::MismatchedA2lBlock`] when the table scan fails, and
    /// [`CalError::UnknownCompuMethod`] when a characteristic or measurement
    /// references a `COMPU_METHOD` its module does not declare.
    pub fn from_a2l(text: &str) -> Result<Self, CalError> {
        let a2l = a2l_parse::A2lProject::parse(text)?;
        let tables = tables::scan(text)?;
        let mut modules = Vec::with_capacity(a2l.modules.len());
        for module in &a2l.modules {
            modules.push(Module::resolve(module, &tables)?);
        }
        Ok(Self {
            a2l,
            dbc: None,
            modules,
            bindings: BTreeMap::new(),
            tables,
        })
    }

    /// Read and resolve an A2L description from disk.
    ///
    /// # Errors
    ///
    /// [`CalError::Io`] when the file cannot be read; otherwise as
    /// [`CalibrationProject::from_a2l`].
    pub fn from_a2l_file(path: &Path) -> Result<Self, CalError> {
        let text = std::fs::read_to_string(path).map_err(|error| CalError::Io {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
        Self::from_a2l(&text)
    }

    /// Bind a CAN database, resolving every characteristic that a signal
    /// matches by name.
    ///
    /// A characteristic is bound when some signal in the DBC carries the
    /// characteristic's **own name**. That convention — the same name on both
    /// sides — is what a2l suppliers and CAN database authors actually
    /// agree on; guessing from a fuzzy match would silently bind the wrong
    /// signal.
    ///
    /// Re-binding replaces the previous bindings wholesale, so calling this
    /// twice with different databases cannot leave a stale binding behind.
    ///
    /// # Errors
    ///
    /// [`CalError::AmbiguousSignal`] when one characteristic's name appears
    /// as a signal in two messages: there is no correct answer, and picking
    /// one would be a guess.
    pub fn attach_dbc(&mut self, dbc: Dbc) -> Result<(), CalError> {
        let mut bindings: BTreeMap<(String, String), SignalBinding> = BTreeMap::new();
        for message in &dbc.messages {
            let can_id = message.id.raw();
            for signal in &message.signals {
                // A multiplexed signal is absent from most frames, so it
                // cannot be the sole carrier of a calibration value.
                if !matches!(
                    signal.multiplexing_info,
                    dbc_parse::MultiplexingInfo::None
                ) {
                    continue;
                }
                let module_name = self
                    .modules
                    .iter()
                    .find(|module| {
                        module
                            .characteristics
                            .iter()
                            .any(|c| c.name == signal.name)
                    })
                    .map(|module| module.name.clone());
                let Some(module_name) = module_name else {
                    continue;
                };
                let key = (module_name, signal.name.clone());
                if let Some(existing) = bindings.get(&key) {
                    if existing.can_id != can_id {
                        return Err(CalError::AmbiguousSignal {
                            name: signal.name.clone(),
                            can_id,
                            other_can_id: existing.can_id,
                        });
                    }
                    continue;
                }
                bindings.insert(
                    key,
                    binding_from(signal.name.clone(), can_id, signal),
                );
            }
        }
        self.bindings = bindings;
        self.dbc = Some(dbc);
        Ok(())
    }

    /// Every resolved module, in file order.
    pub fn modules(&self) -> impl Iterator<Item = &Module> {
        self.modules.iter()
    }

    /// The module named `name`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] when no such module exists.
    pub fn module(&self, name: &str) -> Result<&Module, CalError> {
        self.modules
            .iter()
            .find(|module| module.name == name)
            .ok_or_else(|| CalError::UnknownModule(name.to_owned()))
    }

    /// The characteristic named `name` in `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] or [`CalError::UnknownCharacteristic`].
    pub fn characteristic(
        &self,
        module: &str,
        name: &str,
    ) -> Result<&Characteristic, CalError> {
        self.module(module)?
            .characteristic(name)
            .ok_or_else(|| CalError::UnknownCharacteristic(name.to_owned()))
    }

    /// The measurement named `name` in `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] or [`CalError::UnknownMeasurement`].
    pub fn measurement(&self, module: &str, name: &str) -> Result<&Measurement, CalError> {
        self.module(module)?
            .measurement(name)
            .ok_or_else(|| CalError::UnknownMeasurement(name.to_owned()))
    }

    /// The CAN signal bound to `module`'s characteristic `c`, if any.
    #[must_use]
    pub fn signal_for_characteristic(&self, module: &str, c: &str) -> Option<&SignalBinding> {
        self.bindings.get(&(module.to_owned(), c.to_owned()))
    }

    /// Every characteristic that has a CAN signal binding.
    pub fn bound_characteristics(&self) -> impl Iterator<Item = (&str, &str, &SignalBinding)> {
        self.bindings
            .iter()
            .map(|((module, name), binding)| (module.as_str(), name.as_str(), binding))
    }

    /// The attached CAN database, if any.
    #[must_use]
    pub fn dbc(&self) -> Option<&Dbc> {
        self.dbc.as_ref()
    }

    /// The tables recovered from the description.
    #[must_use]
    pub fn tables(&self) -> &TableSet {
        &self.tables
    }

    /// Number of resolved modules.
    #[must_use]
    pub fn module_count(&self) -> usize {
        self.modules.len()
    }

    /// Convert a raw memory count to its physical value.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] or [`CalError::UnknownCharacteristic`].
    /// A non-finite result is *not* an error here — it is reported by
    /// [`CalibrationProject::check_limits`], which is the single gate a
    /// value must pass before it is written.
    pub fn to_physical(&self, module: &str, name: &str, raw: f64) -> Result<f64, CalError> {
        let characteristic = self.characteristic(module, name)?;
        Ok(characteristic.to_physical(raw as u64))
    }

    /// Convert a physical value back to the raw **memory bit pattern** to
    /// deposit.
    ///
    /// For a signed deposit the pattern is two's complement, so a `SBYTE`
    /// characteristic holding −40 °C yields `0xD8` — the bits that actually
    /// go into memory, and the value
    /// [`Characteristic::to_physical`](crate::Characteristic::to_physical)
    /// reads back. This is what makes a read-modify-write cycle bit-exact
    /// rather than merely numerically close.
    ///
    /// The count is bracket-checked against the deposit's datatype before it
    /// is encoded, so a physical value the ECU cannot represent is
    /// [`CalError::OutOfBounds`] rather than a silently wrapped pattern.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`],
    /// [`CalError::NonFiniteValue`],
    /// [`CalError::UnsupportedConversion`] for a non-invertible conversion,
    /// [`CalError::LimitViolation`] when the value is outside the declared
    /// limits, and [`CalError::OutOfBounds`] when the count falls outside
    /// the deposit datatype's range.
    pub fn to_raw(&self, module: &str, name: &str, physical: f64) -> Result<u64, CalError> {
        let characteristic = self.characteristic(module, name)?;
        if !physical.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: characteristic.name.clone(),
                value: physical,
            });
        }
        // Check the declared limits on the way in: a value outside them is
        // not one this ECU should ever be told to hold, whether or not the
        // conversion can represent it.
        self.check_physical_limits(characteristic, physical)?;
        let count = characteristic.conversion.to_raw_i64(physical)?;
        if let Some(datatype) = characteristic.datatype {
            let (lower, upper) = crate::model::raw_range(datatype);
            #[allow(clippy::cast_precision_loss)]
            let as_f64 = count as f64;
            if as_f64 < lower || as_f64 > upper {
                return Err(CalError::OutOfBounds {
                    name: characteristic.name.clone(),
                    value: physical,
                    lower,
                    upper,
                });
            }
            return Ok(two_complement(count, datatype));
        }
        // No datatype: the only representable reading is the count itself,
        // and a negative one cannot be deposited unsigned.
        Ok(count.unsigned_abs())
    }

    /// Validate a raw count against the characteristic's declared limits.
    ///
    /// Limits are declared on the **physical** scale, so the count is
    /// converted first and the comparison is done in engineering units.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`], and
    /// [`CalError::LimitViolation`] carrying the name, the offending value,
    /// and both limits.
    pub fn check_limits(&self, module: &str, name: &str, raw: u64) -> Result<(), CalError> {
        let characteristic = self.characteristic(module, name)?;
        let physical = characteristic.to_physical(raw);
        self.check_physical_limits(characteristic, physical)
    }

    /// Validate a physical value against a characteristic's limits.
    ///
    /// A characteristic whose declared limits are both zero — the ASAP2 way
    /// of saying "unconstrained" — imposes no constraint.
    ///
    /// # Errors
    ///
    /// [`CalError::NonFiniteValue`] for a NaN or infinite value, and
    /// [`CalError::LimitViolation`] when the value falls outside the
    /// declared range.
    pub fn check_physical_limits(
        &self,
        characteristic: &Characteristic,
        physical: f64,
    ) -> Result<(), CalError> {
        if !physical.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: characteristic.name.clone(),
                value: physical,
            });
        }
        let (lower, upper) = (characteristic.lower_limit, characteristic.upper_limit);
        if lower == 0.0 && upper == 0.0 {
            return Ok(());
        }
        if physical < lower || physical > upper {
            return Err(CalError::LimitViolation {
                name: characteristic.name.clone(),
                value: physical,
                lower,
                upper,
            });
        }
        Ok(())
    }

    /// The byte length a characteristic's deposit occupies.
    ///
    /// A scalar's length is its element size. A block's length is that
    /// element size times the axis point count recovered from the
    /// description, defaulting to one point when the A2L omits it.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`], and
    /// [`CalError::UnsupportedConversion`] when the record layout declares
    /// no element datatype, which leaves a block with no length at all.
    pub fn deposit_len(&self, module: &str, name: &str) -> Result<usize, CalError> {
        let characteristic = self.characteristic(module, name)?;
        let element = characteristic.deposit_size().ok_or_else(|| CalError::UnsupportedConversion {
            name: characteristic.name.clone(),
            detail: "the record layout declares no element datatype, so the deposit has no length",
        })?;
        if !characteristic.kind.is_block() {
            return Ok(element);
        }
        let points = self.tables.axis_points(&characteristic.name).unwrap_or(1);
        Ok(element.saturating_mul(points.max(1)))
    }
}

/// The memory bit pattern of `count` in `datatype`'s two's-complement form.
///
/// An unsigned datatype yields the count as-is. A signed one yields the
/// count re-encoded in `size_bytes * 8` bits, so −40 becomes `0xD8` in an
/// `SBYTE` and `0xFFFF_FFD8` in a `SWORD`.
#[must_use]
fn two_complement(count: i64, datatype: a2l_parse::DataType) -> u64 {
    if !crate::model::is_signed_datatype(datatype) {
        return count.unsigned_abs();
    }
    let bits = u32::try_from(datatype.size_bytes() * 8).unwrap_or(64).min(64);
    #[allow(clippy::cast_sign_loss)]
    let bits = bits as u64;
    if bits == 0 {
        return 0;
    }
    #[allow(clippy::cast_possible_wrap)]
    let masked = (count as u64) & ((1u64 << bits) - 1);
    masked
}

/// Build a binding from a DBC signal and the message that carries it.
fn binding_from(name: String, can_id: u32, signal: &DbcSignal) -> SignalBinding {
    SignalBinding::new(
        name,
        can_id,
        signal.start_bit,
        signal.bit_length,
        signal.byte_order,
        signal.scale,
        signal.offset,
    )
}
