//! The calibration project: an A2L description plus an optional DBC
//! database, resolved into addressable, convertible, limit-checked
//! parameters.
//!
//! # What the L1 substrates do and do not hand over
//!
//! [`a2l_parse`] gives the *declaration* — addresses, limits, record
//! layouts, COMPU_METHODs — and [`dbc_parse`] gives the *bus layout*.
//! `cal-model` owns everything between those two and the wire: resolving
//! every reference the description makes, evaluating and inverting
//! conversions, checking limits, coding deposits into bytes, and pairing
//! characteristics with signals by name.
//!
//! # Reference resolution is strict
//!
//! A CHARACTERISTIC whose `COMPU_METHOD` or `RECORD_LAYOUT` is not declared
//! in its module is a [`CalError::Unsupported`], not a silently defaulted
//! one. A conversion that cannot be resolved is worse than no conversion:
//! the ECU would be written with values the calibration engineer never saw.
//! A `TABLE` conversion resolves once its `COMPU_TAB_REF` points are
//! registered ([`register_table`](Self::register_table)).

use crate::conversion::CompuMethod;
use crate::error::CalError;
use crate::model::{CharKind, Characteristic, Measurement, Module};
use crate::signal::SignalBinding;
use a2l_parse::{A2lProject, DataType};
use dbc_parse::Dbc;
use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::path::Path;

/// A calibration project: an A2L description, optionally bound to a DBC
/// database.
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationProject {
    a2l: A2lProject,
    modules: Vec<Module>,
    dbc: Option<Dbc>,
    tables: BTreeMap<String, Vec<(f64, f64)>>,
    bindings: BTreeMap<String, BTreeMap<String, SignalBinding>>,
    measurement_bindings: BTreeMap<String, BTreeMap<String, SignalBinding>>,
    deposit_byte_order: xcp_core::ByteOrder,
}

impl CalibrationProject {
    /// Parse an A2L description and resolve every reference it makes.
    ///
    /// # Errors
    ///
    /// [`CalError::A2l`] for a malformed file,
    /// [`CalError::Unsupported`] when a characteristic or measurement
    /// references a `COMPU_METHOD` or `RECORD_LAYOUT` its module does not
    /// declare.
    pub fn from_a2l(text: &str) -> Result<Self, CalError> {
        Self::from_parsed(A2lProject::parse(text)?)
    }

    /// Read and resolve an A2L description from disk.
    ///
    /// # Errors
    ///
    /// [`CalError::Io`] when the file cannot be read, otherwise as
    /// [`from_a2l`](Self::from_a2l).
    pub fn from_a2l_file(path: &Path) -> Result<Self, CalError> {
        let text = std::fs::read_to_string(path).map_err(|err| CalError::Io {
            path: path.display().to_string(),
            detail: err.to_string(),
        })?;
        Self::from_a2l(&text)
    }

    /// Resolve an already-parsed A2L project.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] as in [`from_a2l`](Self::from_a2l).
    pub fn from_parsed(a2l: A2lProject) -> Result<Self, CalError> {
        let mut modules = Vec::with_capacity(a2l.modules.len());
        for a2l_module in &a2l.modules {
            let mut characteristics = Vec::with_capacity(a2l_module.characteristics.len());
            for raw in &a2l_module.characteristics {
                let layout = a2l_module
                    .record_layout_by_name(&raw.deposit)
                    .ok_or_else(|| CalError::Unsupported {
                        subject: format!("characteristic `{}`", raw.name),
                        reason: format!(
                            "RECORD_LAYOUT `{}` is not declared in module `{}`",
                            raw.deposit, a2l_module.name
                        ),
                    })?;
                let fnc = layout.fnc_values.ok_or_else(|| CalError::Unsupported {
                    subject: format!("characteristic `{}`", raw.name),
                    reason: format!(
                        "RECORD_LAYOUT `{}` declares no FNC_VALUES, so the deposit datatype is unknown",
                        raw.deposit
                    ),
                })?;
                let method = a2l_module
                    .compu_method_by_name(&raw.conversion)
                    .ok_or_else(|| CalError::Unsupported {
                        subject: format!("characteristic `{}`", raw.name),
                        reason: format!(
                            "COMPU_METHOD `{}` is not declared in module `{}`",
                            raw.conversion, a2l_module.name
                        ),
                    })?;
                characteristics.push(Characteristic {
                    name: raw.name.clone(),
                    kind: CharKind::from_char_type(raw.r#type),
                    address: raw.address,
                    deposit: raw.deposit.clone(),
                    deposit_position: fnc.position,
                    datatype: fnc.datatype,
                    elements: 1,
                    conversion: CompuMethod::from_a2l(method),
                    lower_limit: raw.lower_limit,
                    upper_limit: raw.upper_limit,
                    max_diff: raw.max_diff,
                });
            }
            let mut measurements = Vec::with_capacity(a2l_module.measurements.len());
            for raw in &a2l_module.measurements {
                let method = a2l_module
                    .compu_method_by_name(&raw.conversion)
                    .ok_or_else(|| CalError::Unsupported {
                        subject: format!("measurement `{}`", raw.name),
                        reason: format!(
                            "COMPU_METHOD `{}` is not declared in module `{}`",
                            raw.conversion, a2l_module.name
                        ),
                    })?;
                measurements.push(Measurement {
                    name: raw.name.clone(),
                    address: raw.address,
                    conversion: CompuMethod::from_a2l(method),
                    datatype: raw.datatype,
                });
            }
            modules.push(Module {
                name: a2l_module.name.clone(),
                characteristics,
                measurements,
            });
        }
        Ok(Self {
            a2l,
            modules,
            dbc: None,
            tables: BTreeMap::new(),
            bindings: BTreeMap::new(),
            measurement_bindings: BTreeMap::new(),
            deposit_byte_order: xcp_core::ByteOrder::Intel,
        })
    }

    /// The A2L `PROJECT` name.
    #[must_use]
    pub fn project_name(&self) -> &str {
        &self.a2l.project
    }

    /// The underlying parsed description, for callers that need the record
    /// layouts and compu methods this layer resolved away.
    #[must_use]
    pub const fn a2l(&self) -> &A2lProject {
        &self.a2l
    }

    /// The attached DBC database, when one has been attached.
    #[must_use]
    pub const fn dbc(&self) -> Option<&Dbc> {
        self.dbc.as_ref()
    }

    /// Every module, in declaration order.
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
            .find(|m| m.name == name)
            .ok_or_else(|| CalError::UnknownModule(name.to_string()))
    }

    /// Module names, in declaration order.
    pub fn module_names(&self) -> impl Iterator<Item = &str> {
        self.modules.iter().map(|m| m.name.as_str())
    }

    /// The characteristic `name` in module `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] or [`CalError::UnknownCharacteristic`].
    pub fn characteristic(&self, module: &str, name: &str) -> Result<&Characteristic, CalError> {
        self.module(module)?
            .characteristic(name)
            .ok_or_else(|| CalError::UnknownCharacteristic(qualified(module, name)))
    }

    /// The measurement `name` in module `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] or [`CalError::UnknownCharacteristic`].
    pub fn measurement(&self, module: &str, name: &str) -> Result<&Measurement, CalError> {
        self.module(module)?
            .measurement(name)
            .ok_or_else(|| CalError::UnknownCharacteristic(qualified(module, name)))
    }

    /// Declare how many elements a curve/map/value-block deposits.
    ///
    /// `a2l_parse` deliberately does not retain the ASAP2 `NUMBER` keyword
    /// (nor `NO_AXIS_PTS`, which lives in the `AXIS_DESCR` blocks it
    /// skips), so the element count of a multi-element characteristic is
    /// not in the parsed model. Rather than guess — a wrong count reads the
    /// wrong number of bytes out of ECU memory — the count is declared here,
    /// next to the datatype it multiplies.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`] or [`CalError::UnknownCharacteristic`].
    pub fn declare_elements(
        &mut self,
        module: &str,
        name: &str,
        elements: usize,
    ) -> Result<(), CalError> {
        let target = self
            .modules
            .iter_mut()
            .find(|m| m.name == module)
            .ok_or_else(|| CalError::UnknownModule(module.to_string()))?
            .characteristics
            .iter_mut()
            .find(|c| c.name == name)
            .ok_or_else(|| CalError::UnknownCharacteristic(qualified(module, name)))?;
        target.elements = elements;
        Ok(())
    }

    /// Register the `(raw, physical)` points of an A2L `COMPU_TAB`.
    ///
    /// Points must be non-empty and strictly ascending by raw value — the
    /// order the format specifies, and the order [`Curve`](crate::Curve)
    /// and [`CompuMethod::apply`] rely on.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when the point list is empty or not
    /// strictly ascending.
    pub fn register_table(&mut self, name: &str, points: Vec<(f64, f64)>) -> Result<(), CalError> {
        if points.is_empty() {
            return Err(CalError::Unsupported {
                subject: format!("COMPU_TAB `{name}`"),
                reason: "a table needs at least one point".to_string(),
            });
        }
        // Slice destructuring rather than indexing — `windows(2)` always
        // yields exactly two elements — and `partial_cmp` rather than a
        // negated `<`, so a NaN raw is visibly incomparable rather than
        // silently "not ascending".
        if points.windows(2).any(|pair| match pair {
            [first, second] => !matches!(first.0.partial_cmp(&second.0), Some(Ordering::Less)),
            _ => true,
        }) {
            return Err(CalError::Unsupported {
                subject: format!("COMPU_TAB `{name}`"),
                reason: "points must be strictly ascending by raw value".to_string(),
            });
        }
        self.tables.insert(name.to_string(), points);
        Ok(())
    }

    /// The points registered for `name`.
    #[must_use]
    pub fn table(&self, name: &str) -> Option<&[(f64, f64)]> {
        self.tables.get(name).map(Vec::as_slice)
    }

    /// Every registered table, by `COMPU_TAB` name.
    #[must_use]
    pub const fn tables(&self) -> &BTreeMap<String, Vec<(f64, f64)>> {
        &self.tables
    }

    /// The byte order used to code multi-byte deposit elements.
    ///
    /// XCP slaves advertise their byte order in the CONNECT response; the
    /// default is Intel (little-endian), matching `xcp_core`'s default.
    #[must_use]
    pub const fn deposit_byte_order(&self) -> xcp_core::ByteOrder {
        self.deposit_byte_order
    }

    /// Set the byte order used to code multi-byte deposit elements.
    pub fn set_deposit_byte_order(&mut self, order: xcp_core::ByteOrder) {
        self.deposit_byte_order = order;
    }

    /// Bind the project's characteristics to the signals of `dbc`.
    ///
    /// A characteristic binds to the first signal whose name matches it —
    /// exactly first, then ASCII-case-insensitively, because DBC signal
    /// names are conventionally upper-case while A2L names are lower-case.
    /// Bindings replace any previous attachment. Characteristics with no
    /// same-named signal are simply left unbound;
    /// [`require_signal_binding`](Self::require_signal_binding) is where
    /// that becomes a typed error.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when a matched signal's layout cannot be
    /// bit-coded (a width outside `1..=64`, or a start bit beyond the DBC
    /// range). The database is left unattached.
    pub fn attach_dbc(&mut self, dbc: Dbc) -> Result<(), CalError> {
        let mut bindings: BTreeMap<String, BTreeMap<String, SignalBinding>> = BTreeMap::new();
        let mut measurement_bindings: BTreeMap<String, BTreeMap<String, SignalBinding>> =
            BTreeMap::new();
        for module in &self.modules {
            let mut per_module = BTreeMap::new();
            for characteristic in &module.characteristics {
                if let Some((can_id, signal)) = find_signal(&dbc, &characteristic.name) {
                    let binding = validate_binding(&characteristic.name, can_id, signal)?;
                    per_module.insert(characteristic.name.clone(), binding);
                }
            }
            let mut per_measurement = BTreeMap::new();
            for measurement in &module.measurements {
                if let Some((can_id, signal)) = find_signal(&dbc, &measurement.name) {
                    let binding = validate_binding(&measurement.name, can_id, signal)?;
                    per_measurement.insert(measurement.name.clone(), binding);
                }
            }
            bindings.insert(module.name.clone(), per_module);
            measurement_bindings.insert(module.name.clone(), per_measurement);
        }
        self.bindings = bindings;
        self.measurement_bindings = measurement_bindings;
        self.dbc = Some(dbc);
        Ok(())
    }

    /// Parse a DBC document and bind it (the common case).
    ///
    /// # Errors
    ///
    /// [`CalError::Dbc`] for a malformed database, otherwise as
    /// [`attach_dbc`](Self::attach_dbc).
    pub fn attach_dbc_text(&mut self, text: &str) -> Result<(), CalError> {
        self.attach_dbc(Dbc::parse(text)?)
    }

    /// The CAN signal bound to `characteristic` in `module`, when one is.
    #[must_use]
    pub fn signal_for_characteristic(
        &self,
        module: &str,
        char_name: &str,
    ) -> Option<&SignalBinding> {
        self.bindings.get(module)?.get(char_name)
    }

    /// Every binding in `module`, ordered by characteristic name.
    #[must_use]
    pub fn signal_bindings(&self, module: &str) -> Vec<&SignalBinding> {
        self.bindings
            .get(module)
            .map(|m| m.values().collect())
            .unwrap_or_default()
    }

    /// The CAN signal bound to measurement `name` in `module`, when one is.
    ///
    /// Measurements bind exactly like characteristics do — a bench tool
    /// watching `engine_speed` on the bus wants the same layout it gets for
    /// a calibratable.
    #[must_use]
    pub fn signal_for_measurement(
        &self,
        module: &str,
        measurement: &str,
    ) -> Option<&SignalBinding> {
        self.measurement_bindings.get(module)?.get(measurement)
    }

    /// Every measurement binding in `module`, ordered by measurement name.
    #[must_use]
    pub fn measurement_bindings(&self, module: &str) -> Vec<&SignalBinding> {
        self.measurement_bindings
            .get(module)
            .map(|m| m.values().collect())
            .unwrap_or_default()
    }

    /// The CAN signal bound to measurement `name` in `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`], or
    /// [`CalError::NoSignalBinding`].
    pub fn require_measurement_binding(
        &self,
        module: &str,
        measurement: &str,
    ) -> Result<&SignalBinding, CalError> {
        self.measurement(module, measurement)?;
        self.signal_for_measurement(module, measurement)
            .ok_or_else(|| CalError::NoSignalBinding(qualified(module, measurement)))
    }

    /// The CAN signal bound to `characteristic` in `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`],
    /// or [`CalError::NoSignalBinding`] when no DBC is attached or no
    /// signal matches.
    pub fn require_signal_binding(
        &self,
        module: &str,
        char_name: &str,
    ) -> Result<&SignalBinding, CalError> {
        self.characteristic(module, char_name)?;
        self.signal_for_characteristic(module, char_name)
            .ok_or_else(|| CalError::NoSignalBinding(qualified(module, char_name)))
    }

    /// Resolve a characteristic's conversion, filling a `TABLE` method's
    /// points from the registered tables.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when a `TABLE` conversion has no points
    /// and no registered `COMPU_TAB_REF`.
    pub fn resolved_conversion(
        &self,
        characteristic: &Characteristic,
    ) -> Result<CompuMethod, CalError> {
        characteristic.conversion.resolved(&self.tables)
    }

    /// Raw value → physical value, through the A2L `COMPU_METHOD`.
    ///
    /// `raw` is the memory **bit pattern**: for a signed datatype the low
    /// `element_bits` bits are sign-extended, so `-1 °C` in an `SBYTE`
    /// characteristic is `0xFF`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`],
    /// or [`CalError::Unsupported`] for an unresolvable conversion.
    pub fn to_physical(&self, module: &str, name: &str, raw: u64) -> Result<f64, CalError> {
        let characteristic = self.characteristic(module, name)?;
        let conversion = self.resolved_conversion(characteristic)?;
        Ok(conversion.apply(characteristic.raw_value(raw)))
    }

    /// Physical value → raw value, inverting the A2L `COMPU_METHOD`.
    ///
    /// The value is limit-checked first, then inverted against the deposit's
    /// raw bounds, then rounded to the nearest integer the datatype holds.
    /// The result is the memory **bit pattern** (two's complement for
    /// signed datatypes).
    ///
    /// # Errors
    ///
    /// [`CalError::LimitViolation`] when the physical value is outside the
    /// A2L limits, [`CalError::OutOfBounds`] when the deposit cannot
    /// represent it, [`CalError::Unsupported`] for a non-invertible or
    /// unresolvable conversion, plus the lookup errors.
    pub fn to_raw(&self, module: &str, name: &str, physical: f64) -> Result<u64, CalError> {
        let characteristic = self.characteristic(module, name)?;
        self.check_physical_limits(characteristic, physical)?;
        let conversion = self.resolved_conversion(characteristic)?;
        let (value_min, value_max) = characteristic.value_bounds();
        let value = conversion.invert(&characteristic.name, physical, value_min, value_max)?;
        let rounded = value.round();
        let bounded = if rounded < value_min {
            value_min
        } else if rounded > value_max {
            value_max
        } else {
            rounded
        };
        Ok(characteristic.raw_bits(bounded))
    }

    /// Check a physical value against the A2L limits of a characteristic.
    ///
    /// # Errors
    ///
    /// [`CalError::LimitViolation`] carrying the characteristic name, the
    /// offending value, and both limits.
    pub fn check_physical_limits(
        &self,
        characteristic: &Characteristic,
        physical: f64,
    ) -> Result<(), CalError> {
        if characteristic.within_limits(physical) {
            return Ok(());
        }
        Err(CalError::LimitViolation {
            name: characteristic.name.clone(),
            value: physical,
            lower: characteristic.lower_limit,
            upper: characteristic.upper_limit,
        })
    }

    /// Check a raw value against the A2L limits: convert, then check.
    ///
    /// # Errors
    ///
    /// [`CalError::LimitViolation`] carrying the characteristic name, the
    /// **physical** value the raw produced, and both limits.
    pub fn check_limits(&self, module: &str, name: &str, raw: u64) -> Result<(), CalError> {
        let physical = self.to_physical(module, name, raw)?;
        let characteristic = self.characteristic(module, name)?;
        self.check_physical_limits(characteristic, physical)
    }

    /// Locate a characteristic by name across every module, returning the
    /// module name too. First declaration wins.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownCharacteristic`] when no module declares it.
    pub fn locate(&self, name: &str) -> Result<(&str, &Characteristic), CalError> {
        self.modules
            .iter()
            .find_map(|module| {
                module
                    .characteristic(name)
                    .map(|characteristic| (module.name.as_str(), characteristic))
            })
            .ok_or_else(|| CalError::UnknownCharacteristic(name.to_string()))
    }

    /// Decode a deposit payload into one raw bit pattern per element.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when the datatype is IEEE-754 (a float
    /// deposit carries a value, not a bit pattern) or when `data` is shorter
    /// than the declared deposit.
    pub fn read_raw(
        &self,
        characteristic: &Characteristic,
        data: &[u8],
    ) -> Result<Vec<u64>, CalError> {
        require_integer_datatype(characteristic)?;
        let size = characteristic.datatype.size_bytes();
        // The whole deposit record, reserved bytes ahead of the value array
        // included — the same span `write_raw` produces.
        let needed = usize::from(characteristic.deposit_position) + characteristic.elements * size;
        if data.len() < needed {
            return Err(CalError::Unsupported {
                subject: format!("characteristic `{}`", characteristic.name),
                reason: format!(
                    "deposit needs {needed} bytes ({} element(s) · {size} byte(s) at position {}) but the payload carries {}",
                    characteristic.elements,
                    characteristic.deposit_position,
                    data.len()
                ),
            });
        }
        let mut raws = Vec::with_capacity(characteristic.elements);
        for index in 0..characteristic.elements {
            let start = usize::from(characteristic.deposit_position) + index * size;
            let end = start + size;
            let slice = data.get(start..end).ok_or_else(|| CalError::Unsupported {
                subject: format!("characteristic `{}`", characteristic.name),
                reason: format!(
                    "deposit needs {end} bytes but the payload carries {}",
                    data.len()
                ),
            })?;
            raws.push(decode_element(slice, self.deposit_byte_order));
        }
        Ok(raws)
    }

    /// Encode one raw bit pattern per element into a deposit payload.
    ///
    /// The payload is `deposit_position + elements · element width` bytes
    /// long, so the bytes before `FNC_VALUES POSITION` are present and zero
    /// — an ECU that ignores them is unaffected, and one that does not is
    /// told exactly what the layout is.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when the datatype is IEEE-754, when the
    /// element count overflows the deposit size, or when the deposit exceeds
    /// one XCP UPLOAD (255 bytes).
    pub fn write_raw(
        &self,
        characteristic: &Characteristic,
        raws: &[u64],
    ) -> Result<Vec<u8>, CalError> {
        require_integer_datatype(characteristic)?;
        let size = characteristic.datatype.size_bytes();
        let leading = usize::from(characteristic.deposit_position);
        let needed = leading
            .checked_add(
                raws.len()
                    .checked_mul(size)
                    .ok_or_else(|| CalError::Unsupported {
                        subject: format!("characteristic `{}`", characteristic.name),
                        reason: "element count overflows the deposit size".to_string(),
                    })?,
            )
            .ok_or_else(|| CalError::Unsupported {
                subject: format!("characteristic `{}`", characteristic.name),
                reason: "deposit size overflows".to_string(),
            })?;
        if needed > usize::from(u8::MAX) {
            return Err(CalError::Unsupported {
                subject: format!("characteristic `{}`", characteristic.name),
                reason: format!("deposit of {needed} bytes exceeds one XCP UPLOAD"),
            });
        }
        let mut data = vec![0_u8; needed];
        for (index, &raw) in raws.iter().enumerate() {
            let value = characteristic.raw_value(raw);
            let start = leading + index * size;
            if let Some(slot) = data.get_mut(start..start + size) {
                encode_element(slot, value, self.deposit_byte_order);
            }
        }
        Ok(data)
    }

    /// The physical values of one raw value list.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] for an unresolvable conversion.
    pub fn physical_values(
        &self,
        characteristic: &Characteristic,
        raws: &[u64],
    ) -> Result<Vec<f64>, CalError> {
        let conversion = self.resolved_conversion(characteristic)?;
        Ok(raws
            .iter()
            .map(|&raw| conversion.apply(characteristic.raw_value(raw)))
            .collect())
    }
}

/// Reject IEEE-754 deposits: they carry a value, not a bit pattern, and
/// every raw/physical contract in this crate is stated over bit patterns.
fn require_integer_datatype(characteristic: &Characteristic) -> Result<(), CalError> {
    match characteristic.datatype {
        DataType::Float32 | DataType::Float64 => Err(CalError::Unsupported {
            subject: format!("characteristic `{}`", characteristic.name),
            reason: format!(
                "{} deposits are IEEE-754 values, not integer bit patterns",
                characteristic.datatype.keyword()
            ),
        }),
        _ => Ok(()),
    }
}

/// Turn a matched DBC signal into a binding, refusing a layout that cannot
/// be bit-coded.
fn validate_binding(
    name: &str,
    can_id: u32,
    signal: &dbc_parse::Signal,
) -> Result<SignalBinding, CalError> {
    if signal.bit_length == 0 || signal.bit_length > 64 || signal.start_bit > 511 {
        return Err(CalError::Unsupported {
            subject: format!("signal `{}`", signal.name),
            reason: format!(
                "layout start_bit {} length {} is outside the DBC range (start <= 511, length 1..=64)",
                signal.start_bit, signal.bit_length
            ),
        });
    }
    Ok(SignalBinding::from_signal(name, can_id, signal))
}

/// The first signal named `name` in `dbc` (exact, then case-insensitive),
/// with its message id.
fn find_signal<'a>(dbc: &'a Dbc, name: &str) -> Option<(u32, &'a dbc_parse::Signal)> {
    for message in &dbc.messages {
        for signal in &message.signals {
            if signal.name == name {
                return Some((message.id.raw(), signal));
            }
        }
    }
    for message in &dbc.messages {
        for signal in &message.signals {
            if signal.name.eq_ignore_ascii_case(name) {
                return Some((message.id.raw(), signal));
            }
        }
    }
    None
}

/// `module.characteristic`, the qualified form used in error payloads.
fn qualified(module: &str, name: &str) -> String {
    format!("{module}.{name}")
}

/// Assemble `bytes` into a raw value.
fn decode_element(bytes: &[u8], order: xcp_core::ByteOrder) -> u64 {
    let len = bytes.len();
    let mut value = 0_u64;
    for (index, &byte) in bytes.iter().enumerate() {
        let shift = match order {
            xcp_core::ByteOrder::Intel => index * 8,
            xcp_core::ByteOrder::Motorola => (len - 1 - index) * 8,
        };
        value |= u64::from(byte) << shift;
    }
    value
}

/// Scatter a numeric value into `bytes`.
fn encode_element(bytes: &mut [u8], value: f64, order: xcp_core::ByteOrder) {
    #[allow(clippy::cast_possible_wrap)]
    let bits = value as i64 as u64;
    let len = bytes.len();
    for (index, slot) in bytes.iter_mut().enumerate() {
        let shift = match order {
            xcp_core::ByteOrder::Intel => index * 8,
            xcp_core::ByteOrder::Motorola => (len - 1 - index) * 8,
        };
        #[allow(clippy::cast_possible_truncation)]
        {
            *slot = ((bits >> shift) & 0xFF) as u8;
        }
    }
}
