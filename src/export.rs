//! Serializing a resolved project back to A2L text.
//!
//! The exporter closes the loop the parser opens: a calibrated set of
//! values has to leave the tool in a form the supplier toolchain — and
//! `a2l_parse` itself — can read back. [`CalibrationProject::to_a2l`]
//! emits `PROJECT` / `MODULE` / `RECORD_LAYOUT` / `COMPU_METHOD` /
//! `COMPU_TAB` / `CHARACTERISTIC` / `MEASUREMENT` blocks that re-parse to an
//! identical [`a2l_parse::A2lProject`].
//!
//! # What the export cannot carry
//!
//! `a2l_parse` (L1) is a model of the *calibration semantics*, not of the
//! file format: it retains block structure, addresses, conversions, and
//! limits, and skips description strings, a measurement's declared display
//! range, `FORMAT` strings, and vendor extensions. The exporter therefore
//! emits neutral placeholders for those — a measurement's
//! resolution/accuracy/limit keywords are written as `0`, and descriptions
//! as a fixed string. Both are syntactically valid A2L and semantically
//! empty: the values that survive the round trip are exactly the ones a
//! calibration session acts on.
//!
//! Because layouts are regenerated rather than echoed, the export names
//! its own (`RL_EXPORT_0`, `RL_EXPORT_1`, …) with the datatype each
//! characteristic actually resolved to — including the
//! [`DEFAULT_DEPOSIT_TYPE`](crate::model::DEFAULT_DEPOSIT_TYPE) the resolver
//! assumed for a layout that declared no `FNC_VALUES`.
//!
//! [`CalibrationProject::to_a2l`]: crate::CalibrationProject::to_a2l

use a2l_parse::{Coeffs, ConversionType, DataType};

use crate::convert::CompuMethod;
use crate::model::{CalibrationProject, Measurement, Module};
use crate::table::TableKind;

/// The ASAP2 version line the exporter emits.
pub const ASAP2_VERSION: &str = "ASAP2_VERSION 1 61";

/// The description string every exported block carries.
const DESCRIPTION: &str = "cal-model export";

/// Render `project` as A2L text.
#[must_use]
pub fn write_project(project: &CalibrationProject) -> String {
    let mut out = String::with_capacity(4096);
    out.push_str(ASAP2_VERSION);
    out.push('\n');
    out.push_str("/begin PROJECT ");
    out.push_str(project.project_name());
    out.push_str(" \"");
    out.push_str(DESCRIPTION);
    out.push_str("\"\n\n");
    for module in project.modules() {
        write_module(&mut out, module);
    }
    out.push_str("/end PROJECT\n");
    out
}

/// Append one module's blocks.
fn write_module(out: &mut String, module: &Module) {
    out.push_str("  /begin MODULE ");
    out.push_str(module.name());
    out.push_str(" \"");
    out.push_str(DESCRIPTION);
    out.push_str("\"\n\n");

    // RECORD_LAYOUTs, one per distinct deposit datatype in use.
    for (index, datatype) in datatypes(module).iter().enumerate() {
        out.push_str("    /begin RECORD_LAYOUT ");
        out.push_str(&layout_name(index));
        out.push_str("\n      FNC_VALUES 1 ");
        out.push_str(datatype.keyword());
        out.push_str(" COLUMN_DIR DIRECT\n    /end RECORD_LAYOUT\n\n");
    }

    // COMPU_METHODs, deduplicated in first-use order.
    let mut written: Vec<&str> = Vec::new();
    for method in methods(module) {
        if written.contains(&method.name()) {
            continue;
        }
        written.push(method.name());
        write_compu_method(out, method);
    }
    for method in methods(module) {
        if let Some(table) = method.table() {
            if !written.contains(&table.name()) {
                written.push(table.name());
                write_table(out, table);
            }
        }
    }

    for characteristic in module.characteristics() {
        write_characteristic(out, characteristic, layout_name(index_of(module, characteristic.datatype())));
    }
    for measurement in module.measurements() {
        write_measurement(out, measurement);
    }
    out.push_str("  /end MODULE\n\n");
}

/// Every distinct deposit datatype a module uses, in first-use order.
fn datatypes(module: &Module) -> Vec<DataType> {
    let mut types: Vec<DataType> = Vec::new();
    for datatype in module
        .characteristics()
        .map(crate::model::Characteristic::datatype)
        .chain(module.measurements().map(Measurement::datatype))
    {
        if !types.contains(&datatype) {
            types.push(datatype);
        }
    }
    types
}

/// The index of `datatype` in [`datatypes`].
fn index_of(module: &Module, datatype: DataType) -> usize {
    datatypes(module)
        .iter()
        .position(|candidate| *candidate == datatype)
        .unwrap_or(0)
}

/// The exported RECORD_LAYOUT name for the `index`-th deposit datatype.
fn layout_name(index: usize) -> String {
    format!("RL_EXPORT_{index}")
}

/// Every COMPU_METHOD a module's characteristics and measurements reference.
fn methods(module: &Module) -> Vec<&CompuMethod> {
    module
        .characteristics()
        .map(crate::model::Characteristic::conversion)
        .chain(module.measurements().map(Measurement::conversion))
        .collect()
}

/// Emit one COMPU_METHOD block.
fn write_compu_method(out: &mut String, method: &CompuMethod) {
    out.push_str("    /begin COMPU_METHOD ");
    out.push_str(method.name());
    out.push_str(" \"");
    out.push_str(DESCRIPTION);
    out.push_str("\" ");
    out.push_str(method.kind().keyword());
    out.push_str("\n      \"%.6g\" \"");
    out.push_str(method.unit());
    out.push('"');
    match method.kind() {
        ConversionType::Linear => {
            if let Coeffs::Linear([a, b]) = method.coefficients() {
                out.push_str(" COEFFS_LINEAR ");
                out.push_str(&number(*a));
                out.push(' ');
                out.push_str(&number(*b));
            }
        }
        ConversionType::RatFunc => {
            if let Coeffs::RatFunc(coefficients) = method.coefficients() {
                out.push_str(" COEFFS");
                for coefficient in coefficients {
                    out.push(' ');
                    out.push_str(&number(*coefficient));
                }
            }
        }
        ConversionType::Table => {
            if let Some(table) = method.table() {
                out.push_str(" COMPU_TAB_REF ");
                out.push_str(table.name());
            }
        }
        ConversionType::Identity => {}
    }
    out.push_str("\n    /end COMPU_METHOD\n\n");
}

/// Emit the COMPU_TAB / COMPU_VTAB a tabular COMPU_METHOD references.
fn write_table(out: &mut String, table: &crate::table::CompuTable) {
    let block = match table.kind() {
        TableKind::Verb => "COMPU_VTAB",
        _ => "COMPU_TAB",
    };
    out.push_str("    /begin ");
    out.push_str(block);
    out.push(' ');
    out.push_str(table.name());
    out.push_str(" \"");
    out.push_str(DESCRIPTION);
    out.push_str("\" ");
    out.push_str(table.kind().keyword());
    out.push('\n');
    if let Some((raw, physical)) = table.default_point() {
        out.push_str("      DEFAULT_VALUE_NUMERIC ");
        out.push_str(&number(raw));
        out.push(' ');
        out.push_str(&number(physical));
    }
    for (raw, physical) in table.points() {
        out.push_str("\n      ");
        out.push_str(&number(*raw));
        out.push(' ');
        out.push_str(&number(*physical));
    }
    out.push_str("\n    /end ");
    out.push_str(block);
    out.push_str("\n\n");
}

/// Emit one CHARACTERISTIC block.
fn write_characteristic(out: &mut String, characteristic: &crate::model::Characteristic, deposit: String) {
    out.push_str("    /begin CHARACTERISTIC ");
    out.push_str(characteristic.name());
    out.push_str(" \"");
    out.push_str(DESCRIPTION);
    out.push_str("\" ");
    out.push_str(characteristic.kind().keyword());
    out.push_str(&format!(" 0x{:X}", characteristic.address()));
    out.push(' ');
    out.push_str(&deposit);
    out.push(' ');
    out.push_str(&number(characteristic.max_diff()));
    out.push(' ');
    out.push_str(characteristic.conversion().name());
    out.push(' ');
    out.push_str(&number(characteristic.lower_limit()));
    out.push(' ');
    out.push_str(&number(characteristic.upper_limit()));
    out.push_str("\n    /end CHARACTERISTIC\n\n");
}

/// Emit one MEASUREMENT block.
fn write_measurement(out: &mut String, measurement: &Measurement) {
    out.push_str("    /begin MEASUREMENT ");
    out.push_str(measurement.name());
    out.push_str(" \"");
    out.push_str(DESCRIPTION);
    out.push_str("\" ");
    out.push_str(measurement.datatype().keyword());
    out.push(' ');
    out.push_str(measurement.conversion().name());
    // RESOLUTION, ACCURACY, LOWER_LIMIT, UPPER_LIMIT are required by the
    // grammar and discarded by the L1 model, so the export writes the
    // neutral values a generator emits for an unset display range.
    out.push_str(" 0 0 0 0");
    if measurement.address() != 0 {
        out.push_str(&format!(" ECU_ADDRESS 0x{:X}", measurement.address()));
    }
    out.push_str("\n    /end MEASUREMENT\n\n");
}

/// Format an f64 the way A2L accepts it.
///
/// A2L tooling is not uniform about exponent notation, so a value whose
/// shortest round-trip form would carry an exponent is expanded to plain
/// decimal. Non-finite values have no A2L spelling and are written as `0`.
fn number(value: f64) -> String {
    if !value.is_finite() {
        return "0".to_owned();
    }
    let shortest = format!("{value:?}");
    if shortest.contains(['e', 'E']) {
        let plain = format!("{value:.30}");
        return if plain.contains('.') {
            plain.trim_end_matches('0').trim_end_matches('.').to_owned()
        } else {
            plain
        };
    }
    shortest
}