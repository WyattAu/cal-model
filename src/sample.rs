//! The embedded demo project: a realistic two-ECU powertrain description
//! and the CAN database that binds it.
//!
//! These are the fixtures the crate's own tests, doctests, and the
//! `calibrate` example run against. They are embedded (`include_str!`) so
//! every downstream user gets a working example without a data directory,
//! and they are deliberately *realistic*: vendor `IF_DATA`, `AXIS_DESCR`,
//! and `COMPU_TAB` blocks the A2L parser must skip, every conversion type,
//! and limits on every adjustable.
//!
//! # `COMPU_TAB` points are registered, not parsed
//!
//! The A2L text carries its `COMPU_TAB`/`COMPU_VTAB` blocks for fidelity,
//! but `a2l_parse` skips them (they are unknown sub-blocks of
//! `COMPU_METHOD`). [`tables`] holds the same points as data;
//! [`sample_project`] registers them so the TABLE conversions resolve.
//!
//! # Element counts are declared
//!
//! `a2l_parse` does not retain the ASAP2 `NUMBER`/`NO_AXIS_PTS` keywords, so
//! [`element_counts`] carries them for
//! [`CalibrationProject::declare_elements`](crate::CalibrationProject::declare_elements)
//! — again, a declaration rather than a guess.

/// A realistic two-ECU A2L description: 15 characteristics (including a
/// `CURVE`, two `MAP`s, a `VAL_BLK`, and an `ASCII` identifier), 5
/// measurements, and COMPU_METHODs covering every conversion type
/// (`LINEAR`, `RAT_FUNC` in both its reducible and quadratic forms, `TABLE`
/// in both interpolated and stepped form, and `IDENTITY`).
pub const SAMPLE_A2L: &str = include_str!("../data/powertrain.a2l");

/// The CAN database that binds [`SAMPLE_A2L`]'s characteristics to message
/// signals — Intel and Motorola layouts, `UWORD`/`UBYTE` widths.
pub const SAMPLE_DBC: &str = include_str!("../data/powertrain.dbc");

/// The `COMPU_TAB` points of [`SAMPLE_A2L`], as `(name, points)` pairs.
#[must_use]
pub fn tables() -> Vec<(&'static str, Vec<(f64, f64)>)> {
    vec![
        (
            "boost_curve_tab",
            vec![
                (0.0, 0.0),
                (1000.0, 45.0),
                (2000.0, 78.0),
                (3000.0, 105.0),
                (4000.0, 128.0),
                (5000.0, 142.0),
                (6000.0, 150.0),
            ],
        ),
        (
            "gear_vtab",
            vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0), (4.0, 4.0)],
        ),
        (
            "gear_pos_tab",
            vec![(0.0, 0.0), (10.0, 1.0), (25.0, 2.0), (45.0, 3.0)],
        ),
        (
            "ratio_vtab",
            vec![(1.0, 0.0), (2.0, 1.0), (3.0, 2.0), (4.0, 3.0)],
        ),
    ]
}

/// The ECU memory the demo project starts from: `(address, bytes)` seeds,
/// one per address the tests and example read before their first write.
///
/// Addresses come from [`SAMPLE_A2L`]; the values are a plausible factory
/// calibration (800 rpm idle, 100 Nm torque cap, no boost, rev cut on).
///
/// The sample layouts declare `FNC_VALUES 1`, so every characteristic's
/// deposit record starts with one reserved byte; each seed carries it.
#[must_use]
pub fn seed_memory() -> Vec<(u32, Vec<u8>)> {
    // The leading 0x00 of each characteristic seed is the reserved byte
    // ahead of `FNC_VALUES POSITION 1`.
    vec![
        // engine: eng_load = 400 counts · 0.1 %/count − 10 % = 30.0 %
        (0x72_01_00, vec![0x00, 0x90, 0x01]),
        // engine: eng_torque_max = (0.5·200 + 100)/2 = 100 Nm
        (0x72_01_04, vec![0x00, 0xC8, 0x00]),
        // engine: idle_target_rpm = 3200 counts · 0.25 rpm = 800 rpm
        (0x72_01_08, vec![0x00, 0x80, 0x0C]),
        // engine: boost_target = 0 counts of the quadratic rational = 0 kPa
        (0x72_01_0C, vec![0x00, 0x00, 0x00]),
        // engine: rev_limit_cut = 1 (enabled)
        (0x72_01_10, vec![0x00, 0x01]),
        // engine: ecu_serial = 'A' (identity raw passthrough)
        (0x70_01_00, vec![0x00, 0x41]),
        // engine: boost_curve — 8 elements (UWORD), all 45.0 kPa
        // (raw 1000 on the boost_tab interpolation)
        (
            0x72_10_00,
            vec![
                0x00, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8,
                0x03, 0xE8, 0x03,
            ],
        ),
        // engine: inj_map — 4 elements (UWORD), all 20.0 %
        // (300 counts · 0.1 − 10)
        (
            0x72_20_00,
            vec![0x00, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01],
        ),
        // engine: knock_table — 3 elements (UWORD), all 78.0 kPa
        // (raw 2000 on the boost_tab interpolation)
        (0x72_30_00, vec![0x00, 0xD0, 0x07, 0xD0, 0x07, 0xD0, 0x07]),
        // engine measurements (ECU_ADDRESS records, no leading byte)
        (0x32_00_01, vec![0x00, 0x04]), // engine_speed: 1024 · 0.25 = 256 rpm
        (0x32_01_00, vec![0x64]),       // coolant_temp: 100 · 0.1 − 10 = 0.0 degC
        (0x32_02_00, vec![0xC8, 0x00]), // boost_actual: 0.0001·40000 + 100 = 104 kPa
        // transmission: primary_ratio = 25000 counts · 1e-4 = 2.5
        (0x73_01_00, vec![0x00, 0xA8, 0x61]),
        // transmission: shift_pressure = 800 / 4 = 200 kPa
        (0x73_01_04, vec![0x00, 0x20, 0x03]),
        // transmission: shift_time_ms = 120 · 0.5 − 20 = 40 ms
        (0x73_01_08, vec![0x00, 0x78, 0x00]),
        // transmission: clutch_force = 0.0005·250000 + 0.2·500 = 225 N
        (0x73_01_0C, vec![0x00, 0xF4, 0x01]),
        // transmission: gear_code = 2
        (0x73_01_10, vec![0x00, 0x02]),
        // transmission: shift_map — 4 elements (UWORD), all 60.0 ms
        // (160 counts · 0.5 − 20)
        (
            0x73_20_00,
            vec![0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00],
        ),
        // transmission measurements
        (0x33_00_01, vec![0x10, 0x27]), // turb_speed: 10000 · 1e-4 = 1.0
        (0x33_01_00, vec![0x64, 0x00]), // out_speed: 100 (identity)
    ]
}

/// The element counts of the sample project's multi-element
/// characteristics — the `NUMBER`/`NO_AXIS_PTS` declarations that
/// `a2l_parse` does not retain.
#[must_use]
pub fn element_counts() -> Vec<(&'static str, &'static str, usize)> {
    vec![
        ("engine", "boost_curve", 8),
        ("engine", "inj_map", 4),
        ("engine", "knock_table", 3),
        ("transmission", "shift_map", 4),
    ]
}

/// Build the demo project: [`SAMPLE_A2L`] parsed, its tables registered, and
/// its element counts declared.
///
/// # Errors
///
/// [`CalError::A2l`](crate::CalError::A2l) or
/// [`CalError::Unsupported`](crate::CalError::Unsupported) only if the
/// embedded fixtures are themselves malformed — a bug this crate's own tests
/// would catch first.
pub fn sample_project() -> Result<crate::CalibrationProject, crate::CalError> {
    let mut project = crate::CalibrationProject::from_a2l(SAMPLE_A2L)?;
    for (name, points) in tables() {
        project.register_table(name, points)?;
    }
    for (module, name, elements) in element_counts() {
        project.declare_elements(module, name, elements)?;
    }
    Ok(project)
}
