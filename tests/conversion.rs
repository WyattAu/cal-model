//! COMPU_METHOD conversion: round-trips for every conversion type, monotone
//! table interpolation, identity passthrough, limit checking, and the typed
//! errors an unresolvable or non-invertible conversion produces.
//!
//! Test files: `unwrap`/`expect` and float comparison are by design — a test
//! asserting an exact physical value *is* the specification.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use cal_model::{sample_project, CalError, CalibrationProject, CompuMethod, SAMPLE_A2L};

/// Build a one-characteristic project around `method`, with the given limits.
fn project_with(
    method: &CompuMethod,
    lower: f64,
    upper: f64,
    datatype: &str,
) -> CalibrationProject {
    let coeffs = match method {
        CompuMethod::Linear { slope, intercept } => {
            format!("COEFFS_LINEAR {slope} {intercept}")
        }
        CompuMethod::RatFunc { coeffs } => {
            let [a, b, c, d, e, f] = *coeffs;
            format!("COEFFS {a} {b} {c} {d} {e} {f}")
        }
        // A TABLE declaration carries only the reference; the points are
        // registered on the project afterwards.
        CompuMethod::Table { .. } => "COMPU_TAB_REF t".to_string(),
        CompuMethod::Identity => String::new(),
    };
    let keyword = method.keyword();
    let text = format!(
        "/begin PROJECT p \"d\"\n\
         /begin MODULE m \"m\"\n\
         /begin RECORD_LAYOUT RL FNC_VALUES 1 {datatype}\n\
         /end RECORD_LAYOUT\n\
         /begin COMPU_METHOD cm \"cm\" {keyword} \"%.6f\" \"u\" {coeffs}\n\
         /end COMPU_METHOD\n\
         /begin CHARACTERISTIC value \"Value\" VALUE 0x1000\n\
         RL 0 cm {lower} {upper}\n\
         /end CHARACTERISTIC\n\
         /end MODULE\n\
         /end PROJECT\n"
    );
    let mut project = CalibrationProject::from_a2l(&text).unwrap();
    if let CompuMethod::Table { points, .. } = method {
        project.register_table("t", points.clone()).unwrap();
    }
    project
}

#[test]
fn linear_round_trip_is_exact_within_a_nanounit() {
    // rpm_lin: 0.25 rpm per count. Evaluation is limit-free; inversion is
    // limit-checked, so the round-trip cases are the in-limit raws.
    let project = sample_project().unwrap();
    for raw in [0_u64, 1, 7, 100, 1000, 65535] {
        let physical = project
            .to_physical("engine", "idle_target_rpm", raw)
            .unwrap();
        assert!((physical - raw as f64 * 0.25).abs() < 1e-9, "raw {raw}");
    }
    for raw in [2000_u64, 2400, 3200, 4000, 4800] {
        let physical = project
            .to_physical("engine", "idle_target_rpm", raw)
            .unwrap();
        let back = project
            .to_raw("engine", "idle_target_rpm", physical)
            .unwrap();
        assert_eq!(back, raw, "round-trip through {physical}");
    }
}

#[test]
fn linear_with_an_offset_round_trips() {
    let project = sample_project().unwrap();
    // load_lin: 0.1 %/count − 10 %, limits 0…100 %.
    for raw in [100_u64, 400, 550, 1000, 1100] {
        let physical = project.to_physical("engine", "eng_load", raw).unwrap();
        assert!((physical - (raw as f64 * 0.1 - 10.0)).abs() < 1e-9);
        let back = project.to_raw("engine", "eng_load", physical).unwrap();
        assert_eq!(back, raw);
    }
}

#[test]
fn rat_func_round_trip_recovers_the_raw_count() {
    let project = sample_project().unwrap();
    // press_rat: 0.0001·x² + 0.5·x (a genuinely quadratic form, so this
    // exercises the bisection inverter rather than a closed form), limits
    // 0…250 kPa → in-limit raws below 700.
    // The limits cap boost at 250 kPa, which is raw ≈ 447.
    for raw in [0_u64, 10, 100, 320, 400, 447] {
        let physical = project.to_physical("engine", "boost_target", raw).unwrap();
        assert!(
            (physical - (0.0001 * raw as f64 * raw as f64 + 0.5 * raw as f64)).abs() < 1e-9,
            "raw {raw} → {physical}"
        );
        let back = project.to_raw("engine", "boost_target", physical).unwrap();
        assert_eq!(back, raw, "quadratic round-trip through {physical}");
    }
}

#[test]
fn rat_func_with_a_constant_denominator_round_trips() {
    let project = sample_project().unwrap();
    // trq_lin: (0·x² + 1·x + 0)/(0·x² + 0·x + 4) = x/4, limits 0…100 kPa.
    for raw in [0_u64, 4, 400, 400] {
        let physical = project
            .to_physical("transmission", "shift_pressure", raw)
            .unwrap();
        assert!((physical - raw as f64 / 4.0).abs() < 1e-9);
        assert_eq!(
            project
                .to_raw("transmission", "shift_pressure", physical)
                .unwrap(),
            raw
        );
    }
}

#[test]
fn rat_func_with_an_offset_numerator_round_trips() {
    let project = sample_project().unwrap();
    // torque_lin: (0.5·x + 100)/2, limits 50…700 Nm.
    for raw in [0_u64, 200, 1000, 2600] {
        let physical = project
            .to_physical("engine", "eng_torque_max", raw)
            .unwrap();
        assert!((physical - (0.25 * raw as f64 + 50.0)).abs() < 1e-9);
        assert_eq!(
            project
                .to_raw("engine", "eng_torque_max", physical)
                .unwrap(),
            raw
        );
    }
}

#[test]
fn table_interpolation_is_monotone_increasing() {
    let project = sample_project().unwrap();
    let characteristic = project.characteristic("engine", "boost_curve").unwrap();
    let conversion = project.resolved_conversion(characteristic).unwrap();
    assert!(conversion.points().len() >= 7);

    // Sweep the tabulated domain densely; a TAB_INTP conversion must never
    // step backwards.
    let mut previous = f64::NEG_INFINITY;
    for step in 0..=6000 {
        let raw = step as f64;
        let value = conversion.apply(raw);
        assert!(
            value >= previous,
            "not monotone at raw {raw}: {value} < {previous}"
        );
        previous = value;
    }
    // And the endpoints are the tabulated endpoints.
    assert!((conversion.apply(0.0) - 0.0).abs() < 1e-12);
    assert!((conversion.apply(6000.0) - 150.0).abs() < 1e-12);
}

#[test]
fn table_interpolation_is_monotone_decreasing_for_a_falling_table() {
    let falling = CompuMethod::tab_intp(vec![(0.0, 100.0), (10.0, 50.0), (20.0, 0.0)]);
    let mut previous = f64::INFINITY;
    for step in 0..=20 {
        let value = falling.apply(step as f64);
        assert!(value <= previous, "not monotone at {step}");
        previous = value;
    }
    assert!((falling.apply(5.0) - 75.0).abs() < 1e-12);
}

#[test]
fn table_interpolation_clamps_outside_the_tabulated_domain() {
    let table = CompuMethod::tab_intp(vec![(10.0, 1.0), (20.0, 2.0)]);
    assert!((table.apply(-100.0) - 1.0).abs() < 1e-12);
    assert!((table.apply(1e9) - 2.0).abs() < 1e-12);
}

#[test]
fn table_round_trips_through_its_own_points() {
    let project = sample_project().unwrap();
    for &(raw, physical) in project.table("boost_curve_tab").unwrap() {
        let back = project.to_raw("engine", "boost_curve", physical).unwrap();
        // Tabulated points land on integer raws, so the round-trip is exact.
        assert_eq!(back as f64, raw, "{physical} → {back}");
    }
}

#[test]
fn table_interpolation_inverts_between_points() {
    let project = sample_project().unwrap();
    // 61.5 kPa is halfway between 45 and 78 on the 1000…2000 segment, whose
    // raw midpoint is 1500.
    let raw = project.to_raw("engine", "boost_curve", 61.5).unwrap();
    assert_eq!(raw, 1500);
}

#[test]
fn stepped_table_lookup_holds_each_segment_constant() {
    let steps = CompuMethod::tab_no_intp(vec![(0.0, 10.0), (10.0, 20.0), (20.0, 30.0)]);
    assert!((steps.apply(0.0) - 10.0).abs() < 1e-12);
    assert!((steps.apply(9.99) - 10.0).abs() < 1e-12);
    assert!((steps.apply(10.0) - 20.0).abs() < 1e-12);
    assert!((steps.apply(19.99) - 20.0).abs() < 1e-12);
    assert!((steps.apply(25.0) - 30.0).abs() < 1e-12);
    assert!((steps.apply(1e6) - 30.0).abs() < 1e-12);

    // And the inverse lands on a tabulated raw, never an interpolated one:
    // 20.4 is inside the step that starts at raw 10.
    assert!((steps.invert("s", 20.4, 0.0, 100.0).unwrap() - 10.0).abs() < 1e-12);
    assert!((steps.invert("s", 10.0, 0.0, 100.0).unwrap() - 0.0).abs() < 1e-12);
    assert!((steps.invert("s", 30.0, 0.0, 100.0).unwrap() - 20.0).abs() < 1e-12);
}

#[test]
fn a_stepped_table_is_monotone_when_its_points_are() {
    let steps = CompuMethod::tab_no_intp(vec![(0.0, 0.0), (4.0, 1.0), (8.0, 2.0)]);
    let mut previous = f64::NEG_INFINITY;
    for step in 0..=8 {
        let value = steps.apply(step as f64);
        assert!(value >= previous, "not monotone at {step}");
        previous = value;
    }
}

#[test]
fn identity_conversion_is_a_passthrough() {
    let project = sample_project().unwrap();
    for raw in [0_u64, 1, 42, 255] {
        let physical = project.to_physical("engine", "rev_limit_cut", raw).unwrap();
        assert!(
            (physical - raw as f64).abs() < 1e-12,
            "raw {raw} → {physical}"
        );
    }
    assert!(project
        .characteristic("engine", "rev_limit_cut")
        .unwrap()
        .conversion
        .is_identity());

    // Inversion honours the declared limits (0…1, a boolean flag): 0 and 1
    // round-trip, 42 does not.
    assert_eq!(project.to_raw("engine", "rev_limit_cut", 0.0).unwrap(), 0);
    assert_eq!(project.to_raw("engine", "rev_limit_cut", 1.0).unwrap(), 1);
    assert!(matches!(
        project.to_raw("engine", "rev_limit_cut", 42.0),
        Err(CalError::LimitViolation { .. })
    ));
}

#[test]
fn identity_conversion_over_a_full_word_is_the_identity() {
    let project = project_with(&CompuMethod::Identity, 0.0, 70000.0, "UWORD");
    for raw in [0_u64, 1, 12345, 65535] {
        let physical = project.to_physical("m", "value", raw).unwrap();
        assert!((physical - raw as f64).abs() < 1e-12);
        assert_eq!(project.to_raw("m", "value", physical).unwrap(), raw);
    }
}

#[test]
fn limits_accept_an_in_range_raw() {
    let project = sample_project().unwrap();
    // 500…1200 rpm.
    assert!(project
        .check_limits("engine", "idle_target_rpm", 2000)
        .is_ok());
    assert!(project
        .check_limits("engine", "idle_target_rpm", 3200)
        .is_ok());
    assert!(project
        .check_limits("engine", "idle_target_rpm", 4800)
        .is_ok());
}

#[test]
fn limits_reject_an_out_of_range_raw_with_name_value_and_bounds() {
    let project = sample_project().unwrap();
    let error = project
        .check_limits("engine", "idle_target_rpm", 0)
        .unwrap_err();
    assert_eq!(
        error,
        CalError::LimitViolation {
            name: "idle_target_rpm".to_string(),
            value: 0.0,
            lower: 500.0,
            upper: 1200.0,
        }
    );
    // The high side too, and the error carries the physical value.
    let high = project
        .check_limits("engine", "idle_target_rpm", 4801)
        .unwrap_err();
    match high {
        CalError::LimitViolation { value, upper, .. } => {
            assert!(value > upper, "{value} vs {upper}");
        }
        other => panic!("expected LimitViolation, got {other}"),
    }
}

#[test]
fn to_raw_refuses_an_out_of_limit_value() {
    let project = sample_project().unwrap();
    let error = project
        .to_raw("engine", "idle_target_rpm", 4000.0)
        .unwrap_err();
    assert!(
        matches!(&error, CalError::LimitViolation { value, lower, upper, .. }
            if *value == 4000.0 && *lower == 500.0 && *upper == 1200.0),
        "{error}"
    );
    // Exactly on the limit is fine.
    assert!(project.to_raw("engine", "idle_target_rpm", 1200.0).is_ok());
    assert!(project.to_raw("engine", "idle_target_rpm", 500.0).is_ok());
}

#[test]
fn a_degenerate_limit_range_imposes_no_constraint() {
    // Generators emit `0.0 0.0` for ASCII identifiers; that must not read as
    // "the value must be exactly zero".
    let project = project_with(&CompuMethod::Identity, 0.0, 0.0, "UBYTE");
    let characteristic = project.characteristic("m", "value").unwrap();
    assert!(!characteristic.has_limits());
    assert!(characteristic.within_limits(0.0));
    assert!(characteristic.within_limits(65.0));
    assert!(characteristic.within_limits(255.0));
    assert!(project.check_limits("m", "value", 255).is_ok());
    // But a non-finite value is never in limits.
    assert!(!characteristic.within_limits(f64::NAN));
}

#[test]
fn a_non_finite_physical_value_is_a_limit_violation() {
    let project = sample_project().unwrap();
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let error = project
            .to_raw("engine", "idle_target_rpm", bad)
            .unwrap_err();
        assert!(
            matches!(&error, CalError::LimitViolation { value, .. } if value.is_nan() || value.is_infinite()),
            "{bad} gave {error}"
        );
    }
}

#[test]
fn a_zero_slope_linear_conversion_is_not_invertible() {
    let project = project_with(&CompuMethod::linear(0.0, 42.0), -1e9, 1e9, "UWORD");
    let error = project.to_raw("m", "value", 42.0).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { subject, reason }
            if subject == "`value`" && reason.contains("not invertible")),
        "{error}"
    );
    // Evaluation still works — it is only the inverse that is impossible.
    assert!((project.to_physical("m", "value", 7).unwrap() - 42.0).abs() < 1e-12);
}

#[test]
fn an_empty_table_evaluates_as_the_identity_but_refuses_to_invert() {
    let empty = CompuMethod::Table {
        tab_ref: Some("absent".to_string()),
        points: Vec::new(),
        interpolate: true,
    };
    assert!((empty.apply(42.0) - 42.0).abs() < 1e-12);
    let error = empty.invert("value", 42.0, 0.0, 255.0).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("no points")),
        "{error}"
    );
}

#[test]
fn a_table_with_no_tab_ref_cannot_be_resolved() {
    let anonymous = CompuMethod::tab_intp(Vec::new());
    let error = anonymous
        .resolved(&std::collections::BTreeMap::new())
        .unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("COMPU_TAB_REF")),
        "{error}"
    );
}

#[test]
fn resolution_fills_points_from_the_registry() {
    let method = CompuMethod::Table {
        tab_ref: Some("t".to_string()),
        points: Vec::new(),
        interpolate: true,
    };
    let mut tables = std::collections::BTreeMap::new();
    assert!(method.resolved(&tables).is_err());

    // A registered-but-empty table is still unresolvable.
    tables.insert("t".to_string(), Vec::new());
    let error = method.resolved(&tables).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("no points")),
        "{error}"
    );

    tables.insert("t".to_string(), vec![(0.0, 0.0), (1.0, 2.0)]);
    let resolved = method.resolved(&tables).unwrap();
    assert_eq!(resolved.points(), &[(0.0, 0.0), (1.0, 2.0)]);
    assert_eq!(resolved.tab_ref(), Some("t"));
    assert!((resolved.apply(0.5) - 1.0).abs() < 1e-12);
}

#[test]
fn inline_points_win_over_the_registry() {
    let method = CompuMethod::tab_intp(vec![(0.0, 5.0), (10.0, 6.0)]);
    let mut tables = std::collections::BTreeMap::new();
    tables.insert("other".to_string(), vec![(0.0, 100.0)]);
    let resolved = method.resolved(&tables).unwrap();
    assert!((resolved.apply(0.0) - 5.0).abs() < 1e-12);
}

#[test]
fn a_non_table_conversion_resolves_to_itself() {
    for method in [
        CompuMethod::Identity,
        CompuMethod::linear(2.0, 1.0),
        CompuMethod::rat_func([0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
    ] {
        let resolved = method.resolved(&std::collections::BTreeMap::new()).unwrap();
        assert_eq!(resolved, method);
    }
}

#[test]
fn a_rational_conversion_with_a_zero_denominator_is_unsupported_not_infinite() {
    let project = project_with(
        &CompuMethod::rat_func([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
        -1e9,
        1e9,
        "UWORD",
    );
    let error = project.to_raw("m", "value", 100.0).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("not finite")),
        "{error}"
    );
    // Evaluation is total and yields a non-finite value, which the limit
    // check then rejects.
    assert!(!project.to_physical("m", "value", 5).unwrap().is_finite());
}

#[test]
fn a_descending_conversion_inverts_by_bisection() {
    let project = project_with(
        &CompuMethod::rat_func([0.0, -1.0, 1000.0, 0.0, 0.0, 1.0]),
        0.0,
        1000.0,
        "UWORD",
    );
    for raw in [0_u64, 100, 500, 999] {
        let physical = project.to_physical("m", "value", raw).unwrap();
        assert!((physical - (1000.0 - raw as f64)).abs() < 1e-9);
        assert_eq!(project.to_raw("m", "value", physical).unwrap(), raw);
    }
}

#[test]
fn set_points_replaces_a_table_point_list() {
    let mut method = CompuMethod::tab_intp(vec![(0.0, 0.0)]);
    method.set_points(vec![(0.0, 1.0), (10.0, 2.0)]);
    assert_eq!(method.points(), &[(0.0, 1.0), (10.0, 2.0)]);
    // A no-op on a non-table.
    let mut linear = CompuMethod::linear(1.0, 0.0);
    linear.set_points(vec![(0.0, 1.0)]);
    assert!(linear.points().is_empty());
}

#[test]
fn constructors_and_keywords_agree() {
    assert!(CompuMethod::identity().is_identity());
    assert_eq!(CompuMethod::identity().keyword(), "IDENTITY");
    assert_eq!(CompuMethod::linear(1.0, 0.0).keyword(), "LINEAR");
    assert_eq!(CompuMethod::rat_func([0.0; 6]).keyword(), "RAT_FUNC");
    assert_eq!(CompuMethod::tab_intp(vec![(0.0, 0.0)]).keyword(), "TABLE");
    assert_eq!(
        CompuMethod::tab_no_intp(vec![(0.0, 0.0)]).keyword(),
        "TABLE"
    );
    assert!(CompuMethod::linear(1.0, 0.0).tab_ref().is_none());
    assert!(CompuMethod::linear(1.0, 0.0).points().is_empty());
}

#[test]
fn a_linear_method_without_coefficients_does_not_invent_a_scale() {
    // The format requires COEFFS_LINEAR, but a sloppy generator omits it.
    let text = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL FNC_VALUES 1 UBYTE
            /end RECORD_LAYOUT
            /begin COMPU_METHOD cm "cm" LINEAR "%.0f" "-"
            /end COMPU_METHOD
            /begin CHARACTERISTIC value "Value" VALUE 0x1000
              RL 0 cm 0.0 255.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    let project = CalibrationProject::from_a2l(text).unwrap();
    let conversion = &project.characteristic("m", "value").unwrap().conversion;
    assert!(conversion.is_identity());
    assert!((project.to_physical("m", "value", 42).unwrap() - 42.0).abs() < 1e-12);
}

#[test]
fn a_table_conversion_outside_its_ordinate_range_is_out_of_bounds() {
    let project = project_with(
        &CompuMethod::tab_intp(vec![(0.0, 0.0), (10.0, 10.0)]),
        -1e6,
        1e6,
        "UWORD",
    );
    let error = project.to_raw("m", "value", 50.0).unwrap_err();
    assert!(
        matches!(&error, CalError::OutOfBounds { lower, upper, .. } if *lower == 0.0 && *upper == 10.0),
        "{error}"
    );
}

#[test]
fn every_sample_characteristic_converts_its_own_seed() {
    // A blanket check that the resolution of the whole sample is coherent:
    // every characteristic reads its seed, and that value is inside its own
    // declared limits.
    let project = sample_project().unwrap();
    for module in project.modules() {
        let module_name = module.name.clone();
        for characteristic in &module.characteristics {
            if characteristic.conversion.keyword() == "TABLE" {
                continue; // checked separately; the tabulation is not the identity
            }
            // Round-trip through a physical value inside the declared limits.
            let midpoint = (characteristic.lower_limit + characteristic.upper_limit) / 2.0;
            if !midpoint.is_finite() || !characteristic.has_limits() {
                continue;
            }
            let raw = project
                .to_raw(&module_name, &characteristic.name, midpoint)
                .unwrap_or_else(|error| panic!("{module_name}.{}: {error}", characteristic.name));
            let physical = project
                .to_physical(&module_name, &characteristic.name, raw)
                .unwrap();
            assert!(
                characteristic.within_limits(physical),
                "{module_name}.{} raw {raw} → {physical} outside [{}, {}]",
                characteristic.name,
                characteristic.lower_limit,
                characteristic.upper_limit
            );
            // And the raw is inside the deposit's own datatype range.
            let (value_min, value_max) = characteristic.value_bounds();
            assert!(characteristic.raw_value(raw) >= value_min);
            assert!(characteristic.raw_value(raw) <= value_max);
        }
    }
    let _ = SAMPLE_A2L;
}
