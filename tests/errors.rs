//! `Display` and `Error` coverage for every `CalError` variant, plus the
//! `std::error::Error` contract the crate promises.
//!
//! Test files: `unwrap`/`expect` are by design.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use cal_model::{CalError, CalibrationProject, SAMPLE_A2L};
use std::error::Error as _;

/// Every variant, once, so the coverage assertion below cannot drift.
fn all_variants() -> Vec<(&'static str, CalError)> {
    vec![
        (
            "A2l",
            CalError::A2l(cal_model::a2l::A2lError::MissingProject),
        ),
        ("Xcp", CalError::Xcp(cal_model::xcp::XcpError::PageNotValid)),
        (
            "Dbc",
            CalError::Dbc(cal_model::dbc::DbcError::InvalidId {
                raw: 0x999,
                max: 0x7FF,
            }),
        ),
        (
            "UnknownModule",
            CalError::UnknownModule("gearbox2".to_string()),
        ),
        (
            "UnknownCharacteristic",
            CalError::UnknownCharacteristic("engine.torque_gain".to_string()),
        ),
        (
            "LimitViolation",
            CalError::LimitViolation {
                name: "idle_target_rpm".to_string(),
                value: 4000.0,
                lower: 500.0,
                upper: 1200.0,
            },
        ),
        (
            "OutOfBounds",
            CalError::OutOfBounds {
                name: "rev_limit".to_string(),
                value: 70000.0,
                lower: 0.0,
                upper: 65535.0,
            },
        ),
        (
            "NoSignalBinding",
            CalError::NoSignalBinding("engine.idle_target_rpm".to_string()),
        ),
        (
            "Transport",
            CalError::Transport("slave not answering".to_string()),
        ),
        ("Convergence", CalError::Convergence { iterations: 400 }),
        (
            "Unsupported",
            CalError::Unsupported {
                subject: "COMPU_TAB_REF `gear_vtab`".to_string(),
                reason: "no table registered under that name".to_string(),
            },
        ),
        (
            "NonMonotone",
            CalError::NonMonotone {
                name: "boost_point_3".to_string(),
                value: 42.0,
                previous: 90.0,
            },
        ),
        (
            "Io",
            CalError::Io {
                path: "/tmp/absent.a2l".to_string(),
                detail: "No such file or directory (os error 2)".to_string(),
            },
        ),
    ]
}

#[test]
fn every_variant_renders_a_distinct_non_empty_message() {
    let variants = all_variants();
    let mut seen: Vec<String> = Vec::new();
    for (name, error) in &variants {
        let rendered = error.to_string();
        assert!(!rendered.is_empty(), "{name} rendered empty");
        assert!(
            !seen.contains(&rendered),
            "{name} renders identically to another variant: {rendered}"
        );
        seen.push(rendered);
    }
    assert_eq!(variants.len(), seen.len());
}

#[test]
fn every_variant_is_reachable_from_the_public_api() {
    // Not a re-listing of the same values: each error below is produced by
    // calling the crate, which is what "coverage" means for an error type.
    let mut produced: Vec<CalError> = Vec::new();

    // A2l
    produced.push(CalibrationProject::from_a2l("not an a2l file").unwrap_err());
    // Xcp — a DAQ-only slave cannot be calibrated.
    let project = cal_model::sample_project().unwrap();
    produced.push(
        cal_model::CalibrationSession::connect(
            &project,
            Box::new(cal_model::mock::MockTransport::new()),
            cal_model::ResourceMode::DAQ,
        )
        .unwrap_err(),
    );
    // Dbc
    let mut bad_dbc = cal_model::sample_project().unwrap();
    produced.push(bad_dbc.attach_dbc_text("BO_ nonsense").unwrap_err());
    // UnknownModule / UnknownCharacteristic
    produced.push(project.module("gearbox2").unwrap_err());
    produced.push(project.characteristic("engine", "torque_gain").unwrap_err());
    // LimitViolation
    produced.push(
        project
            .to_raw("engine", "idle_target_rpm", 4000.0)
            .unwrap_err(),
    );
    // OutOfBounds — inside the A2L limits, outside what a UWORD can hold.
    // The sample's idle target caps at 1200 rpm, which is comfortably inside
    // a UWORD, so the reachability check uses its own description.
    let wide = CalibrationProject::from_a2l(
        r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL FNC_VALUES 1 UWORD
            /end RECORD_LAYOUT
            /begin COMPU_METHOD cm "cm" IDENTITY "%.0f" "rpm"
            /end COMPU_METHOD
            /begin CHARACTERISTIC rev_limit "Rev limit" VALUE 0x1000
              RL 0 cm 0.0 70000.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#,
    )
    .unwrap();
    produced.push(wide.to_raw("m", "rev_limit", 70000.0).unwrap_err());
    // NoSignalBinding
    produced.push(
        project
            .require_signal_binding("engine", "idle_target_rpm")
            .unwrap_err(),
    );
    // Transport
    let mut failing = cal_model::CalibrationSession::connect(
        &project,
        Box::new(cal_model::mock::MockTransport::new().fail_reads("slave not answering")),
        cal_model::ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    produced.push(
        failing
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap_err(),
    );
    // Convergence
    produced.push(
        cal_model::optimize_working_point(
            &[cal_model::CalParameter::new("a", 0.0, 0.0, 1.0)],
            &|_: &[f64]| f64::NAN,
        )
        .unwrap_err(),
    );
    // Unsupported — an undeclared COMPU_METHOD.
    let orphan = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL FNC_VALUES 1 UBYTE
            /end RECORD_LAYOUT
            /begin CHARACTERISTIC orphan "Orphan" VALUE 0x1000
              RL 0 no_such_method 0.0 1.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    produced.push(CalibrationProject::from_a2l(orphan).unwrap_err());
    // NonMonotone
    produced.push(
        cal_model::calibrate_curve(&[
            cal_model::CalParameter::new("a", 10.0, -1.0, 1.0),
            cal_model::CalParameter::new("b", 20.0, 9.0, 11.0),
            cal_model::CalParameter::new("c", 15.0, 19.0, 21.0),
        ])
        .unwrap_err(),
    );
    // Io
    produced.push(
        CalibrationProject::from_a2l_file(std::path::Path::new("/nonexistent/absent.a2l"))
            .unwrap_err(),
    );

    // Each produced error's Display matches the hand-written expectation for
    // its variant, which pins both reachability and rendering.
    /// A variant name and a predicate that recognises its rendering.
    type Expectation = (&'static str, fn(&CalError) -> bool);
    let expected: Vec<Expectation> = vec![
        ("A2l", |e| {
            matches!(e, CalError::A2l(_)) && e.to_string().starts_with("A2L description error:")
        }),
        ("Xcp", |e| {
            matches!(e, CalError::Xcp(_)) && e.to_string().starts_with("XCP error:")
        }),
        ("Dbc", |e| {
            matches!(e, CalError::Dbc(_)) && e.to_string().starts_with("DBC error:")
        }),
        ("UnknownModule", |e| {
            e.to_string() == "unknown module `gearbox2`"
        }),
        ("UnknownCharacteristic", |e| {
            e.to_string() == "unknown characteristic `engine.torque_gain`"
        }),
        ("LimitViolation", |e| {
            e.to_string() == "`idle_target_rpm` = 4000 violates its calibration limits [500, 1200]"
        }),
        ("OutOfBounds", |e| {
            e.to_string() == "`rev_limit` = 70000 is outside the representable range [0, 65535]"
        }),
        ("NoSignalBinding", |e| {
            e.to_string() == "no CAN signal binding for `engine.idle_target_rpm`"
        }),
        ("Transport", |e| {
            e.to_string() == "transport failure: slave not answering"
        }),
        ("Convergence", |e| {
            e.to_string()
                .starts_with("optimiser did not converge after ")
                && e.to_string().ends_with(" iterations")
        }),
        ("Unsupported", |e| {
            e.to_string() == "characteristic `orphan`: COMPU_METHOD `no_such_method` is not declared in module `m`"
        }),
        ("NonMonotone", |e| {
            e.to_string() == "non-monotone curve at `c`: 15 follows 20"
        }),
        ("Io", |e| {
            e.to_string()
                .starts_with("cannot read `/nonexistent/absent.a2l`:")
        }),
    ];
    assert_eq!(produced.len(), expected.len(), "one error per variant");

    // Match produced errors to their expected rendering, in any order.
    let mut unmatched = expected.len();
    for (name, check) in &expected {
        let found = produced.iter().any(check);
        assert!(found, "no produced error matched `{name}`");
        if found {
            unmatched -= 1;
        }
    }
    assert_eq!(unmatched, 0);
}

#[test]
fn the_wrapped_substrate_errors_render_their_own_messages() {
    assert_eq!(
        CalError::A2l(cal_model::a2l::A2lError::MissingProject).to_string(),
        "A2L description error: no `/begin PROJECT` block found"
    );
    assert_eq!(
        CalError::Xcp(cal_model::xcp::XcpError::PageNotValid).to_string(),
        "XCP error: ERR 0x26: selected page not available"
    );
    assert_eq!(
        CalError::Dbc(cal_model::dbc::DbcError::BufferTooShort { need: 8, have: 3 }).to_string(),
        "DBC error: buffer of 3 bytes is too short: need 8"
    );
}

#[test]
fn a_cal_error_is_a_std_error_and_kinds_propagate() {
    // The `std::error::Error` contract, including `source()` through the
    // wrapped substrate errors.
    let a2l = CalError::A2l(cal_model::a2l::A2lError::MissingProject);
    assert!(a2l.source().is_some());
    let xcp = CalError::Xcp(cal_model::xcp::XcpError::CmdBusy);
    assert!(xcp.source().is_some());
    let dbc = CalError::Dbc(cal_model::dbc::DbcError::InvalidId { raw: 1, max: 2 });
    assert!(dbc.source().is_some());

    // A domain error has no underlying cause.
    assert!(CalError::UnknownModule("m".to_string()).source().is_none());
    assert!(CalError::Transport("t".to_string()).source().is_none());

    // Usable as a boxed error, which is how callers will hold it.
    let boxed: Box<dyn std::error::Error> = Box::new(a2l);
    assert!(boxed.to_string().contains("A2L description error"));

    // And it is Debug + Clone + PartialEq.
    let cloned = CalError::Transport("t".to_string()).clone();
    assert_eq!(cloned, CalError::Transport("t".to_string()));
    assert!(format!("{cloned:?}").contains("Transport"));
}

#[test]
fn errors_from_real_failures_carry_their_context() {
    let project = cal_model::sample_project().unwrap();

    // The limit error names the characteristic, not the module.
    let error = project
        .to_raw("engine", "idle_target_rpm", 9000.0)
        .unwrap_err();
    assert!(error.to_string().contains("`idle_target_rpm`"), "{error}");

    // An unresolved table names the COMPU_TAB_REF.
    let unresolved = cal_model::CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let error = unresolved
        .to_physical("engine", "boost_curve", 1000)
        .unwrap_err();
    assert!(
        error
            .to_string()
            .contains("COMPU_TAB_REF `boost_curve_tab`"),
        "{error}"
    );

    // A missing module names the module.
    let error = project.module("gearbox").unwrap_err();
    assert!(error.to_string().contains("gearbox"), "{error}");
}
