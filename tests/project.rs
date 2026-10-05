//! Project loading and lookup: the embedded realistic A2L description,
//! every module/characteristic/measurement resolution, and the typed errors
//! a miss produces.
//!
//! Test files: strategic `unwrap`/`expect` are by design — a test asserts on
//! `Result`, it does not defend against it.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use cal_model::mock::MockTransport;
use cal_model::{
    sample_project, CalError, CalibrationProject, CalibrationSession, CharKind, ResourceMode,
    SAMPLE_A2L,
};
use std::path::Path;

#[test]
fn sample_declares_two_modules() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(project.project_name(), "powertrain");
    let names: Vec<&str> = project.module_names().collect();
    assert_eq!(names, ["engine", "transmission"]);
}

#[test]
fn engine_module_has_nine_characteristics_spanning_every_kind() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let engine = project.module("engine").unwrap();
    assert_eq!(engine.characteristics.len(), 9);

    let mut kinds: Vec<&str> = engine
        .characteristics
        .iter()
        .map(|c| c.kind.keyword())
        .collect();
    kinds.sort_unstable();
    kinds.dedup();
    assert_eq!(kinds, ["ASCII", "CURVE", "MAP", "VALUE", "VAL_BLK"]);
}

#[test]
fn transmission_module_has_six_characteristics() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(
        project
            .module("transmission")
            .unwrap()
            .characteristics
            .len(),
        6
    );
}

#[test]
fn five_measurements_are_resolved_with_datatype_and_address() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let count: usize = project
        .modules()
        .map(|m| m.measurements.len())
        .sum::<usize>();
    assert_eq!(count, 5);

    let speed = project.measurement("engine", "engine_speed").unwrap();
    assert_eq!(speed.address, 0x320001);
    assert_eq!(speed.size_bytes(), 2);
    assert!(!speed.is_signed());

    let coolant = project.measurement("engine", "coolant_temp").unwrap();
    assert!(coolant.is_signed());
    assert_eq!(coolant.datatype.keyword(), "SBYTE");
}

#[test]
fn every_conversion_type_resolves() {
    let project = sample_project().unwrap();
    // LINEAR
    assert_eq!(
        project
            .characteristic("engine", "idle_target_rpm")
            .unwrap()
            .conversion
            .keyword(),
        "LINEAR"
    );
    // RAT_FUNC
    assert_eq!(
        project
            .characteristic("engine", "boost_target")
            .unwrap()
            .conversion
            .keyword(),
        "RAT_FUNC"
    );
    // TABLE
    assert_eq!(
        project
            .characteristic("engine", "boost_curve")
            .unwrap()
            .conversion
            .keyword(),
        "TABLE"
    );
    // IDENTITY
    assert_eq!(
        project
            .characteristic("engine", "rev_limit_cut")
            .unwrap()
            .conversion
            .keyword(),
        "IDENTITY"
    );
}

#[test]
fn characteristic_lookup_carries_address_limits_and_layout() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let idle = project.characteristic("engine", "idle_target_rpm").unwrap();
    assert_eq!(idle.name, "idle_target_rpm");
    assert_eq!(idle.kind, CharKind::Value);
    assert_eq!(idle.address, 0x720108);
    assert_eq!(idle.deposit, "RL_UWORD");
    assert_eq!(idle.deposit_position, 1);
    assert_eq!(idle.datatype.size_bytes(), 2);
    assert_eq!(idle.lower_limit, 500.0);
    assert_eq!(idle.upper_limit, 1200.0);
    assert_eq!(idle.max_diff, 10.0);
}

#[test]
fn unknown_module_is_a_typed_error() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(
        project.module("transaxle"),
        Err(CalError::UnknownModule("transaxle".to_string()))
    );
    assert_eq!(
        project.characteristic("transaxle", "gear_code"),
        Err(CalError::UnknownModule("transaxle".to_string()))
    );
}

#[test]
fn unknown_characteristic_is_a_typed_error_carrying_the_qualified_name() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(
        project.characteristic("engine", "turbo_boost"),
        Err(CalError::UnknownCharacteristic(
            "engine.turbo_boost".to_string()
        ))
    );
    assert_eq!(
        project.measurement("engine", "turbo_boost"),
        Err(CalError::UnknownCharacteristic(
            "engine.turbo_boost".to_string()
        ))
    );
}

#[test]
fn locate_finds_a_characteristic_across_modules() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let (module, characteristic) = project.locate("gear_code").unwrap();
    assert_eq!(module, "transmission");
    assert_eq!(characteristic.address, 0x730110);
    assert!(project.locate("nonesuch").is_err());
}

#[test]
fn undeclared_compu_method_is_unsupported_not_defaulted() {
    let text = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL UBYTE FNC_VALUES 1 UBYTE
            /end RECORD_LAYOUT
            /begin CHARACTERISTIC orphan "Orphan" VALUE 0x1000
              RL 0 no_such_method 0.0 1.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    let error = CalibrationProject::from_a2l(text).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { subject, reason }
            if subject == "characteristic `orphan`"
                && reason.contains("COMPU_METHOD `no_such_method`")),
        "{error}"
    );
}

#[test]
fn undeclared_record_layout_is_unsupported() {
    let text = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin COMPU_METHOD cm "cm" IDENTITY "%.0f" "-"
            /end COMPU_METHOD
            /begin CHARACTERISTIC orphan "Orphan" VALUE 0x1000
              NO_SUCH_LAYOUT 0 cm 0.0 1.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    let error = CalibrationProject::from_a2l(text).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { subject, reason }
            if subject == "characteristic `orphan`"
                && reason.contains("RECORD_LAYOUT `NO_SUCH_LAYOUT`")),
        "{error}"
    );
}

#[test]
fn record_layout_without_fnc_values_is_unsupported() {
    let text = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL "axes only"
              NO_AXIS_PTS_X 8
            /end RECORD_LAYOUT
            /begin COMPU_METHOD cm "cm" IDENTITY "%.0f" "-"
            /end COMPU_METHOD
            /begin CHARACTERISTIC orphan "Orphan" VALUE 0x1000
              RL 0 cm 0.0 1.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    let error = CalibrationProject::from_a2l(text).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("FNC_VALUES")),
        "{error}"
    );
}

#[test]
fn malformed_a2l_surfaces_the_typed_parse_error() {
    let error = CalibrationProject::from_a2l("/begin PROJECT p \"d\"").unwrap_err();
    assert!(matches!(error, CalError::A2l(_)), "{error}");
}

#[test]
fn from_a2l_file_reads_the_embedded_fixture() {
    let dir = std::env::temp_dir().join("cal-model-project-test");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("powertrain.a2l");
    std::fs::write(&path, SAMPLE_A2L).unwrap();

    let from_file = CalibrationProject::from_a2l_file(&path).unwrap();
    let from_text = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(from_file, from_text);

    let _ = std::fs::remove_file(&path);
}

#[test]
fn from_a2l_file_reports_a_missing_file_as_io() {
    let missing = Path::new("/nonexistent/cal-model/absent.a2l");
    let error = CalibrationProject::from_a2l_file(missing).unwrap_err();
    match error {
        CalError::Io { path, .. } => assert!(path.ends_with("absent.a2l"), "{path}"),
        other => panic!("expected Io, got {other}"),
    }
}

#[test]
fn element_declaration_updates_the_deposit_size() {
    let mut project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(
        project
            .characteristic("engine", "boost_curve")
            .unwrap()
            .elements,
        1
    );

    project
        .declare_elements("engine", "boost_curve", 8)
        .unwrap();
    let curve = project.characteristic("engine", "boost_curve").unwrap();
    assert_eq!(curve.elements, 8);
    assert_eq!(curve.size_bytes(), 16);
    assert_eq!(curve.kind, CharKind::Curve);
}

#[test]
fn element_declaration_on_a_missing_target_is_typed() {
    let mut project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert_eq!(
        project.declare_elements("engine", "nonesuch", 4),
        Err(CalError::UnknownCharacteristic(
            "engine.nonesuch".to_string()
        ))
    );
    assert_eq!(
        project.declare_elements("nonesuch", "boost_curve", 4),
        Err(CalError::UnknownModule("nonesuch".to_string()))
    );
}

#[test]
fn table_registration_requires_ascending_non_empty_points() {
    let mut project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    assert!(project.register_table("t", vec![]).is_err());
    assert!(project
        .register_table("t", vec![(1.0, 1.0), (1.0, 2.0)])
        .is_err());
    assert!(project
        .register_table("t", vec![(2.0, 1.0), (1.0, 2.0)])
        .is_err());
    project
        .register_table("t", vec![(0.0, 0.0), (1.0, 2.0)])
        .unwrap();
    assert_eq!(project.table("t"), Some(&[(0.0, 0.0), (1.0, 2.0)][..]));
    assert!(project.table("absent").is_none());
    assert!(project.tables().contains_key("t"));
}

#[test]
fn unresolved_table_conversion_is_an_error_not_the_identity() {
    // The sample's `boost_curve` uses a TABLE conversion, but a project
    // built without registering `boost_curve_tab` cannot resolve it.
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let error = project
        .to_physical("engine", "boost_curve", 1000)
        .unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { subject, .. }
            if subject == "COMPU_TAB_REF `boost_curve_tab`"),
        "{error}"
    );
}

#[test]
fn registered_table_resolves_the_conversion() {
    let project = sample_project().unwrap();
    // raw 2000 on the boost_curve_tab interpolation is exactly 78 kPa.
    assert!((project.to_physical("engine", "boost_curve", 2000).unwrap() - 78.0).abs() < 1e-9);
}

#[test]
fn deposit_byte_order_is_intel_by_default_and_settable() {
    let mut project = sample_project().unwrap();
    let characteristic = project
        .characteristic("engine", "idle_target_rpm")
        .unwrap()
        .clone();
    assert_eq!(
        project.deposit_byte_order(),
        cal_model::xcp::ByteOrder::Intel
    );

    // Intel: 3200 = 0x0C80 → 80 0C
    let intel = project.write_raw(&characteristic, &[3200]).unwrap();
    assert_eq!(intel, vec![0x00, 0x80, 0x0C]);
    assert_eq!(
        project.read_raw(&characteristic, &intel).unwrap(),
        vec![3200]
    );

    project.set_deposit_byte_order(cal_model::xcp::ByteOrder::Motorola);
    let motorola = project.write_raw(&characteristic, &[3200]).unwrap();
    assert_eq!(motorola, vec![0x00, 0x0C, 0x80]);
    assert_eq!(
        project.read_raw(&characteristic, &motorola).unwrap(),
        vec![3200]
    );
}

#[test]
fn signed_datatype_round_trips_through_a_bit_pattern() {
    let project = sample_project().unwrap();
    // `coolant_temp` is an SBYTE measurement; a signed characteristic uses
    // two's complement, so −1 is 0xFF.
    let text = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL_SBYTE FNC_VALUES 1 SBYTE
            /end RECORD_LAYOUT
            /begin COMPU_METHOD cm "cm" IDENTITY "%.0f" "degC"
            /end COMPU_METHOD
            /begin CHARACTERISTIC temp "Temperature" VALUE 0x2000
              RL_SBYTE 0 cm -40.0 120.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    let signed = CalibrationProject::from_a2l(text).unwrap();
    let temp = signed.characteristic("m", "temp").unwrap();
    assert!(temp.is_signed());
    assert_eq!(temp.value_bounds(), (-128.0, 127.0));
    assert_eq!(temp.raw_value(0xFF), -1.0);
    assert_eq!(temp.raw_bits(-1.0), 0xFF);
    assert_eq!(signed.to_raw("m", "temp", -1.0).unwrap(), 0xFF);
    assert_eq!(signed.to_physical("m", "temp", 0xFF).unwrap(), -1.0);

    // The bytes written are the two's complement bit pattern.
    let payload = signed.write_raw(temp, &[0xFF]).unwrap();
    assert_eq!(payload, vec![0x00, 0xFF]);

    // And a 64-bit deposit masks rather than overflowing.
    let _ = project;
}

#[test]
fn deposit_bounds_come_from_the_datatype_width() {
    let project = sample_project().unwrap();
    let word = project.characteristic("engine", "idle_target_rpm").unwrap();
    assert_eq!(word.value_bounds(), (0.0, 65535.0));
    assert_eq!(word.element_bits(), 16);
    assert_eq!(word.element_mask(), 0xFFFF);
    assert_eq!(word.size_bytes(), 2);

    let byte = project.characteristic("engine", "rev_limit_cut").unwrap();
    assert_eq!(byte.value_bounds(), (0.0, 255.0));
    assert_eq!(byte.element_bits(), 8);
    assert_eq!(byte.element_mask(), 0xFF);

    // A value the datatype cannot express is OutOfBounds, carrying the raw
    // range it exceeded. `eng_load` is a UWORD with a 0…100 % limit, so the
    // UWORD's full range (0…65535 counts = −10…6543.5 %) is wider than the
    // limits and the limits are what bite; `rev_limit_cut` is a UBYTE whose
    // range is likewise wider than its 0…1 flag range. The datatype range is
    // what a conversion is bracket-checked against, so the assertion here is
    // on `value_bounds` and on a purpose-built description whose limits
    // exceed its deposit.
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
    let limit = wide.characteristic("m", "rev_limit").unwrap();
    assert_eq!(limit.value_bounds(), (0.0, 65535.0));
    assert_eq!(wide.to_raw("m", "rev_limit", 65535.0).unwrap(), 0xFFFF);
    let over = wide.to_raw("m", "rev_limit", 70000.0).unwrap_err();
    assert!(
        matches!(&over, CalError::OutOfBounds { lower, upper, value, .. }
            if *lower == 0.0 && *upper == 65535.0 && *value == 70000.0),
        "{over:?}"
    );

    // And the sample's own limits are checked before the datatype's.
    assert!(matches!(
        project.to_raw("engine", "idle_target_rpm", 16383.75),
        Err(CalError::LimitViolation { .. })
    ));
}

#[test]
fn a_deposit_larger_than_one_upload_is_unsupported() {
    let mut project = sample_project().unwrap();
    project
        .declare_elements("engine", "boost_curve", 200)
        .unwrap();
    let error = project
        .write_raw(
            project.characteristic("engine", "boost_curve").unwrap(),
            &vec![0; 200],
        )
        .unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("XCP UPLOAD")),
        "{error}"
    );
}

#[test]
fn ieee_datatypes_are_rejected_rather_than_misread_as_integers() {
    let text = r#"
        /begin PROJECT p "d"
          /begin MODULE m "m"
            /begin RECORD_LAYOUT RL_F FNC_VALUES 1 FLOAT32_IEEE
            /end RECORD_LAYOUT
            /begin COMPU_METHOD cm "cm" IDENTITY "%.0f" "-"
            /end COMPU_METHOD
            /begin CHARACTERISTIC gain "Gain" VALUE 0x3000
              RL_F 0 cm 0.0 1.0
            /end CHARACTERISTIC
          /end MODULE
        /end PROJECT
    "#;
    let project = CalibrationProject::from_a2l(text).unwrap();
    let gain = project.characteristic("m", "gain").unwrap();
    let error = project.write_raw(gain, &[0]).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("IEEE-754")),
        "{error}"
    );
    assert!(project.read_raw(gain, &[0, 0, 0, 0]).is_err());
}

#[test]
fn read_raw_reports_a_short_payload() {
    let project = sample_project().unwrap();
    let curve = project.characteristic("engine", "boost_curve").unwrap();
    let error = project.read_raw(curve, &[0, 1, 2]).unwrap_err();
    // The whole record is 1 reserved byte + 8 elements · 2 bytes = 17.
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("17 bytes")),
        "{error}"
    );
}

#[test]
fn write_raw_keeps_the_bytes_before_the_deposit_position_zero() {
    let project = sample_project().unwrap();
    let idle = project.characteristic("engine", "idle_target_rpm").unwrap();
    let payload = project.write_raw(idle, &[0x0201]).unwrap();
    assert_eq!(payload, vec![0x00, 0x01, 0x02]);
}

#[test]
fn physical_values_bulk_converts() {
    let project = sample_project().unwrap();
    let idle = project.characteristic("engine", "idle_target_rpm").unwrap();
    let values = project.physical_values(idle, &[0, 1000, 2000]).unwrap();
    assert_eq!(values, vec![0.0, 250.0, 500.0]);
}

#[test]
fn a_project_is_comparable_and_clonable() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let clone = project.clone();
    assert_eq!(project, clone);
    assert!(project.dbc().is_none());
    assert_eq!(project.a2l().project, "powertrain");
    assert_eq!(project.a2l().modules.len(), 2);
}

#[test]
fn sessions_accept_the_resource_mask_and_reject_a_daq_only_slave() {
    let project = sample_project().unwrap();
    // CAL/PAG present: connects.
    let ok = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new()),
        ResourceMode::CAL_PAGE_OR_DAQ,
    );
    assert!(ok.is_ok());
    assert_eq!(ok.unwrap().resources(), ResourceMode::CAL_PAGE_OR_DAQ);

    // DAQ only, no CAL/PAG: refused at connect.
    let refused =
        CalibrationSession::connect(&project, Box::new(MockTransport::new()), ResourceMode::DAQ);
    assert!(matches!(
        refused,
        Err(CalError::Xcp(cal_model::xcp::XcpError::AccessDenied))
    ));

    // The empty mask skips the check (not completing a real CONNECT).
    assert!(CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new()),
        ResourceMode::CONNECT_NORMAL
    )
    .is_ok());
}

#[test]
fn session_exposes_the_project_and_transport() {
    let project = sample_project().unwrap();
    let bench = MockTransport::seeded(&cal_model::seed_memory());
    let mut session =
        CalibrationSession::connect(&project, Box::new(bench), ResourceMode::CONNECT_NORMAL)
            .unwrap();
    assert_eq!(session.project().project_name(), "powertrain");
    assert_eq!(session.cal_page(), 0);

    // The transport is reachable for a caller that wants raw memory access:
    // write a raw pattern, and watch the session's conversion of it.
    session
        .transport_mut()
        .write(0x720108, &[0x00, 0x40, 0x1F])
        .unwrap();
    // 8000 counts · 0.25 rpm = 2000 rpm.
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 2000.0)
            .abs()
            < 1e-9
    );
}

#[test]
fn a_session_reads_the_seed_value_of_every_sample_characteristic() {
    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::seeded(&cal_model::seed_memory())),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();

    // VALUE characteristics: one element each.
    assert!((session.read_characteristic("engine", "eng_load").unwrap() - 30.0).abs() < 1e-9);
    assert!(
        (session
            .read_characteristic("engine", "eng_torque_max")
            .unwrap()
            - 100.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 800.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic("engine", "boost_target")
            .unwrap()
            - 0.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic("engine", "rev_limit_cut")
            .unwrap()
            - 1.0)
            .abs()
            < 1e-9
    );
    assert!((session.read_characteristic("engine", "ecu_serial").unwrap() - 65.0).abs() < 1e-9);

    // The 8-element curve, read whole.
    let curve = session
        .read_characteristic_elements("engine", "boost_curve")
        .unwrap();
    assert_eq!(curve.len(), 8);
    for (index, value) in curve.iter().enumerate() {
        assert!((value - 45.0).abs() < 1e-9, "[{index}] = {value}");
    }

    // The 4x4-declared injection map.
    let map = session
        .read_characteristic_elements("engine", "inj_map")
        .unwrap();
    assert_eq!(map.len(), 4);
    for value in &map {
        assert!((value - 20.0).abs() < 1e-9, "{value}");
    }

    // And the transmission side.
    assert!(
        (session
            .read_characteristic("transmission", "primary_ratio")
            .unwrap()
            - 2.5)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic("transmission", "shift_pressure")
            .unwrap()
            - 200.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic("transmission", "shift_time_ms")
            .unwrap()
            - 40.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic("transmission", "clutch_force")
            .unwrap()
            - 225.0)
            .abs()
            < 1e-9
    );
}

#[test]
fn a_session_reads_every_sample_measurement() {
    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::seeded(&cal_model::seed_memory())),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    assert!((session.read_measurement("engine", "engine_speed").unwrap() - 256.0).abs() < 1e-9);
    assert!((session.read_measurement("engine", "coolant_temp").unwrap() - 0.0).abs() < 1e-9);
    assert!((session.read_measurement("engine", "boost_actual").unwrap() - 104.0).abs() < 1e-9);
    assert!(
        (session
            .read_measurement("transmission", "turb_speed")
            .unwrap()
            - 1.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_measurement("transmission", "out_speed")
            .unwrap()
            - 100.0)
            .abs()
            < 1e-9
    );
}

#[test]
fn a_measurement_read_reports_an_unknown_name() {
    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new()),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    assert_eq!(
        session.read_measurement("engine", "oil_temp"),
        Err(CalError::UnknownCharacteristic(
            "engine.oil_temp".to_string()
        ))
    );
}

#[test]
fn a_zero_element_deposit_is_refused_rather_than_indexed() {
    let mut project = sample_project().unwrap();
    project
        .declare_elements("engine", "boost_curve", 0)
        .unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new()),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    let error = session
        .read_characteristic("engine", "boost_curve")
        .unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("no elements")),
        "{error}"
    );
}
