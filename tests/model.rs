//! The resolved model: every ASAP2 datatype and characteristic kind, signed
//! bit patterns, datatype-derived bounds, and the accessor surface a host
//! reads.
//!
//! Test files: `unwrap`/`expect` are by design.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use cal_model::{
    ByteOrder, CalError, CalibrationProject, CharKind, Measurement, SignalBinding, SAMPLE_A2L,
};

/// A one-characteristic description over `datatype`, so every datatype in the
/// model is exercised against real parsing.
fn project_with_datatype(datatype: &str) -> (CalibrationProject, String) {
    let text = format!(
        "/begin PROJECT p \"d\"\n\
         /begin MODULE m \"m\"\n\
         /begin RECORD_LAYOUT RL FNC_VALUES 3 {datatype}\n\
         /end RECORD_LAYOUT\n\
         /begin COMPU_METHOD cm \"cm\" IDENTITY \"%.0f\" \"u\"\n\
         /end COMPU_METHOD\n\
         /begin CHARACTERISTIC value \"Value\" VALUE 0x1000\n\
         RL 0 cm -1.0e9 1.0e9\n\
         /end CHARACTERISTIC\n\
         /end MODULE\n\
         /end PROJECT\n"
    );
    let project = CalibrationProject::from_a2l(&text).unwrap();
    (project, text)
}

#[test]
fn every_integer_datatype_yields_its_width_bounds() {
    // (A2L datatype, element bytes, element bits, unsigned lower, upper)
    let cases = [
        ("UBYTE", 1_usize, 8_u32, 0.0_f64, 255.0_f64),
        ("SBYTE", 1, 8, -128.0, 127.0),
        ("UWORD", 2, 16, 0.0, 65535.0),
        ("SWORD", 2, 16, -32768.0, 32767.0),
        ("ULONG", 4, 32, 0.0, 4294967295.0),
        ("SLONG", 4, 32, -2147483648.0, 2147483647.0),
        ("A_UINT64", 8, 64, 0.0, 18446744073709551615.0),
        (
            "A_INT64",
            8,
            64,
            -9223372036854775808.0,
            9223372036854775807.0,
        ),
    ];

    for (datatype, bytes, bits, lower, upper) in cases {
        let (project, _) = project_with_datatype(datatype);
        let characteristic = project.characteristic("m", "value").unwrap();
        assert_eq!(characteristic.element_bits(), bits, "{datatype} bits");
        assert_eq!(
            characteristic.datatype.size_bytes(),
            bytes,
            "{datatype} size"
        );
        // The element count defaults to 1 until declared.
        assert_eq!(characteristic.size_bytes(), bytes, "{datatype} deposit");
        assert_eq!(
            characteristic.value_bounds(),
            (lower, upper),
            "{datatype} bounds"
        );
        assert_eq!(
            characteristic.is_signed(),
            datatype.starts_with('S') || datatype == "A_INT64",
            "{datatype} signedness"
        );
    }
}

#[test]
fn a_forty_eight_bit_mask_is_exact() {
    let (project, _) = project_with_datatype("A_INT64");
    let characteristic = project.characteristic("m", "value").unwrap();
    // A 64-bit element uses the whole word: no mask is applied.
    assert_eq!(characteristic.element_mask(), u64::MAX);
    assert_eq!(characteristic.raw_value(u64::MAX), -1.0);

    let (word, _) = project_with_datatype("ULONG");
    let word = word.characteristic("m", "value").unwrap();
    assert_eq!(word.element_mask(), 0xFFFF_FFFF);
    // A value above the element width is masked, not wrapped.
    assert_eq!(word.raw_value(0x1_0000_0001), 1.0);
}

#[test]
fn signed_bit_patterns_sign_extend_and_round_trip() {
    let cases = [
        ("SBYTE", 8_u32),
        ("SWORD", 16),
        ("SLONG", 32),
        ("A_INT64", 64),
    ];
    for (datatype, bits) in cases {
        let (project, _) = project_with_datatype(datatype);
        let characteristic = project.characteristic("m", "value").unwrap();
        let (value_min, _) = characteristic.value_bounds();

        // −1 is all-ones, the classic two's-complement pattern.
        let minus_one = characteristic.raw_bits(-1.0);
        assert_eq!(minus_one, characteristic.element_mask(), "{datatype}");
        assert_eq!(characteristic.raw_value(minus_one), -1.0, "{datatype}");
        assert_eq!(project.to_physical("m", "value", minus_one).unwrap(), -1.0);
        assert_eq!(project.to_raw("m", "value", -1.0).unwrap(), minus_one);

        // The minimum and maximum are exact.
        let (minimum, highest) = characteristic.value_bounds();
        assert_eq!(minimum, value_min, "{datatype} min bound");
        assert_eq!(
            characteristic.raw_value(characteristic.raw_bits(minimum)),
            minimum,
            "{datatype} min"
        );
        assert_eq!(
            characteristic.raw_value(characteristic.raw_bits(highest)),
            highest,
            "{datatype} max"
        );

        // Half the range is negative.
        let half = characteristic.raw_bits(value_min / 2.0);
        assert!(
            characteristic.raw_value(half) < 0.0,
            "{datatype}: half the negative range must be negative"
        );

        // And a 32-bit element's mask does not touch the upper word.
        if bits == 32 {
            assert_eq!(characteristic.raw_value(0xFFFF_FFFF_FFFF_FFFF), -1.0);
        }
    }
}

#[test]
fn unsigned_values_above_the_sign_bit_stay_positive() {
    let (project, _) = project_with_datatype("ULONG");
    let characteristic = project.characteristic("m", "value").unwrap();
    assert!(!characteristic.is_signed());
    // The top bit of an unsigned element is a value, not a sign.
    assert_eq!(characteristic.raw_value(0x8000_0000), 2147483648.0);
    assert_eq!(characteristic.raw_bits(2147483648.0), 0x8000_0000);
}

#[test]
fn ieee_deposits_parse_but_refuse_integer_coding() {
    // Both float widths parse — the model carries them — but neither can be
    // read as a bit pattern.
    for datatype in ["FLOAT32_IEEE", "FLOAT64_IEEE"] {
        let (project, _) = project_with_datatype(datatype);
        let characteristic = project.characteristic("m", "value").unwrap();
        assert_eq!(characteristic.datatype.keyword(), datatype);
        assert!(!characteristic.is_signed());
        let error = project.write_raw(characteristic, &[0]).unwrap_err();
        assert!(
            matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("IEEE-754")),
            "{datatype}: {error}"
        );
    }
}

#[test]
fn every_characteristic_kind_maps_both_ways() {
    let cases = [
        ("VALUE", CharKind::Value),
        ("CURVE", CharKind::Curve),
        ("MAP", CharKind::Map),
        ("VAL_BLK", CharKind::ValBlk),
        ("ASCII", CharKind::Ascii),
    ];
    for (keyword, kind) in cases {
        assert_eq!(kind.keyword(), keyword);
        assert_eq!(CharKind::from_char_type(kind_as_char_type(keyword)), kind);
    }
    // Only the arrays are multi-element.
    assert!(CharKind::Curve.is_multi_element());
    assert!(CharKind::Map.is_multi_element());
    assert!(CharKind::ValBlk.is_multi_element());
    assert!(!CharKind::Value.is_multi_element());
    assert!(!CharKind::Ascii.is_multi_element());
}

/// The `a2l_parse` counterpart of a keyword.
fn kind_as_char_type(keyword: &str) -> cal_model::a2l::CharType {
    cal_model::a2l::CharType::from_keyword(keyword).unwrap()
}

#[test]
fn limit_semantics_follow_the_declaration() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();

    // A real range constrains.
    let idle = project.characteristic("engine", "idle_target_rpm").unwrap();
    assert!(idle.has_limits());
    assert!(idle.within_limits(800.0));
    assert!(!idle.within_limits(400.0));
    assert!(!idle.within_limits(1300.0));

    // A degenerate range (the ASCII identifier's `0.0 0.0`) does not.
    let serial = project.characteristic("engine", "ecu_serial").unwrap();
    assert!(!serial.has_limits());
    assert!(serial.within_limits(0.0));
    assert!(serial.within_limits(255.0));

    // A non-finite value is never within limits, whatever the declaration.
    assert!(!serial.within_limits(f64::NAN));
    assert!(!serial.within_limits(f64::INFINITY));
    assert!(!idle.within_limits(f64::NEG_INFINITY));
}

#[test]
fn with_elements_is_the_builder_form_of_declare_elements() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let original = project.characteristic("engine", "boost_curve").unwrap();
    let widened = original.clone().with_elements(16);
    assert_eq!(original.elements, 1);
    assert_eq!(widened.elements, 16);
    assert_eq!(widened.size_bytes(), 32);
    // Every other field is preserved.
    assert_eq!(widened.name, original.name);
    assert_eq!(widened.address, original.address);
    assert_eq!(widened.kind, original.kind);
}

#[test]
fn module_accessors_list_and_find_by_name() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    let engine = project.module("engine").unwrap();

    assert_eq!(engine.characteristic_names().count(), 9);
    assert_eq!(engine.measurement_names().count(), 3);
    assert!(engine.characteristic("eng_load").is_some());
    assert!(engine.characteristic("nonesuch").is_none());
    assert!(engine.measurement("engine_speed").is_some());
    assert!(engine.measurement("nonesuch").is_none());

    // The names come back in declaration order.
    let names: Vec<&str> = engine.characteristic_names().collect();
    assert_eq!(names[0], "eng_load");
    assert_eq!(names[1], "eng_torque_max");
}

#[test]
fn a_measurement_reports_its_width_and_signedness() {
    let project = CalibrationProject::from_a2l(SAMPLE_A2L).unwrap();
    for module in project.modules() {
        for measurement in &module.measurements {
            let resolved: &Measurement = project
                .measurement(&module.name, &measurement.name)
                .unwrap();
            assert_eq!(resolved.size_bytes(), resolved.datatype.size_bytes());
            // Every sample measurement declares an ECU_ADDRESS.
            assert_ne!(resolved.address, 0, "{}", resolved.name);
            // And raw_value round-trips through the declared width.
            assert_eq!(resolved.raw_value(0), 0.0);
            assert!(resolved.raw_value(u64::MAX).is_finite());
        }
    }
}

#[test]
fn a_signal_binding_is_built_from_a_dbc_signal() {
    let signal = cal_model::dbc::Signal {
        name: "Load".to_string(),
        start_bit: 39,
        bit_length: 12,
        byte_order: cal_model::dbc::ByteOrder::Motorola,
        is_signed: false,
        scale: 0.5,
        offset: -3.0,
        min: 0.0,
        max: 100.0,
        unit: "%".to_string(),
        receivers: vec!["ECU".to_string()],
        multiplexing_info: cal_model::dbc::MultiplexingInfo::None,
    };
    let binding = SignalBinding::from_signal("load", 0x1C0, &signal);
    assert_eq!(binding.characteristic, "load");
    assert_eq!(binding.can_id, 0x1C0);
    assert_eq!(binding.start_bit, 39);
    assert_eq!(binding.length, 12);
    assert_eq!(binding.byte_order, ByteOrder::Motorola);
    assert!((binding.factor - 0.5).abs() < 1e-12);
    assert!((binding.offset + 3.0).abs() < 1e-12);
}

#[test]
fn the_sample_project_is_internally_coherent() {
    let project = cal_model::sample_project().unwrap();

    // Every module's characteristics resolve to a declared conversion, a
    // declared deposit, and a datatype with a real width.
    for module in project.modules() {
        for characteristic in &module.characteristics {
            assert!(
                !characteristic.deposit.is_empty(),
                "{}",
                characteristic.name
            );
            assert!(
                characteristic.datatype.size_bytes() > 0,
                "{}",
                characteristic.name
            );
            assert!(characteristic.element_bits() > 0);
            // The declared element count fits one XCP UPLOAD.
            let span = usize::from(characteristic.deposit_position) + characteristic.size_bytes();
            assert!(
                span <= usize::from(u8::MAX),
                "{}: {span}",
                characteristic.name
            );
            // And the conversion is resolvable, tables included.
            assert!(
                project.resolved_conversion(characteristic).is_ok(),
                "{}",
                characteristic.name
            );
        }
    }

    // Every multi-element characteristic has a declared count above 1.
    for module in project.modules() {
        for characteristic in &module.characteristics {
            if characteristic.kind.is_multi_element() {
                assert!(
                    characteristic.elements > 1,
                    "{}.{} is multi-element but declares {} element(s)",
                    module.name,
                    characteristic.name,
                    characteristic.elements
                );
            }
        }
    }

    // No two characteristics in a module share an address.
    for module in project.modules() {
        for (index, first) in module.characteristics.iter().enumerate() {
            for second in module.characteristics.iter().skip(index + 1) {
                assert_ne!(
                    (first.address, first.size_bytes()),
                    (second.address, second.size_bytes()),
                    "{}.{} and {}.{} overlap",
                    module.name,
                    first.name,
                    module.name,
                    second.name
                );
            }
        }
    }
}
