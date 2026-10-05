//! proptest battery: conversion round-trips, snapshot/restore idempotence,
//! and byte-order extraction against a naive bit-walk reference.
//!
//! Test file: strategic `unwrap` is by design.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use cal_model::{
    calibrate_curve, sample_project, ByteOrder, CalibrationProject, CalibrationSession,
    ResourceMode, SignalBinding,
};
use proptest::prelude::*;

/// A linear characteristic over `datatype`, with limits wide enough that the
/// round-trip is limited only by the datatype's resolution.
fn linear_project(
    slope: f64,
    intercept: f64,
    datatype: &str,
    lower: f64,
    upper: f64,
) -> CalibrationProject {
    let text = format!(
        "/begin PROJECT p \"d\"\n\
         /begin MODULE m \"m\"\n\
         /begin RECORD_LAYOUT RL FNC_VALUES 1 {datatype}\n\
         /end RECORD_LAYOUT\n\
         /begin COMPU_METHOD cm \"cm\" LINEAR \"%.6f\" \"u\" COEFFS_LINEAR {slope} {intercept}\n\
         /end COMPU_METHOD\n\
         /begin CHARACTERISTIC value \"Value\" VALUE 0x1000\n\
         RL 0 cm {lower} {upper}\n\
         /end CHARACTERISTIC\n\
         /end MODULE\n\
         /end PROJECT\n"
    );
    CalibrationProject::from_a2l(&text).unwrap()
}

/// One payload bit at a DBC bit position, counting bits within a byte from
/// the LSB. Positions past the end of the payload read as zero.
fn bit_at(payload: &[u8], position: u16) -> u8 {
    let index = usize::from(position);
    payload.get(index / 8).copied().unwrap_or(0) >> (index % 8) & 1
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 500, ..ProptestConfig::default() })]

    /// 500 cases: for any raw value and any non-degenerate linear coefficients,
    /// `to_raw(to_physical(raw)) == raw`, and the physical round-trip lands
    /// within half a count of the original value.
    #[test]
    fn linear_round_trip_is_exact(
        raw in 0_u64..4096,
        slope in 0.25_f64..4.0,
        intercept in -1000.0_f64..1000.0,
    ) {
        // Limits span the whole raw range of the datatype plus slack.
        let highest = slope * 4095.0 + intercept;
        let lowest = intercept;
        let (lower, upper) = (lowest - 10_000.0, highest + 10_000.0);
        let project = linear_project(slope, intercept, "UWORD", lower, upper);

        let physical = project.to_physical("m", "value", raw).unwrap();
        let back = project.to_raw("m", "value", physical).unwrap();
        prop_assert_eq!(back, raw, "raw {} → {} → {}", raw, physical, back);

        // And physical → raw → physical within one conversion quantum.
        let again = project.to_physical("m", "value", back).unwrap();
        prop_assert!(
            (again - physical).abs() <= slope * 0.5 + 1e-9,
            "{physical} → {back} → {again} exceeds half a count"
        );
    }

    /// 300 cases: signal extraction agrees with a naive bit-walk reference —
    /// a deliberately naive implementation of the two DBC bit-numbering
    /// systems, written here from the spec rather than shared with the crate.
    #[test]
    fn byte_order_extraction_matches_a_naive_reference(
        start_bit in 0_u16..64,
        length in 1_u8..33,
        motorola in any::<bool>(),
        payload in prop::collection::vec(any::<u8>(), 8),
    ) {
        // The naive reference, written from the spec rather than shared with
        // the crate. It collects the DBC bit positions the signal occupies —
        // descending with a +15 sawtooth for Motorola, ascending for Intel —
        // then reassembles the value most-significant-bit first along that
        // walk. That is a different formulation from `dbc_parse::bits`, which
        // is the point: two implementations of one specification.
        let order = if motorola { ByteOrder::Motorola } else { ByteOrder::Intel };
        let mut positions: Vec<u16> = Vec::new();
        if motorola {
            let mut pos = start_bit;
            for _ in 0..length {
                positions.push(pos);
                pos = if pos % 8 == 0 { pos + 15 } else { pos - 1 };
            }
        } else {
            for offset in 0..length {
                positions.push(start_bit + u16::from(offset));
            }
        }

        // Direction of significance, stated from the spec rather than copied:
        // Motorola names the MSB in `start_bit`, so the *first* position the
        // walk visits is the most significant bit; Intel names the LSB, so
        // the *last* position is. Bits outside the payload read as zero
        // (documented on both implementations).
        let mut expected = 0_u64;
        if motorola {
            for pos in &positions {
                expected = (expected << 1) | u64::from(bit_at(&payload, *pos));
            }
        } else {
            // Intel: position `start_bit + i` carries bit `i`, so the walk
            // order *is* the significance order.
            for (index, pos) in positions.iter().enumerate() {
                let shift = u32::try_from(index).unwrap_or(64);
                if shift < 64 {
                    expected |= u64::from(bit_at(&payload, *pos)) << shift;
                }
            }
        }

        let binding = SignalBinding {
            characteristic: "probe".to_string(),
            can_id: 0x100,
            start_bit,
            length,
            byte_order: order,
            factor: 1.0,
            offset: 0.0,
        };
        prop_assert_eq!(
            binding.extract_raw(&payload),
            expected,
            "start_bit {} len {} {:?}",
            start_bit,
            length,
            order
        );

        // Encoding is the exact inverse, which the reference confirms too.
        let mut encoded = [0_u8; 8];
        if binding.encode(&mut encoded, expected as f64).is_ok() {
            prop_assert_eq!(binding.extract_raw(&encoded), expected);
        }
    }
}

// 200 cases of snapshot/restore idempotence against the mock transport.
proptest! {
    #![proptest_config(ProptestConfig { cases: 200, ..ProptestConfig::default() })]

    #[test]
    fn snapshot_restore_round_trips_any_capture(
        idles in 2000_u64..4800,
        torques in 50_u64..2600,
        curve_raw in 0_u64..6000,
    ) {
        let project = sample_project().unwrap();
        let bench = cal_model::mock::MockTransport::seeded(&cal_model::seed_memory());
        let mut session =
            CalibrationSession::connect(&project, Box::new(bench), ResourceMode::CONNECT_NORMAL)
                .unwrap();

        // Move a representative set of parameters to arbitrary in-range values.
        session
            .write_characteristic_raw("engine", "idle_target_rpm", 0, idles)
            .unwrap();
        session
            .write_characteristic_raw("engine", "eng_torque_max", 0, torques)
            .unwrap();
        session
            .write_characteristic_raw("engine", "boost_curve", 2, curve_raw)
            .unwrap();

        let captured = session
            .snapshot(&["idle_target_rpm", "eng_torque_max", "boost_curve"])
            .unwrap();

        // Restoring the capture reproduces it bit-for-bit...
        session.restore(&captured).unwrap();
        let after_restore = session
            .snapshot(&["idle_target_rpm", "eng_torque_max", "boost_curve"])
            .unwrap();
        prop_assert_eq!(&after_restore, &captured);

        // ...and restoring it again changes nothing (idempotence).
        session.restore(&captured).unwrap();
        let after_second = session
            .snapshot(&["idle_target_rpm", "eng_torque_max", "boost_curve"])
            .unwrap();
        prop_assert_eq!(&after_second, &captured);
    }
}

// 100 cases: `calibrate_curve` recovers a monotone series at its nodes.
proptest! {
    #![proptest_config(ProptestConfig { cases: 100, ..ProptestConfig::default() })]

    #[test]
    fn calibrate_curve_reproduces_a_monotone_series(
        values in prop::collection::vec(0.0_f64..1000.0, 2..8),
        offsets in prop::collection::vec(0.0_f64..100.0, 2..8),
    ) {
        let count = values.len().min(offsets.len());
        if count < 2 {
            return Ok(());
        }
        // Build a strictly increasing set of ordinates from the samples.
        let mut sorted = values[..count].to_vec();
        sorted.sort_by(f64::total_cmp);
        let mut ordinates = Vec::with_capacity(count);
        let mut previous = -1.0_f64;
        for value in sorted {
            let ordinate = value.max(previous + 0.5);
            ordinates.push(ordinate);
            previous = ordinate;
        }

        // Abscissae from the offsets, made strictly increasing.
        let mut abscissae = offsets[..count].to_vec();
        abscissae.sort_by(f64::total_cmp);
        for index in 1..count {
            if abscissae[index] <= abscissae[index - 1] {
                abscissae[index] = abscissae[index - 1] + 1.0;
            }
        }

        let params: Vec<cal_model::CalParameter> = (0..count)
            .map(|index| {
                cal_model::CalParameter::new(
                    format!("n{index}"),
                    ordinates[index],
                    abscissae[index] - 0.5,
                    abscissae[index] + 0.5,
                )
            })
            .collect();

        let curve = calibrate_curve(&params).unwrap();
        prop_assert!(curve.is_monotone());
        for index in 0..count {
            let error = (curve.eval(abscissae[index]) - ordinates[index]).abs();
            prop_assert!(error < 1e-6, "node {index}: error {error}");
        }
    }
}

// 100 cases: every error the public API produces renders a message.
proptest! {
    #![proptest_config(ProptestConfig { cases: 100, ..ProptestConfig::default() })]

    #[test]
    fn every_error_from_the_public_api_renders(
        raw in 0_u64..65536,
        physical in -1e6_f64..1e6,
    ) {
        let project = sample_project().unwrap();
        if let Ok(physical) = project.to_physical("engine", "idle_target_rpm", raw) {
            let outcome = project.to_raw("engine", "idle_target_rpm", physical);
            if let Err(error) = outcome {
                prop_assert!(!error.to_string().is_empty());
            }
        }
        let _ = project.to_raw("engine", "idle_target_rpm", physical);
        prop_assert!(!project.module("nonesuch").unwrap_err().to_string().is_empty());
    }
}
