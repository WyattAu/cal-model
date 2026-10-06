//! The calibration session: read-modify-write against the in-memory mock
//! transport, page switching, snapshot/restore, diff reporting, byte-order
//! extraction against hand-computed signals, and error propagation.
//!
//! Test files: `unwrap`/`expect` are by design — asserting on `Result` is
//! what these tests do.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use cal_model::mock::{MockTransport, Request};
use cal_model::{
    diff_snapshots, render_deltas, sample_project, seed_memory, ByteOrder, CalError,
    CalibrationDelta, CalibrationProject, CalibrationSession, ResourceMode, SignalBinding,
    XcpTransport, SAMPLE_DBC,
};

/// Connect a session to a project with the sample seed memory loaded.
fn bench_on(project: &CalibrationProject) -> CalibrationSession<'_> {
    CalibrationSession::connect(
        project,
        Box::new(MockTransport::seeded(&seed_memory())),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap()
}

#[test]
fn read_characteristic_returns_the_a2l_seed_value() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    // The seed memory holds 800 rpm (3200 counts · 0.25).
    let idle = session
        .read_characteristic("engine", "idle_target_rpm")
        .unwrap();
    assert!((idle - 800.0).abs() < 1e-9, "{idle}");
    // The engine load seed is 30 % (400 counts · 0.1 − 10).
    let load = session.read_characteristic("engine", "eng_load").unwrap();
    assert!((load - 30.0).abs() < 1e-9, "{load}");
}

#[test]
fn write_characteristic_updates_memory_and_read_back_reflects_it() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    session
        .write_characteristic("engine", "idle_target_rpm", 950.0)
        .unwrap();
    let read = session
        .read_characteristic("engine", "idle_target_rpm")
        .unwrap();
    assert!((read - 950.0).abs() < 1e-9, "{read}");

    // The raw memory really changed: 950 rpm = 3800 counts = 0x0ED8.
    assert_eq!(
        MockTransport::from_transport(session.transport())
            .unwrap()
            .peek(0x720108, 3),
        vec![0x00, 0xD8, 0x0E]
    );
}

#[test]
fn a_write_is_read_back_verified() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    // A write records both the DOWNLOAD and the confirming UPLOAD.
    session
        .write_characteristic("engine", "eng_torque_max", 250.0)
        .unwrap();
    let writes: Vec<&Request> = MockTransport::from_transport(session.transport())
        .unwrap()
        .requests()
        .iter()
        .filter(|r| matches!(r, Request::Write { .. }))
        .collect();
    assert_eq!(writes.len(), 1);
    let reads: Vec<&Request> = MockTransport::from_transport(session.transport())
        .unwrap()
        .requests()
        .iter()
        .filter(|r| matches!(r, Request::Read { .. }))
        .collect();
    assert_eq!(reads.len(), 1, "the write is confirmed by exactly one read");
    assert!(matches!(reads[0], Request::Read { addr, len } if addr == &0x720104 && len == &3));
}

#[test]
fn a_write_of_an_out_of_limit_value_never_reaches_the_bus() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    let error = session
        .write_characteristic("engine", "idle_target_rpm", 5000.0)
        .unwrap_err();
    assert!(matches!(error, CalError::LimitViolation { .. }), "{error}");
    // Nothing was transmitted.
    assert!(MockTransport::from_transport(session.transport())
        .unwrap()
        .requests()
        .is_empty());
}

#[test]
fn writing_a_curve_element_leaves_the_other_elements_intact() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    let before = session
        .read_characteristic_elements("engine", "boost_curve")
        .unwrap();
    assert_eq!(before.len(), 8);

    // Adjust point 3 only, from 45 kPa up to a tabulated 128 kPa (an exact
    // table point, so no quantisation shows up in the read-back).
    session
        .write_characteristic_element("engine", "boost_curve", 3, 128.0)
        .unwrap();

    let after = session
        .read_characteristic_elements("engine", "boost_curve")
        .unwrap();
    assert_eq!(after.len(), 8);
    for (index, value) in after.iter().enumerate() {
        if index == 3 {
            assert!((value - 128.0).abs() < 1e-9, "[3] = {value}");
        } else {
            assert!((value - 45.0).abs() < 1e-9, "[{index}] = {value}");
        }
    }
}

#[test]
fn writing_past_the_end_of_a_curve_is_out_of_bounds() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let error = session
        .write_characteristic_raw("engine", "boost_curve", 8, 0)
        .unwrap_err();
    assert!(
        matches!(&error, CalError::OutOfBounds { lower, upper, .. } if *lower == 0.0 && *upper == 7.0),
        "{error}"
    );
}

#[test]
fn switching_pages_leaves_the_other_page_untouched() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    assert_eq!(session.cal_page(), 0);

    // On page 0 the idle target reads 800 rpm.
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 800.0)
            .abs()
            < 1e-9
    );

    session.set_cal_page(1).unwrap();
    assert_eq!(session.cal_page(), 1);
    assert!(MockTransport::from_transport(session.transport())
        .unwrap()
        .requests()
        .contains(&Request::SetPage(1)));

    // Page 1 starts erased: all zeros.
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 0.0)
            .abs()
            < 1e-9
    );

    session
        .write_characteristic("engine", "idle_target_rpm", 750.0)
        .unwrap();
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 750.0)
            .abs()
            < 1e-9
    );

    // Back to page 0: the original 800 rpm is intact.
    session.set_cal_page(0).unwrap();
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 800.0)
            .abs()
            < 1e-9
    );
    assert_eq!(
        MockTransport::from_transport(session.transport())
            .unwrap()
            .byte_on(0, 0x72010A),
        0x0C
    );
    assert_eq!(
        MockTransport::from_transport(session.transport())
            .unwrap()
            .byte_on(1, 0x72010A),
        0x0B
    );
}

#[test]
fn a_failed_page_switch_leaves_the_session_on_its_old_page() {
    let project = sample_project().unwrap();

    // A transport whose SET_CAL_PAGE is refused. The session
    // borrows its transport, so a fresh session over a failing transport is
    // the honest way to exercise this.
    struct RefusesPages(MockTransport);
    impl cal_model::XcpTransport for RefusesPages {
        fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
            self.0.read(addr, len)
        }
        fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError> {
            self.0.write(addr, data)
        }
        fn set_page(&mut self, _page: u32) -> Result<(), CalError> {
            Err(CalError::Transport("page 5 not available".to_string()))
        }
    }

    let mut refusing = CalibrationSession::connect(
        &project,
        Box::new(RefusesPages(MockTransport::seeded(&seed_memory()))),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    assert_eq!(refusing.cal_page(), 0);
    let error = refusing.set_cal_page(5).unwrap_err();
    assert!(matches!(&error, CalError::Transport(msg) if msg.contains("not available")));
    // The session's page did not move.
    assert_eq!(refusing.cal_page(), 0);
    // And the memory it reads is still page 0's.
    assert!(
        (refusing
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 800.0)
            .abs()
            < 1e-9
    );
}

#[test]
fn a_transport_default_set_page_accepts_the_switch() {
    // The trait's default `set_page` is a no-op, so a single-page transport
    // satisfies the contract without implementing anything.
    struct SinglePage;
    impl cal_model::XcpTransport for SinglePage {
        fn read(&mut self, _addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
            Ok(vec![0; usize::from(len)])
        }
        fn write(&mut self, _addr: u32, _data: &[u8]) -> Result<(), CalError> {
            Ok(())
        }
    }
    let project = sample_project().unwrap();
    let mut session =
        CalibrationSession::connect(&project, Box::new(SinglePage), ResourceMode::CONNECT_NORMAL)
            .unwrap();
    assert!(session.set_cal_page(3).is_ok());
    assert_eq!(session.cal_page(), 3);
}

#[test]
fn snapshot_and_restore_round_trip_every_value() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    let original = session.snapshot_module("engine").expect("engine snapshots");
    assert_eq!(original.entries.len(), 9);
    assert_eq!(original.page, 0);
    // 6 single-element characteristics + 8 + 4 + 3 curve elements.
    assert_eq!(original.element_count(), 6 + 8 + 4 + 3);

    // Move a representative of each shape.
    session
        .write_characteristic("engine", "idle_target_rpm", 1000.0)
        .unwrap();
    session
        .write_characteristic("engine", "eng_torque_max", 400.0)
        .unwrap();
    session
        .write_characteristic_element("engine", "boost_curve", 0, 10.0)
        .unwrap();
    session
        .write_characteristic_element("engine", "boost_curve", 5, 142.0)
        .unwrap();
    session
        .write_characteristic_element("engine", "inj_map", 2, 80.0)
        .unwrap();
    session
        .write_characteristic_element("engine", "knock_table", 1, 100.0)
        .unwrap();

    // Everything really moved.
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 1000.0)
            .abs()
            < 1e-9
    );
    assert!(
        (session
            .read_characteristic_elements("engine", "boost_curve")
            .unwrap()[5]
            - 142.0)
            .abs()
            < 1e-9
    );

    // Restore, and the whole deposit is bit-identical to the capture.
    session.restore(&original).unwrap();
    let restored = session.snapshot_module("engine").unwrap();
    assert_eq!(restored, original);
}

#[test]
fn restore_is_bit_exact_not_value_exact() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let before = session.snapshot(&["eng_torque_max"]).unwrap();

    session
        .write_characteristic("engine", "eng_torque_max", 300.0)
        .unwrap();
    session.restore(&before).unwrap();

    // The raw bit pattern came back, not a re-derived float.
    let raw = session
        .read_characteristic_raw("engine", "eng_torque_max")
        .unwrap();
    let original_raw = before.entries[0].elements[0].raw;
    assert_eq!(raw, vec![original_raw]);
}

#[test]
fn restore_switches_to_the_snapshot_page_first() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let mut snapshot = session.snapshot(&["idle_target_rpm"]).unwrap();
    snapshot.page = 2;

    session.restore(&snapshot).unwrap();
    assert_eq!(session.cal_page(), 2);
    // And the value landed on page 2, not page 0.
    assert!(
        (session
            .read_characteristic("engine", "idle_target_rpm")
            .unwrap()
            - 800.0)
            .abs()
            < 1e-9
    );
    assert_eq!(
        MockTransport::from_transport(session.transport())
            .unwrap()
            .byte_on(2, 0x72010A),
        0x0C
    );
}

#[test]
fn restore_of_an_empty_snapshot_is_a_no_op() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    session.restore(&cal_model::Snapshot::empty(0)).unwrap();
    assert_eq!(session.cal_page(), 0);
}

#[test]
fn restore_reports_a_parameter_the_project_no_longer_declares() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let mut snapshot = session.snapshot(&["idle_target_rpm"]).unwrap();
    snapshot.entries[0].name = "deleted_parameter".to_string();
    let error = session.restore(&snapshot).unwrap_err();
    assert!(
        matches!(&error, CalError::UnknownCharacteristic(name) if name == "engine.deleted_parameter"),
        "{error}"
    );
}

#[test]
fn snapshots_compare_equal_and_look_up_by_name() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let snapshot = session
        .snapshot(&["idle_target_rpm", "eng_torque_max"])
        .unwrap();

    assert_eq!(snapshot.entries.len(), 2);
    assert!(snapshot
        .physical("idle_target_rpm")
        .is_some_and(|v| (v - 800.0).abs() < 1e-9));
    assert!(snapshot
        .physical("eng_torque_max")
        .is_some_and(|v| (v - 100.0).abs() < 1e-9));
    assert!(snapshot.physical("absent").is_none());
    assert!(snapshot.find("idle_target_rpm").is_some());
    assert!(snapshot.entry("engine", "idle_target_rpm").is_some());
    assert!(snapshot.entry("transmission", "idle_target_rpm").is_none());
    assert_eq!(snapshot.element_count(), 2);

    // PartialEq: an identical re-capture equals it.
    let again = session
        .snapshot(&["idle_target_rpm", "eng_torque_max"])
        .unwrap();
    assert_eq!(snapshot, again);

    // And a moved value does not.
    session
        .write_characteristic("engine", "idle_target_rpm", 810.0)
        .unwrap();
    let moved = session
        .snapshot(&["idle_target_rpm", "eng_torque_max"])
        .unwrap();
    assert_ne!(snapshot, moved);
}

#[test]
fn snapshot_of_an_unknown_name_is_typed() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    assert_eq!(
        session.snapshot(&["no_such_parameter"]),
        Err(CalError::UnknownCharacteristic(
            "no_such_parameter".to_string()
        ))
    );
}

#[test]
fn a_snapshot_renders_a_readable_report() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let snapshot = session.snapshot(&["idle_target_rpm"]).unwrap();
    let report = snapshot.to_string();
    assert!(report.contains("calibration snapshot: page 0"));
    assert!(report.contains("1 characteristic(s), 1 element(s)"));
    assert!(report.contains("engine.idle_target_rpm (1 element)"));
    assert!(report.contains("[0] raw 0xC80 physical 800.000000"));
}

#[test]
fn a_curve_entry_renders_every_element() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let snapshot = session.snapshot(&["boost_curve"]).unwrap();
    let report = snapshot.to_string();
    assert!(report.contains("engine.boost_curve (8 elements)"));
    assert_eq!(report.matches("[0] raw").count(), 1);
    // One line per element.
    assert_eq!(report.lines().count(), 1 + 1 + 8);
}

#[test]
fn snapshot_element_accessors_agree() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let snapshot = session.snapshot(&["boost_curve"]).unwrap();
    let entry = snapshot.find("boost_curve").unwrap();
    assert!(entry.physical(0).is_some());
    assert!(entry.physical(7).is_some());
    assert!(entry.physical(8).is_none());
    assert_eq!(entry.element_name(0), "boost_curve[0]");
    let idle = session.snapshot(&["idle_target_rpm"]).unwrap();
    assert_eq!(idle.entries[0].element_name(0), "idle_target_rpm");
}

#[test]
fn diff_reports_the_deltas_and_flags_out_of_limit_changes() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let before = session
        .snapshot(&["idle_target_rpm", "eng_torque_max", "boost_curve"])
        .unwrap();

    session
        .write_characteristic("engine", "idle_target_rpm", 900.0)
        .unwrap();
    session
        .write_characteristic("engine", "eng_torque_max", 300.0)
        .unwrap();
    // 20 kPa is between tabulated points: the deposit can only hold the
    // nearest quantum (raw 444 → 19.98 kPa).
    session
        .write_characteristic_element("engine", "boost_curve", 0, 20.0)
        .unwrap();
    let after = session
        .snapshot(&["idle_target_rpm", "eng_torque_max", "boost_curve"])
        .unwrap();

    let deltas = diff_snapshots(Some(&before), &after, &project);
    assert_eq!(deltas.len(), 1 + 1 + 8);

    // idle: 800 → 900, delta +100.
    let idle = deltas.iter().find(|d| d.name == "idle_target_rpm").unwrap();
    assert!((idle.before - 800.0).abs() < 1e-9);
    assert!((idle.after - 900.0).abs() < 1e-9);
    assert!((idle.delta - 100.0).abs() < 1e-9);
    assert!(idle.within_limits);
    assert!(!idle.is_unchanged());
    assert!(!idle.is_new());

    // torque: 100 → 300, delta +200.
    let torque = deltas.iter().find(|d| d.name == "eng_torque_max").unwrap();
    assert!((torque.delta - 200.0).abs() < 1e-9);
    assert!(torque.within_limits);

    // The curve is reported per element, and only element 0 moved.
    let curve: Vec<&CalibrationDelta> = deltas
        .iter()
        .filter(|d| d.name.starts_with("boost_curve"))
        .collect();
    assert_eq!(curve.len(), 8);
    assert!(!curve[0].is_unchanged());
    // The TABLE conversion resolves only to the tabulation's resolution, so
    // the read-back lands a quantum above the requested 20 kPa — which is
    // exactly what the ECU will interpolate.
    assert!((curve[0].after - 19.98).abs() < 1e-6, "{}", curve[0].after);
    assert!((curve[0].before - 45.0).abs() < 1e-9);
    assert!(curve[0].delta < 0.0, "boost dropped from 45 to ~20");
    for element in &curve[1..] {
        assert!(
            element.is_unchanged(),
            "{} should not have moved",
            element.name
        );
    }
}

#[test]
fn diff_flags_a_change_that_is_now_out_of_limits() {
    // A snapshot whose value exceeds the project's current limits: the diff
    // evaluates against the limits that now apply, not the ones in force when
    // the capture was taken.
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let before = session.snapshot(&["idle_target_rpm"]).unwrap();

    // Hand-build an "after" that claims 5000 rpm.
    let after = cal_model::Snapshot {
        page: 0,
        entries: vec![cal_model::SnapshotEntry {
            module: "engine".to_string(),
            name: "idle_target_rpm".to_string(),
            elements: vec![cal_model::SnapshotValue {
                raw: 20000,
                physical: 5000.0,
            }],
        }],
    };

    let deltas = diff_snapshots(Some(&before), &after, &project);
    assert_eq!(deltas.len(), 1);
    assert!(!deltas[0].within_limits);
    assert!((deltas[0].delta - 4200.0).abs() < 1e-9);
    assert!(deltas[0].to_string().contains("OUT OF LIMITS"));
}

#[test]
fn diff_against_no_baseline_reports_everything_as_new() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let after = session.snapshot(&["idle_target_rpm"]).unwrap();
    let deltas = diff_snapshots(None, &after, &project);
    assert_eq!(deltas.len(), 1);
    assert!(deltas[0].is_new());
    assert!(deltas[0].before.is_nan());
    assert!(deltas[0].delta.is_nan());
    assert!(deltas[0].to_string().contains("(new)"));
}

#[test]
fn a_session_diff_measures_against_its_baseline() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);

    // No baseline yet: everything is new.
    let first = session.snapshot(&["idle_target_rpm"]).unwrap();
    assert!(session.diff(&first).iter().all(CalibrationDelta::is_new));
    assert!(session.baseline().is_none());

    session.set_baseline(first.clone());
    assert_eq!(session.baseline(), Some(&first));

    session
        .write_characteristic("engine", "idle_target_rpm", 1100.0)
        .unwrap();
    let second = session.snapshot(&["idle_target_rpm"]).unwrap();
    let deltas = session.diff(&second);
    assert_eq!(deltas.len(), 1);
    assert!((deltas[0].before - 800.0).abs() < 1e-9);
    assert!((deltas[0].after - 1100.0).abs() < 1e-9);
    assert!((deltas[0].delta - 300.0).abs() < 1e-9);
}

#[test]
fn capture_baseline_snapshots_every_module() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    session.capture_baseline().unwrap();
    let baseline = session.baseline().unwrap();
    assert_eq!(baseline.entries.len(), 9 + 6);
    assert!(baseline.entry("transmission", "gear_code").is_some());
    assert!(baseline.entry("engine", "boost_curve").is_some());
}

#[test]
fn a_session_hands_out_the_xcp_frames_for_each_operation() {
    let project = sample_project().unwrap();
    let session = bench_on(&project);

    // SET_MTA + UPLOAD for a 3-byte read at 0x720108.
    let upload = session.upload_frames(0x720108, 3);
    assert_eq!(upload[0], 0xF6, "SET_MTA");
    assert_eq!(
        &upload[1..4],
        &[0x00, 0x00, 0x00],
        "reserved + address extension 0"
    );
    assert_eq!(&upload[4..8], &[0x08, 0x01, 0x72, 0x00], "0x720108, Intel");
    assert_eq!(upload[8], 0xF5, "UPLOAD");
    assert_eq!(upload[9], 3, "element count");

    // SET_MTA + DOWNLOAD for a 3-byte write.
    let download = session.download_frames(0x720108, &[0x00, 0xD8, 0x0E]);
    assert_eq!(download[0], 0xF6, "SET_MTA");
    assert_eq!(&download[4..8], &[0x08, 0x01, 0x72, 0x00]);
    assert_eq!(download[8], 0xF0, "DOWNLOAD");
    assert_eq!(download[9], 3);
    assert_eq!(&download[10..], &[0x00, 0xD8, 0x0E]);

    // SET_CAL_PAGE for page 2.
    let page = session.set_cal_page_frame(2);
    assert_eq!(page[0], 0xEB, "SET_CAL_PAGE");
    assert_eq!(page[1], 0x03, "switch on the ECU, all segments");
    assert_eq!(&page[2..], &[0x02, 0x00]);
}

#[test]
fn transport_read_failures_surface_unchanged() {
    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new().fail_reads("slave not answering")),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();

    let error = session
        .read_characteristic("engine", "idle_target_rpm")
        .unwrap_err();
    assert_eq!(
        error,
        CalError::Transport("slave not answering".to_string())
    );
    assert_eq!(error.to_string(), "transport failure: slave not answering");
}

#[test]
fn transport_write_failures_surface_unchanged() {
    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new().fail_writes("page write protected")),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();

    let error = session
        .write_characteristic("engine", "idle_target_rpm", 900.0)
        .unwrap_err();
    assert!(matches!(&error, CalError::Transport(msg) if msg.contains("write protected")));
}

#[test]
fn a_snapshot_over_a_failing_transport_reports_the_failure() {
    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(MockTransport::new().fail_reads("bus-off")),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    assert!(matches!(
        session.snapshot(&["idle_target_rpm"]),
        Err(CalError::Transport(_))
    ));
    assert!(matches!(
        session.snapshot_module("engine"),
        Err(CalError::Transport(_))
    ));
    assert!(matches!(
        session.capture_baseline(),
        Err(CalError::Transport(_))
    ));
}

#[test]
fn a_read_back_mismatch_is_a_transport_error() {
    // A transport that silently drops writes: the session must notice rather
    // than report a calibration it did not perform.
    struct DropsWrites(MockTransport);
    impl cal_model::XcpTransport for DropsWrites {
        fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
            self.0.read(addr, len)
        }
        fn write(&mut self, _addr: u32, _data: &[u8]) -> Result<(), CalError> {
            Ok(()) // accepted, not applied
        }
    }

    let project = sample_project().unwrap();
    let mut session = CalibrationSession::connect(
        &project,
        Box::new(DropsWrites(MockTransport::seeded(&seed_memory()))),
        ResourceMode::CONNECT_NORMAL,
    )
    .unwrap();
    let error = session
        .write_characteristic("engine", "idle_target_rpm", 900.0)
        .unwrap_err();
    assert!(
        matches!(&error, CalError::Transport(msg) if msg.contains("write-back mismatch")),
        "{error}"
    );
}

#[test]
fn byte_order_extraction_matches_a_hand_computed_motorola_signal() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();

    // `boost_target : 39|12@0+ (0.5,0)` — Motorola, 12 bits starting at DBC
    // bit 39. Hand-computed: bit 39 is the MSB, so the walk is 39, 38, 37,
    // 36, 35, 34, 33, 32, 47, 46, 45, 44. Payload 0x12 0x34 → 0xABC.
    let binding = project
        .require_signal_binding("engine", "boost_target")
        .unwrap();
    assert_eq!(binding.byte_order, ByteOrder::Motorola);
    assert_eq!(binding.start_bit, 39);
    assert_eq!(binding.length, 12);
    assert_eq!(binding.can_id, 192);
    assert!((binding.factor - 0.5).abs() < 1e-12);
    assert_eq!(binding.offset, 0.0);

    let payload = [0x00_u8, 0x00, 0x00, 0x00, 0x12, 0x34, 0x00, 0x00];
    // Hand-computed: DBC bit 39 is byte 4 / in-byte bit 7, so the walk is
    // 39, 38, 37, 36, 35, 34, 33, 32 (all of byte 4, MSB first) and then
    // the sawtooth jumps to 47, 46, 45, 44 — the top nibble of byte 5.
    // Byte 4 = 0b0001_0010 and byte 5's top nibble = 0b0011, giving
    // 0b0001_0010_0011 = 0x123.
    assert_eq!(binding.extract_raw(&payload), 0x123);
    assert!((binding.decode(&payload) - 0x123 as f64 * 0.5).abs() < 1e-9);
    // 0x123 = 291 counts · 0.5 = 145.5 kPa.
    assert!((binding.decode(&payload) - 145.5).abs() < 1e-9);
}

#[test]
fn byte_order_extraction_matches_a_hand_computed_intel_signal() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();

    // `engine_speed : 16|16@1+ (0.25,0)` — Intel, 16 bits at DBC bit 16:
    // bytes 2…3, least significant byte first. Payload 34 12 → 0x1234.
    let binding = project
        .require_measurement_binding("engine", "engine_speed")
        .unwrap();
    assert_eq!(binding.byte_order, ByteOrder::Intel);
    assert_eq!(binding.start_bit, 16);
    assert_eq!(binding.length, 16);

    let payload = [0x00_u8, 0x00, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00];
    assert_eq!(binding.extract_raw(&payload), 0x1234);
    // 0x1234 = 4660 · 0.25 = 1165 rpm.
    assert!((binding.decode(&payload) - 1165.0).abs() < 1e-9);

    // `eng_load : 0|12@1+ (0.1,-10)` — Intel, 12 bits at bit 0.
    let load = project
        .require_signal_binding("engine", "eng_load")
        .unwrap();
    assert_eq!(load.start_bit, 0);
    assert_eq!(load.length, 12);
    let low = [0x9A_u8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    assert_eq!(load.extract_raw(&low), 0x09A);
    // 154 · 0.1 − 10 = 5.4 %.
    assert!((load.decode(&low) - 5.4).abs() < 1e-9);
}

#[test]
fn motorola_and_intel_disagree_on_the_same_payload() {
    // The whole point of the two byte orders: identical bytes, different
    // signals. This is the naive-reference check done by hand.
    let payload = [0x12_u8, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];
    let intel = SignalBinding {
        characteristic: "intel".to_string(),
        can_id: 0x100,
        start_bit: 0,
        length: 16,
        byte_order: ByteOrder::Intel,
        factor: 1.0,
        offset: 0.0,
    };
    // Intel, bits 0…15: byte 0 LSB-first → 0x3412.
    assert_eq!(intel.extract_raw(&payload), 0x3412);

    let motorola = SignalBinding {
        characteristic: "motorola".to_string(),
        can_id: 0x100,
        start_bit: 7,
        length: 16,
        byte_order: ByteOrder::Motorola,
        factor: 1.0,
        offset: 0.0,
    };
    // Motorola, start bit 7 (the MSB of byte 0): descend 7…0, then jump to
    // 15 and descend 15…8 → bytes 0 and 1 read big-endian → 0x1234.
    assert_eq!(motorola.extract_raw(&payload), 0x1234);
    assert_ne!(intel.extract_raw(&payload), motorola.extract_raw(&payload));
}

#[test]
fn a_binding_round_trips_a_physical_value_through_a_payload() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();
    let binding = project
        .require_measurement_binding("engine", "engine_speed")
        .unwrap();

    let mut payload = [0_u8; 8];
    binding.encode(&mut payload, 3000.0).unwrap();
    // 3000 rpm / 0.25 = 12000 counts = 0x2EE0, Intel at bytes 2…3.
    assert_eq!(&payload[2..4], &[0xE0, 0x2E]);
    assert!((binding.decode(&payload) - 3000.0).abs() < 1e-9);
}

#[test]
fn a_motorola_binding_round_trips_through_the_sawtooth() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();
    let binding = project
        .require_signal_binding("engine", "boost_target")
        .unwrap();

    let mut payload = [0_u8; 8];
    binding.encode(&mut payload, 145.5).unwrap();
    assert_eq!(binding.extract_raw(&payload), 0x123);
    assert!((binding.decode(&payload) - 145.5).abs() < 1e-9);
    // The bits landed where the hand computation says: 0x12 0x34.
    assert_eq!(payload[4], 0x12);
    assert_eq!(payload[5] & 0xF0, 0x30);
}

#[test]
fn a_binding_rejects_a_value_too_wide_for_its_signal() {
    let binding = SignalBinding {
        characteristic: "narrow".to_string(),
        can_id: 0x100,
        start_bit: 0,
        length: 8,
        byte_order: ByteOrder::Intel,
        factor: 1.0,
        offset: 0.0,
    };
    let mut payload = [0_u8; 2];
    let error = binding.encode(&mut payload, 256.0).unwrap_err();
    assert!(
        matches!(&error, CalError::Dbc(cal_model::dbc::DbcError::RawOutOfRange { bit_length, .. }) if *bit_length == 8),
        "{error}"
    );
    assert!(binding.encode(&mut payload, 255.0).is_ok());
}

#[test]
fn a_binding_rejects_a_zero_factor_and_a_short_buffer() {
    let flat = SignalBinding {
        characteristic: "flat".to_string(),
        can_id: 0x100,
        start_bit: 0,
        length: 8,
        byte_order: ByteOrder::Intel,
        factor: 0.0,
        offset: 5.0,
    };
    let mut payload = [0_u8; 2];
    assert!(matches!(
        flat.encode(&mut payload, 5.0),
        Err(CalError::Unsupported { .. })
    ));

    let binding = SignalBinding {
        characteristic: "wide".to_string(),
        can_id: 0x100,
        start_bit: 16,
        length: 32,
        byte_order: ByteOrder::Intel,
        factor: 1.0,
        offset: 0.0,
    };
    let mut short = [0_u8; 2];
    assert!(matches!(
        binding.encode(&mut short, 1.0),
        Err(CalError::Dbc(cal_model::dbc::DbcError::BufferTooShort {
            need: 6,
            have: 2
        }))
    ));
}

#[test]
fn extraction_is_total_for_a_truncated_payload() {
    let binding = SignalBinding {
        characteristic: "truncated".to_string(),
        can_id: 0x100,
        start_bit: 7,
        length: 16,
        byte_order: ByteOrder::Motorola,
        factor: 1.0,
        offset: 0.0,
    };
    // Positions past the end read as zero rather than panicking.
    assert_eq!(binding.extract_raw(&[0xFF]), 0xFF00);
    assert_eq!(binding.extract_raw(&[]), 0);
}

#[test]
fn a_binding_knows_how_big_a_payload_must_be() {
    let intel = SignalBinding {
        characteristic: "i".to_string(),
        can_id: 0,
        start_bit: 16,
        length: 16,
        byte_order: ByteOrder::Intel,
        factor: 1.0,
        offset: 0.0,
    };
    assert_eq!(intel.payload_len(), 4);

    // Motorola `39|12@0` reaches byte 5.
    let motorola = SignalBinding {
        characteristic: "m".to_string(),
        can_id: 0,
        start_bit: 39,
        length: 12,
        byte_order: ByteOrder::Motorola,
        factor: 1.0,
        offset: 0.0,
    };
    assert_eq!(motorola.payload_len(), 6);
}

#[test]
fn a_binding_renders_readably() {
    let binding = SignalBinding {
        characteristic: "eng_load".to_string(),
        can_id: 192,
        start_bit: 0,
        length: 12,
        byte_order: ByteOrder::Intel,
        factor: 0.1,
        offset: -10.0,
    };
    let rendered = binding.to_string();
    assert!(rendered.contains("0x0C0 eng_load"));
    assert!(rendered.contains("start_bit 0 len 12"));
    assert!(rendered.contains("intel"));
    assert!(rendered.contains("factor 0.1 offset -10"));

    let motorola = SignalBinding {
        byte_order: ByteOrder::Motorola,
        ..binding
    };
    assert!(motorola.to_string().contains("motorola"));
}

#[test]
fn bindings_decode_together_from_one_payload() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();
    let bindings: Vec<SignalBinding> = project
        .signal_bindings("engine")
        .into_iter()
        .cloned()
        .collect();
    assert_eq!(bindings.len(), 2);

    let payload = [0x00_u8, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00];
    let decoded = SignalBinding::decode_all(&bindings, &payload);
    assert_eq!(decoded.len(), 2);
    for (can_id, name, _value) in &decoded {
        assert_eq!(*can_id, 192);
        assert!(!name.is_empty());
    }
}

#[test]
fn binding_falls_back_to_a_case_insensitive_signal_match() {
    let dbc = r#"
        BO_ 100 Status: 4 ECU
         SG_ Coolant_Temp : 0|8@1+ (1,-40) [-40|87] "degC" ECU
    "#;
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(dbc).unwrap();
    // `coolant_temp` matches `Coolant_Temp` case-insensitively.
    assert!(project
        .signal_for_measurement("engine", "coolant_temp")
        .is_some());
    assert!(project
        .require_measurement_binding("engine", "coolant_temp")
        .is_ok());
    // Nothing else does.
    assert!(project
        .signal_for_measurement("engine", "engine_speed")
        .is_none());
    assert!(project
        .signal_for_characteristic("engine", "boost_curve")
        .is_none());
}

#[test]
fn a_project_without_a_dbc_reports_no_binding() {
    // Built from SAMPLE_A2L directly rather than via `sample_project`, which now
    // attaches the DBC — a project *with* no bus attached is the point here, and
    // relying on a fixture being incomplete to express that is how the fixture
    // ended up incomplete in the first place.
    let project = CalibrationProject::from_a2l(cal_model::SAMPLE_A2L).unwrap();
    assert!(project.dbc().is_none());
    assert!(project
        .signal_for_characteristic("engine", "eng_load")
        .is_none());
    assert_eq!(
        project.require_signal_binding("engine", "eng_load"),
        Err(CalError::NoSignalBinding("engine.eng_load".to_string()))
    );
    assert!(project.signal_bindings("engine").is_empty());
}

#[test]
fn require_signal_binding_validates_the_lookup_first() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();
    assert_eq!(
        project.require_signal_binding("engine", "no_such"),
        Err(CalError::UnknownCharacteristic(
            "engine.no_such".to_string()
        ))
    );
    assert_eq!(
        project.require_signal_binding("no_module", "eng_load"),
        Err(CalError::UnknownModule("no_module".to_string()))
    );
    // A real characteristic with no same-named signal.
    assert_eq!(
        project.require_signal_binding("engine", "boost_curve"),
        Err(CalError::NoSignalBinding("engine.boost_curve".to_string()))
    );
}

#[test]
fn attaching_a_dbc_twice_replaces_the_bindings() {
    let mut project = sample_project().unwrap();
    project.attach_dbc_text(SAMPLE_DBC).unwrap();
    assert_eq!(project.signal_bindings("engine").len(), 2);
    assert_eq!(project.measurement_bindings("engine").len(), 3);

    let replacement = r#"
        BO_ 200 Only: 1 ECU
         SG_ eng_load : 0|8@1+ (1,0) [0|255] "-" ECU
    "#;
    project.attach_dbc_text(replacement).unwrap();
    assert_eq!(project.signal_bindings("engine").len(), 1);
    assert_eq!(project.dbc().unwrap().messages.len(), 1);
    let binding = project
        .require_signal_binding("engine", "eng_load")
        .unwrap();
    assert_eq!(binding.can_id, 200);
    assert_eq!(binding.length, 8);
}

#[test]
fn a_malformed_dbc_is_a_dbc_error() {
    let mut project = sample_project().unwrap();
    let error = project.attach_dbc_text("BO_ nonsense").unwrap_err();
    assert!(matches!(error, CalError::Dbc(_)), "{error}");
}

#[test]
fn an_out_of_range_signal_layout_is_refused_and_leaves_no_dbc() {
    // dbc-parse cannot produce one from text, so build the database by hand —
    // the point is that `attach_dbc` validates what it is handed.
    let signal = cal_model::dbc::Signal {
        name: "eng_load".to_string(),
        start_bit: 600, // beyond the DBC's 511
        bit_length: 12,
        byte_order: ByteOrder::Intel,
        is_signed: false,
        scale: 0.1,
        offset: -10.0,
        min: 0.0,
        max: 100.0,
        unit: "%".to_string(),
        receivers: vec!["ECU".to_string()],
        multiplexing_info: cal_model::dbc::MultiplexingInfo::None,
    };
    let dbc = cal_model::dbc::Dbc {
        messages: vec![cal_model::dbc::Message {
            id: cal_model::dbc::CanId::Standard(192),
            name: "Status".to_string(),
            dlc: 8,
            sender: "ECU".to_string(),
            signals: vec![signal],
        }],
        ..cal_model::dbc::Dbc::default()
    };

    // From SAMPLE_A2L directly, so the assertion that a *refused* attachment
    // leaves no DBC starts from a project that had none.
    let mut project = CalibrationProject::from_a2l(cal_model::SAMPLE_A2L).unwrap();
    let error = project.attach_dbc(dbc).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { subject, reason }
            if subject == "signal `eng_load`" && reason.contains("start <= 511")),
        "{error}"
    );
    assert!(
        project.dbc().is_none(),
        "a refused attachment leaves no DBC"
    );
    assert!(project
        .signal_for_characteristic("engine", "eng_load")
        .is_none());
}

#[test]
fn the_diff_report_renders_a_readable_table() {
    let project = sample_project().unwrap();
    let mut session = bench_on(&project);
    let before = session
        .snapshot(&["idle_target_rpm", "eng_torque_max"])
        .unwrap();
    session
        .write_characteristic("engine", "idle_target_rpm", 900.0)
        .unwrap();
    let after = session
        .snapshot(&["idle_target_rpm", "eng_torque_max"])
        .unwrap();

    let report = render_deltas(&diff_snapshots(Some(&before), &after, &project));
    assert!(report.contains("parameter"));
    assert!(report.contains("idle_target_rpm"));
    assert!(report.contains("eng_torque_max"));
    assert!(report.contains("1 changed, 1 unchanged, 0 out of limits"));
    // The unchanged row carries an explicit zero delta.
    assert!(report.contains("+0.000000"));
}

#[test]
fn an_empty_snapshot_renders_and_compares() {
    let empty = cal_model::Snapshot::empty(0);
    assert_eq!(empty, cal_model::Snapshot::default());
    assert_eq!(empty.entries.len(), 0);
    assert_eq!(empty.element_count(), 0);
    assert!(empty
        .to_string()
        .contains("0 characteristic(s), 0 element(s)"));
    assert!(render_deltas(&[]).contains("0 changed, 0 unchanged, 0 out of limits"));
    // An empty "after" produces no deltas at all.
    let project = sample_project().unwrap();
    assert!(diff_snapshots(Some(&empty), &empty, &project).is_empty());
}

#[test]
fn a_multi_element_entry_reports_a_plural_in_its_display() {
    let single = cal_model::SnapshotEntry {
        module: "m".to_string(),
        name: "one".to_string(),
        elements: vec![cal_model::SnapshotValue {
            raw: 1,
            physical: 1.0,
        }],
    };
    assert!(single.to_string().contains("(1 element)"));
    let plural = cal_model::SnapshotEntry {
        module: "m".to_string(),
        name: "many".to_string(),
        elements: vec![
            cal_model::SnapshotValue {
                raw: 1,
                physical: 1.0
            };
            3
        ],
    };
    assert!(plural.to_string().contains("(3 elements)"));
}

#[test]
fn a_debug_render_of_a_session_shows_its_state() {
    let project = sample_project().unwrap();
    let session = bench_on(&project);
    let rendered = format!("{session:?}");
    assert!(rendered.contains("CalibrationSession"));
    assert!(rendered.contains("page: 0"));
    assert!(rendered.contains("resources:"));
}

#[test]
fn mock_transport_reports_its_traffic_and_page() {
    let mut bench = MockTransport::seeded(&[(0x1000, vec![1, 2, 3])]);
    assert_eq!(bench.page(), 0);
    assert_eq!(bench.byte(0x1000), 1);
    assert_eq!(bench.byte(0x1002), 3);
    // Unset addresses read as zero, like erased flash.
    assert_eq!(bench.byte(0x9999), 0);
    assert_eq!(bench.byte_on(1, 0x1000), 0);

    bench.set_page(1).unwrap();
    bench.write(0x1000, &[9]).unwrap();
    assert_eq!(bench.byte_on(1, 0x1000), 9);
    assert_eq!(bench.byte_on(0, 0x1000), 1);
    assert_eq!(bench.page(), 1);

    assert_eq!(
        bench.requests(),
        [
            Request::SetPage(1),
            Request::Write {
                addr: 0x1000,
                len: 1
            },
        ]
    );
    assert_eq!(MockTransport::default().page(), 0);
}
