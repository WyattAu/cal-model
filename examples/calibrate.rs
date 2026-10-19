//! A full calibration cycle against an in-memory bench ECU: inventory the
//! project, print every characteristic with its limits, run a
//! read-modify-write adjustment, snapshot before and after, and print the
//! diff report.
//!
//! ```sh
//! # the embedded demo project
//! cargo run --example calibrate
//! # a real description, optionally with its CAN database
//! cargo run --example calibrate -- path/to/powertrain.a2l [path/to/powertrain.dbc]
//! ```
//!
//! The example exercises exactly what a calibration engineer tool calls:
//! [`CalibrationProject`] to resolve the description, [`CalibrationSession`]
//! to drive the ECU, and the snapshot/diff pair to report what changed.
//!
//! Test file: `unwrap` is deliberate — this is a demo, not production code.

use cal_model::mock::MockTransport;
use cal_model::{
    diff_snapshots, render_deltas, CalibrationProject, CalibrationSession, ResourceMode,
};
use std::path::Path;
use std::process::ExitCode;

/// The demo adjustment: the idle target goes from whatever the ECU holds to
/// 950 rpm.
const TARGET_IDLE_RPM: f64 = 950.0;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("calibration cycle failed: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Resolve the project from the command line, or fall back to the embedded
/// demo.
fn run() -> Result<(), cal_model::CalError> {
    let mut arguments = std::env::args().skip(1);
    let project = match arguments.next() {
        Some(path) => {
            let mut project = CalibrationProject::from_a2l_file(Path::new(&path))?;
            // `a2l_parse` skips the COMPU_TAB blocks and does not retain the
            // element counts, so a project loaded from a file needs both
            // supplied before its TABLE conversions resolve. A host with its
            // own source for either would call `register_table` /
            // `declare_elements` directly.
            cal_model::complete(&mut project)?;
            if let Some(dbc) = arguments.next() {
                project.attach_dbc_text(&std::fs::read_to_string(&dbc).map_err(|error| {
                    cal_model::CalError::Io {
                        path: dbc.clone(),
                        detail: error.to_string(),
                    }
                })?)?;
                println!("bound CAN database: {dbc}\n");
            } else {
                println!("no CAN database given; characteristic bindings are absent\n");
            }
            project
        }
        None => {
            println!("no A2L file given; using the embedded demo project\n");
            cal_model::sample_project()?
        }
    };

    print_inventory(&project)?;

    // Seed the bench with a plausible factory calibration.
    let bench = MockTransport::seeded(&cal_model::seed_memory());
    let mut session =
        CalibrationSession::connect(&project, Box::new(bench), ResourceMode::CAL_PAGE_OR_DAQ)?;
    println!(
        "connected: resources {}, page {}\n",
        session.resources(),
        session.cal_page()
    );

    // ---- read-modify-write -------------------------------------------------
    let names = ["idle_target_rpm", "eng_torque_max", "boost_curve"];
    let before = session.snapshot(&names)?;
    print!("{}", before);
    println!();

    let observed = session.read_characteristic("engine", "idle_target_rpm")?;
    println!("idle target as found: {observed:.1} rpm");
    println!("writing {TARGET_IDLE_RPM:.1} rpm …");
    session.write_characteristic("engine", "idle_target_rpm", TARGET_IDLE_RPM)?;
    let confirmed = session.read_characteristic("engine", "idle_target_rpm")?;
    println!(
        "read back {confirmed:.1} rpm (delta {:+.1})\n",
        confirmed - observed
    );

    // A write the ECU must refuse: outside the A2L limits.
    match session.write_characteristic("engine", "idle_target_rpm", 4000.0) {
        Ok(()) => println!("unexpected: 4000 rpm was accepted"),
        Err(error) => println!("rejected as expected: {error}\n"),
    }

    // ---- page switching ----------------------------------------------------
    session.set_cal_page(1)?;
    let alt = session.read_characteristic("engine", "idle_target_rpm")?;
    println!("page 1 idle target (erased): {alt:.1} rpm");
    session.set_cal_page(0)?;
    let back = session.read_characteristic("engine", "idle_target_rpm")?;
    println!("page 0 idle target (intact): {back:.1} rpm\n");

    // ---- diff --------------------------------------------------------------
    let after = session.snapshot(&names)?;
    println!("diff report");
    println!(
        "{}",
        render_deltas(&diff_snapshots(Some(&before), &after, &project))
    );

    // ---- curve calibration -------------------------------------------------
    let nodes = [
        cal_model::CalParameter::new("idle_700", 12.0, 650.0, 750.0),
        cal_model::CalParameter::new("idle_800", 14.5, 750.0, 850.0),
        cal_model::CalParameter::new("idle_900", 16.0, 850.0, 950.0),
        cal_model::CalParameter::new("idle_1000", 17.0, 950.0, 1050.0),
    ];
    match cal_model::calibrate_curve(&nodes) {
        Ok(curve) => {
            print!("{curve}");
            println!("curve at 875 rpm: {:.3}\n", curve.eval(875.0));
        }
        Err(error) => println!("curve fit refused the nodes: {error}\n"),
    }

    Ok(())
}

/// Print every module's characteristics and measurements with their limits.
fn print_inventory(project: &CalibrationProject) -> Result<(), cal_model::CalError> {
    println!("project: {}", project.project_name());
    for module in project.modules() {
        println!(
            "\nmodule {} — {} characteristic(s), {} measurement(s)",
            module.name,
            module.characteristics.len(),
            module.measurements.len()
        );

        println!(
            "  {:<20} {:<8} {:<10} {:>8} {:>12} {:>12} {:>10}",
            "characteristic", "type", "conversion", "address", "lower", "upper", "elements"
        );
        for characteristic in &module.characteristics {
            let limits = if characteristic.has_limits() {
                format!(
                    "{:.3} {:>12.3}",
                    characteristic.lower_limit, characteristic.upper_limit
                )
            } else {
                "     (none)        —".to_string()
            };
            println!(
                "  {:<20} {:<8} {:<10} {:#010x} {} {:>10}",
                characteristic.name,
                characteristic.kind.keyword(),
                characteristic.conversion.keyword(),
                characteristic.address,
                limits,
                characteristic.elements,
            );
        }

        println!(
            "  {:<20} {:<8} {:<10} {:>10} {:>8}",
            "measurement", "datatype", "conversion", "address", "bytes"
        );
        for measurement in &module.measurements {
            let measurement = project.measurement(&module.name, &measurement.name)?;
            println!(
                "  {:<20} {:<8} {:<10} {:#010x} {:>8}",
                measurement.name,
                measurement.datatype.keyword(),
                measurement.conversion.keyword(),
                measurement.address,
                measurement.size_bytes(),
            );
        }
    }
    println!();
    Ok(())
}
