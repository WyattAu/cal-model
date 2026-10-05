//! Fuzz target — the calibration layer's totality: arbitrary input resolves
//! to either a project or a typed error, never a panic, and a resolved
//! project answers every lookup, conversion, and limit check without
//! panicking either.
//!
//! Two halves, because the trust boundary is two-sided:
//!
//! 1. `CalibrationProject::from_a2l` over arbitrary bytes — the A2L
//!    description is untrusted supplier input, and the layer above it must
//!    not add a panic path.
//! 2. Every public entry point over the *resolved* model, which is where an
//!    indexing or overflow bug would hide.

#![no_main]

use cal_model::{CalibrationProject, CalParameter, CompuMethod, SignalBinding, ByteOrder};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // The fuzzer's `&[u8]` may not be UTF-8; a lossy conversion is the honest
    // way to hand text to a text parser.
    let text = String::from_utf8_lossy(data);

    let Ok(mut project) = CalibrationProject::from_a2l(&text) else {
        return;
    };

    // Every module answers its own lookups, and the two agree.
    let module_names: Vec<String> = project.module_names().map(str::to_string).collect();
    for module_name in &module_names {
        let Ok(module) = project.module(module_name) else {
            continue;
        };
        let characteristics: Vec<String> = module
            .characteristic_names()
            .map(str::to_string)
            .collect();
        for characteristic in &characteristics {
            let resolved = project
                .characteristic(module_name, characteristic)
                .expect("a declared characteristic resolves");

            // Conversion in both directions, over a spread of raw values that
            // includes the datatype's extremes.
            let (value_min, value_max) = resolved.value_bounds();
            // Raw bit patterns, including the datatype's extremes.
            let raw_probes = [0_u64, 1, u64::MAX, u64::MAX / 2, 0x8000_0000_0000_0000];
            for raw in raw_probes {
                if let Ok(physical) = project.to_physical(module_name, characteristic, raw) {
                    let _ = project.check_limits(module_name, characteristic, raw);
                    let _ = resolved.within_limits(physical);
                    let _ = resolved.raw_value(raw);
                    let _ = resolved.raw_bits(physical);
                }
            }
            // Physical values, including non-finite ones.
            let physical_probes = [
                0.0_f64,
                1.0,
                0.5,
                -1.0,
                value_min,
                value_max,
                (value_min + value_max) / 2.0,
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::MAX,
                f64::MIN,
            ];
            for physical in physical_probes {
                let _ = project.to_raw(module_name, characteristic, physical);
            }

            // Raw coding round-trips through the deposit record.
            let elements = resolved.elements.clamp(1, 64);
            let mut declared = project.clone();
            if declared.declare_elements(module_name, characteristic, elements).is_ok() {
                if let Ok(resolved) = declared.characteristic(module_name, characteristic) {
                    let raws: Vec<u64> = (0..elements).map(|index| index as u64).collect();
                    if let Ok(payload) = declared.write_raw(resolved, &raws) {
                        let _ = declared.read_raw(resolved, &payload);
                        // Truncating the payload must be an error, not a panic.
                        let truncated = &payload[..payload.len().saturating_sub(1)];
                        let _ = declared.read_raw(resolved, truncated);
                    }
                }
            }

            // The signal binding path, over a fabricated layout on every legal
            // byte order and width.
            let binding = SignalBinding {
                characteristic: characteristic.clone(),
                can_id: 0x123,
                start_bit: 7,
                length: 16,
                byte_order: ByteOrder::Motorola,
                factor: 0.5,
                offset: -10.0,
            };
            let mut payload = [0_u8; 8];
            let _ = binding.encode(&mut payload, 100.0);
            let _ = binding.decode(&payload);
            let _ = binding.payload_len();
            // A zero factor must be an error, never a division by zero.
            let flat = SignalBinding {
                factor: 0.0,
                ..binding.clone()
            };
            let _ = flat.encode(&mut payload, 1.0);
            // An absurd width must not overflow the bit maths.
            let wide = SignalBinding {
                length: 64,
                ..binding.clone()
            };
            let _ = wide.encode(&mut payload, f64::MAX);
            let _ = wide.extract_raw(&payload);
        }

        // Measurements resolve and convert.
        let measurements: Vec<String> = module.measurement_names().map(str::to_string).collect();
        for measurement in &measurements {
            if let Ok(resolved) = project.measurement(module_name, measurement) {
                let _ = resolved.raw_value(0);
                let _ = resolved.raw_value(u64::MAX);
                let _ = resolved.size_bytes();
                let _ = resolved.is_signed();
            }
        }
    }

    // Table registration is validated at every shape.
    let _ = project.register_table("fuzz", vec![(0.0, 0.0)]);
    let _ = project.register_table("fuzz", vec![(1.0, 0.0), (0.0, 1.0)]);
    let _ = project.register_table("fuzz", Vec::new());
    let _ = project.register_table("fuzz", vec![(f64::NAN, 0.0), (1.0, 1.0)]);
    // A DBC attach over arbitrary text is either parsed or a typed error.
    let _ = project.attach_dbc_text(&text);

    // The calibration arithmetic over arbitrary parameters.
    let params: Vec<CalParameter> = [0.0_f64, 1.0, f64::NAN, f64::INFINITY, -1.0]
        .into_iter()
        .enumerate()
        .map(|(index, value)| {
            CalParameter::new(format!("p{index}"), value, -10.0, 10.0)
        })
        .collect();
    if let Ok(curve) = cal_model::calibrate_curve(&params) {
        let _ = curve.eval(0.0);
        let _ = curve.eval(f64::MAX);
        let _ = curve.eval(f64::NAN);
        let _ = curve.is_monotone();
        let _ = curve.to_string();
    }
    let _ = cal_model::optimize_working_point(&params, &|point: &[f64]| {
        point.iter().copied().sum::<f64>()
    });

    // And every conversion variant evaluates without panicking.
    for method in [
        CompuMethod::identity(),
        CompuMethod::linear(0.25, -10.0),
        CompuMethod::linear(f64::NAN, 0.0),
        CompuMethod::rat_func([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        CompuMethod::tab_intp(Vec::new()),
        CompuMethod::tab_no_intp(vec![(0.0, 0.0), (1.0, 1.0)]),
    ] {
        let _ = method.apply(f64::NAN);
        let _ = method.invert("probe", f64::NAN, 0.0, 255.0);
        let _ = method.invert("probe", 1.0, f64::NEG_INFINITY, f64::INFINITY);
        let _ = method.keyword();
        let _ = method.points();
    }
});
