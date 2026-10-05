//! Calibration fitting: monotone piecewise-linear curves and bounded
//! working-point optimisation.
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
    calibrate_curve, optimal_point, optimize_working_point, CalError, CalParameter, Curve,
};

/// A node of a curve: `value` is the ordinate, `lower`/`upper` the abscissa
/// interval it was measured over (the midpoint is the node's abscissa).
fn node(name: &str, value: f64, midpoint: f64) -> CalParameter {
    CalParameter::new(name, value, midpoint - 1.0, midpoint + 1.0)
}

#[test]
fn a_parameter_reports_its_midpoint_and_clamps() {
    let p = CalParameter::new("idle", 42.0, 700.0, 900.0);
    assert_eq!(p.name, "idle");
    assert_eq!(p.midpoint(), 800.0);
    assert_eq!(p.clamp(500.0), 700.0);
    assert_eq!(p.clamp(1000.0), 900.0);
    assert_eq!(p.clamp(800.0), 800.0);
    assert!(p
        .to_string()
        .contains("idle = 42.000000 in [700.000000, 900.000000]"));
}

#[test]
fn calibrate_curve_recovers_synthetic_monotone_data() {
    // y = x²  sampled at five operating points; the fit must reproduce every
    // node exactly and interpolate linearly between them.
    let xs = [0.0_f64, 10.0, 20.0, 30.0, 40.0];
    let params: Vec<CalParameter> = xs
        .iter()
        .enumerate()
        .map(|(index, &x)| node(&format!("n{index}"), x * x, x))
        .collect();

    let curve = calibrate_curve(&params).unwrap();
    assert_eq!(curve.len(), 5);
    assert!(curve.is_monotone());
    assert!(!curve.is_empty());

    // Every node is recovered within 1e-6.
    for (index, &x) in xs.iter().enumerate() {
        let error = (curve.eval(x) - x * x).abs();
        assert!(error < 1e-6, "node {index}: error {error}");
    }

    // And the maximum deviation over a dense sweep of the tabulated domain is
    // also under 1e-6 at the nodes themselves — the interpolation error
    // between them is the fit's documented linear approximation.
    for step in 0..=40 {
        let x = step as f64;
        let fitted = curve.eval(x);
        assert!((0.0..=1600.0).contains(&fitted), "{x} → {fitted}");
    }
    assert!((curve.eval(40.0) - 1600.0).abs() < 1e-6);
}

#[test]
fn calibrate_curve_interpolates_linearly_between_nodes() {
    // y = 2x + 1 is *linear*, so the piecewise fit reproduces it exactly
    // everywhere — that is the tightest statement the fit can make.
    let params: Vec<CalParameter> = [0.0_f64, 5.0, 10.0, 15.0]
        .iter()
        .enumerate()
        .map(|(index, &x)| node(&format!("n{index}"), 2.0 * x + 1.0, x))
        .collect();
    let curve = calibrate_curve(&params).unwrap();
    for step in 0..=150 {
        let x = step as f64 / 10.0;
        assert!((curve.eval(x) - (2.0 * x + 1.0)).abs() < 1e-9, "x = {x}");
    }
}

#[test]
fn calibrate_curve_reproduces_every_node_exactly() {
    let params: Vec<CalParameter> = [0.0_f64, 2.0, 5.0, 9.0]
        .iter()
        .enumerate()
        .map(|(index, &x)| node(&format!("n{index}"), x * 3.0 + 7.0, x))
        .collect();
    let curve = calibrate_curve(&params).unwrap();
    for (index, &x) in [0.0_f64, 2.0, 5.0, 9.0].iter().enumerate() {
        assert!(
            (curve.eval(x) - (x * 3.0 + 7.0)).abs() < 1e-6,
            "node {index}"
        );
    }
}

#[test]
fn calibrate_curve_sorts_nodes_by_abscissa() {
    // Deliberately out of order: the fit orders by midpoint, not by input.
    let params = vec![
        node("late", 30.0, 30.0),
        node("early", 10.0, 10.0),
        node("middle", 20.0, 20.0),
    ];
    let curve = calibrate_curve(&params).unwrap();
    let points = curve.points();
    assert_eq!(points.len(), 3);
    assert!((points[0].0 - 10.0).abs() < 1e-12);
    assert!((points[1].0 - 20.0).abs() < 1e-12);
    assert!((points[2].0 - 30.0).abs() < 1e-12);
    assert!(curve.is_monotone());
}

#[test]
fn calibrate_curve_accepts_a_monotone_decreasing_series() {
    let params: Vec<CalParameter> = [0.0_f64, 10.0, 20.0]
        .iter()
        .enumerate()
        .map(|(index, &x)| node(&format!("n{index}"), 100.0 - x, x))
        .collect();
    let curve = calibrate_curve(&params).unwrap();
    assert!(curve.is_monotone());
    assert!((curve.eval(0.0) - 100.0).abs() < 1e-9);
    assert!((curve.eval(20.0) - 80.0).abs() < 1e-9);
}

#[test]
fn calibrate_curve_accepts_a_flat_series() {
    let params: Vec<CalParameter> = [0.0_f64, 1.0, 2.0]
        .iter()
        .enumerate()
        .map(|(index, &x)| node(&format!("n{index}"), 5.0, x))
        .collect();
    let curve = calibrate_curve(&params).unwrap();
    assert!(curve.is_monotone());
    assert!((curve.eval(1.0) - 5.0).abs() < 1e-12);
}

#[test]
fn calibrate_curve_rejects_a_non_monotone_series() {
    let params = vec![
        node("a", 10.0, 0.0),
        node("b", 20.0, 10.0), // rising
        node("c", 15.0, 20.0), // falling — the turn
    ];
    let error = calibrate_curve(&params).unwrap_err();
    assert_eq!(
        error,
        CalError::NonMonotone {
            name: "c".to_string(),
            value: 15.0,
            previous: 20.0,
        }
    );
    assert!(error.to_string().contains("non-monotone curve at `c`"));
}

#[test]
fn calibrate_curve_rejects_a_turn_upwards_too() {
    let params = vec![
        node("a", 20.0, 0.0),
        node("b", 10.0, 10.0), // falling
        node("c", 15.0, 20.0), // rising
    ];
    let error = calibrate_curve(&params).unwrap_err();
    assert!(
        matches!(&error, CalError::NonMonotone { name, value, previous }
            if name == "c" && *value == 15.0 && *previous == 10.0),
        "{error}"
    );
}

#[test]
fn calibrate_curve_needs_at_least_two_nodes() {
    let error = calibrate_curve(&[]).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("0 point(s)")),
        "{error}"
    );
    let one = calibrate_curve(&[node("a", 1.0, 0.0)]).unwrap_err();
    assert!(
        matches!(&one, CalError::Unsupported { reason, .. } if reason.contains("1 point(s)")),
        "{one}"
    );
    assert!(calibrate_curve(&[node("a", 1.0, 0.0), node("b", 2.0, 1.0)]).is_ok());
}

#[test]
fn calibrate_curve_rejects_two_nodes_at_the_same_abscissa() {
    let params = vec![node("a", 1.0, 5.0), node("b", 2.0, 5.0)];
    let error = calibrate_curve(&params).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("share the abscissa 5")),
        "{error}"
    );
}

#[test]
fn calibrate_curve_rejects_degenerate_and_non_finite_bounds() {
    let inverted = CalParameter::new("bad", 1.0, 10.0, 5.0);
    let error = calibrate_curve(&[inverted, node("b", 2.0, 20.0)]).unwrap_err();
    assert!(
        matches!(&error, CalError::OutOfBounds { name, lower, upper, .. }
            if name == "bad" && *lower == 10.0 && *upper == 5.0),
        "{error}"
    );

    let infinite = CalParameter::new("inf", f64::INFINITY, 0.0, 1.0);
    let error = calibrate_curve(&[infinite, node("b", 2.0, 20.0)]).unwrap_err();
    assert!(
        matches!(&error, CalError::Unsupported { reason, .. } if reason.contains("finite")),
        "{error}"
    );

    let nan_bounds = CalParameter::new("nan", 1.0, f64::NAN, 1.0);
    assert!(calibrate_curve(&[nan_bounds, node("b", 2.0, 20.0)]).is_err());
}

#[test]
fn a_curve_clamps_outside_its_domain_and_renders() {
    let curve = calibrate_curve(&[node("a", 0.0, 0.0), node("b", 10.0, 10.0)]).unwrap();
    assert!((curve.eval(-100.0) - 0.0).abs() < 1e-12);
    assert!((curve.eval(1000.0) - 10.0).abs() < 1e-12);
    assert!((curve.eval(2.5) - 2.5).abs() < 1e-12);
    let rendered = curve.to_string();
    assert!(
        rendered.contains("calibration curve (2 points)"),
        "{rendered}"
    );
    assert!(rendered.contains("x =       0.0000"), "{rendered}");
    assert!(rendered.contains("y =     0.000000"), "{rendered}");
    assert!(rendered.contains("x =      10.0000"), "{rendered}");
}

#[test]
fn curve_construction_validates_its_points() {
    assert!(Curve::new(vec![(0.0, 0.0)]).is_err());
    assert!(Curve::new(Vec::new()).is_err());
    assert!(Curve::new(vec![(1.0, 0.0), (0.0, 1.0)]).is_err());
    assert!(Curve::new(vec![(0.0, 0.0), (0.0, 1.0)]).is_err());
    assert!(Curve::new(vec![(0.0, f64::NAN), (1.0, 1.0)]).is_err());
    assert!(Curve::new(vec![(0.0, 0.0), (1.0, 1.0)]).is_ok());
    assert!(Curve::new(vec![(0.0, 1.0), (1.0, 0.0)]).is_ok());

    // A zigzag curve is constructible but reports itself as non-monotone.
    let zigzag = Curve::new(vec![(0.0, 0.0), (1.0, 5.0), (2.0, 1.0)]).unwrap();
    assert!(!zigzag.is_monotone());
}

#[test]
fn optimize_working_point_finds_a_known_bounded_maximum() {
    // f(x, y) = -(x² + (y - 3)²) over [-10, 10]² has its maximum of 0 at
    // (0, 3).
    let params = vec![
        CalParameter::new("x", 0.0, -10.0, 10.0),
        CalParameter::new("y", 0.0, -10.0, 10.0),
    ];
    let best = optimize_working_point(&params, &|point: &[f64]| {
        -(point[0] * point[0] + (point[1] - 3.0) * (point[1] - 3.0))
    })
    .unwrap();
    assert!((best - 0.0).abs() < 1e-6, "best = {best}");

    let (point, value) = optimal_point(&params, &|p: &[f64]| {
        -(p[0] * p[0] + (p[1] - 3.0) * (p[1] - 3.0))
    })
    .unwrap();
    assert!((point[0] - 0.0).abs() < 1e-4, "x = {}", point[0]);
    assert!((point[1] - 3.0).abs() < 1e-4, "y = {}", point[1]);
    assert!((value - 0.0).abs() < 1e-6);
}

#[test]
fn optimize_working_point_finds_a_corner_maximum() {
    // A monotone objective: the maximum is at the upper corner.
    let params = vec![
        CalParameter::new("a", 0.0, 0.0, 5.0),
        CalParameter::new("b", 0.0, 0.0, 7.0),
    ];
    let best = optimize_working_point(&params, &|p: &[f64]| 3.0 * p[0] + 2.0 * p[1]).unwrap();
    assert!((best - (3.0 * 5.0 + 2.0 * 7.0)).abs() < 1e-6, "{best}");
}

#[test]
fn optimize_working_point_finds_a_lower_corner_maximum() {
    // Negative slope in both coordinates: the maximum is at the lower corner.
    let params = vec![CalParameter::new("a", 0.0, 2.0, 5.0)];
    let best = optimize_working_point(&params, &|p: &[f64]| -p[0]).unwrap();
    assert!((best - (-2.0)).abs() < 1e-6, "{best}");
}

#[test]
fn optimize_working_point_never_leaves_the_box() {
    let cases: Vec<(&str, Vec<CalParameter>)> = vec![
        (
            "quadratic bowl",
            vec![
                CalParameter::new("a", 0.0, -3.0, 4.0),
                CalParameter::new("b", 0.0, 10.0, 20.0),
            ],
        ),
        (
            "single point box",
            vec![CalParameter::new("a", 7.0, 7.0, 7.0)],
        ),
        (
            "reversed bounds",
            vec![CalParameter::new("a", 0.0, -100.0, -50.0)],
        ),
        ("wide range", vec![CalParameter::new("a", 0.0, -1e6, 1e6)]),
    ];
    for (name, params) in cases {
        let (point, _) = optimal_point(&params, &|p: &[f64]| {
            // A deliberately awkward surface with many local features.
            (p[0] * p[0]).sin() + (p[0] * 0.5).cos()
        })
        .unwrap();
        for (index, param) in params.iter().enumerate() {
            let value = point[index];
            assert!(
                value >= param.lower - 1e-9 && value <= param.upper + 1e-9,
                "{name}: {value} outside [{}, {}]",
                param.lower,
                param.upper
            );
        }
    }
}

#[test]
fn optimize_working_point_never_returns_outside_the_box_for_the_value_either() {
    // A 1-D identity objective: the maximum is exactly the upper bound.
    let params = vec![CalParameter::new("a", 0.0, -5.0, 12.0)];
    let best = optimize_working_point(&params, &|p: &[f64]| p[0]).unwrap();
    assert!((best - 12.0).abs() < 1e-9, "{best}");
    assert!(best >= params[0].lower && best <= params[0].upper);

    // And the reversed objective: maximising -x picks the lower bound, so the
    // returned value is +5.
    let flipped = optimize_working_point(&params, &|p: &[f64]| -p[0]).unwrap();
    assert!((flipped - 5.0).abs() < 1e-9, "{flipped}");
    assert!(flipped >= params[0].lower && flipped <= params[0].upper);
}

#[test]
fn optimize_working_point_reports_non_convergence_with_an_iteration_count() {
    // An objective that never yields a finite value cannot be descended, and
    // the crate reports that rather than returning -inf as an answer.
    let params = vec![CalParameter::new("a", 0.0, 0.0, 1.0)];
    let error = optimize_working_point(&params, &|_: &[f64]| f64::NAN).unwrap_err();
    match error {
        CalError::Convergence { iterations } => assert!(iterations >= 1, "{iterations}"),
        other => panic!("expected Convergence, got {other}"),
    }
    assert!(optimize_working_point(&params, &|_: &[f64]| f64::INFINITY).is_err());
}

#[test]
fn optimize_working_point_handles_a_constant_objective() {
    let params = vec![CalParameter::new("a", 0.0, 0.0, 10.0)];
    let best = optimize_working_point(&params, &|_: &[f64]| 42.0).unwrap();
    assert!((best - 42.0).abs() < 1e-12);
}

#[test]
fn optimize_working_point_handles_a_three_dimensional_problem() {
    // Maximum of -(x² + y² + z²) at the origin, over an off-centre box.
    let params = vec![
        CalParameter::new("x", 5.0, 1.0, 9.0),
        CalParameter::new("y", 5.0, 1.0, 9.0),
        CalParameter::new("z", 5.0, 1.0, 9.0),
    ];
    let (point, value) = optimal_point(&params, &|p: &[f64]| {
        -(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
    })
    .unwrap();
    for coordinate in &point {
        assert!((coordinate - 1.0).abs() < 1e-4, "{coordinate}");
    }
    assert!((value + 3.0).abs() < 1e-6, "{value}");
}

#[test]
fn optimize_working_point_is_deterministic() {
    let params = vec![
        CalParameter::new("a", 0.0, -5.0, 5.0),
        CalParameter::new("b", 0.0, -5.0, 5.0),
    ];
    let objective = |p: &[f64]| -(p[0] * p[0] + 2.0 * (p[1] - 1.0).powi(2));
    let first = optimal_point(&params, &objective).unwrap();
    for _ in 0..5 {
        let again = optimal_point(&params, &objective).unwrap();
        assert_eq!(first, again, "the same inputs must give the same answer");
    }
}

#[test]
fn optimize_working_point_rejects_an_empty_or_degenerate_parameter_set() {
    let empty: Vec<CalParameter> = Vec::new();
    assert!(
        matches!(
            optimal_point(&empty, &|_: &[f64]| 0.0),
            Err(CalError::Unsupported { .. })
        ),
        "empty parameter set"
    );

    let inverted = vec![CalParameter::new("a", 0.0, 5.0, 1.0)];
    let error = optimal_point(&inverted, &|_: &[f64]| 0.0).unwrap_err();
    assert!(matches!(error, CalError::OutOfBounds { .. }), "{error}");

    let infinite = vec![CalParameter::new("a", 0.0, f64::NEG_INFINITY, 1.0)];
    assert!(optimal_point(&infinite, &|_: &[f64]| 0.0).is_err());
}

#[test]
fn optimize_working_point_respects_a_degenerate_single_point_box() {
    let params = vec![CalParameter::new("a", 3.0, 3.0, 3.0)];
    let best = optimize_working_point(&params, &|p: &[f64]| p[0] * p[0]).unwrap();
    assert!((best - 9.0).abs() < 1e-12, "{best}");
}

#[test]
fn optimize_working_point_multi_start_finds_a_secondary_basin() {
    // A surface whose global maximum is at a corner and whose midpoint is a
    // local ridge — multi-start is what makes the corner findable.
    let params = vec![CalParameter::new("a", 0.0, 0.0, 100.0)];
    let best = optimize_working_point(&params, &|p: &[f64]| {
        // Maximised at x = 100; the midpoint 50 is a shallow local flat.
        let x = p[0];
        if x > 90.0 {
            1000.0 - (100.0 - x)
        } else {
            x
        }
    })
    .unwrap();
    assert!(best > 900.0, "the corner basin was missed: {best}");
}

#[test]
fn a_realistic_idle_gain_search_converges() {
    // The shape a calibration engineer actually searches: maximise a fuel
    // economy proxy subject to an idle-speed and a torque bound.
    let params = vec![
        CalParameter::new("idle_gain", 1.0, 0.5, 1.5),
        CalParameter::new("torque_cap", 100.0, 50.0, 200.0),
    ];
    let best = optimize_working_point(&params, &|p: &[f64]| {
        let gain = p[0];
        let torque = p[1];
        // Reward a high gain, penalise torque beyond the sweet spot, and
        // penalise the corners.
        gain * 100.0 - (torque - 120.0).abs() - gain * gain * 20.0
    })
    .unwrap();
    assert!(best > 0.0, "{best}");
    let (point, _) = optimal_point(&params, &|p: &[f64]| {
        let gain = p[0];
        let torque = p[1];
        gain * 100.0 - (torque - 120.0).abs() - gain * gain * 20.0
    })
    .unwrap();
    assert!(point[0] >= 0.5 && point[0] <= 1.5, "gain {}", point[0]);
    assert!(point[1] >= 50.0 && point[1] <= 200.0, "torque {}", point[1]);
    assert!(
        (point[1] - 120.0).abs() < 1e-3,
        "torque settled at {}",
        point[1]
    );
}
