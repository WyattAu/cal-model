//! Calibration fitting: monotone piecewise-linear curves and bounded
//! working-point optimisation.
//!
//! These are the two computations a calibration engineer reaches for when
//! the ECU will not tell them the answer — a measured set of points that
//! must become a curve the ECU can interpolate, and a set of adjustable
//! parameters whose best combination has to be found inside their limits.
//!
//! # Curve nodes are parameter *intervals*
//!
//! A [`CalParameter`] is one node of a curve under construction: `value` is
//! the ordinate (the calibrated output at that node) and `lower`/`upper`
//! are the abscissa bounds of the operating condition it was measured at —
//! "`idle` between 700 and 900 rpm". The node's abscissa is the interval's
//! midpoint, which is what makes the fit reproducible from the measurement
//! record rather than from an implicit index order. Nodes are sorted by
//! abscissa before the fit, so the caller may pass them in any order.
//!
//! # Monotonicity is enforced, not assumed
//!
//! A calibration curve whose ordinates change direction makes the ECU's
//! interpolation physically wrong at the turn, so [`calibrate_curve`]
//! rejects it with [`CalError::NonMonotone`] rather than fitting a curve
//! that lies. Both directions are accepted — a gain that falls with
//! temperature is as legitimate as one that rises with load.
//!
//! # The optimiser is deterministic
//!
//! [`optimize_working_point`] is bounded coordinate descent from a fixed
//! set of starting points (each parameter's midpoint and the box corners),
//! halving the step on every sweep without improvement. Same inputs, same
//! answer, every run — which is the only acceptable behaviour for something
//! that ends up in an ECU's flash. A non-finite objective is reported as
//! [`CalError::Convergence`] carrying the iteration count, never silently
//! returning a number from a diverged search.

use crate::error::CalError;
use std::fmt;

/// Sweep budget for coordinate descent before the optimiser gives up.
const MAX_SWEEPS: u32 = 400;

/// Relative step below which a sweep counts as converged.
const STEP_TOLERANCE: f64 = 1e-9;

/// One node of a curve under construction, or one adjustable of an
/// optimisation.
#[derive(Debug, Clone, PartialEq)]
pub struct CalParameter {
    /// What this node is — the operating condition or the parameter name.
    pub name: String,
    /// For a curve node, the ordinate (the calibrated value). For an
    /// adjustable, the starting point.
    pub value: f64,
    /// Lower bound: the abscissa interval's start (a curve node) or the
    /// parameter's calibration limit.
    pub lower: f64,
    /// Upper bound: the abscissa interval's end (a curve node) or the
    /// parameter's calibration limit.
    pub upper: f64,
}

impl CalParameter {
    /// A parameter with a name, a value, and bounds.
    #[must_use]
    pub fn new(name: impl Into<String>, value: f64, lower: f64, upper: f64) -> Self {
        Self {
            name: name.into(),
            value,
            lower,
            upper,
        }
    }

    /// The abscissa of a curve node: the midpoint of its interval.
    #[must_use]
    pub fn midpoint(&self) -> f64 {
        (self.lower + self.upper) / 2.0
    }

    /// Clamp `x` into this parameter's bounds.
    #[must_use]
    pub fn clamp(&self, x: f64) -> f64 {
        if x < self.lower {
            self.lower
        } else if x > self.upper {
            self.upper
        } else {
            x
        }
    }
}

impl fmt::Display for CalParameter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} = {:.6} in [{:.6}, {:.6}]",
            self.name, self.value, self.lower, self.upper
        )
    }
}

/// A monotone piecewise-linear calibration curve.
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    points: Vec<(f64, f64)>,
}

impl Curve {
    /// Build a curve from `(abscissa, ordinate)` points.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when fewer than two points are given, when
    /// the abscissae are not strictly ascending, or when any point is
    /// non-finite.
    pub fn new(points: Vec<(f64, f64)>) -> Result<Self, CalError> {
        if points.len() < 2 {
            return Err(CalError::Unsupported {
                subject: "calibration curve".to_string(),
                reason: format!("{} point(s) given, at least 2 are required", points.len()),
            });
        }
        if points.iter().any(|&(x, y)| !x.is_finite() || !y.is_finite()) {
            return Err(CalError::Unsupported {
                subject: "calibration curve".to_string(),
                reason: "points must be finite".to_string(),
            });
        }
        if points.windows(2).any(|w| !(w[0].0 < w[1].0)) {
            return Err(CalError::Unsupported {
                subject: "calibration curve".to_string(),
                reason: "abscissae must be strictly ascending".to_string(),
            });
        }
        Ok(Self { points })
    }

    /// The fitted points, ascending by abscissa.
    #[must_use]
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// Number of nodes.
    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// `true` when the curve carries no nodes (never true for a curve built
    /// through [`Curve::new`]).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// `true` when the ordinates never change direction.
    #[must_use]
    pub fn is_monotone(&self) -> bool {
        let mut rising = false;
        let mut falling = false;
        for pair in self.points.windows(2) {
            if pair[1].1 > pair[0].1 {
                rising = true;
            } else if pair[1].1 < pair[0].1 {
                falling = true;
            }
        }
        !(rising && falling)
    }

    /// Evaluate the curve at `x`: linear interpolation between the
    /// bracketing nodes, clamped to the end nodes outside the domain (the
    /// ASAP2 `TAB_INTP` convention).
    #[must_use]
    pub fn eval(&self, x: f64) -> f64 {
        let Some((&first_x, &first_y)) = self.points.first() else {
            return f64::NAN;
        };
        if x <= first_x {
            return first_y;
        }
        let Some((&last_x, &last_y)) = self.points.last() else {
            return f64::NAN;
        };
        if x >= last_x {
            return last_y;
        }
        for pair in self.points.windows(2) {
            let (x0, y0) = pair[0];
            let (x1, y1) = pair[1];
            if x >= x0 && x <= x1 {
                let span = x1 - x0;
                if span == 0.0 {
                    return y0;
                }
                return y0 + (y1 - y0) * (x - x0) / span;
            }
        }
        last_y
    }
}

impl fmt::Display for Curve {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "calibration curve ({} points):", self.points.len())?;
        for &(x, y) in &self.points {
            writeln!(f, "  x = {x:>12.4}   y = {y:>12.6}")?;
        }
        Ok(())
    }
}

/// Fit a monotone piecewise-linear curve through the given nodes.
///
/// Nodes are ordered by their abscissa (the midpoint of `lower..upper`) and
/// the ordinates must not change direction. The fit reproduces every node
/// exactly and interpolates linearly between them — which is precisely what
/// an ECU's `TAB_INTP` conversion does, so what this returns is what the ECU
/// will compute.
///
/// # Errors
///
/// [`CalError::Unsupported`] for fewer than two nodes, non-finite bounds, or
/// two nodes sharing an abscissa; [`CalError::OutOfBounds`] when a node's
/// `lower` exceeds its `upper`; [`CalError::NonMonotone`] when the ordinates
/// change direction.
pub fn calibrate_curve(params: &[CalParameter]) -> Result<Curve, CalError> {
    let mut nodes: Vec<(f64, &CalParameter)> = Vec::with_capacity(params.len());
    for param in params {
        validate_bounds(param)?;
        nodes.push((param.midpoint(), param));
    }
    nodes.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.1.name.cmp(&b.1.name))
    });
    for pair in nodes.windows(2) {
        if pair[0].0 == pair[1].0 {
            return Err(CalError::Unsupported {
                subject: "calibration curve".to_string(),
                reason: format!(
                    "nodes `{}` and `{}` share the abscissa {}",
                    pair[0].1.name, pair[1].1.name, pair[0].0
                ),
            });
        }
    }
    let mut points: Vec<(f64, f64)> = Vec::with_capacity(nodes.len());
    let mut direction = 0_i8;
    for (abscissa, param) in &nodes {
        if let Some(&(_, previous)) = points.last() {
            if param.value > previous && direction < 0 {
                return Err(CalError::NonMonotone {
                    name: param.name.clone(),
                    value: param.value,
                    previous,
                });
            }
            if param.value < previous && direction > 0 {
                return Err(CalError::NonMonotone {
                    name: param.name.clone(),
                    value: param.value,
                    previous,
                });
            }
            if param.value > previous {
                direction = 1;
            } else if param.value < previous {
                direction = -1;
            }
        }
        points.push((*abscissa, param.value));
    }
    Curve::new(points)
}

/// Check a parameter's bounds are usable.
fn validate_bounds(param: &CalParameter) -> Result<(), CalError> {
    if !param.lower.is_finite() || !param.upper.is_finite() || !param.value.is_finite() {
        return Err(CalError::Unsupported {
            subject: format!("parameter `{}`", param.name),
            reason: "value and bounds must be finite".to_string(),
        });
    }
    if param.lower > param.upper {
        return Err(CalError::OutOfBounds {
            name: param.name.clone(),
            value: param.lower,
            lower: param.lower,
            upper: param.upper,
        });
    }
    Ok(())
}

/// Maximise `objective` over the box defined by `params`, returning the
/// point and its value.
///
/// Coordinate descent from each parameter's midpoint and from the box
/// corners, halving the step after every sweep without improvement, until
/// the step falls below [`STEP_TOLERANCE`] or [`MAX_SWEEPS`] sweeps pass.
/// The returned point is always inside the box.
///
/// # Errors
///
/// [`CalError::OutOfBounds`] for an empty parameter set or a degenerate
/// bound, [`CalError::Unsupported`] for non-finite bounds,
/// [`CalError::Convergence`] when the objective never yields a finite value.
pub fn optimal_point(
    params: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
) -> Result<(Vec<f64>, f64), CalError> {
    if params.is_empty() {
        return Err(CalError::Unsupported {
            subject: "optimiser".to_string(),
            reason: "no parameters to optimise".to_string(),
        });
    }
    for param in params {
        validate_bounds(param)?;
    }
    let n = params.len();
    let widths: Vec<f64> = params.iter().map(|p| p.upper - p.lower).collect();

    // Deterministic multi-start: the midpoint, then every corner of the box
    // for small problems, so a descent that would otherwise stall in a
    // corner's basin starts from that corner too.
    let mut starts: Vec<Vec<f64>> = vec![params.iter().map(CalParameter::midpoint).collect()];
    if n <= 8 {
        for corner in 0..n {
            for high_corner in [true, false] {
                let mut point: Vec<f64> = params
                    .iter()
                    .map(|p| if high_corner { p.lower } else { p.upper })
                    .collect();
                let Some(param) = params.get(corner) else {
                    continue;
                };
                let opposite = if high_corner { param.upper } else { param.lower };
                if let Some(slot) = point.get_mut(corner) {
                    *slot = opposite;
                }
                starts.push(point);
            }
        }
    }

    let mut best: Option<(Vec<f64>, f64)> = None;
    let mut worst_sweeps = 0_u32;
    for start in starts {
        match descend(params, &widths, objective, start) {
            Ok(found) => {
                let replace = best.as_ref().is_none_or(|(_, value)| found.1 > *value);
                if replace {
                    best = Some(found);
                }
            }
            Err(sweeps) => worst_sweeps = worst_sweeps.max(sweeps),
        }
    }
    best.ok_or(CalError::Convergence {
        iterations: worst_sweeps.max(1),
    })
}

/// One coordinate-descent run from `start`.
///
/// `Ok` carries the point and its value; `Err` carries the number of sweeps
/// the run consumed before giving up.
fn descend(
    params: &[CalParameter],
    widths: &[f64],
    objective: &dyn Fn(&[f64]) -> f64,
    mut point: Vec<f64>,
) -> Result<(Vec<f64>, f64), u32> {
    for (index, param) in params.iter().enumerate() {
        if let Some(slot) = point.get_mut(index) {
            *slot = param.clamp(*slot);
        }
    }
    // A non-finite start is still worth walking away from (the box corners
    // may well be finite), but a run that never sees a finite value reports
    // failure rather than returning -inf as an answer.
    let initial = objective(&point);
    let mut value = if initial.is_finite() {
        initial
    } else {
        f64::NEG_INFINITY
    };
    let mut steps: Vec<f64> = widths.iter().map(|w| 0.5 * w).collect();
    let mut sweep = 0_u32;
    loop {
        if sweep >= MAX_SWEEPS {
            return Err(MAX_SWEEPS);
        }
        sweep += 1;
        let mut improved = false;
        let mut finite_trial = false;
        for index in 0..point.len() {
            for direction in [1.0_f64, -1.0_f64] {
                let step = steps.get(index).copied().unwrap_or(0.0);
                let current = point.get(index).copied().unwrap_or(0.0);
                let Some(param) = params.get(index) else {
                    continue;
                };
                let trial = param.clamp(current + direction * step);
                if trial == current {
                    continue;
                }
                let mut candidate = point.clone();
                if let Some(slot) = candidate.get_mut(index) {
                    *slot = trial;
                }
                let score = objective(&candidate);
                if !score.is_finite() {
                    continue;
                }
                finite_trial = true;
                if score > value {
                    value = score;
                    point = candidate;
                    improved = true;
                    break;
                }
            }
        }
        if improved {
            continue;
        }
        if !finite_trial {
            return Err(sweep);
        }
        let mut moving = false;
        for step in &mut steps {
            *step *= 0.5;
            if *step > STEP_TOLERANCE {
                moving = true;
            }
        }
        if !moving {
            return Ok((point, value));
        }
    }
}

/// Maximise `objective` over the box defined by `params` and return the best
/// value found.
///
/// See [`optimal_point`] for the search itself and its guarantees; this is
/// the value-only form calibration tools usually want.
///
/// # Errors
///
/// As [`optimal_point`].
pub fn optimize_working_point(
    params: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
) -> Result<f64, CalError> {
    optimal_point(params, objective).map(|(_, value)| value)
}
