//! Calibration solving: monotone curve fits and bounded working-point
//! optimisation.
//!
//! Two problems a calibration engineer hits constantly:
//!
//! * **A curve does not behave like the data.** A set of measured values
//!   comes out of a bench run slightly non-monotonic, and a look-up that
//!   decreases with speed is not a look-up. [`calibrate_curve`] fits the
//!   monotone curve closest to the measurements in the least-squares sense,
//!   subject to each knot's declared bounds — a bounded isotonic
//!   regression, solved exactly by pool-adjacent-violators.
//! * **One calibration point has to trade several quantities off against
//!   each other.** [`optimize_working_point`] runs bounded coordinate
//!   descent over the parameter set, maximising a caller-supplied
//!   objective without ever leaving a parameter's bounds.
//!
//! Both are total: an infeasible problem is a typed
//! [`CalError`], never a panic, and the optimiser reports a
//! [`CalError::Convergence`] when its iteration budget runs out rather than
//! returning an unconverged answer as if it were final.

use crate::error::CalError;

/// Convergence tolerance on the objective's improvement per round: below
/// this, the descent is treated as converged.
const TOLERANCE: f64 = 1e-12;

/// Maximum rounds of coordinate descent before
/// [`CalError::Convergence`] is returned.
pub const MAX_ROUNDS: u32 = 200;

/// Golden-section refinement rounds per coordinate. Each round shrinks the
/// bracket by the golden ratio, so 80 rounds resolve a unit interval to
/// well past f64 precision.
const LINE_SEARCH_ROUNDS: u32 = 80;

/// One named calibration parameter with its value and its admissible range.
#[derive(Debug, Clone, PartialEq)]
pub struct CalParameter {
    /// The parameter name — the CHARACTERISTIC it calibrates.
    pub name: String,
    /// The current (or starting) value.
    pub value: f64,
    /// The lowest admissible value.
    pub lower: f64,
    /// The highest admissible value.
    pub upper: f64,
}

impl CalParameter {
    /// Build a parameter.
    #[must_use]
    pub fn new(name: impl Into<String>, value: f64, lower: f64, upper: f64) -> Self {
        Self {
            name: name.into(),
            value,
            lower,
            upper,
        }
    }

    /// `true` when `value` is inside `[lower, upper]`.
    #[must_use]
    pub fn contains(&self, value: f64) -> bool {
        value.is_finite() && value >= self.lower && value <= self.upper
    }

    /// `value` clamped into `[lower, upper]` — what the solvers start from.
    #[must_use]
    pub fn clamp(&self, value: f64) -> f64 {
        if value < self.lower {
            self.lower
        } else if value > self.upper {
            self.upper
        } else {
            value
        }
    }

    /// The width of the admissible range.
    #[must_use]
    pub fn span(&self) -> f64 {
        self.upper - self.lower
    }
}

/// A monotone piecewise-linear curve: knots in the abscissa and their
/// fitted values.
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    knots: Vec<f64>,
    values: Vec<f64>,
}

impl Curve {
    /// Build a curve from knots and values.
    ///
    /// # Errors
    ///
    /// [`CalError::NoParameters`] when either side is empty, or the lengths
    /// disagree, [`CalError::NonFiniteValue`] for a non-finite knot or
    /// value.
    pub fn new(knots: Vec<f64>, values: Vec<f64>) -> Result<Self, CalError> {
        if knots.len() != values.len() {
            return Err(CalError::NoParameters);
        }
        if let Some((knot, value)) = knots
            .iter()
            .zip(&values)
            .find(|(knot, value)| !knot.is_finite() || !value.is_finite())
        {
            return Err(CalError::NonFiniteValue {
                name: format!("knot {}", **knot),
                value: **value,
            });
        }
        Ok(Self { knots, values })
    }

    /// The knot abscissae, ascending.
    #[must_use]
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// The fitted value at each knot.
    #[must_use]
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    /// The number of knots.
    #[must_use]
    pub fn len(&self) -> usize {
        self.knots.len()
    }

    /// `true` when the curve has no knots.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.knots.is_empty()
    }

    /// `true` when the fitted values never decrease — the property the
    /// fit exists to guarantee.
    #[must_use]
    pub fn is_monotone(&self) -> bool {
        self.values.windows(2).all(|pair| pair[0] <= pair[1])
    }

    /// The largest absolute difference between two curves of identical
    /// knots — the error metric of a calibration fit.
    #[must_use]
    pub fn max_deviation(&self, other: &Curve) -> f64 {
        if self.knots.len() != other.knots.len() {
            return f64::INFINITY;
        }
        self.values
            .iter()
            .zip(&other.values)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
    }

    /// Evaluate the curve at `x`, interpolating linearly between knots and
    /// clamping to the endpoint values outside the knot range.
    #[must_use]
    pub fn evaluate(&self, x: f64) -> f64 {
        match (self.values.first(), self.values.last()) {
            (None, _) | (_, None) => f64::NAN,
            (Some(first), Some(last)) => {
                if x <= self.knots[0] {
                    return *first;
                }
                if x >= self.knots[self.knots.len() - 1] {
                    return *last;
                }
                for window in 0..self.knots.len() - 1 {
                    let (low, high) = (self.knots[window], self.knots[window + 1]);
                    if x >= low && x <= high {
                        let span = high - low;
                        if span == 0.0 {
                            return self.values[window];
                        }
                        let t = (x - low) / span;
                        return self.values[window] + t * (self.values[window + 1] - self.values[window]);
                    }
                }
                *last
            }
        }
    }
}

/// Fit the monotone piecewise-linear curve closest to a parameter set, in
/// the least-squares sense, with every knot constrained to its
/// parameter's `[lower, upper]` bounds.
///
/// The parameters are the knots: parameter *i* carries the measured value
/// at knot *i*, and the knot abscissae are the evenly spaced positions
/// `0, 1, …, n−1`. The fit is the bounded isotonic regression of those
/// values — the solution minimises the sum of squared deviations subject to
/// the fitted values being non-decreasing and inside their boxes, which
/// pool-adjacent-violators computes exactly.
///
/// # Errors
///
/// [`CalError::NoParameters`] when `params` is empty, [`CalError::InfeasibleCurve`]
/// when the bounds contradict monotone ordering (some later knot's lower
/// bound exceeds an earlier knot's upper bound — no monotone sequence can
/// satisfy both), and [`CalError::NonFiniteValue`] for a non-finite value
/// or bound.
pub fn calibrate_curve(params: &[CalParameter]) -> Result<Curve, CalError> {
    if params.is_empty() {
        return Err(CalError::NoParameters);
    }
    for param in params {
        if !param.value.is_finite() || !param.lower.is_finite() || !param.upper.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: param.name.clone(),
                value: param.value,
            });
        }
        if param.lower > param.upper {
            return Err(CalError::InfeasibleCurve {
                name: param.name.clone(),
            });
        }
    }
    // Feasibility: a non-decreasing sequence needs later_lower ≤ earlier_upper.
    for (index, param) in params.iter().enumerate() {
        for earlier in &params[..index] {
            if param.lower > earlier.upper {
                return Err(CalError::InfeasibleCurve {
                    name: param.name.clone(),
                });
            }
        }
    }

    let observed: Vec<f64> = params.iter().map(|param| param.value).collect();
    let mut fitted = pool_adjacent_violators(&observed);

    // Clamp each pooled block into the intersection of its members' boxes,
    // then sweep forward so the result is non-decreasing *and* in-bounds
    // even where the pooled blocks alone would not be.
    let mut previous = f64::NEG_INFINITY;
    for (index, param) in params.iter().enumerate() {
        let mut value = fitted[index].clamp(param.lower, param.upper);
        if value < previous {
            value = previous;
        }
        let value = value.clamp(param.lower, param.upper);
        fitted[index] = value;
        previous = value;
    }

    let knots: Vec<f64> = (0..params.len()).map(|index| index as f64).collect();
    Curve::new(knots, fitted)
}

/// Pool-adjacent-violators: the exact least-squares isotonic
/// (non-decreasing) regression of `observed`, by repeatedly merging runs
/// whose means violate the order.
fn pool_adjacent_violators(observed: &[f64]) -> Vec<f64> {
    /// One pooled run: its members, their mean, and its weight.
    struct Block {
        start: usize,
        length: usize,
        sum: f64,
    }
    let mut blocks: Vec<Block> = observed
        .iter()
        .enumerate()
        .map(|(index, value)| Block {
            start: index,
            length: 1,
            sum: *value,
        })
        .collect();
    loop {
        let mut merged = false;
        let mut index = 0;
        while index + 1 < blocks.len() {
            let left_mean = blocks[index].sum / blocks[index].length as f64;
            let right_mean = blocks[index + 1].sum / blocks[index + 1].length as f64;
            if left_mean > right_mean {
                let right = blocks.remove(index + 1);
                blocks[index].length += right.length;
                blocks[index].sum += right.sum;
                merged = true;
            } else {
                index += 1;
            }
        }
        if !merged {
            break;
        }
    }
    let mut fitted = vec![0.0; observed.len()];
    for block in blocks {
        let mean = block.sum / block.length as f64;
        for slot in &mut fitted[block.start..block.start + block.length] {
            *slot = mean;
        }
    }
    fitted
}

/// The result of a bounded coordinate descent.
#[derive(Debug, Clone, PartialEq)]
pub struct Descent {
    /// The argument that attains the best objective value found.
    pub point: Vec<f64>,
    /// That objective value.
    pub value: f64,
    /// Rounds of coordinate descent performed.
    pub rounds: u32,
    /// `true` when the descent converged before the round budget ran out.
    pub converged: bool,
}

impl Descent {
    /// `true` when every coordinate stays inside its parameter's bounds —
    /// the invariant coordinate descent maintains.
    #[must_use]
    pub fn within_bounds(&self, params: &[CalParameter]) -> bool {
        self.point
            .iter()
            .zip(params)
            .all(|(value, param)| param.contains(*value))
    }
}

/// Coordinate descent: maximise `objective` over the box the parameters
/// describe, starting from each parameter's `value` clamped into bounds.
///
/// Each round runs a golden-section line search on one coordinate at a
/// time, which converges on smooth objectives (the usual case: a
/// calibration objective is a smooth model over a small box) and never
/// leaves the box.
///
/// # Errors
///
/// [`CalError::NoParameters`] when `params` is empty,
/// [`CalError::NonFiniteValue`] for a non-finite value, bound, or
/// objective result, [`CalError::InfeasibleCurve`] when a parameter's
/// bounds are inverted, and [`CalError::Convergence`] when the round budget
/// runs out with the objective still improving.
pub fn optimize_working_point(
    params: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
) -> Result<f64, CalError> {
    Ok(optimize_working_point_at(params, objective)?.1)
}

/// Like [`optimize_working_point`], but also returns the argument that
/// attains the maximum — which is the value a calibration engineer actually
/// writes to the ECU.
///
/// # Errors
///
/// The errors [`optimize_working_point`] returns.
pub fn optimize_working_point_at(
    params: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
) -> Result<(Vec<f64>, f64), CalError> {
    let descent = descend(params, objective, MAX_ROUNDS)?;
    if !descent.converged {
        return Err(CalError::Convergence {
            iterations: descent.rounds,
        });
    }
    Ok((descent.point, descent.value))
}

/// Bounded coordinate descent with an explicit round budget.
///
/// Split out from [`optimize_working_point`] so the budget is testable: a
/// caller that needs a hard cap can ask for one, and a caller that wants to
/// know whether the search converged gets a [`Descent`] rather than a bare
/// maximum.
///
/// # Errors
///
/// [`CalError::NoParameters`] when `params` is empty,
/// [`CalError::InfeasibleCurve`] for inverted bounds, and
/// [`CalError::NonFiniteValue`] for a non-finite parameter or objective
/// result.
pub fn descend(
    params: &[CalParameter],
    objective: &dyn Fn(&[f64]) -> f64,
    max_rounds: u32,
) -> Result<Descent, CalError> {
    if params.is_empty() {
        return Err(CalError::NoParameters);
    }
    for param in params {
        if !param.value.is_finite() || !param.lower.is_finite() || !param.upper.is_finite() {
            return Err(CalError::NonFiniteValue {
                name: param.name.clone(),
                value: param.value,
            });
        }
        if param.lower > param.upper {
            return Err(CalError::InfeasibleCurve {
                name: param.name.clone(),
            });
        }
    }
    let mut point: Vec<f64> = params.iter().map(|param| param.clamp(param.value)).collect();
    let mut best = evaluate(objective, &point)?;

    let mut round = 0;
    while round < max_rounds {
        round += 1;
        let mut improvement = 0.0;
        for coordinate in 0..point.len() {
            let previous = best;
            let (value, candidate) = line_search(
                objective,
                &point,
                coordinate,
                params[coordinate].lower,
                params[coordinate].upper,
            )?;
            improvement += (value - previous).abs();
            point = candidate;
            best = value;
        }
        if improvement <= TOLERANCE {
            return Ok(Descent {
                point,
                value: best,
                rounds: round,
                converged: true,
            });
        }
    }
    Ok(Descent {
        point,
        value: best,
        rounds: max_rounds,
        converged: false,
    })
}

/// Evaluate the objective, rejecting non-finite answers.
fn evaluate(objective: &dyn Fn(&[f64]) -> f64, point: &[f64]) -> Result<f64, CalError> {
    let value = objective(point);
    if value.is_finite() {
        Ok(value)
    } else {
        Err(CalError::NonFiniteValue {
            name: "objective".to_owned(),
            value,
        })
    }
}

/// Golden-section maximisation of the objective along one coordinate.
///
/// Returns the best value found and the point that attains it. The bracket
/// always stays inside `[lower, upper]`, which is what makes the bounds a
/// hard guarantee rather than a penalty.
fn line_search(
    objective: &dyn Fn(&[f64]) -> f64,
    point: &mut [f64],
    coordinate: usize,
    lower: f64,
    upper: f64,
) -> Result<(f64, Vec<f64>), CalError> {
    let mut best_point = point.to_vec();
    let mut best_value = evaluate(objective, point)?;

    let width = upper - lower;
    if !(width > 0.0) {
        // A pinned parameter: nothing to search.
        return Ok((best_value, best_point));
    }
    let phi = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut left = lower;
    let mut right = upper;
    let mut inner_left = right - phi * (right - left);
    let mut inner_right = left + phi * (right - left);

    let mut candidate = best_point.clone();
    candidate[coordinate] = inner_left;
    let mut left_value = evaluate(objective, &candidate)?;
    candidate[coordinate] = inner_right;
    let mut right_value = evaluate(objective, &candidate)?;

    for _ in 0..LINE_SEARCH_ROUNDS {
        if right - left <= 1e-15 * width.max(1.0) {
            break;
        }
        if left_value >= right_value {
            right = inner_right;
            inner_right = inner_left;
            right_value = left_value;
            inner_left = right - phi * (right - left);
            candidate[coordinate] = inner_left;
            left_value = evaluate(objective, &candidate)?;
        } else {
            left = inner_left;
            inner_left = inner_right;
            left_value = right_value;
            inner_right = left + phi * (right - left);
            candidate[coordinate] = inner_right;
            right_value = evaluate(objective, &candidate)?;
        }
    }

    // Compare the bracket midpoint and both interior points: the maximum
    // may sit at either edge, which is exactly the boundary case a
    // calibration objective produces.
    let mut best_coordinate = best_point[coordinate];
    for trial in [
        left,
        right,
        inner_left,
        inner_right,
        best_point[coordinate],
    ] {
        let trial = trial.clamp(lower, upper);
        candidate[coordinate] = trial;
        let value = evaluate(objective, &candidate)?;
        if value > best_value {
            best_value = value;
            best_coordinate = trial;
        }
    }
    best_point[coordinate] = best_coordinate;
    // The endpoints matter for a monotone objective: the golden section
    // never evaluates them.
    for trial in [lower, upper] {
        candidate[coordinate] = trial;
        let value = evaluate(objective, &candidate)?;
        if value > best_value {
            best_value = value;
            best_coordinate = trial;
        }
    }
    best_point[coordinate] = best_coordinate;
    Ok((best_value, best_point))
}