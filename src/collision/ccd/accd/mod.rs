//! Additive CCD implementation
//! ref: Codimensional Incremental Potential Contact, Li et. al, 2021

use na::SVector;

use crate::{flog, types::Float};

pub mod edge_edge;
pub mod line_line;
pub mod point_line;
pub mod point_point;

const DEFAULT_MAX_ITERATIONS: usize = 10_000_000;
const DEFAULT_CONSERVATIVE_RESCALING: Float = 0.9;

/// Computes the time of impact between two objects using additive continuous collision detection.
/// x: Initial positions
/// dx: Displacements
/// distance_squared: A function that computes the squared distance between the two objects at a given time
/// max_disp_mag: Maximum displacement magnitude
/// min_distance: The minimum distance between the objects
/// tmax: The maximum time to check for collisions
/// Returns (whether collision, time of impact)
pub fn additive_ccd<F>(
    mut x: SVector<Float, 12>,
    dx: SVector<Float, 12>,
    distance_squared: F,
    max_disp_mag: Float,
    min_distance: Float,
    tmax: Float,
) -> (bool, Float)
where
    F: Fn(&SVector<Float, 12>) -> Float,
{
    let conservative_rescaling = DEFAULT_CONSERVATIVE_RESCALING;

    let min_distance_sq = min_distance * min_distance;

    let mut d_sq = distance_squared(&x);
    let mut d = d_sq.sqrt();
    assert!(d > min_distance);

    let mut d_func = d_sq - min_distance_sq;
    assert!(d_func > 0.);

    // (d - ξ) = (d² - ξ²) / (d + ξ)
    let gap = (1. - conservative_rescaling) * d_func / (d + min_distance);
    if gap < Float::EPSILON {
        // TODO(log): log in warning
        flog!(
            "Small gap {:e} ≤ ε={} in Additive CCD can lead to missed collisions",
            gap,
            Float::EPSILON
        )
    }

    let mut toi = 0.;
    let max_iterations = DEFAULT_MAX_ITERATIONS;
    for i in 0..max_iterations {
        let toi_lower_bound = conservative_rescaling * d_func / ((d + min_distance) * max_disp_mag);

        x += toi_lower_bound * dx;

        d_sq = distance_squared(&x);
        d = d_sq.sqrt();

        d_func = d_sq - min_distance_sq;
        assert!(d_func > 0.);
        if toi > 0. && d_func / (d + min_distance) < gap {
            break; // distance (including thickness) is less than gap
        }

        toi += toi_lower_bound;
        if toi > tmax {
            return (false, Float::INFINITY); // collision occurs after tmax
        }

        if i == max_iterations - 1 {
            // TODO(log): log in warning
            flog!(
                "Slow convergence in Additive CCD. Perhaps the gap is too small (gap={:e})?",
                gap
            )
        }
    }

    (true, toi)
}
