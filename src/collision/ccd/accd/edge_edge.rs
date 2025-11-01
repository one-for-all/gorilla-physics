use std::ops::Range;

use na::{DVector, Matrix2, Normed, SVector, UnitVector3, Vector, Vector2, Vector3};

use crate::{
    collision::ccd::accd::{
        additive_ccd, line_line::line_line_distance, point_line::point_line_distance,
        point_point::point_point_distance, stack,
    },
    flog,
    types::Float,
};

#[allow(non_camel_case_types)]
enum EdgeEdgeDistanceType {
    /// The edges are closest at vertex 0 of edge A and 0 of edge B.
    EA0_EB0,
    /// The edges are closest at vertex 0 of edge A and 1 of edge B.
    EA0_EB1,
    /// The edges are closest at vertex 1 of edge A and 0 of edge B.
    EA1_EB0,
    /// The edges are closest at vertex 1 of edge A and 1 of edge B.
    EA1_EB1,
    /// The edges are closest at the interior of edge A and vertex 0 of edge B.
    EA_EB0,
    /// The edges are closest at the interior of edge A and vertex 1 of edge B.
    EA_EB1,
    /// The edges are closest at vertex 0 of edge A and the interior of edge B.
    EA0_EB,
    /// The edges are closest at vertex 1 of edge A and the interior of edge B.
    EA1_EB,
    /// The edges are closest at an interior point of edge A and B.
    EA_EB,
}

// A more robust implementation of http://geomalgorithms.com/a07-_distance.html
// ref: https://github.com/ipc-sim/ipc-toolkit/blob/main/src/ipc/distance/distance_type.cpp
fn edge_edge_distance_type(
    ea0: &Vector3<Float>,
    ea1: &Vector3<Float>,
    eb0: &Vector3<Float>,
    eb1: &Vector3<Float>,
) -> EdgeEdgeDistanceType {
    let PARALLEL_THRESHOLD = 1e-20;

    let u = ea1 - ea0;
    let v = eb1 - eb0;
    let w = ea0 - eb0;

    let a = u.norm_squared();
    let b = u.dot(&v);
    let c = v.norm_squared();
    let d = u.dot(&w);
    let e = v.dot(&w);
    let D = a * c - b * b;

    // Degenerate cases should not happen in practice, but we handle them
    if a == 0.0 && c == 0.0 {
        return EdgeEdgeDistanceType::EA0_EB0;
    } else if a == 0.0 {
        return EdgeEdgeDistanceType::EA0_EB;
    } else if c == 0.0 {
        return EdgeEdgeDistanceType::EA_EB0;
    }

    // Special handling for parallel edges
    let parallel_tolerance = PARALLEL_THRESHOLD * (a * c).max(1.0);
    if u.cross(&v).norm_squared() < parallel_tolerance {
        return edge_edge_parallel_distance_type(ea0, ea1, eb0, eb1);
    }

    let mut default_case = EdgeEdgeDistanceType::EA_EB;

    // compute the line parameters of the two closest points
    let sN = b * e - c * d;
    let mut tN;
    let mut tD; // tc = tN / tD
    if sN <= 0.0 {
        // sc < 0 ⟹ the s=0 edge is visible
        tN = e;
        tD = c;
        default_case = EdgeEdgeDistanceType::EA0_EB;
    } else if sN >= D {
        // sc > 1 ⟹ the s=1 edge is visible
        tN = e + b;
        tD = c;
        default_case = EdgeEdgeDistanceType::EA1_EB;
    } else {
        tN = a * e - b * d;
        tD = D; // default tD = D ≥ 0
        if tN > 0.0 && tN < tD && u.cross(&v).norm_squared() < parallel_tolerance {
            // avoid coplanar or nearly parallel EE
            if sN < D / 2. {
                tN = e;
                tD = c;
                default_case = EdgeEdgeDistanceType::EA0_EB;
            } else {
                tN = e + b;
                tD = c;
                default_case = EdgeEdgeDistanceType::EA1_EB;
            }
        }
        // else default_case stays EdgeEdgeDistanceType::EA_EB
    }

    if tN <= 0.0 {
        // tc < 0 ⟹ the t=0 edge is visible
        // recompute sc for this edge
        if -d <= 0.0 {
            return EdgeEdgeDistanceType::EA0_EB0;
        } else if -d >= a {
            return EdgeEdgeDistanceType::EA1_EB0;
        } else {
            return EdgeEdgeDistanceType::EA_EB0;
        }
    } else if tN >= tD {
        // tc > 1 ⟹ the t=1 edge is visible
        // recompute sc for this edge
        if (-d + b) <= 0.0 {
            return EdgeEdgeDistanceType::EA0_EB1;
        } else if (-d + b) >= a {
            return EdgeEdgeDistanceType::EA1_EB1;
        } else {
            return EdgeEdgeDistanceType::EA_EB1;
        }
    }

    default_case
}

fn edge_edge_parallel_distance_type(
    ea0: &Vector3<Float>,
    ea1: &Vector3<Float>,
    eb0: &Vector3<Float>,
    eb1: &Vector3<Float>,
) -> EdgeEdgeDistanceType {
    let ea = ea1 - ea0;
    let alpha = (eb0 - ea0).dot(&ea) / ea.norm_squared();
    let beta = (eb1 - ea0).dot(&ea) / ea.norm_squared();

    let eac; // 0: EA0, 1: EA1, 2: EA
    let ebc; // 0: EB0, 1: EB1, 2: EB
    if alpha < 0. {
        eac = if 0. <= beta && beta <= 1. { 2 } else { 0 };
        ebc = if beta <= alpha {
            0
        } else {
            if beta <= 1. {
                1
            } else {
                2
            }
        };
    } else if alpha > 1. {
        eac = if 0. <= beta && beta <= 1. { 2 } else { 1 };
        ebc = if beta >= alpha {
            0
        } else {
            if 0. <= beta {
                1
            } else {
                2
            }
        };
    } else {
        eac = 2;
        ebc = 0;
    }

    // f(0, 0) = 0000 = 0 -> EA0_EB0
    // f(0, 1) = 0001 = 1 -> EA0_EB1
    // f(0, 2) = 0110 = 6 -> EA0_EB
    // f(1, 0) = 0010 = 2 -> EA1_EB0
    // f(1, 1) = 0011 = 3 -> EA1_EB1
    // f(1, 2) = 0111 = 7 -> EA1_EB
    // f(2, 0) = 0100 = 4 -> EA_EB0
    // f(2, 1) = 0101 = 5 -> EA_EB1
    // f(2, 2) = 1000 = 8 -> EA_EB

    assert!(eac != 2 || ebc != 2); // This case results in a degenerate line-line

    return if eac == 0 {
        if ebc == 0 {
            EdgeEdgeDistanceType::EA0_EB0
        } else if ebc == 1 {
            EdgeEdgeDistanceType::EA0_EB1
        } else {
            EdgeEdgeDistanceType::EA0_EB
        }
    } else if eac == 1 {
        if ebc == 0 {
            EdgeEdgeDistanceType::EA1_EB0
        } else if ebc == 1 {
            EdgeEdgeDistanceType::EA1_EB1
        } else {
            EdgeEdgeDistanceType::EA1_EB
        }
    } else {
        if ebc == 0 {
            EdgeEdgeDistanceType::EA_EB0
        } else if ebc == 1 {
            EdgeEdgeDistanceType::EA_EB1
        } else {
            EdgeEdgeDistanceType::EA_EB
        }
    };
}

/// Computes the shortest distance (squared) between two line segments (ea0, ea1) and (eb0, eb1)
pub fn edge_edge_distance(
    ea0: &Vector3<Float>,
    ea1: &Vector3<Float>,
    eb0: &Vector3<Float>,
    eb1: &Vector3<Float>,
) -> Float {
    let dtype = edge_edge_distance_type(ea0, ea1, eb0, eb1);

    match dtype {
        EdgeEdgeDistanceType::EA0_EB0 => point_point_distance(ea0, eb0),
        EdgeEdgeDistanceType::EA0_EB1 => point_point_distance(ea0, eb1),
        EdgeEdgeDistanceType::EA1_EB0 => point_point_distance(ea1, eb0),
        EdgeEdgeDistanceType::EA1_EB1 => point_point_distance(ea1, eb1),
        EdgeEdgeDistanceType::EA_EB0 => point_line_distance(eb0, ea0, ea1),
        EdgeEdgeDistanceType::EA_EB1 => point_line_distance(eb1, ea0, ea1),
        EdgeEdgeDistanceType::EA0_EB => point_line_distance(ea0, eb0, eb1),
        EdgeEdgeDistanceType::EA1_EB => point_line_distance(ea1, eb0, eb1),
        EdgeEdgeDistanceType::EA_EB => line_line_distance(ea0, ea1, eb0, eb1),
    }
}

/// Computes the time of impact between two edges using Additive CCD
/// initial positions and final positions of two edges are given.
/// Returns (whether collision, time of impact)
pub fn edge_edge_accd(
    ea0_t0: &Vector3<Float>,
    ea1_t0: &Vector3<Float>,
    eb0_t0: &Vector3<Float>,
    eb1_t0: &Vector3<Float>,
    ea0_t1: &Vector3<Float>,
    ea1_t1: &Vector3<Float>,
    eb0_t1: &Vector3<Float>,
    eb1_t1: &Vector3<Float>,
    min_distance: Float,
    tmax: Float,
) -> (bool, Float) {
    let initial_distance = edge_edge_distance(ea0_t0, ea1_t0, eb0_t0, eb1_t0);

    if initial_distance <= min_distance * min_distance {
        // TODO(log): log this only at warning level
        flog!(
            "Initial distance {} ≤ d_min={}, returning toi=0!",
            initial_distance.sqrt(),
            min_distance
        );
        return (true, 0.);
    }

    let mut dea0 = ea0_t1 - ea0_t0;
    let mut dea1 = ea1_t1 - ea1_t0;
    let mut deb0 = eb0_t1 - eb0_t0;
    let mut deb1 = eb1_t1 - eb1_t0;
    // subtract mean
    let mean = (dea0 + dea1 + deb0 + deb1) / 4.;
    dea0 -= mean;
    dea1 -= mean;
    deb0 -= mean;
    deb1 -= mean;

    let max_disp_mag = dea0.norm_squared().max(dea1.norm_squared()).sqrt()
        + deb0.norm_squared().max(deb1.norm_squared()).sqrt();
    if max_disp_mag == 0. {
        return (false, Float::INFINITY);
    }

    let min_distance_sq = min_distance * min_distance;
    let distance_squared = |x: &SVector<Float, 12>| {
        let ea0: Vector3<Float> = x.fixed_rows::<3>(0).into();
        let ea1: Vector3<Float> = x.fixed_rows::<3>(3).into();
        let eb0: Vector3<Float> = x.fixed_rows::<3>(6).into();
        let eb1: Vector3<Float> = x.fixed_rows::<3>(9).into();

        let mut d_sq = edge_edge_distance(&ea0, &ea1, &eb0, &eb1);
        if d_sq - min_distance_sq <= 0. {
            // since we ensured other place that all dist smaller than d̂ are
            // positive, this must be some far away nearly parallel edges
            d_sq = (ea0 - eb0)
                .norm_squared()
                .min((ea0 - eb1).norm_squared())
                .min((ea1 - eb0).norm_squared())
                .min((ea1 - eb1).norm_squared());
        }
        return d_sq;
    };

    let x = stack(ea0_t0, ea1_t0, eb0_t0, eb1_t0);
    let dx = stack(&dea0, &dea1, &deb0, &deb1);

    additive_ccd(x, dx, distance_squared, max_disp_mag, min_distance, tmax)
}

/// Returns (contact point, contact normal, barycentric coords of cp on edge 1, and that on edge 2)
/// contact normal points from edge a to edge b
pub fn edge_edge_contact(
    ea0_t0: &Vector3<Float>,
    ea1_t0: &Vector3<Float>,
    eb0_t0: &Vector3<Float>,
    eb1_t0: &Vector3<Float>,
    ea0_t1: &Vector3<Float>,
    ea1_t1: &Vector3<Float>,
    eb0_t1: &Vector3<Float>,
    eb1_t1: &Vector3<Float>,
    toi: Float,
) -> (Vector3<Float>, UnitVector3<Float>, [Float; 2], [Float; 2]) {
    let dea0 = ea0_t1 - ea0_t0;
    let dea1 = ea1_t1 - ea1_t0;
    let deb0 = eb0_t1 - eb0_t0;
    let deb1 = eb1_t1 - eb1_t0;

    let ea0_tc = ea0_t0 + dea0 * toi;
    let ea1_tc = ea1_t0 + dea1 * toi;
    let eb0_tc = eb0_t0 + deb0 * toi;
    let eb1_tc = eb1_t0 + deb1 * toi;

    // Compute the contact point, given two contacting edges
    // ref: Contact and Friction Simulation for Comptuer Graphics, 2022
    // 2.2.3 Continuous Collision Detection of Edge-Edge Pair
    let x1 = ea0_tc;
    let x2 = ea1_tc;
    let x3 = eb0_tc;
    let x4 = eb1_tc;

    let x12 = x2 - x1;
    let x34 = x4 - x3;
    let A00 = x12.dot(&x12);
    let A01 = -x12.dot(&x34);
    let A10 = A01;
    let A11 = x34.dot(&x34);
    let A = Matrix2::new(A00, A01, A10, A11);

    let x13 = x3 - x1;
    let b = Vector2::new(x12.dot(&x13), -x34.dot(&x13));

    if let Some(x) = A.lu().solve(&b) {
        let a = x[0];
        let b = x[1];

        let cp = x1 + a * x12;
        let mut n = UnitVector3::new_normalize(x12.cross(&x34));

        // contact points on edge a and b at t0
        let ea_cp_t0 = ea0_t0 + a * (ea1_t0 - ea0_t0);
        let eb_cp_t0 = eb0_t0 + a * (eb1_t0 - eb0_t0);
        if (eb_cp_t0 - ea_cp_t0).dot(&n) < 0. {
            n = -n;
        }

        return (cp, n, [1. - a, a], [1. - b, b]);
    } else {
        panic!(
            "Cannot solve for normal equation Ax=b. Edges are probably parallel. A: {}, b: {}",
            A, b
        );
    }
}
