use na::{Matrix2x3, SVector, UnitVector3, Vector3};

use crate::{
    collision::{
        ccd::accd::{
            additive_ccd, point_line::point_line_distance, point_plane::point_plane_distance,
            point_point::point_point_distance, stack,
        },
        mesh::projected_barycentric_coord,
    },
    flog,
    types::Float,
};

#[allow(non_camel_case_types)]
enum PointTriangleDistanceType {
    ///< The point is closest to triangle vertex zero.
    P_T0,
    ///< The point is closest to triangle vertex one.
    P_T1,
    ///< The point is closest to triangle vertex two.
    P_T2,
    ///< The point is closest to triangle edge zero (vertex zero to one).
    P_E0,
    ///< The point is closest to triangle edge one (vertex one to two).
    P_E1,
    ///< The point is closest to triangle edge two (vertex two to zero).
    P_E2,
    ///< The point is closest to the interior of the triangle.
    P_T,
}

/// Determine the closest pair between a point and a triangle
fn point_triangle_distance_type(
    p: &Vector3<Float>,
    t0: &Vector3<Float>,
    t1: &Vector3<Float>,
    t2: &Vector3<Float>,
) -> PointTriangleDistanceType {
    let normal = (t1 - t0).cross(&(t2 - t0));

    let mut basis = Matrix2x3::zeros();
    let mut param = Matrix2x3::zeros();

    let row0 = t1 - t0;
    basis.row_mut(0).copy_from(&row0.transpose());
    let row1 = row0.cross(&normal);
    basis.row_mut(1).copy_from(&row1.transpose());

    let col0 = (basis * basis.transpose())
        .cholesky() // equivalent to LDLT decomposition
        .unwrap()
        .solve(&(basis * (p - t0)));
    param.column_mut(0).copy_from(&col0);
    if param[(0, 0)] > 0. && param[(0, 0)] < 1. && param[(1, 0)] >= 0. {
        return PointTriangleDistanceType::P_E0; // edge 0 is the closest
    }

    let row0 = t2 - t1;
    basis.row_mut(0).copy_from(&row0.transpose());
    let row1 = row0.cross(&(normal));
    basis.row_mut(1).copy_from(&row1.transpose());

    let col1 = (basis * basis.transpose())
        .cholesky()
        .unwrap()
        .solve(&(basis * (p - t1)));
    param.column_mut(1).copy_from(&col1);
    if param[(0, 1)] > 0. && param[(0, 1)] < 1. && param[(1, 1)] >= 0. {
        return PointTriangleDistanceType::P_E1; // edge 1 is the closest
    }

    let row0 = t0 - t2;
    basis.row_mut(0).copy_from(&row0.transpose());
    let row1 = row0.cross(&normal);
    basis.row_mut(1).copy_from(&row1.transpose());

    let col2 = (basis * basis.transpose())
        .cholesky()
        .unwrap()
        .solve(&(basis * (p - t2)));
    param.column_mut(2).copy_from(&col2);
    if param[(0, 2)] > 0. && param[(0, 2)] < 1. && param[(1, 2)] >= 0. {
        return PointTriangleDistanceType::P_E2; // edge 2 is the closest
    }

    if param[(0, 0)] <= 0. && param[(0, 2)] >= 1. {
        return PointTriangleDistanceType::P_T0; // vertex 0 is the closest
    } else if param[(0, 1)] <= 0. && param[(0, 0)] >= 1. {
        return PointTriangleDistanceType::P_T1; // vertex 1 is the closest
    } else if param[(0, 2)] <= 0. && param[(0, 1)] >= 1. {
        return PointTriangleDistanceType::P_T2; // vertex 2 is the closest
    }

    PointTriangleDistanceType::P_T
}

pub fn point_triangle_distance(
    p: &Vector3<Float>,
    t0: &Vector3<Float>,
    t1: &Vector3<Float>,
    t2: &Vector3<Float>,
) -> Float {
    let dtype = point_triangle_distance_type(p, t0, t1, t2);

    match dtype {
        PointTriangleDistanceType::P_T0 => point_point_distance(p, t0),
        PointTriangleDistanceType::P_T1 => point_point_distance(p, t1),
        PointTriangleDistanceType::P_T2 => point_point_distance(p, t2),
        PointTriangleDistanceType::P_E0 => point_line_distance(p, t0, t1),
        PointTriangleDistanceType::P_E1 => point_line_distance(p, t1, t2),
        PointTriangleDistanceType::P_E2 => point_line_distance(p, t2, t0),
        PointTriangleDistanceType::P_T => point_plane_distance(p, t0, t1, t2),
    }
}

/// Computes the time of impact between a point and a triangle using Additive CCD.
/// initial positions and final positions of the point and the triangle are given.
/// Returns (whether collision, time of impact)
pub fn point_triangle_accd(
    p_t0: &Vector3<Float>,
    t0_t0: &Vector3<Float>,
    t1_t0: &Vector3<Float>,
    t2_t0: &Vector3<Float>,
    p_t1: &Vector3<Float>,
    t0_t1: &Vector3<Float>,
    t1_t1: &Vector3<Float>,
    t2_t1: &Vector3<Float>,
    min_distance: Float,
    tmax: Float,
) -> Option<Float> {
    let initial_distance = point_triangle_distance(p_t0, t0_t0, t1_t0, t2_t0);
    if initial_distance <= min_distance * min_distance {
        // TODO(log): log this only at warning level
        flog!(
            "Initial distance {} ≤ d_min={}, returning toi=0!",
            initial_distance.sqrt(),
            min_distance
        );
        return Some(0.);
    }

    let mut dp = p_t1 - p_t0;
    let mut dt0 = t0_t1 - t0_t0;
    let mut dt1 = t1_t1 - t1_t0;
    let mut dt2 = t2_t1 - t2_t0;
    // subtract mean
    let mean = (dp + dt0 + dt1 + dt2) / 4.;
    dp -= mean;
    dt0 -= mean;
    dt1 -= mean;
    dt2 -= mean;

    let max_disp_mag = dp.norm()
        + dt0
            .norm_squared()
            .max(dt1.norm_squared())
            .max(dt2.norm_squared())
            .sqrt();
    if max_disp_mag == 0. {
        return None;
    }

    let distance_squared = |x: &SVector<Float, 12>| {
        let p: Vector3<Float> = x.fixed_rows::<3>(0).into();
        let t0: Vector3<Float> = x.fixed_rows::<3>(3).into();
        let t1: Vector3<Float> = x.fixed_rows::<3>(6).into();
        let t2: Vector3<Float> = x.fixed_rows::<3>(9).into();
        return point_triangle_distance(&p, &t0, &t1, &t2);
    };

    let x = stack(p_t0, t0_t0, t1_t0, t2_t0);
    let dx = stack(&dp, &dt0, &dt1, &dt2);

    additive_ccd(x, dx, distance_squared, max_disp_mag, min_distance, tmax)
}

/// Returns (contact point, contact normal, barycentric coords of cp on triangle)
/// contact normal points away from the triangle
pub fn point_triangle_contact(
    p_t0: &Vector3<Float>,
    t0_t0: &Vector3<Float>,
    t1_t0: &Vector3<Float>,
    t2_t0: &Vector3<Float>,
    p_t1: &Vector3<Float>,
    t0_t1: &Vector3<Float>,
    t1_t1: &Vector3<Float>,
    t2_t1: &Vector3<Float>,
    toi: Float,
) -> (Vector3<Float>, UnitVector3<Float>, [Float; 3]) {
    let dp = p_t1 - p_t0;
    let dt0 = t0_t1 - t0_t0;
    let dt1 = t1_t1 - t1_t0;
    let dt2 = t2_t1 - t2_t0;

    let p_tc = p_t0 + dp * toi;
    let t0_tc = t0_t0 + dt0 * toi;
    let t1_tc = t1_t0 + dt1 * toi;
    let t2_tc = t2_t0 + dt2 * toi;

    let u = t1_tc - t0_tc;
    let v = t2_tc - t0_tc;

    let cp = p_tc;
    let n = UnitVector3::new_normalize(u.cross(&v));

    let (w1, w2, w3) = projected_barycentric_coord(&cp, &t0_tc, &u, &v);
    (cp, n, [w1, w2, w3])
}
