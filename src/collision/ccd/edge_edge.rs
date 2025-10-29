use na::{Matrix2, UnitVector3, Vector2, Vector3};

use crate::{collision::ccd::solver::solve_cubic, types::Float};

/// Returns (contact point on edge 1, contact normal, barycentric coords of cp on edge 1 )
/// contact normal is from edge 1 to edge 2
/// TODO: still not fail-proof at super-fast speeds
pub fn edge_edge_ccd(
    e1: &[Vector3<Float>; 2],
    e2: &[Vector3<Float>; 2],
    v1: &Vector3<Float>,
    v2: &Vector3<Float>,
    v3: &Vector3<Float>,
    v4: &Vector3<Float>,
    t_end: Float,
) -> Option<(Vector3<Float>, UnitVector3<Float>, [Float; 2])> {
    let x1 = e1[0];
    let x2 = e1[1];
    let x3 = e2[0];
    let x4 = e2[1];

    // Skip the test if edges are parallel
    if (x4 - x3).cross(&(x2 - x1)).norm() <= 1e-6 {
        return None;
    }

    let a = (v2 - v1).cross(&(v4 - v3));
    let b = (x2 - x1).cross(&(v4 - v3)) + (v2 - v1).cross(&(x4 - x3));
    let c = (x2 - x1).cross(&(x4 - x3));

    let d = x3 - x1;
    let e = v3 - v1;

    let p3 = a.dot(&e);
    let p2 = a.dot(&d) + b.dot(&e);
    let p1 = b.dot(&d) + c.dot(&e);
    let p0 = c.dot(&d);

    let t = solve_cubic(p3, p2, p1, p0);
    if t.is_none() {
        return None;
    }
    let t = t.unwrap();
    if t < 0. || t > t_end {
        return None;
    }

    // ref: Contact and Friction Simulation for Comptuer Graphics, 2022
    // 2.2.3 Continuous Collision Detection of Edge-Edge Pair
    let x1 = x1 + v1 * t;
    let x2 = x2 + v2 * t;
    let x3 = x3 + v3 * t;
    let x4 = x4 + v4 * t;

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
        if a > 1. || a < 0. || b > 1. || b < 0. {
            return None;
        }

        // Use current points to find cp and n?
        // let x1 = e1[0];
        // let x2 = e1[1];
        // let x3 = e2[0];
        // let x4 = e2[1];
        // let x12 = x2 - x1;
        // let x34 = x4 - x3;

        let cp = x1 + a * x12;
        let mut n = UnitVector3::new_normalize(x12.cross(&x34));

        // TODO: this might be wrong. fix this.
        if (e2[0] - e1[0]).dot(&n) < 0. {
            n = -n;
        }
        return Some((cp, n, [1. - a, a]));
    } else {
        return None;
        // panic!("no solution / infinite solution on normal equation for edge-edge CCD");
    }
}

#[cfg(test)]
mod edge_ccd_tests {
    use na::{vector, Vector3};

    use crate::{assert_vec_close, collision::ccd::edge_edge::edge_edge_ccd, flog, types::Float};

    #[test]
    fn edge_edge_1() {
        // Arrange
        let e1 = [vector![0., -1., 0.], vector![0., 1., 0.]];
        let e2: [Vector3<Float>; 2] =
            [vector![0., 0., -1.], vector![0., 0., 1.]].map(|e| e + vector![0.1, 0., 0.]);

        let v1 = Vector3::zeros();
        let v2 = Vector3::zeros();
        let v3 = vector![-1., 0., 0.];
        let v4 = v3.clone();

        // Act
        let ccd = edge_edge_ccd(&e1, &e2, &v1, &v2, &v3, &v4, 1.0);

        // Assert
        if let Some((cp, n, ws)) = ccd {
            assert_eq!(cp, vector![0., 0., 0.]);
            assert_eq!(n, Vector3::x_axis());
            assert_eq!(ws, [0.5, 0.5]);
        } else {
            panic!("not detecting collision");
        }
    }

    #[test]
    fn edge_edge_2() {
        // Arrange
        let e1 = [vector![0., -1., 0.], vector![0., 1., 0.]];
        let e2: [Vector3<Float>; 2] =
            [vector![0., 0., -1.], vector![0., 0., 1.]].map(|e| e + vector![0.1, 0., 0.]);

        let v1 = vector![-0.1, 0., 0.];
        let v2 = v1.clone();
        let v3 = vector![-1.1, 0., 0.];
        let v4 = v3.clone();

        // Act
        let ccd = edge_edge_ccd(&e1, &e2, &v1, &v2, &v3, &v4, 1.0);

        // Assert
        if let Some((cp, n, ws)) = ccd {
            assert_vec_close!(cp, vector![-0.01, 0., 0.], 1e-5);
            assert_eq!(n, Vector3::x_axis());
            assert_eq!(ws, [0.5, 0.5]);
        } else {
            panic!("not detecting collision");
        }
    }
}
