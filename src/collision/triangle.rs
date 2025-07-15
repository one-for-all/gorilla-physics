use na::{UnitVector3, Vector3};

use crate::types::Float;

/// TODO: Not fully-developed. Not used now. Could be useful in the future.
/// Returns (contact, normal) if two triangles are in contact
pub fn compute_triangle_contact(
    A: &[Vector3<Float>; 3],
    B: &[Vector3<Float>; 3],
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let A_v0_v1 = A[1] - A[0];
    let A_v0_v2 = A[2] - A[0];
    let A_normal = UnitVector3::new_normalize(A_v0_v1.cross(&A_v0_v2)); // outward normal
    let A_d = -A_normal.dot(&A[0]); // distance from triangle to origin

    // compute distances from B vertices to A plane, and return None if all vertices on
    // same side
    let B_v0_d = A_normal.dot(&B[0]) + A_d;
    let B_v1_d = A_normal.dot(&B[1]) + A_d;
    let B_v2_d = A_normal.dot(&B[2]) + A_d;
    if B_v0_d.signum() == B_v1_d.signum() && B_v0_d.signum() == B_v2_d.signum() {
        return None;
    }

    let B_v0_v1 = B[1] - B[0];
    let B_v0_v2 = B[2] - B[0];
    let B_normal = UnitVector3::new_normalize(B_v0_v1.cross(&B_v0_v2)); // outward normal
    let B_d = -B_normal.dot(&B[0]); // distance from triangle to origin

    // compute distances from A vertices to B plane, and return None if all vertices on
    // same side
    let A_v0_d = B_normal.dot(&A[0]) + B_d;
    let A_v1_d = B_normal.dot(&A[1]) + B_d;
    let A_v2_d = B_normal.dot(&A[2]) + B_d;
    if A_v0_d.signum() == A_v1_d.signum() && A_v0_d.signum() == A_v2_d.signum() {
        return None;
    }

    // TODO: Fix the following. Currently hack & trial
    let B_min_d = B_v0_d.min(B_v1_d).min(B_v2_d);
    let A_min_d = A_v0_d.min(A_v1_d).min(A_v2_d);
    let min_d = B_min_d.min(A_min_d);
    if B_v0_d == min_d {
        return Some((B[0], A_normal));
    }
    if B_v1_d == min_d {
        return Some((B[1], A_normal));
    }
    if B_v2_d == min_d {
        return Some((B[2], A_normal));
    }
    if A_v0_d == min_d {
        return Some((A[0], B_normal));
    }
    if A_v1_d == min_d {
        return Some((A[1], B_normal));
    }
    if A_v2_d == min_d {
        return Some((A[2], B_normal));
    }
    panic!("should not reach here");
}

/// TODO: Not fully-developed. Not used now. Could be useful in the future.
pub fn triangle_triangle_collision_detection(
    a: &[Vector3<Float>; 3],
    b: &[Vector3<Float>; 3],
) -> bool {
    // equation parameters for triangle A
    let a_v0_v1 = a[1] - a[0];
    let a_v0_v2 = a[2] - a[0];
    let a_normal = a_v0_v1.cross(&a_v0_v2);
    let a_d = -a_normal.dot(&a[0]);

    // compute distances from B vertices to A plane, and return false if all vertices on
    // same side
    let b_v0_d = a_normal.dot(&b[0]) + a_d;
    let b_v1_d = a_normal.dot(&b[1]) + a_d;
    let b_v2_d = a_normal.dot(&b[2]) + a_d;
    if b_v0_d.signum() == b_v1_d.signum() && b_v0_d.signum() == b_v2_d.signum() {
        return false;
    }

    // equation parameters for triangle B
    let b_v0_v1 = b[1] - b[0];
    let b_v0_v2 = b[2] - b[0];
    let b_normal = b_v0_v1.cross(&b_v0_v2);
    let b_d = -b_normal.dot(&b[0]);

    // compute distances from A vertices to B plane, and return false if all vertices on
    // same side
    let a_v0_d = b_normal.dot(&a[0]) + b_d;
    let a_v1_d = b_normal.dot(&a[1]) + b_d;
    let a_v2_d = b_normal.dot(&a[2]) + b_d;
    if a_v0_d.signum() == a_v1_d.signum() && a_v0_d.signum() == a_v2_d.signum() {
        return false;
    }

    let D = a_normal.cross(&b_normal); // direction of intersection line

    // if a_v0_d.signum() == a_v2_d.signum() {
    let p0 = D.dot(&a[0]);
    let p1 = D.dot(&a[1]);
    let _t1 = p0 + (p1 - p0) * a_v0_d / (a_v0_d - a_v1_d);

    let p2 = D.dot(&a[2]);
    let _t2 = p2 + (p1 - p2) * a_v2_d / (a_v2_d - a_v1_d);
    // }

    let p0 = D.dot(&b[0]);
    let p2 = D.dot(&b[2]);
    let _t3 = p0 + (p2 - p0) * b_v0_d / (b_v0_d - b_v2_d);

    let p1 = D.dot(&b[1]);
    let _t4 = p2 + (p1 - p2) * b_v2_d / (b_v2_d - b_v1_d);

    true
}

#[cfg(test)]
mod triangle_collision_tests {
    use na::vector;

    use super::triangle_triangle_collision_detection;

    #[test]
    fn triangle_triangle_test1() {
        // Arrange
        let a = [
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![1., 1., 0.],
        ];
        let b = [
            vector![0., 0., 1.],
            vector![1., 0., 1.],
            vector![1., 1., 1.],
        ];

        // Act
        let col = triangle_triangle_collision_detection(&a, &b);

        // Assert
        assert_eq!(col, false);
    }

    #[test]
    fn triangle_triangle_test2() {
        // Arrange
        let a = [
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![1., 1., 0.],
        ];
        let b = [
            vector![0., 0., 1.],
            vector![1., 0., 1.],
            vector![0., 0., 2.],
        ];

        // Act
        let col = triangle_triangle_collision_detection(&a, &b);

        // Assert
        assert_eq!(col, false);
    }

    #[test]
    fn triangle_triangle_test3() {
        // Arrange
        let a = [
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![1., 1., 0.],
        ];
        let b = [
            vector![0., 0.5, 0.5],
            vector![1., 0.5, -0.5],
            vector![1., 0.5, 0.5],
        ];

        // Act
        let col = triangle_triangle_collision_detection(&b, &a);

        // Assert
        assert_eq!(col, true);
    }
}
