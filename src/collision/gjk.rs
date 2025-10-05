use na::{vector, Vector3};

use crate::types::Float;

use super::{
    cuboid::Cuboid,
    polytope::{support_fn, Polytope},
};

const ORIGIN: Vector3<Float> = vector![0., 0., 0.];

/// Perform GJK collision detection.
/// Returns the Minkowski difference polytope, if in collision
pub(super) fn gjk(A: &Cuboid, B: &Cuboid) -> Option<Polytope> {
    // Keeps track of the vertices
    let mut simplex = vec![];

    // Pick an initial direction
    let mut direction = {
        let center_A = A.isometry.translation.vector;
        let center_B = B.isometry.translation.vector;
        if center_A == center_B {
            vector![1., 0., 0.]
        } else {
            center_A - center_B
        }
    };

    loop {
        let new_vertex = support_fn(A, B, &direction);
        if new_vertex.v == ORIGIN {
            // origin is a vertex => A and B are touching
            return None;
        }
        if direction.dot(&new_vertex.v) <= 0.0 {
            // new vertex not past origin in direction => (A-B) shape cannot enclose origin
            return None;
        }
        simplex.push(new_vertex.clone());

        match simplex.len() {
            1 => {
                direction = ORIGIN - new_vertex.v;
            }
            2 => {
                let v1 = &simplex[0];
                let v2 = &simplex[1];
                let v1v2 = v2.v - v1.v;
                // direction perpendicular from v1v2 segment to origin
                direction = v1.v.cross(&v2.v).cross(&v1v2);

                if direction.norm() == 0.0 {
                    // origin on edge
                    let candidate_direction = v1v2.cross(&Vector3::x_axis());
                    if candidate_direction.norm() > 0.0 {
                        // v1v2 not aligned with x-axis, ok
                        direction = candidate_direction;
                    } else {
                        direction = v1v2.cross(&Vector3::y_axis());
                    }
                }
            }
            3 => {
                let v1 = &simplex[0];
                let v2 = &simplex[1];
                let v3 = &simplex[2];
                direction = (v3.v - v1.v).cross(&(v2.v - v1.v));
                if direction.dot(&(ORIGIN - v1.v)) < 0.0 {
                    direction = -direction;

                    // switch v2 and v3, to ensure direction is still v1v3 cross v1v2
                    simplex.swap(1, 2);
                }

                if direction.norm() == 0.0 {
                    panic!("v1 v2 v3 of the simplex cannot be colinear");
                }
            }
            4 => {
                let v1 = &simplex[0];
                let v2 = &simplex[1];
                let v3 = &simplex[2];
                let v4 = &simplex[3];
                let v1v3 = v3.v - v1.v;
                let v1v2 = v2.v - v1.v;
                let v1v4 = v4.v - v1.v;

                let normal_v143 = v1v3.cross(&v1v4);
                if normal_v143.dot(&v1v2) > 0.0 {
                    panic!("normal should be pointing outward by construction");
                }
                if normal_v143.dot(&v1.v) < 0.0 {
                    // origin is outside in this direction
                    direction = normal_v143;
                    simplex = vec![v1.clone(), v4.clone(), v3.clone()];
                    continue;
                }

                let v2v3 = v3.v - v2.v;
                let v2v4 = v4.v - v2.v;
                let v2v1 = -v1v2;
                let normal_v234 = v2v4.cross(&v2v3);
                if normal_v234.dot(&v2v1) > 0.0 {
                    panic!("normal should be pointing outward by construction");
                }
                if normal_v234.dot(&v2.v) < 0.0 {
                    // origin is outside in this direction
                    direction = normal_v234;
                    simplex = vec![v2.clone(), v3.clone(), v4.clone()];
                    continue;
                }

                let normal_v124 = v1v4.cross(&v1v2);
                if normal_v124.dot(&v1v3) > 0.0 {
                    panic!("normal should be pointing outward by construction");
                }
                if normal_v124.dot(&v1.v) < 0.0 {
                    // origin is outside in this direction
                    direction = normal_v124;
                    simplex = vec![v1.clone(), v2.clone(), v4.clone()];
                    continue;
                }

                // origin not in any of the three possible regions, it must be
                // inside
                return Some(Polytope::from_simplex(&simplex));
            }
            _ => panic!("should not have more than 4 vertices"),
        }
    }
}

#[cfg(test)]
mod gjk_tests {
    use na::UnitQuaternion;
    use rand::{rng, Rng};

    use crate::{util::test_utils::random_vector3, PI};

    use super::*;

    #[test]
    fn gjk_sample() {
        // Arrange
        let box1 = Cuboid::new(Vector3::zeros(), UnitQuaternion::identity(), 3.0, 1.0, 2.0);
        let box2 = Cuboid::new_cube(
            vector![1.5, 0.0, 1.0],
            UnitQuaternion::from_euler_angles(0., PI / 4.0, 0.0),
            1.0,
        );

        // Assert
        assert!(gjk(&box1, &box2).is_some());
    }

    #[test]
    fn gjk_sample2() {
        // Arrange
        let box1 = Cuboid::new_cube(Vector3::zeros(), UnitQuaternion::identity(), 1.0);
        let box2 = Cuboid::new_cube(vector![1.5, 0.0, 1.0], UnitQuaternion::identity(), 1.0);

        // Assert
        assert!(gjk(&box1, &box2).is_none());
    }

    #[test]
    fn gjk_sample3() {
        // Arrange
        let l = 2.0;
        let distance: Float = (2.0 as Float).sqrt();
        let box1 = Cuboid::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l);
        let box2 = Cuboid::new_cube(
            vector![-distance, 0.0, distance],
            UnitQuaternion::identity(),
            l,
        );

        // Assert
        assert!(gjk(&box1, &box2).is_some());
    }

    #[test]
    fn gjk_sample4() {
        // Arrange
        let l = 2.0;
        let distance: Float = (2.0 as Float).sqrt();
        let box1 = Cuboid::new_cube(
            Vector3::zeros(),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
            l,
        );
        let box2 = Cuboid::new_cube(
            vector![-distance - 1e-3, 0.0, distance],
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
            l,
        );

        // Assert
        assert!(gjk(&box1, &box2).is_none());
    }

    #[test]
    fn gjk_face_shallow_penetration() {
        let mut rng = rng();

        for _ in 0..5000 {
            // Arrange
            let center = random_vector3(&mut rng, 1.0);
            let l_a = rng.random_range(0.5..2.0);
            let mut A = Cuboid::new_cube(center, UnitQuaternion::identity(), l_a);
            let l_b = rng.random_range(0.5..2.0);

            // move B in a either x, y or z direction, to make it touch A
            let move_dir = rng.random_range(0..2);
            let mut b_center = center
                + vector![
                    rng.random_range(-1e-2..1e-2),
                    rng.random_range(-1e-2..1e-2),
                    rng.random_range(-1e-2..1e-2),
                ];
            b_center[move_dir] = center[move_dir] + l_a / 2.0 + l_b / 2.0;
            let mut B = Cuboid::new_cube(b_center, UnitQuaternion::identity(), l_b);

            // create a small penetration
            let penetration = rng.random_range(0.0..1e-3);
            B.isometry.translation.vector[move_dir] -= penetration;

            A.set_rotation(UnitQuaternion::from_euler_angles(
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
            ));
            B.set_rotation(UnitQuaternion::from_euler_angles(
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
            ));
            A.recompute_points();
            B.recompute_points();

            // Act
            let poly = gjk(&A, &B);
            let poly2 = gjk(&B, &A);

            // Assert
            assert!(poly.is_some());
            assert!(poly.unwrap().contains_origin());
            assert!(poly2.is_some());
            assert!(poly2.unwrap().contains_origin());
        }
    }

    #[test]
    fn gjk_no_penetration() {
        let mut rng = rng();

        for _ in 0..5000 {
            // Arrange
            let center = random_vector3(&mut rng, 1.0);
            let l_a = rng.random_range(0.5..2.0);
            let mut A = Cuboid::new_cube(center, UnitQuaternion::identity(), l_a);
            let l_b = rng.random_range(0.5..2.0);

            // move B in a either x, y or z direction, to put it away from A
            let move_dir = rng.random_range(0..2);
            let mut b_center = center
                + vector![
                    rng.random_range(-0.1..0.1),
                    rng.random_range(-0.1..0.1),
                    rng.random_range(-0.1..0.1)
                ];
            b_center[move_dir] = center[move_dir] + l_a + l_b;
            let mut B = Cuboid::new_cube(b_center, UnitQuaternion::identity(), l_b);

            let disturbance = rng.random_range(-1e-2..1e-2);
            B.isometry.translation.vector[move_dir] += disturbance;

            A.set_rotation(UnitQuaternion::from_euler_angles(
                rng.random_range(-PI..PI),
                rng.random_range(-PI..PI),
                rng.random_range(-PI..PI),
            ));
            B.set_rotation(UnitQuaternion::from_euler_angles(
                rng.random_range(-PI..PI),
                rng.random_range(-PI..PI),
                rng.random_range(-PI..PI),
            ));
            A.recompute_points();
            B.recompute_points();

            // Act
            let poly = gjk(&A, &B);
            let poly2 = gjk(&B, &A);

            // Assert
            assert!(poly.is_none());
            assert!(poly2.is_none());
        }
    }

    #[test]
    fn gjk_small_separation() {
        let mut rng = rng();

        for _ in 0..5000 {
            // Arrange
            let center = random_vector3(&mut rng, 1.0);
            let l_a = rng.random_range(0.5..2.0);
            let mut A = Cuboid::new_cube(center, UnitQuaternion::identity(), l_a);
            let l_b = rng.random_range(0.5..2.0);

            // move B in a either x, y or z direction, to put it away from A
            let move_dir = rng.random_range(0..2);
            let mut b_center = center
                + vector![
                    rng.random_range(-0.1..0.1),
                    rng.random_range(-0.1..0.1),
                    rng.random_range(-0.1..0.1)
                ];
            b_center[move_dir] = center[move_dir] + l_a / 2.0 + l_b / 2.0;
            let mut B = Cuboid::new_cube(b_center, UnitQuaternion::identity(), l_b);

            // create a small separation
            let separation = rng.random_range(1e-1..2e-1);
            B.isometry.translation.vector[move_dir] += separation;

            A.set_rotation(UnitQuaternion::from_euler_angles(
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
            ));
            B.set_rotation(UnitQuaternion::from_euler_angles(
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
                rng.random_range(-1e-2..1e-2),
            ));
            A.recompute_points();
            B.recompute_points();

            // Act
            let poly = gjk(&A, &B);
            let poly2 = gjk(&B, &A);

            // Assert
            assert!(poly.is_none());
            assert!(poly2.is_none());
        }
    }
}
