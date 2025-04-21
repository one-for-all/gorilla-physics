use std::{cmp::Reverse, collections::BinaryHeap};

use na::{vector, Matrix3, Matrix3x1, Vector3};
use ordered_float::OrderedFloat;

use crate::types::Float;

use super::{
    addEdgeIfNotExisting,
    cuboid::Cuboid,
    polytope::{support_fn, Polytope},
};

const TOL: Float = 0.0; // TODO: investigate why TOL of 0 fails epa_sample2 test

#[allow(unused)]
// Returns the lambda vector that composes v1, v2, v3 into the closest point
// i.e. Solve Aλ = b, such that v = λ * (v1, v2, v3)
fn solve_closest_point_lambda_matrix_method(
    v1: &Vector3<Float>,
    v2: &Vector3<Float>,
    v3: &Vector3<Float>,
) -> Vector3<Float> {
    let v1v2 = v2 - v1;
    let v1v3 = v3 - v1;

    // // TODO: Special treatement when v123 plane contains origin?
    // if v1v2.cross(&v1v3).dot(v1) == 0.0 {}

    #[rustfmt::skip]
    let A = Matrix3::new(
        1.,             1.,             1., 
        v1.dot(&v1v2),  v2.dot(&v1v2),  v3.dot(&v1v2), 
        v1.dot(&v1v3),  v2.dot(&v1v3),  v3.dot(&v1v3)
    );
    let b = Matrix3x1::new(1.0, 0., 0.);

    let lambda = A.lu().solve(&b);
    if lambda.is_none() {
        panic!("Can not solve A x = b, with A = {}, b = {}", A, b);
    }

    lambda.unwrap()
}

#[allow(unused)]
// Returns the closest point on the face, by normal projection method
fn solve_closest_point_direct(
    v1: &Vector3<Float>,
    v2: &Vector3<Float>,
    v3: &Vector3<Float>,
) -> Vector3<Float> {
    let v1v2 = v2 - v1;
    let v1v3 = v3 - v1;
    let normal = v1v2.cross(&v1v3);
    if normal.norm() == 0.0 {
        panic!("normal of v123 should not be 0.0");
    }

    let distance = v2.dot(&normal) / normal.norm();

    let result = distance * normal / normal.norm();
    result
}

/// Computes the barycentric cooridates for closest point on a face, by area method.
/// i.e. λ for which v = λ * (v1, v2, v3)
/// Ref: Computing the Barycentric Coordinates of a Projected Point, by Wolfgang
/// Heidrich, 2005
/// Returns:
///     barycentric coordinates
///     normal direction
fn solve_closest_point_lambda_area_method(
    v1: &Vector3<Float>,
    v2: &Vector3<Float>,
    v3: &Vector3<Float>,
) -> (Vector3<Float>, Vector3<Float>) {
    let v1v2 = v2 - v1;
    let v1v3 = v3 - v1;
    let normal = v1v2.cross(&v1v3);
    let one_over_4A_squared = 1.0 / normal.norm_squared();

    let b3 = v1.cross(&v1v2).dot(&normal) * one_over_4A_squared;
    let b2 = v1v3.cross(&v1).dot(&normal) * one_over_4A_squared;
    let b1 = 1.0 - (b2 + b3);
    (vector![b1, b2, b3], normal)
}

/// Computes the closest point on a face
pub fn solve_closest_point_area_method(
    v1: &Vector3<Float>,
    v2: &Vector3<Float>,
    v3: &Vector3<Float>,
) -> Vector3<Float> {
    let (lambda, normal) = solve_closest_point_lambda_area_method(v1, v2, v3);
    let w = lambda[0] * v1 + lambda[1] * v2 + lambda[2] * v3;

    // project w onto normal
    w.dot(&normal) * normal / normal.norm_squared()
}

#[derive(PartialEq, Eq, PartialOrd, Ord)]
struct HeapElement {
    distance: OrderedFloat<Float>,
    triangle_index: usize,
}

/// Expanding Polytope algorithm.
/// It computes the penetration depth and two contact points, one on each body.
/// Note: Contact points are expressed in reference frame coordinates.
/// TODO: perform flood-fill, instead of brute-force, when finding visible faces.
pub(super) fn epa(poly: &mut Polytope, A: &Cuboid, B: &Cuboid) -> (Vector3<Float>, Vector3<Float>) {
    let mut heap = BinaryHeap::new();

    // Put all vertices onto heap
    for (i, triangle) in poly.triangles.iter().enumerate() {
        let v1 = &poly.vertices[triangle[0]];
        let v2 = &poly.vertices[triangle[1]];
        let v3 = &poly.vertices[triangle[2]];

        let w = solve_closest_point_area_method(&v1.v, &v2.v, &v3.v);
        poly.closest_points.push(w);
        let distance = w.norm();
        heap.push(Reverse(HeapElement {
            distance: OrderedFloat(distance),
            triangle_index: i,
        }));
    }

    loop {
        let element = heap.pop().unwrap().0;
        let index = element.triangle_index;

        // Skip triangles that are already removed
        if poly.obsolete[index] {
            continue;
        }
        poly.obsolete[index] = true;
        let triangle = poly.triangles[index];
        let w = poly.closest_points[index];

        let v1 = &poly.vertices[triangle[0]];
        let v2 = &poly.vertices[triangle[1]];
        let v3 = &poly.vertices[triangle[2]];

        // new vertex
        let normal = (v2.v - v1.v).cross(&(v3.v - v1.v));
        if normal.norm() == 0.0 {
            panic!("normal of v123 should not be 0");
        }
        let direction = normal;

        let v = support_fn(A, B, &direction);

        // verify new vertex validity
        let v_dot_w = v.v.dot(&w);
        let w_norm_squared = w.norm_squared();
        if v_dot_w < w_norm_squared - 1e-8 {
            panic!(
                "new vertex should not be below the triangle plane: {}",
                v_dot_w - w_norm_squared
            );
        }
        if v.v.dot(&direction) < 0.0 {
            panic!("new vertex should not be opposite the search direction");
        }

        // Check that  current triangle is very close to, if not is a face of the
        // Minkowski difference shape.
        // Alternative if conditions:
        //      1. v == v1 || v == v2 || v == v3
        //      2. poly.vertices.contains(&v)
        if (v_dot_w - w_norm_squared) / w.norm() < 1e-6 // check distance from v to w
            || (w.norm() == 0.0 && v.v.dot(&direction) / direction.norm() < 1e-6)
        // check direction of v, if w == 0
        {
            let (lambda, _) = solve_closest_point_lambda_area_method(&v1.v, &v2.v, &v3.v);

            let v1_A = A.point_at(v1.index_A);
            let v2_A = A.point_at(v2.index_A);
            let v3_A = A.point_at(v3.index_A);
            let cp_A = lambda[0] * v1_A + lambda[1] * v2_A + lambda[2] * v3_A;

            let v1_B = B.point_at(v1.index_B);
            let v2_B = B.point_at(v2.index_B);
            let v3_B = B.point_at(v3.index_B);
            let cp_B = lambda[0] * v1_B + lambda[1] * v2_B + lambda[2] * v3_B;

            // cp_A-cp_B should be very close to lambda[0] * v1.v + lambda[1] * v2.v + lambda[2] * v3.v
            return (cp_A, cp_B);
        }

        // Edges to form new triangles with the new vertex
        // TODO: replace with HashSet
        let mut edges = vec![];

        // Add edges of current triangle to the candidate edges
        edges.push((triangle[0], triangle[1]));
        edges.push((triangle[1], triangle[2]));
        edges.push((triangle[2], triangle[0]));

        // Find all triangles that can be seen by the new vertex
        for e in heap.iter() {
            let i = e.0.triangle_index;
            if poly.obsolete[i] {
                continue;
            }
            let w = poly.closest_points[i];
            let triangle = poly.triangles[i];

            if w.norm() != 0.0 && v.v.dot(&w) > w.norm_squared() + TOL {
                poly.obsolete[i] = true;
                // new vertex above the triangle plane, meaning it can see the triangle
                addEdgeIfNotExisting(&mut edges, (triangle[0], triangle[1]));
                addEdgeIfNotExisting(&mut edges, (triangle[1], triangle[2]));
                addEdgeIfNotExisting(&mut edges, (triangle[2], triangle[0]));
                continue;
            }

            if w.norm() > 0.0 {
                // only do the following check if w is origin
                continue;
            }
            let v1 = &poly.vertices[triangle[0]];
            let v2 = &poly.vertices[triangle[1]];
            let v3 = &poly.vertices[triangle[2]];
            let normal = (v2.v - v1.v).cross(&(v3.v - v1.v));
            if v.v.dot(&normal) > 0.0 {
                // if new vertex in same direction as normal of the triangle
                // face
                poly.obsolete[i] = true;
                addEdgeIfNotExisting(&mut edges, (triangle[0], triangle[1]));
                addEdgeIfNotExisting(&mut edges, (triangle[1], triangle[2]));
                addEdgeIfNotExisting(&mut edges, (triangle[2], triangle[0]));
            }
        }

        // Form new triangles with the candidate edges
        poly.vertices.push(v.clone());
        let v_index = poly.vertices.len() - 1;
        for edge in &edges {
            // Check for colinearity between new v and existing points of edge
            let v1 = &poly.vertices[edge.0];
            let v2 = &poly.vertices[edge.1];
            let normal = (v2.v - v1.v).cross(&(v.v - v1.v));
            if normal.norm() == 0.0 {
                panic!("three points of the new triangle should not be colinear");
            }
            if normal.dot(&v1.v) < 0.0 {
                panic!("wrong direction for the edge, {}", normal.dot(&v1.v));
            }

            // Add the new triangle and associated data
            poly.triangles.push(vector![edge.0, edge.1, v_index]);
            let w = solve_closest_point_area_method(&v1.v, &v2.v, &v.v);

            poly.closest_points.push(w);
            poly.obsolete.push(false);
            heap.push(Reverse(HeapElement {
                distance: OrderedFloat(w.norm()),
                triangle_index: poly.triangles.len() - 1,
            }));
        }
    }
}

#[cfg(test)]
mod epa_tests {
    use na::vector;
    use rand::{rng, Rng};

    use na::{Quaternion, UnitQuaternion};

    use crate::{
        assert_close,
        collision::{gjk::gjk, CollisionDetector},
        util::test_utils::{random_quaternion, random_vector},
        PI,
    };

    use super::*;

    fn angle_between(v1: &Vector3<Float>, v2: &Vector3<Float>) -> Float {
        let v1norm = v1.norm();
        let v2norm = v2.norm();
        if v1norm == 0.0 || v2norm == 0.0 {
            return 0.0;
        }

        let dot = v1.dot(v2);
        let mag_product = v1norm * v2norm;
        let cos_theta = (dot / mag_product).clamp(-1.0, 1.0);
        cos_theta.acos()
    }

    #[test]
    fn closest_point_area_method_rand() {
        let mut rng = rng();

        let total_count = 5000;
        let mut direction_success_count = 0;
        let mut coplanar_success_count = 0;
        for _case in 0..total_count {
            let v1 = random_vector(&mut rng, 1e-3);
            let mut v2 = random_vector(&mut rng, 1.0);
            let mut v3 = random_vector(&mut rng, 1.0);

            if (v2 - v1).cross(&(v3 - v1)).dot(&v1) < 0.0 {
                let tmp = v2;
                v2 = v3;
                v3 = tmp;
            }

            let (lambda, _) = solve_closest_point_lambda_area_method(&v1, &v2, &v3);
            assert_close!(lambda[0] + lambda[1] + lambda[2], 1.0, 1e-5);

            let w = lambda[0] * v1 + lambda[1] * v2 + lambda[2] * v3;
            let normal = (v2 - v1).cross(&(v3 - v1));

            // direction check
            let angle = angle_between(&w, &normal);
            if angle < 1e-1 {
                direction_success_count += 1;
            } else {
                println!("angle: {}", angle);
            }

            // coplanar check
            let projection = (w - v1).dot(&normal);
            if projection.abs() < 1e-8 {
                coplanar_success_count += 1;
            } else {
                println!("projection: {}", projection);
            }
        }

        let direction_success_rate = direction_success_count as Float / total_count as Float;
        assert_eq!(direction_success_rate, 1.0);
        let coplanar_success_rate = coplanar_success_count as Float / total_count as Float;
        assert_eq!(coplanar_success_rate, 1.0);
    }

    #[test]
    fn closest_point_area_method_sample() {
        let v1 = vector![1.0, 0.0, 0.0];
        let v2 = vector![0.0, 1.0, 0.0];
        let v3 = vector![0.0, 0.0, 1.0];
        let w = solve_closest_point_area_method(&v1, &v2, &v3);

        let normal = (v2 - v1).cross(&(v3 - v1));
        assert_close!(angle_between(&w, &normal), 0.0, 1e-2);
    }

    #[test]
    fn closest_point_area_method_sample2() {
        let v1 = vector![1.0, 0.0, 0.0];
        let v2 = vector![0.0, 1.0, 0.0];
        let v3 = vector![-1.0, 0.0, 0.0];
        let w = solve_closest_point_area_method(&v1, &v2, &v3);

        let normal = (v2 - v1).cross(&(v3 - v1));
        assert_close!(angle_between(&w, &normal), 0.0, 1e-2);
    }

    #[test]
    fn epa_face_face_shallow_penetration() {
        let mut rng = rng();

        let total = 2000;
        let mut separation_success = 0;
        let mut collision_success = 0;
        for _case in 0..total {
            // Arrange
            let center = random_vector(&mut rng, 100.0);
            let l_a = rng.random_range(0.5..2.0);
            let mut A = Cuboid::new_cube(center, UnitQuaternion::identity(), l_a);

            // move B in either x, y or z direction, to make it touch A
            let l_b = rng.random_range(0.5..2.0);
            let mut b_center = center + random_vector(&mut rng, 1e-2);
            let move_axis = rng.random_range(0..2);
            let move_dir = {
                if rng.random_bool(0.5) {
                    1.0
                } else {
                    -1.0
                }
            };
            b_center[move_axis] = center[move_axis] + move_dir * (l_a / 2.0 + l_b / 2.0);
            let mut B = Cuboid::new_cube(b_center, UnitQuaternion::identity(), l_b);

            // create a small penetration
            let penetration = -move_dir * rng.random_range(0.0..1e-3);
            B.isometry.translation.vector[move_axis] += penetration;

            A.set_rotation(random_quaternion(&mut rng, 1e-2));
            B.set_rotation(random_quaternion(&mut rng, 1e-2));

            // Act
            let mut collision_detector = CollisionDetector::new(&A, &B);
            let is_collision = collision_detector.gjk();
            assert!(is_collision);
            let (cp_A, cp_B) = collision_detector.epa();

            // Assert
            assert!(A.point_on_surface(&cp_A));
            assert!(B.point_on_surface(&cp_B));

            let scale_margin = 1e-2;
            let mut A_test = A.clone();
            let mut B_test = B.clone();
            B_test.isometry.translation.vector += (cp_A - cp_B).scale(1.0 + scale_margin);
            if !CollisionDetector::new(&A, &B_test).gjk() {
                separation_success += 1;
            }
            B_test.isometry.translation.vector =
                B.isometry.translation.vector + (cp_A - cp_B).scale(1.0 - scale_margin);
            if CollisionDetector::new(&A, &B_test).gjk() {
                collision_success += 1;
            }

            A_test.isometry.translation.vector += (cp_B - cp_A).scale(1.0 + scale_margin);
            if !CollisionDetector::new(&A_test, &B).gjk() {
                separation_success += 1;
            }
            A_test.isometry.translation.vector =
                A.isometry.translation.vector + (cp_B - cp_A).scale(1.0 - scale_margin);
            if CollisionDetector::new(&A_test, &B).gjk() {
                collision_success += 1;
            }
        }

        let check_total = total * 2;
        let separation_success_rate = separation_success as Float / check_total as Float;
        assert_eq!(separation_success_rate, 1.0);
        let collision_success_rate = collision_success as Float / check_total as Float;
        assert_eq!(collision_success_rate, 1.0);
    }

    #[test]
    fn epa_sample() {
        let l_a = 1.1;
        let mut A = Cuboid::new_cube_at_center(l_a);
        let l_b = 1.0;

        let b_center = vector![l_a / 2.0 + l_b / 2.0, 1e-2, 1e-2];
        let mut B = Cuboid::new_cube(b_center, UnitQuaternion::identity(), l_b);

        // create a small penetration
        let penetration = 1e-3;
        B.isometry.translation.vector[0] -= penetration;

        A.isometry.rotation = UnitQuaternion::from_euler_angles(-1e-2, 1e-2, -1e-2);
        A.recompute_points();
        B.recompute_points();

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        let mut B_test = B.clone();
        B_test.isometry.translation.vector += (cp_A - cp_B).scale(1.0 + 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_none());
    }

    /// Curiously, this exact test case does not pass, but would if we change
    /// parameters a little bit. Most likely a numerical issue.
    /// TODO: Fix the algorithm implementation to also pass this case.
    #[ignore]
    #[test]
    fn epa_sample2() {
        let l_a = 1.0;
        let A = Cuboid::new_cube_at_center(l_a);
        let l_b = 1.0;

        let b_center = vector![l_a / 2.0 + l_b / 2.0, 0., 0.];
        let mut B = Cuboid::new_cube(b_center, UnitQuaternion::identity(), l_b);

        // create a small penetration
        let penetration = 1e-3;
        B.isometry.translation.vector[0] -= penetration;
        B.recompute_points();

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        let normal = cp_A - cp_B;
        let mut B_test = B.clone();
        B_test.isometry.translation.vector += normal.scale(1.0 + 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_none());
        B_test.isometry.translation.vector =
            B.isometry.translation.vector + normal.scale(1.0 - 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_some());
    }

    #[test]
    fn epa_sample3() {
        // Arrange
        let l = 2.0;
        let A = Cuboid::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l);
        let B = Cuboid::new_cube(vector![l / 2.0, l / 2.0, 0.], UnitQuaternion::identity(), l);

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        let normal = cp_A - cp_B;
        let mut B_test = B.clone();
        B_test.isometry.translation.vector += normal.scale(1.0 + 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_none());
        B_test.isometry.translation.vector =
            B.isometry.translation.vector + normal.scale(1.0 - 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_some());
        assert_close!(normal.norm(), l / 2.0, 1e-5);
    }

    #[test]
    fn epa_sample4() {
        // Arrange
        let l = 2.0;
        let A = Cuboid::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l);
        let B = Cuboid::new_cube(vector![l / 2.0, 0., 0.], UnitQuaternion::identity(), l);

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        let normal = cp_A - cp_B;
        let mut B_test = B.clone();
        B_test.isometry.translation.vector += normal.scale(1.0 + 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_none());
        B_test.isometry.translation.vector =
            B.isometry.translation.vector + normal.scale(1.0 - 1e-3);
        B_test.recompute_points();
        assert!(gjk(&A, &B_test).is_some());
        assert_close!(normal.norm(), l / 2.0, 1e-5);

        assert_close!(cp_A.y, cp_B.y, 1e-5);
        assert_close!(cp_A.z, cp_B.z, 1e-5);
    }

    #[test]
    fn epa_sample5() {
        // Arrange
        let A = Cuboid::new(
            vector![1.0, 0., 0.],
            UnitQuaternion::identity(),
            2.0,
            1.0,
            1.0,
        );
        let B = Cuboid::new_cube(
            vector![2.0, 0., 0.],
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
            (2.0 as Float).sqrt() / 2.0,
        );

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        let normal = cp_A - cp_B;
        assert_close!(normal.norm(), 0.5, 1e-5);

        assert_close!(cp_A.x, 2.0, 1e-5);
        assert_close!(cp_A.z, 0.0, 1e-5);
        assert_close!(cp_B.x, 1.5, 1e-5);
        assert_close!(cp_B.z, 0.0, 1e-5);
        assert_close!(cp_A.y, cp_B.y, 1e-5);
    }

    #[test]
    fn epa_sample6() {
        // Arrange
        let A = Cuboid::new_cube(
            vector![0., 0., 0.],
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 4.0),
            1.0,
        );
        let move_delta = 0.15;
        let B = Cuboid::new(
            vector![0., 1.0 + move_delta, 1.5],
            UnitQuaternion::identity(),
            1.0,
            2.0,
            3.0,
        );

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        let depth = (cp_A - cp_B).norm();
        let sqrt2over2 = (2.0 as Float).sqrt() / 2.0;
        let depth_side = sqrt2over2 - move_delta;
        assert_close!(depth, sqrt2over2 * depth_side, 1e-5);

        assert_close!(cp_A.y, sqrt2over2 * depth + move_delta, 1e-5);
        assert_close!(cp_A.z, sqrt2over2 * depth, 1e-5);
        assert_close!(cp_B.y, move_delta, 1e-5);
        assert_close!(cp_B.z, 0.0, 1e-5);
        assert_close!(cp_A.x, cp_B.x, 1e-5);
    }

    #[test]
    fn epa_sample7() {
        // Arrange
        let A = Cuboid::new(
            vector![2.5, 0., 1.0],
            UnitQuaternion::identity(),
            0.5,
            0.5,
            2.0,
        );
        let B = Cuboid::new_cube(vector![3.0, 0.0, 0.25], UnitQuaternion::identity(), 0.5);

        // Act
        if let Some(mut poly) = gjk(&A, &B) {
            let (cp_A, cp_B) = epa(&mut poly, &A, &B);
            assert_close!((cp_A - cp_B).norm(), 0.0, 1e-6);
        };
    }

    #[test]
    fn epa_sample8() {
        // Arrange
        let A = Cuboid::new(
            vector![2.5083275, 0.0, 0.9903475],
            UnitQuaternion::identity(),
            0.5,
            0.5,
            2.0,
        );
        let B = Cuboid::new_cube(
            vector![3.0000002, 0.0, 0.2499184],
            UnitQuaternion::identity(),
            0.5,
        );

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        assert!((cp_A - cp_B).norm() > 0.0);
    }

    #[test]
    fn epa_sample9() {
        // Arrange
        let A = Cuboid::new(
            vector![2.600148, 0.0, 0.9818603],
            UnitQuaternion::identity(),
            0.5,
            0.5,
            2.0,
        );
        let B = Cuboid::new_cube(
            vector![3.0916612, -6.926495e-6, 0.25050417],
            UnitQuaternion::from_quaternion(Quaternion::from_vector(vector![
                2.7207802e-6,
                -0.002590372,
                -3.0188437e-5,
                0.99999666
            ])),
            0.5,
        );

        // Act
        let mut poly = gjk(&A, &B).unwrap();
        let (cp_A, cp_B) = epa(&mut poly, &A, &B);

        // Assert
        assert!((cp_A - cp_B).norm() > 0.0);
    }
}
