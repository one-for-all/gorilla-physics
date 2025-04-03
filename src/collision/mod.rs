use std::collections::HashSet;

use na::{
    vector, zero, DVector, Matrix3, Matrix3x1, Matrix3xX, UnitQuaternion, UnitVector3, Vector3,
};
use polytope::{Polytope, Vertex};

use crate::{types::Float, PI};

pub mod polytope;

/// Box shape for collision detection
pub struct Box {
    pub center: Vector3<Float>,
    pub rotation: UnitQuaternion<Float>,

    pub w: Float,
    pub d: Float,
    pub h: Float,
}

impl Box {
    pub fn new(
        center: Vector3<Float>,
        rotation: UnitQuaternion<Float>,
        w: Float,
        d: Float,
        h: Float,
    ) -> Box {
        Box {
            center,
            rotation,
            w,
            d,
            h,
        }
    }

    /// Create a box with equal side lengths
    pub fn new_cube(center: Vector3<Float>, rotation: UnitQuaternion<Float>, l: Float) -> Box {
        Box {
            center,
            rotation,
            w: l,
            d: l,
            h: l,
        }
    }

    /// Returns the support point in the given direction, i.e. the furthest
    /// point in the given direction
    pub fn support_point(&self, direction: UnitVector3<Float>) -> Vector3<Float> {
        let w = self.w;
        let d = self.d;
        let h = self.h;
        let points = Matrix3xX::from_columns(&[
            vector![w / 2.0, d / 2.0, h / 2.0],
            vector![-w / 2.0, d / 2.0, h / 2.0],
            vector![w / 2.0, -d / 2.0, h / 2.0],
            vector![-w / 2.0, -d / 2.0, h / 2.0],
            vector![w / 2.0, d / 2.0, -h / 2.0],
            vector![-w / 2.0, d / 2.0, -h / 2.0],
            vector![w / 2.0, -d / 2.0, -h / 2.0],
            vector![-w / 2.0, -d / 2.0, -h / 2.0],
        ]);

        // Apply rotation and translation
        let mut transformed_points = self.rotation.to_rotation_matrix() * points;
        for mut col in transformed_points.column_iter_mut() {
            col += self.center;
        }

        // Dot product between direction and points
        let distances = direction.transpose() * &transformed_points;

        let (max_index, _) = DVector::from_row_slice(distances.as_slice()).argmax();
        transformed_points.column(max_index).into()
    }
}

/// Computes the vertex in Minkowski polytope of (a-b), in given direction
fn compute_vertex(a: &Box, b: &Box, direction: UnitVector3<Float>) -> Vertex {
    let sp_a = a.support_point(direction);
    let sp_b = b.support_point(-direction);
    Vertex::new(sp_a, sp_b)
}

/// Perform GJK collision detection.
/// Return:
///     whether in collision
///     a polytope, in Minkowski difference space, that encloses the origin
/// TODO: clean up repetitive code.
pub fn gjk(a: &Box, b: &Box) -> (bool, Polytope) {
    let origin = Vector3::zeros();

    let direction = {
        if a.center == b.center {
            Vector3::x_axis() // a randomly picked direction
        } else {
            UnitVector3::new_normalize(a.center - b.center)
        }
    };
    let mut w1 = compute_vertex(a, b, direction);

    if w1.v == origin {
        // origin on a vertex, which means two shapes are touching
        return (false, Polytope::empty());
    }
    let direction = UnitVector3::new_normalize(origin - w1.v);
    let mut w2 = compute_vertex(a, b, direction);

    if w2.v == origin {
        // origin on a vertex, which means two shapes are touching
        return (false, Polytope::empty());
    }
    if w2.v.dot(&direction) < 0.0 {
        // w2 is not past origin, which means that the whole convex polytope
        // cannot enclose origin.
        return (false, Polytope::empty());
    }

    let normal_ow1w2 = w1.v.cross(&w2.v);
    if normal_ow1w2.norm() == 0.0 {
        // origin on edge w1w2

        // Construct a tetrahedron from line segment
        let direction_w1w2 = UnitVector3::new_normalize(w2.v - w1.v);
        // Find the one-vector ei that has max angle with w1w2 line, so that
        // their cross-product can have max norm >> 0, to make the direction
        // computation well-behaved.
        let mut min_component = Float::INFINITY;
        let mut min_i = 0;
        for (i, component) in direction_w1w2.iter().enumerate() {
            if component.abs() < min_component {
                min_component = component.abs();
                min_i = i;
            }
        }
        let mut ei = vector![0., 0., 0.];
        ei[min_i] = 1.;
        let ei = UnitVector3::new_normalize(ei);

        // Find a vertex perpendicular to w1w2
        let direction = UnitVector3::new_normalize(direction_w1w2.cross(&ei));
        let v1 = compute_vertex(a, b, direction);

        // Find another vertex perpendicular to w1w2, rotated by 120 degrees.
        let direction =
            UnitQuaternion::from_axis_angle(&direction_w1w2, 2.0 * PI / 3.0) * direction;
        let v2 = compute_vertex(a, b, direction);

        return (true, Polytope::tetrahedron(vec![w1, w2, v1, v2]));
    }

    let normal_w1w2_to_o = UnitVector3::new_normalize(normal_ow1w2.cross(&(w2.v - w1.v)));
    let direction = normal_w1w2_to_o;
    let mut w3 = compute_vertex(a, b, direction);

    if w3.v == origin {
        // origin on a vertex, which means two shapes are touching
        return (false, Polytope::empty());
    }
    if w3.v.dot(&direction) < 0.0 {
        // w3 is not past origin, which means that the whole convex polytope
        // cannot enclose origin.
        return (false, Polytope::empty());
    }

    let mut normal = UnitVector3::new_normalize((w2.v - w1.v).cross(&(w3.v - w1.v)));
    if normal.dot(&(origin - w1.v)) < 0.0 {
        normal = -normal;

        // correct the orientation of face w123
        let tmp = w2;
        w2 = w3;
        w3 = tmp;
    }
    let direction = normal;
    let mut w4 = compute_vertex(a, b, direction);

    if w4.v == origin {
        // origin on a vertex, which means two shapes are touching
        return (false, Polytope::empty());
    }
    if w4.v.dot(&direction) < 0.0 {
        // new vertex is not past origin, which means that the whole convex polytope
        // cannot enclose origin.
        return (false, Polytope::empty());
    }

    // Before and throughout the loop, it is ensured that w123 is clockwise
    // when viewed from outside the tetrahedron
    loop {
        let normal_w124 = UnitVector3::new_normalize((w2.v - w1.v).cross(&(w4.v - w1.v)));
        if normal_w124.dot(&(w3.v - w1.v)) > 0.0 {
            panic!("wrong face direction");
        }
        if normal_w124.dot(&(origin - w1.v)) > 0.0 {
            // origin outside tetrahedron, so expand in this direction
            let direction = normal_w124;
            let w_new = compute_vertex(a, b, direction);

            if w_new.v == origin {
                // origin on a vertex, which means two shapes are touching
                return (false, Polytope::empty());
            }
            if w_new.v.dot(&direction) < 0.0 {
                // new vertex is not past origin, which means that the whole
                // convex polytope cannot enclose origin.
                return (false, Polytope::empty());
            }

            w3 = w4;
            w4 = w_new;
            continue;
        }

        let normal_w134 = UnitVector3::new_normalize((w4.v - w1.v).cross(&(w3.v - w1.v)));
        if normal_w134.dot(&(w2.v - w1.v)) > 0.0 {
            panic!("wrong face direction");
        }
        if normal_w134.dot(&(origin - w1.v)) > 0.0 {
            // origin outside tetrahedron, so expand in this direction
            let direction = normal_w134;
            let w_new = compute_vertex(a, b, direction);

            if w_new.v == origin {
                // origin on a vertex, which means two shapes are touching
                return (false, Polytope::empty());
            }
            if w_new.v.dot(&direction) < 0.0 {
                // new vertex is not past origin, which means that the whole
                // convex polytope cannot enclose origin.
                return (false, Polytope::empty());
            }

            w2 = w4;
            w4 = w_new;
            continue;
        }

        let normal_w234 = UnitVector3::new_normalize((w3.v - w2.v).cross(&(w4.v - w2.v)));
        if normal_w234.dot(&(w1.v - w2.v)) > 0.0 {
            panic!("wrong face direction");
        }
        if normal_w234.dot(&(origin - w2.v)) > 0.0 {
            // origin outside tetrahedron, so expand in this direction
            let direction = normal_w234;
            let w_new = compute_vertex(a, b, direction);

            if w_new.v == origin {
                // origin on a vertex, which means two shapes are touching
                return (false, Polytope::empty());
            }
            if w_new.v.dot(&direction) < 0.0 {
                // new vertex is not past origin, which means that the whole
                // convex polytope cannot enclose origin.
                return (false, Polytope::empty());
            }

            w1 = w4;
            w4 = w_new;
            continue;
        }

        // origin within the current tetrahedron
        return (true, Polytope::tetrahedron(vec![w1, w2, w3, w4]));
    }
}

/// Expanding Polytope algorithm. It computes the penetration depth and
/// two contact points, one on each body.
/// Note: Contact points are expressed in world coordinate.
/// TODO: perform flood-fill, instead of brute-force, when finding visible faces.
pub fn epa(mut poly: Polytope, a: &Box, b: &Box) -> (Float, Vector3<Float>, Vector3<Float>) {
    // TODO: store and re-use the computations for each face
    loop {
        let mut min_distance = Float::INFINITY;
        let mut closest_normal = UnitVector3::new_normalize(Vector3::x());
        let mut closest_face: &Vector3<usize> = &zero();
        let mut closest_lambda: Vector3<Float> = zero();
        let mut vs = vec![];
        for face in poly.triangles.iter() {
            let w0 = &poly.vertices[face[0]];
            let w1 = &poly.vertices[face[1]];
            let w2 = &poly.vertices[face[2]];
            let normal = UnitVector3::new_normalize((w1.v - w0.v).cross(&(w2.v - w0.v)));
            let distance = w0.v.dot(&normal);

            if distance < 0.0 {
                panic!("wrong normal direction");
            }

            let w0_w1 = w1.v - w0.v;
            let w0_w2 = w2.v - w0.v;

            // Solve for v = λw, the closest point on the face to the origin.
            // λ must sum up to 1, and v must be perpendicular to w0_w1 and
            // w0_w2, which gives the following linear equation Aλ = b.
            #[rustfmt::skip]
            let A = Matrix3::new(
                1.0,              1.0,              1.0,
                w0.v.dot(&w0_w1), w1.v.dot(&w0_w1), w2.v.dot(&w0_w1),
                w0.v.dot(&w0_w2), w1.v.dot(&w0_w2), w2.v.dot(&w0_w2),
            );
            let b = Matrix3x1::new(1.0, 0., 0.);
            let lambda = A.lu().solve(&b).unwrap();
            let v = lambda[0] * w0.v + lambda[1] * w1.v + lambda[2] * w2.v;

            vs.push(v);

            if distance < min_distance {
                min_distance = distance;
                closest_normal = normal;
                closest_face = face;
                closest_lambda = lambda;
            }
        }

        if min_distance == Float::INFINITY {
            panic!("no faces in polytope");
        }

        let direction = closest_normal;
        let new_w = compute_vertex(a, b, direction);

        if (closest_normal.dot(&new_w.v) - min_distance).abs() < 1e-5 {
            // new w is close enough to the existing closest face

            // Compute contact point on A and B.
            // cp_a = λ * sp_a, cp_b = λ * sp_b
            let w0 = &poly.vertices[closest_face[0]];
            let w1 = &poly.vertices[closest_face[1]];
            let w2 = &poly.vertices[closest_face[2]];
            let lambda = closest_lambda;
            let contact_point_a = lambda[0] * w0.sp_a + lambda[1] * w1.sp_a + lambda[2] * w2.sp_a;
            let contact_point_b = lambda[0] * w0.sp_b + lambda[1] * w1.sp_b + lambda[2] * w2.sp_b;

            return (min_distance, contact_point_a, contact_point_b);
        }

        let mut edges: HashSet<(usize, usize)> = HashSet::new();
        for (i, v) in vs.iter().enumerate() {
            if v.dot(&new_w.v) >= v.dot(&v) {
                // face visible from new_w, so add its edges
                let triangle = poly.triangles[i];
                addEdgeIfNotExisting(&mut edges, (triangle[0], triangle[1]));
                addEdgeIfNotExisting(&mut edges, (triangle[1], triangle[2]));
                addEdgeIfNotExisting(&mut edges, (triangle[2], triangle[0]));
            }
        }

        for (i, v) in vs.iter().enumerate().rev() {
            if v.dot(&new_w.v) >= v.dot(&v) {
                // remove the visible faces
                poly.triangles.remove(i);
            }
        }

        // Construct new faces
        poly.vertices.push(new_w);
        let new_w_index = poly.vertices.len() - 1;
        for edge in edges {
            let w0_index = edge.0;
            let w1_index = edge.1;
            poly.triangles
                .push(vector![w0_index, w1_index, new_w_index]);
        }
    }
}

fn addEdgeIfNotExisting(E: &mut HashSet<(usize, usize)>, edge: (usize, usize)) {
    // remove reverse edge
    if !E.remove(&(edge.1, edge.0)) {
        // add if not existing
        E.insert(edge);
    }
}

#[cfg(test)]
mod collision_tests {
    use crate::{assert_close, PI};

    use super::*;

    #[test]
    fn box_collision_checking_1() {
        // Arrange
        let box1 = Box::new(Vector3::zeros(), UnitQuaternion::identity(), 3.0, 1.0, 2.0);
        let box2 = Box::new_cube(
            vector![1.5, 0.0, 1.0],
            UnitQuaternion::from_euler_angles(0., PI / 4.0, 0.0),
            1.0,
        );

        // Act
        let (is_colliding, _) = gjk(&box1, &box2);

        // Assert
        assert_eq!(is_colliding, true);
    }

    #[test]
    fn box_collision_checking_2() {
        // Arrange
        let box1 = Box::new_cube(Vector3::zeros(), UnitQuaternion::identity(), 1.0);
        let box2 = Box::new_cube(vector![1.5, 0.0, 1.0], UnitQuaternion::identity(), 1.0);

        // Act
        let (is_colliding, _) = gjk(&box1, &box2);

        // Assert
        assert_eq!(is_colliding, false);
    }

    #[test]
    fn box_collision_checking_3() {
        // Arrange
        let l = 2.0;
        let distance: Float = (2.0 as Float).sqrt();
        let box1 = Box::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l);
        let box2 = Box::new_cube(
            vector![-distance, 0.0, distance],
            UnitQuaternion::identity(),
            l,
        );

        // Act
        let (is_colliding, _) = gjk(&box1, &box2);

        // Assert
        assert_eq!(is_colliding, true);
    }

    #[test]
    fn box_collision_checking_4() {
        // Arrange
        let l = 2.0;
        let distance: Float = (2.0 as Float).sqrt();
        let box1 = Box::new_cube(
            Vector3::zeros(),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
            l,
        );
        let box2 = Box::new_cube(
            vector![-distance - 1e-3, 0.0, distance],
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
            l,
        );

        // Act
        let (is_colliding, _) = gjk(&box1, &box2);

        // Assert
        assert_eq!(is_colliding, false);
    }

    #[test]
    fn box_collision_penetration_1() {
        // Arrange
        let l = 2.0;
        let box1 = Box::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l);
        let box2 = Box::new_cube(vector![l / 2.0, l / 2.0, 0.], UnitQuaternion::identity(), l);

        // Act
        let (is_colliding, poly) = gjk(&box1, &box2);
        assert_eq!(is_colliding, true);
        let (depth, _, _) = epa(poly, &box1, &box2);

        // Assert
        assert_close!(depth, l / 2.0, 1e-5);
    }

    #[test]
    fn box_collision_penetration_1_1() {
        // Arrange
        let l = 2.0;
        let box1 = Box::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l);
        let box2 = Box::new_cube(vector![l / 2.0, 0., 0.], UnitQuaternion::identity(), l);

        // Act
        let (is_colliding, poly) = gjk(&box1, &box2);
        assert_eq!(is_colliding, true);
        let (depth, cp_a, cp_b) = epa(poly, &box1, &box2);

        // Assert
        assert_close!(depth, l / 2.0, 1e-5);
        assert_close!(cp_a.y, cp_b.y, 1e-5);
        assert_close!(cp_a.z, cp_b.z, 1e-5);
    }

    #[test]
    fn box_collision_penetration_2() {
        // Arrange=
        let box1 = Box::new(
            vector![1.0, 0., 0.],
            UnitQuaternion::identity(),
            2.0,
            1.0,
            1.0,
        );
        let box2 = Box::new_cube(
            vector![2.0, 0., 0.],
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.0),
            (2.0 as Float).sqrt() / 2.0,
        );

        // Act
        let (is_colliding, poly) = gjk(&box1, &box2);
        assert_eq!(is_colliding, true);
        let (depth, cp_a, cp_b) = epa(poly, &box1, &box2);

        // Assert
        assert_close!(depth, 0.5, 1e-5);

        assert_close!(cp_a.x, 2.0, 1e-5);
        assert_close!(cp_a.z, 0.0, 1e-5);
        assert_close!(cp_b.x, 1.5, 1e-5);
        assert_close!(cp_b.z, 0.0, 1e-5);
        assert_close!(cp_a.y, cp_b.y, 1e-5);
    }

    #[test]
    fn box_collision_penetration_3() {
        // Arrange
        let box1 = Box::new_cube(
            vector![0., 0., 0.],
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 4.0),
            1.0,
        );
        let move_delta = 0.15;
        let box2 = Box::new(
            vector![0., 1.0 + move_delta, 1.5],
            UnitQuaternion::identity(),
            1.0,
            2.0,
            3.0,
        );

        // Act
        let (is_colliding, poly) = gjk(&box1, &box2);
        assert_eq!(is_colliding, true);
        let (depth, cp_a, cp_b) = epa(poly, &box1, &box2);

        // Assert
        let sqrt2over2 = (2.0 as Float).sqrt() / 2.0;
        let depth_side = sqrt2over2 - move_delta;
        assert_close!(depth, sqrt2over2 * depth_side, 1e-5);

        assert_close!(cp_a.y, sqrt2over2 * depth + move_delta, 1e-5);
        assert_close!(cp_a.z, sqrt2over2 * depth, 1e-5);
        assert_close!(cp_b.y, move_delta, 1e-5);
        assert_close!(cp_b.z, 0.0, 1e-5);
        assert_close!(cp_a.x, cp_b.x, 1e-5);
    }
}
