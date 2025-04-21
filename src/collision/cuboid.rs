use na::{vector, DVector, Isometry3, Matrix3xX, UnitQuaternion, Vector3};

use crate::types::Float;

#[derive(Clone, PartialEq, Debug)]
/// Box shape for collision detection
pub struct Cuboid {
    pub isometry: Isometry3<Float>, // transform from colllider frame to reference frame

    pub w: Float,
    pub d: Float,
    pub h: Float,

    points: Matrix3xX<Float>,
}

impl Cuboid {
    pub fn recompute_points(&mut self) {
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
        let mut points = self.isometry.rotation.to_rotation_matrix() * points;
        for mut col in points.column_iter_mut() {
            col += self.isometry.translation.vector;
        }

        self.points = points;
    }

    pub fn new(
        center: Vector3<Float>,
        rotation: UnitQuaternion<Float>,
        w: Float,
        d: Float,
        h: Float,
    ) -> Cuboid {
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
        let mut points = rotation.to_rotation_matrix() * points;
        for mut col in points.column_iter_mut() {
            col += center;
        }

        Cuboid {
            isometry: Isometry3::from_parts(center.into(), rotation),
            w,
            d,
            h,
            points,
        }
    }

    pub fn new_at_center(w: Float, d: Float, h: Float) -> Cuboid {
        Cuboid::new(Vector3::zeros(), UnitQuaternion::identity(), w, d, h)
    }

    /// Create a box with equal side lengths
    pub fn new_cube(center: Vector3<Float>, rotation: UnitQuaternion<Float>, l: Float) -> Cuboid {
        Cuboid::new(center, rotation, l, l, l)
    }

    pub fn new_cube_at_center(l: Float) -> Cuboid {
        Cuboid::new_cube(Vector3::zeros(), UnitQuaternion::identity(), l)
    }

    /// Returns the support point in the given direction, i.e. the furthest
    /// point in the given direction
    pub fn compute_support(&self, direction: &Vector3<Float>) -> (Vector3<Float>, usize) {
        let transformed_points = &self.points;

        // Dot product between direction and points
        let distances = direction.transpose() * transformed_points;

        let (max_index, _) = DVector::from_row_slice(distances.as_slice()).argmax();

        (transformed_points.column(max_index).into(), max_index)
    }

    pub fn point_at(&self, index: usize) -> Vector3<Float> {
        self.points.column(index).into()
    }

    pub fn set_rotation(&mut self, rotation: UnitQuaternion<Float>) {
        self.isometry.rotation = rotation;
    }

    /// Returns true if point is on the surface of the cuboid
    pub fn point_on_surface(&self, point: &Vector3<Float>) -> bool {
        // point position relative to cuboid center
        let point = point - self.isometry.translation.vector;

        // x, y, z axes in cuboid frame
        let rotation = self.isometry.rotation;
        let x_axis = rotation * Vector3::x_axis();
        let y_axis = rotation * Vector3::y_axis();
        let z_axis = rotation * Vector3::z_axis();

        // x, y, z coordinates in cuboid frame
        let x = point.dot(&x_axis);
        let y = point.dot(&y_axis);
        let z = point.dot(&z_axis);

        let half_w = self.w / 2.0;
        let half_d = self.d / 2.0;
        let half_h = self.h / 2.0;

        // First ensure that the point is close to being within the cuboid
        // interior
        let tol = 1e-3;
        if x.abs() > half_w + tol {
            return false;
        }
        if y.abs() > half_d + tol {
            return false;
        }
        if z.abs() > half_h + tol {
            return false;
        }

        // Check if on at least one face
        if half_w - x < tol || x - -half_w < tol {
            return true;
        }
        if half_d - y < tol || y - -half_d < tol {
            return true;
        }
        if half_h - z < tol || z - -half_h < tol {
            return true;
        }

        false
    }
}

#[cfg(test)]
mod cuboid_tests {
    use super::*;

    #[test]
    fn test_point_on_surface() {
        let cuboid = Cuboid::new_at_center(1.0, 2.0, 3.0);

        assert_eq!(cuboid.point_on_surface(&vector![0.0, 0.0, 0.0]), false);
        assert_eq!(cuboid.point_on_surface(&vector![0.5, 0.0, 0.0]), true);
        assert_eq!(cuboid.point_on_surface(&vector![0.1, 1.0, -0.1]), true);
    }
}
