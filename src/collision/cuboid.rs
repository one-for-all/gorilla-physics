use na::{vector, DVector, Matrix3xX, UnitQuaternion, Vector3};

use crate::types::Float;

#[derive(Clone, PartialEq, Debug)]
/// Box shape for collision detection
pub struct Cuboid {
    pub center: Vector3<Float>,
    pub rotation: UnitQuaternion<Float>,

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
        let mut points = self.rotation.to_rotation_matrix() * points;
        for mut col in points.column_iter_mut() {
            col += self.center;
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
            center,
            rotation,
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
}
