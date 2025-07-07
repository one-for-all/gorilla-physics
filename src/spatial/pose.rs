use na::{Isometry3, Matrix4, Translation3, UnitQuaternion, Vector3};

use crate::types::Float;

#[derive(Clone, Debug, PartialEq, Copy)]
pub struct Pose {
    pub rotation: UnitQuaternion<Float>,
    pub translation: Vector3<Float>,
}

impl Pose {
    pub fn to_matrix(&self) -> Matrix4<Float> {
        let mut matrix = Matrix4::identity();
        let rotation = self.rotation.to_rotation_matrix();

        matrix
            .view_mut((0, 0), (3, 3))
            .copy_from(&rotation.matrix());
        matrix.view_mut((0, 3), (3, 1)).copy_from(&self.translation);
        matrix
    }

    pub fn identity() -> Self {
        Pose {
            rotation: UnitQuaternion::identity(),
            translation: Vector3::zeros(),
        }
    }

    pub fn to_isometry(&self) -> Isometry3<Float> {
        let translation = Translation3::from(self.translation);
        Isometry3::from_parts(translation, self.rotation)
    }
}
