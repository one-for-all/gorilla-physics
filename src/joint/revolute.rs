use na::{Isometry3, Matrix3, Matrix3xX, Matrix4, Translation3, UnitQuaternion, UnitVector3};
use nalgebra::Vector3;
use std::ops::Mul;

use crate::{
    spatial::{
        geometric_jacobian::GeometricJacobian, spatial_acceleration::SpatialAcceleration,
        transform::Transform3D,
    },
    types::Float,
};

/// Represents a revolute joint connecting a predecessor and a successor body.
///
/// Note: joint frame is defined as the successor body frame
pub struct RevoluteJoint {
    pub init_mat: Matrix4<Float>, // initial transform from successor frame to predecessor frame
    pub transform: Transform3D,   // transform from successor frame to predecessor frame
    pub axis: Vector3<Float>,     // axis of rotation expressed in successor body frame
}

impl RevoluteJoint {
    pub fn default() -> Self {
        RevoluteJoint {
            init_mat: Matrix4::identity(),
            transform: Transform3D::default(),
            axis: Vector3::z(),
        }
    }

    pub fn new(transform: Transform3D, axis: Vector3<Float>) -> Self {
        Self {
            init_mat: transform.mat.clone(),
            transform,
            axis,
        }
    }

    /// Return the spatial acceleration of the successor with respect
    /// to its predecessor, expressed in the successor frame.
    pub fn spatial_acceleration(&self, vdot: &Float) -> SpatialAcceleration {
        SpatialAcceleration {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: self.axis.mul(*vdot),
            linear: Vector3::zeros(),
        }
    }

    pub fn motion_subspace(&self) -> GeometricJacobian {
        GeometricJacobian {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: Matrix3xX::from_column_slice(self.axis.as_slice()),
            linear: Matrix3xX::zeros(1),
        }
    }

    /// Update the transform to be intial transform rotated around axis by q
    pub fn update(&mut self, q: &Float) {
        // TODO: get rid of using Matrix4 as transformation matrix. Use
        // Isometry3 instead. It is more precise.
        let init_translation = Translation3::new(
            self.init_mat[(0, 3)],
            self.init_mat[(1, 3)],
            self.init_mat[(2, 3)],
        );
        let init_rotation_matrix: Matrix3<Float> = self.init_mat.fixed_view::<3, 3>(0, 0).into();
        let init_rot: UnitQuaternion<Float> = UnitQuaternion::from_matrix(&init_rotation_matrix);
        let init_iso = Isometry3::from_parts(init_translation.into(), init_rot);
        let new_iso =
            init_iso * UnitQuaternion::from_axis_angle(&UnitVector3::new_normalize(self.axis), *q);

        self.transform = Transform3D {
            from: self.transform.from.clone(),
            to: self.transform.to.clone(),
            mat: new_iso.to_homogeneous(),
        };
    }
}

#[cfg(test)]
mod revolute_tests {
    use super::*;

    #[test]
    fn test_joint_spatial_acceleration() {
        // Arrange
        let joint = RevoluteJoint::default();
        let vdot = 1.0;

        // Act
        let spatial_acceleration = joint.spatial_acceleration(&vdot);

        // Assert
        assert_eq!(spatial_acceleration.angular, Vector3::new(0.0, 0.0, vdot));
    }
}
