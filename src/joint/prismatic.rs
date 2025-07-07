use na::{zero, Isometry3, Matrix3, Matrix3xX, Matrix4, Transform3, Translation3, UnitQuaternion};
use nalgebra::Vector3;

use crate::{
    spatial::{
        geometric_jacobian::GeometricJacobian, spatial_acceleration::SpatialAcceleration,
        transform::Transform3D,
    },
    types::Float,
};

/// Give springiness to a joint
pub struct JointSpring {
    pub k: Float, // spring constant
    pub l: Float, // rest length
}

/// Represents a prismatic joint connecting a predecessor and a successor body.
///
/// Note: joint frame is defined as the successor body frame
pub struct PrismaticJoint {
    pub init_mat: Matrix4<Float>,
    pub transform: Transform3D,
    pub axis: Vector3<Float>, // axis expressed in successor body frame

    pub spring: Option<JointSpring>,
}

impl PrismaticJoint {
    pub fn new(transform: Transform3D, axis: Vector3<Float>) -> Self {
        Self {
            init_mat: transform.iso.to_homogeneous(),
            transform,
            axis,
            spring: None,
        }
    }

    pub fn new_with_spring(
        transform: Transform3D,
        axis: Vector3<Float>,
        spring: JointSpring,
    ) -> Self {
        Self {
            init_mat: transform.iso.to_homogeneous(),
            transform,
            axis,
            spring: Some(spring),
        }
    }

    /// Return the spatial acceleration of the successor with respect
    /// to its predecessor, expressed in the successor frame.
    pub fn spatial_acceleration(&self, vdot: &Float) -> SpatialAcceleration {
        SpatialAcceleration {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: zero(),
            linear: self.axis * (*vdot),
        }
    }

    pub fn motion_subspace(&self) -> GeometricJacobian {
        GeometricJacobian {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: Matrix3xX::zeros(1),
            linear: Matrix3xX::from_column_slice(self.axis.as_slice()),
        }
    }

    /// Update the transform to be intial transform moved along axis by q
    pub fn update(&mut self, q: &Float) {
        let mat = self.init_mat * Transform3D::translation(&self.axis, q);
        let translation = Translation3::new(mat[(0, 3)], mat[(1, 3)], mat[(2, 3)]);
        let rotation_matrix: Matrix3<Float> = mat.fixed_view::<3, 3>(0, 0).into();
        let rot: UnitQuaternion<Float> = UnitQuaternion::from_matrix(&rotation_matrix);
        let iso = Isometry3::from_parts(translation.into(), rot);

        self.transform = Transform3D {
            from: self.transform.from.clone(),
            to: self.transform.to.clone(),
            iso: iso,
        };
    }
}
