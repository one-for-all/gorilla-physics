use na::{zero, Matrix3xX, Matrix4};
use nalgebra::Vector3;

use crate::{
    geometric_jacobian::GeometricJacobian, spatial_acceleration::SpatialAcceleration,
    transform::Transform3D, types::Float,
};

/// Represents a prismatic joint connecting a predecessor and a successor body.
///
/// Note: joint frame is defined as the successor body frame
pub struct PrismaticJoint {
    pub init_mat: Matrix4<Float>,
    pub transform: Transform3D,
    pub axis: Vector3<Float>, // axis expressed in successor body frame
}

impl PrismaticJoint {
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
        self.transform = Transform3D {
            from: self.transform.from.clone(),
            to: self.transform.to.clone(),
            mat: self.init_mat * Transform3D::translation(&self.axis, q),
        };
    }
}
