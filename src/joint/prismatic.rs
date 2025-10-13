use na::{zero, Isometry3, Matrix3xX, Translation3, UnitVector3};
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
    pub init_iso: Isometry3<Float>,
    pub transform: Transform3D,
    pub axis: UnitVector3<Float>, // axis expressed in successor body frame

    pub v: Float,

    pub spring: Option<JointSpring>,
}

impl PrismaticJoint {
    pub fn new(transform: Transform3D, axis: UnitVector3<Float>) -> Self {
        Self {
            init_iso: transform.iso,
            transform,
            axis,
            v: 0.,
            spring: None,
        }
    }

    pub fn new_with_spring(
        transform: Transform3D,
        axis: UnitVector3<Float>,
        spring: JointSpring,
    ) -> Self {
        Self {
            init_iso: transform.iso,
            transform,
            axis,
            v: 0.,
            spring: Some(spring),
        }
    }

    /// Return the spatial acceleration of the successor with respect
    /// to its predecessor, expressed in the successor frame.
    pub fn spatial_acceleration(&self, vdot: Float) -> SpatialAcceleration {
        SpatialAcceleration {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: zero(),
            linear: self.axis.scale(vdot),
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
    pub fn update(&mut self, q: Float) {
        let iso = self.init_iso * Translation3::from(self.axis.scale(q));

        self.transform = Transform3D {
            from: self.transform.from.clone(),
            to: self.transform.to.clone(),
            iso: iso,
        };
    }
}
