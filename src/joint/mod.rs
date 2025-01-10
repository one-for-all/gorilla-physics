use na::Vector3;
use prismatic::PrismaticJoint;
use revolute::RevoluteJoint;

use crate::{
    geometric_jacobian::GeometricJacobian, spatial_acceleration::SpatialAcceleration,
    transform::Transform3D, types::Float,
};

pub mod prismatic;
pub mod revolute;

pub enum Joint {
    RevoluteJoint(RevoluteJoint),
    PrismaticJoint(PrismaticJoint),
}

impl Joint {
    pub fn transform(&self) -> &Transform3D {
        match self {
            Joint::RevoluteJoint(joint) => &joint.transform,
            Joint::PrismaticJoint(joint) => &joint.transform,
        }
    }

    pub fn axis(&self) -> &Vector3<Float> {
        match self {
            Joint::RevoluteJoint(joint) => &joint.axis,
            Joint::PrismaticJoint(joint) => &joint.axis,
        }
    }

    pub fn motion_subspace(&self) -> GeometricJacobian {
        match self {
            Joint::RevoluteJoint(joint) => joint.motion_subspace(),
            Joint::PrismaticJoint(joint) => joint.motion_subspace(),
        }
    }

    /// Return the spatial acceleration of the successor with respect
    /// to its predecessor, expressed in the successor frame.
    pub fn spatial_acceleration(&self, vdot: &Float) -> SpatialAcceleration {
        match self {
            Joint::RevoluteJoint(joint) => joint.spatial_acceleration(vdot),
            Joint::PrismaticJoint(joint) => joint.spatial_acceleration(vdot),
        }
    }
}
