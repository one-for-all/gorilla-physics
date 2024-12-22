use na::Vector3;
use std::ops::Mul;

use crate::{transform::Transform3D, types::Float};

/// A geometric Jacobian maps a vector of joint velocities to a twist.
/// Currently, it only supports revolute joints.
///
/// Given a joint velocity value, it gives out the corresponding twist.
#[derive(PartialEq, Debug)]
pub struct GeometricJacobian {
    pub body: String,
    pub base: String,
    pub frame: String,
    pub angular: Vector3<Float>,
    pub linear: Vector3<Float>,
}

impl GeometricJacobian {
    /// Transform the twist to be expressed in the "to" frame of transform
    pub fn transform(&self, transform: &Transform3D) -> GeometricJacobian {
        if self.frame != transform.from {
            panic!(
                "twist {} frame is not equal to transform `from` {} frame!",
                self.frame, transform.from
            );
        }

        let rot = transform.rot();
        let trans = transform.trans();
        let angular = rot.mul(self.angular);
        let linear = rot.mul(self.linear) + trans.cross(&angular);

        GeometricJacobian {
            body: self.body.clone(),
            base: self.base.clone(),
            frame: transform.to.clone(),
            angular,
            linear,
        }
    }
}
