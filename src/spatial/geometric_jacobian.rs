use na::Matrix3xX;
use std::ops::Mul;

use crate::{spatial::transform::Transform3D, types::Float, util::colwise_cross};

/// A geometric Jacobian maps a vector of joint velocities to a twist.
#[derive(PartialEq, Debug)]
pub struct GeometricJacobian {
    pub body: String,
    pub base: String,
    pub frame: String,
    pub angular: Matrix3xX<Float>,
    pub linear: Matrix3xX<Float>,
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
        let angular = rot.mul(&self.angular);
        let linear = rot.mul(&self.linear) + colwise_cross(&trans, &angular);

        GeometricJacobian {
            body: self.body.clone(),
            base: self.base.clone(),
            frame: transform.to.clone(),
            angular,
            linear,
        }
    }

    pub fn dim(&self) -> usize {
        if self.angular.ncols() != self.linear.ncols() {
            panic!("Geometric Jacobian's angular dimension does not match linear dimension");
        }

        self.angular.ncols()
    }
}
