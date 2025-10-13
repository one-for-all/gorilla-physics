use na::{DMatrix, Isometry3, Matrix3xX, Matrix6xX};
use std::ops::Mul;

use crate::{
    inertia::SpatialInertia,
    spatial::{pose::Pose, transform::Transform3D},
    types::Float,
    util::colwise_cross,
};

pub struct Momentum {
    pub angular: Matrix3xX<Float>,
    pub linear: Matrix3xX<Float>,
}

impl Momentum {
    pub fn mul(inertia: &SpatialInertia, motion_subspace: &MotionSubspace) -> Self {
        let Jw = &motion_subspace.angular;
        let Jv = &motion_subspace.linear;
        let J = inertia.moment;
        let m = inertia.mass;
        let c = inertia.cross_part;

        let ang = J * Jw + colwise_cross(&c, &Jv);
        let lin = m * Jv - colwise_cross(&c, &Jw);

        Self {
            angular: ang,
            linear: lin,
        }
    }

    /// The result should be a joint-space inertia matrix
    pub fn transpose_mul(&self, motion_subspace: &MotionSubspace) -> DMatrix<Float> {
        let result = self.angular.transpose() * &motion_subspace.angular
            + self.linear.transpose() * &motion_subspace.linear;
        result
    }
}

pub struct MotionSubspace {
    pub angular: Matrix3xX<Float>,
    pub linear: Matrix3xX<Float>,
}

impl MotionSubspace {
    pub fn new(angular: Matrix3xX<Float>, linear: Matrix3xX<Float>) -> Self {
        Self { angular, linear }
    }

    pub fn transform(&self, pose: &Pose) -> Self {
        let rot = pose.rotation.to_rotation_matrix();
        let trans = pose.translation;
        let angular = rot * &self.angular;
        let linear = rot * &self.linear + colwise_cross(&trans, &angular);

        Self { angular, linear }
    }
}

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
    pub fn as_s(&self) -> MotionSubspace {
        MotionSubspace::new(self.angular.clone(), self.linear.clone())
    }

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

    pub fn as_matrix(&self) -> Matrix6xX<Float> {
        let mut out = Matrix6xX::<Float>::zeros(self.angular.ncols());

        out.view_mut((0, 0), (3, self.angular.ncols()))
            .copy_from(&self.angular);
        out.view_mut((3, 0), (3, self.linear.ncols()))
            .copy_from(&self.linear);
        out
    }
}
