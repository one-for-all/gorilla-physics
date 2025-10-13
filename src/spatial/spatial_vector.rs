use std::ops::{Add, Div, Mul};

use crate::{
    spatial::{pose::Pose, transform::Transform3D},
    types::Float,
};
use na::{dvector, zero, DVector, Vector3};

#[derive(Clone, Debug, Copy)]
pub struct SpatialVector {
    pub angular: Vector3<Float>,
    pub linear: Vector3<Float>,
}

impl SpatialVector {
    pub fn zero() -> Self {
        SpatialVector {
            angular: zero(),
            linear: zero(),
        }
    }

    pub fn transform(&self, pose: &Pose) -> SpatialVector {
        let rot = pose.rotation;
        let trans = pose.translation;

        let angular = rot.mul(self.angular);
        let linear = rot.mul(self.linear) + trans.cross(&angular);

        let result = SpatialVector { angular, linear };

        result
    }

    pub fn new(angular: Vector3<Float>, linear: Vector3<Float>) -> Self {
        SpatialVector { angular, linear }
    }

    pub fn angular(angular: Vector3<Float>) -> Self {
        SpatialVector {
            angular,
            linear: zero(),
        }
    }

    pub fn linear(linear: Vector3<Float>) -> Self {
        SpatialVector {
            angular: zero(),
            linear,
        }
    }

    pub fn as_dvector(&self) -> DVector<Float> {
        dvector![
            self.angular.x,
            self.angular.y,
            self.angular.z,
            self.linear.x,
            self.linear.y,
            self.linear.z
        ]
    }
}

impl Mul<Float> for &SpatialVector {
    type Output = SpatialVector;

    fn mul(self, rhs: Float) -> Self::Output {
        SpatialVector {
            angular: self.angular * rhs,
            linear: self.linear * rhs,
        }
    }
}

impl Div<Float> for &SpatialVector {
    type Output = SpatialVector;

    fn div(self, rhs: Float) -> Self::Output {
        SpatialVector {
            angular: self.angular / rhs,
            linear: self.linear / rhs,
        }
    }
}

impl Add for &SpatialVector {
    type Output = SpatialVector;

    fn add(self, rhs: Self) -> Self::Output {
        SpatialVector {
            angular: self.angular + rhs.angular,
            linear: self.linear + rhs.linear,
        }
    }
}

impl Add for SpatialVector {
    type Output = SpatialVector;

    fn add(self, rhs: Self) -> Self::Output {
        SpatialVector {
            angular: self.angular + rhs.angular,
            linear: self.linear + rhs.linear,
        }
    }
}
