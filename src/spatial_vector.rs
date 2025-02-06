use std::ops::{Add, Mul};

use crate::{transform::Transform3D, types::Float};
use na::{zero, Vector3};

#[derive(Clone, Debug)]
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

    pub fn transform(&self, transform: &Transform3D) -> SpatialVector {
        let rot = transform.rot();
        let trans = transform.trans();

        let angular = rot.mul(self.angular);
        let linear = rot.mul(self.linear) + trans.cross(&angular);

        let result = SpatialVector { angular, linear };

        result
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

impl Add for &SpatialVector {
    type Output = SpatialVector;

    fn add(self, rhs: Self) -> Self::Output {
        SpatialVector {
            angular: self.angular + rhs.angular,
            linear: self.linear + rhs.linear,
        }
    }
}
