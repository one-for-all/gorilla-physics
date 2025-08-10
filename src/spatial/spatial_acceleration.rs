use std::ops::Add;
use std::ops::Mul;

use na::{zero, Vector3};

use crate::spatial::transform::Transform3D;
use crate::types::Float;
use crate::GRAVITY;
use crate::WORLD_FRAME;

/// A spatial acceleration is the time derivative of a twist
#[derive(PartialEq, Debug, Clone)]
pub struct SpatialAcceleration {
    pub body: String,
    pub base: String,
    pub frame: String,
    pub angular: Vector3<Float>,
    pub linear: Vector3<Float>,
}

impl SpatialAcceleration {
    pub fn zero(body: &str, base: &str) -> SpatialAcceleration {
        SpatialAcceleration {
            body: body.to_string(),
            base: base.to_string(),
            frame: body.to_string(),
            angular: zero(),
            linear: zero(),
        }
    }

    /// Transform the spatial acceleration to be expressed in the "to" frame of transform
    pub fn transform(&self, transform: &Transform3D) -> SpatialAcceleration {
        if self.frame != transform.from {
            panic!(
                "spatial acceleration {} frame is not equal to transform `from` {} frame!",
                self.frame, transform.from
            );
        }

        let rot = transform.rot();
        let trans = transform.trans();
        let angular = rot.mul(self.angular);
        let linear = rot.mul(self.linear) + trans.cross(&angular);

        SpatialAcceleration {
            body: self.body.clone(),
            base: self.base.clone(),
            frame: transform.to.clone(),
            angular,
            linear,
        }
    }

    pub fn inv_gravitational_spatial_acceleration() -> SpatialAcceleration {
        SpatialAcceleration {
            body: WORLD_FRAME.to_string(),
            base: WORLD_FRAME.to_string(),
            frame: WORLD_FRAME.to_string(),
            angular: Vector3::zeros(),
            linear: Vector3::new(0.0, 0.0, GRAVITY),
        }
    }
}

impl<'a, 'b> Add<&'b SpatialAcceleration> for &'a SpatialAcceleration {
    type Output = SpatialAcceleration;

    /// lhs is A to B spatial acceleration, rhs is B to C spatial acceleration,
    /// returns A to C spatial acceleration.
    fn add(self, rhs: &SpatialAcceleration) -> SpatialAcceleration {
        if self.frame != rhs.frame {
            panic!("lhs and rhs are not expressed in the same frame!");
        }

        if self.body != rhs.base && !(self.base == rhs.base && self.body == rhs.body) {
            panic!("lhs frames do not match as rhs frames!");
        }
        SpatialAcceleration {
            body: rhs.body.clone(),
            base: self.base.clone(),
            frame: self.frame.clone(),
            angular: self.angular + rhs.angular,
            linear: self.linear + rhs.linear,
        }
    }
}
