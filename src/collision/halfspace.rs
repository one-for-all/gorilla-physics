use na::{UnitVector3, Vector3};

use crate::types::Float;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct HalfSpace {
    pub point: Vector3<Float>,      // A point on the half-space
    pub normal: UnitVector3<Float>, // Outward normal direction of the half-space
    pub alpha: Float, // roughly how much velocity is lost, coefficient of restitution e ~= 1-a*v_in
    pub mu: Float,    // coefficient of friction
}

impl HalfSpace {
    // Create a half-space that is moved along normal by distance, from origin
    pub fn new(normal: UnitVector3<Float>, distance: Float) -> Self {
        HalfSpace {
            point: normal.scale(distance),
            normal,
            alpha: 0.9,
            mu: 0.5,
        }
    }

    pub fn new_with_params(
        normal: UnitVector3<Float>,
        distance: Float,
        alpha: Float,
        mu: Float,
    ) -> Self {
        HalfSpace {
            point: normal.scale(distance),
            normal,
            alpha,
            mu,
        }
    }

    // True if the point is inside the half-space
    pub fn has_inside(&self, point: &Vector3<Float>) -> bool {
        (point - self.point).dot(&self.normal) <= 0.0
    }
}
