use na::{UnitVector3, Vector3};

use crate::types::Float;

/// Halfspace with a finite size
pub struct SizedHalfspace {
    pub center: Vector3<Float>,
    pub normal: UnitVector3<Float>,

    pub width_half: Float,
    pub depth_half: Float,
}

impl SizedHalfspace {
    pub fn new(
        center: Vector3<Float>,
        normal: UnitVector3<Float>,
        width_half: Float,
        depth_half: Float,
    ) -> Self {
        Self {
            center,
            normal,
            width_half,
            depth_half,
        }
    }
}
