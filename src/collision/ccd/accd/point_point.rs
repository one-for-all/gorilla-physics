use na::Vector3;

use crate::types::Float;

/// distance squared
pub fn point_point_distance(p0: &Vector3<Float>, p1: &Vector3<Float>) -> Float {
    (p1 - p0).norm_squared()
}
