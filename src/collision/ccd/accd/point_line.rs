use na::Vector3;

use crate::types::Float;

/// Distance squared between a point and a infinite line
pub fn point_line_distance(p: &Vector3<Float>, e0: &Vector3<Float>, e1: &Vector3<Float>) -> Float {
    (e0 - p).cross(&(e1 - p)).norm_squared() / (e1 - e0).norm_squared()
}
