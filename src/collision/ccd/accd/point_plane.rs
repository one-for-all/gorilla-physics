use na::Vector3;

use crate::types::Float;

/// Distance squared between a point and an infinite plane
pub fn point_plane_distance(
    p: &Vector3<Float>,
    t0: &Vector3<Float>,
    t1: &Vector3<Float>,
    t2: &Vector3<Float>,
) -> Float {
    let normal = (t1 - t0).cross(&(t2 - t0));
    let origin = t0;

    let point_to_plane = (p - origin).dot(&normal);
    point_to_plane * point_to_plane / normal.norm_squared()
}
