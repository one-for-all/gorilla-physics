use na::Vector3;

use crate::types::Float;

/// Distance squared between two infinite lines
/// Warning: if the lines are parallel, result is undefined
pub fn line_line_distance(
    ea0: &Vector3<Float>,
    ea1: &Vector3<Float>,
    eb0: &Vector3<Float>,
    eb1: &Vector3<Float>,
) -> Float {
    let normal = (ea1 - ea0).cross(&(eb1 - eb0));
    let line_to_line = (eb0 - ea0).dot(&normal);
    line_to_line * line_to_line / normal.norm_squared()
}
