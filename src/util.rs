use nalgebra::{Matrix3, Vector3};

use crate::types::Float;

/// Mulitiply a spatial momentum with a spatial vector
/// | J         c_hat | | w |   | Jw        + c_hat v |
/// | c_hat^T   m     | | v | = | c_hat^T w + mv      |
pub fn mul_inertia(
    J: &Matrix3<Float>,
    c: &Vector3<Float>,
    m: Float,
    w: &Vector3<Float>,
    v: &Vector3<Float>,
) -> (Vector3<Float>, Vector3<Float>) {
    let angular = J * w + c.cross(v);
    let linear = m * v - c.cross(w);
    (angular, linear)
}