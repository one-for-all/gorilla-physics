use na::DVector;
use nalgebra::{Matrix3, Vector3};
use web_sys::{self};

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

/// Also known as spatial motion cross product
/// Reference: Chapter 2.9 Spatial Cross Products in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn se3_commutator(
    xw: &Vector3<Float>,
    xv: &Vector3<Float>,
    yw: &Vector3<Float>,
    yv: &Vector3<Float>,
) -> (Vector3<Float>, Vector3<Float>) {
    let anguar = xw.cross(yw);
    let linear = xw.cross(yv) + xv.cross(yw);
    (anguar, linear)
}

// Helper function to log to the browser console
pub fn console_log(message: &str) {
    web_sys::console::log_1(&message.into());
}

pub fn assert_close(a: &DVector<Float>, b: &DVector<Float>, tol: Float) {
    for (a, b) in a.iter().zip(b.iter()) {
        assert!((a - b).abs() < tol, "{} != {}", a, b);
    }
}
