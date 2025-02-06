use na::{DVector, Matrix3xX, Matrix4x3, Quaternion, UnitQuaternion};
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

/// Perform column-wise cross product
pub fn colwise_cross(a: &Vector3<Float>, b: &Matrix3xX<Float>) -> Matrix3xX<Float> {
    let ncols = b.ncols();
    let mut result = Matrix3xX::zeros(ncols);
    for i in 0..ncols {
        result.set_column(i, &a.cross(&b.column(i)));
    }
    result
}

/// Compute the derivative of quaternion, given angular velocity:
/// qdot = 1/2 * q \quaternion_product ω, 
/// where q is orientation as quaternion, and ω is angular velocity in body frame
/// 
/// Ref: 1.5.2 & 1.5.4 in Quaternions and Dynamics, Basile Graf, 2007
#[rustfmt::skip]
pub fn quaternion_derivative(q: &UnitQuaternion<Float>, omega: &Vector3<Float>) -> Quaternion<Float> {
    let w = q.w;
    let x = q.coords.x;
    let y = q.coords.y;
    let z = q.coords.z;

    let mat = Matrix4x3::new(
        -x, -y, -z, 
         w, -z,  y, 
         z,  w, -x,
        -y,  x,  w,
    ) / 2.0;

    let quaternion_dot = mat * omega;
    Quaternion::from_parts(quaternion_dot[0], quaternion_dot.fixed_view::<3, 1>(1, 0))
}

// Helper function to log to the browser console
pub fn console_log(message: &str) {
    web_sys::console::log_1(&message.into());
}

pub fn assert_close(a: &Float, b: &Float, tol: Float) {
    assert!((a - b).abs() < tol, "{} != {}", a, b);
}

pub fn assert_dvec_close(a: &DVector<Float>, b: &DVector<Float>, tol: Float) {
    for (a, b) in a.iter().zip(b.iter()) {
        assert!((a - b).abs() < tol, "{} != {}", a, b);
    }
}
