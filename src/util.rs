use std::{
    fs::File,
    io::{BufReader, Read},
};

use na::{
    DMatrix, DVector, Matrix3xX, Matrix4x3, Quaternion, SymmetricEigen, UnitQuaternion, UnitVector3,
};
use nalgebra::{Matrix3, Vector3};
use web_sys::{self};

use crate::{inertia::SpatialInertia, spatial::spatial_vector::SpatialVector, types::Float};

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

pub fn inertia_mul(inertia: &SpatialInertia, vector: &SpatialVector) -> SpatialVector {
    let (angular, linear) = mul_inertia(
        &inertia.moment,
        &inertia.cross_part,
        inertia.mass,
        &vector.angular,
        &vector.linear,
    );
    SpatialVector::new(angular, linear)
}

/// Also known as spatial motion cross product
/// cross product between two motion vectors
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

pub fn spatial_motion_cross(x: &SpatialVector, y: &SpatialVector) -> SpatialVector {
    let (angular, linear) = se3_commutator(&x.angular, &x.linear, &y.angular, &y.linear);
    SpatialVector::new(angular, linear)
}

/// cross product between a motion vector and a force vector
/// reference: equation (2.34) in Featherstone
pub fn spatial_force_cross(v: &SpatialVector, f: &SpatialVector) -> SpatialVector {
    let angular = v.angular.cross(&f.angular) + v.linear.cross(&f.linear);
    let linear = v.angular.cross(&f.linear);
    SpatialVector::new(angular, linear)
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
pub fn quaternion_derivative(
    q: &UnitQuaternion<Float>,
    omega: &Vector3<Float>,
) -> Quaternion<Float> {
    let w = q.w;
    let x = q.coords.x;
    let y = q.coords.y;
    let z = q.coords.z;

    #[rustfmt::skip]
    let mat = Matrix4x3::new(
        -x, -y, -z,
         w, -z,  y,
         z,  w, -x,
        -y,  x,  w,
    ) / 2.0;

    let quaternion_dot = mat * omega;
    Quaternion::from_parts(quaternion_dot[0], quaternion_dot.fixed_view::<3, 1>(1, 0))
}

/// Constructs skew-symmetric matrix of v, i.e. skew_symmetric(v) * k == v.cross(k)
pub fn skew_symmetric(v: &Vector3<Float>) -> Matrix3<Float> {
    #[rustfmt::skip]
    return Matrix3::new(
        0.0,    -v.z,   v.y,
        v.z,    0.0,    -v.x,
        -v.y,   v.x,    0.0
    );
}

/// Given a normal direction, produces two tangential directions
pub fn tangentials(n: &UnitVector3<Float>) -> (UnitVector3<Float>, UnitVector3<Float>) {
    let t = {
        let candidate = n.cross(&Vector3::x_axis());
        if candidate.norm() != 0.0 {
            UnitVector3::new_normalize(candidate)
        } else {
            UnitVector3::new_normalize(n.cross(&Vector3::y_axis()))
        }
    };
    let b = UnitVector3::new_normalize(n.cross(&t));
    (t, b)
}

/// Project a matrix to be symmetric & positive semi-definite
pub fn project_symmetric_psd(matrix: &DMatrix<Float>) -> DMatrix<Float> {
    // Ensure the matrix is square
    assert!(matrix.is_square(), "Matrix must be square");

    // Symmetrize: M = (A + Aᵀ)/2
    let symmetric_matrix = (matrix + matrix.transpose()) * 0.5;

    // Eigen decomposition of the symmetric matrix
    let eigen = SymmetricEigen::new(symmetric_matrix);
    let eigenvalues = eigen.eigenvalues;
    let eigenvectors = eigen.eigenvectors;

    // Threshold eigenvalues: replace negatives with zero
    let thresholded_eigenvalues: DVector<Float> =
        eigenvalues.map(|e| if e < 0.0 { 0.0 } else { e });

    // Reconstruct: M_psd = Q * diag(λ₊) * Qᵀ
    let diag = DMatrix::from_diagonal(&thresholded_eigenvalues);
    &eigenvectors * diag * eigenvectors.transpose()
}

// Helper function to log to the browser console
pub fn console_log(message: &str) {
    web_sys::console::log_1(&message.into());
}

#[macro_export]
macro_rules! flog {
    ($($t:tt)*) => {
        #[cfg(debug_assertions)]
        {
            #[cfg(target_arch = "wasm32")]
            {
                use web_sys::console;
                console::log_1(&format!($($t)*).into());
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
                println!($($t)*);
            }
        }
    };
}

/// Read a file into a string
pub fn read_file(file_path: &str) -> String {
    let file = File::open(file_path).expect(&format!("{} should exist", file_path));
    let mut reader = BufReader::new(file);

    let mut buf: String = String::new();
    let _ = reader.read_to_string(&mut buf);

    buf
}

#[macro_export]
macro_rules! assert_close {
    ($left:expr, $right:expr, $tolerance:expr) => {
        let left = $left;
        let right = $right;
        let tol = $tolerance;
        let diff = (left - right).abs();
        assert!(
            diff < tol,
            "assertion failed: {} ~= {} \
                (tolerance: {}, difference: {})",
            left,
            right,
            tol,
            diff
        );
    };
}

#[macro_export]
macro_rules! assert_vec_close {
    ($left:expr, $right:expr, $tolerance:expr) => {
        let left = $left;
        let right = $right;
        let tol = $tolerance;
        for (a, b) in left.iter().zip(right.iter()) {
            crate::assert_close!(a, b, tol);
        }
    };
}

#[cfg(test)]
pub mod test_utils {
    use na::{vector, UnitQuaternion, Vector3};
    use rand::{rngs::ThreadRng, Rng};

    use crate::types::Float;

    /// Build a Vector3 where each element is random between (-range, range)
    pub fn random_vector3(rng: &mut ThreadRng, range: Float) -> Vector3<Float> {
        vector![
            rng.random_range(-range..range),
            rng.random_range(-range..range),
            rng.random_range(-range..range)
        ]
    }

    /// Build a UnitQuaternion from Euler angles, where each angle is random
    /// between (-range, range)
    pub fn random_quaternion(rng: &mut ThreadRng, range: Float) -> UnitQuaternion<Float> {
        UnitQuaternion::from_euler_angles(
            rng.random_range(-range..range),
            rng.random_range(-range..range),
            rng.random_range(-range..range),
        )
    }
}
