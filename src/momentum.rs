use na::{DMatrix, Matrix3xX};

use crate::{
    inertia::SpatialInertia, spatial::geometric_jacobian::GeometricJacobian, types::Float,
    util::colwise_cross,
};

/// A momentum matrix maps a joint velocity vector to a momentum.
pub struct MomentumMatrix {
    frame: String,
    angular: Matrix3xX<Float>,
    linear: Matrix3xX<Float>,
}

impl MomentumMatrix {
    // Computes the momentum matrix given spatial inertia and jacobian
    pub fn mul(inertia: &SpatialInertia, jacobian: &GeometricJacobian) -> MomentumMatrix {
        let Jw = &jacobian.angular;
        let Jv = &jacobian.linear;
        let J = inertia.moment;
        let m = inertia.mass;
        let c = inertia.cross_part;

        let ang = J * Jw + colwise_cross(&c, &Jv);
        let lin = m * Jv - colwise_cross(&c, &Jw);

        MomentumMatrix {
            frame: jacobian.frame.clone(),
            angular: ang,
            linear: lin,
        }
    }

    /// The result should be a joint-space inertia matrix
    pub fn transpose_mul(&self, jacobian: &GeometricJacobian) -> DMatrix<Float> {
        if self.frame != jacobian.frame {
            panic!(
                "self frame {} and jacobian frame {} differ!",
                self.frame, jacobian.frame
            );
        }

        let result = self.angular.transpose() * &jacobian.angular
            + self.linear.transpose() * &jacobian.linear;
        result
    }
}
