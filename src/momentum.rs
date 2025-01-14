use na::Vector3;

use crate::{geometric_jacobian::GeometricJacobian, inertia::SpatialInertia, types::Float};

/// A momentum matrix maps a joint velocity vector to a momentum.
pub struct MomentumMatrix {
    frame: String,
    angular: Vector3<Float>,
    linear: Vector3<Float>,
}

impl MomentumMatrix {
    // Computes the momentum matrix given spatial inertia and jacobian
    pub fn mul(inertia: &SpatialInertia, jacobian: &GeometricJacobian) -> MomentumMatrix {
        let Jw = jacobian.angular;
        let Jv = jacobian.linear;
        let J = inertia.moment;
        let m = inertia.mass;
        let c = inertia.cross_part;

        let ang = J * Jw + c.cross(&Jv);
        let lin = m * Jv - c.cross(&Jw);

        MomentumMatrix {
            frame: jacobian.frame.clone(),
            angular: ang,
            linear: lin,
        }
    }

    pub fn transpose_mul(&self, jacobian: &GeometricJacobian) -> Float {
        if self.frame != jacobian.frame {
            panic!(
                "self frame {} and jacobian frame {} differ!",
                self.frame, jacobian.frame
            );
        }

        (self.angular.transpose() * jacobian.angular + self.linear.transpose() * jacobian.linear)[0]
    }
}
