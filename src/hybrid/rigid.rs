use na::{dvector, DMatrix, DVector, Matrix3, Vector3};

use crate::{
    inertia::SpatialInertia,
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    types::Float,
    WORLD_FRAME,
};

/// Rigid body
pub struct Rigid {
    pub inertia: SpatialInertia,
    pub pose: Pose,           // TODO: use Iso instead of Pose?
    pub twist: SpatialVector, // body velocity expressed in world frame
}

impl Rigid {
    /// Assuming self.inertia is in body frame, return the inertia expressed in world frame
    pub fn inertia_in_world_frame(&self) -> SpatialInertia {
        let R = self.pose.rotation.to_rotation_matrix();
        let p = self.pose.translation;

        let inertia = &self.inertia;
        let J = inertia.moment;
        let mc = inertia.cross_part;
        let m = inertia.mass;

        let Rmc = R * mc;
        let mp = m * p;
        let mcnew = Rmc + mp;
        let X = Rmc * p.transpose();
        let Y = X + X.transpose() + mp * p.transpose();
        let Jnew = R * J * R.transpose() - Y + Y.trace() * DMatrix::identity(Y.nrows(), Y.ncols());

        SpatialInertia {
            frame: WORLD_FRAME.to_string(),
            moment: Jnew,
            cross_part: mcnew,
            mass: m,
        }
    }

    pub fn new_sphere() -> Self {
        let m = 1.0;
        let r = 1.0;
        let moment = 2. / 5. * m * r * r;
        let moment = Matrix3::from_diagonal_element(moment);
        let cross = Vector3::zeros();
        let inertia = SpatialInertia::new(moment, cross, m, "sphere");
        Rigid {
            inertia,
            pose: Pose::identity(),
            twist: SpatialVector::zero(),
        }
    }

    /// free-motion velocity in body frame
    pub fn free_velocity(&self, dt: Float) -> DVector<Float> {
        let mut v_free = dvector![];
        v_free.extend(self.twist.angular.iter().cloned());
        v_free.extend(self.twist.linear.iter().cloned());
        assert_eq!(v_free.len(), 6);
        v_free
    }

    /// Computes linear momentum in world frame
    pub fn linear_momentum(&self) -> Vector3<Float> {
        let linear = self.inertia.mass * self.twist.linear
            - self.inertia.cross_part.cross(&self.twist.angular);
        self.pose.rotation * linear
    }
}
