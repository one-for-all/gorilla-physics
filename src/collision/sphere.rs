use na::{Isometry3, Vector3};

use crate::{contact::HalfSpace, types::Float};

#[derive(Clone, PartialEq, Debug)]
pub struct Sphere {
    pub isometry: Isometry3<Float>,

    pub radius: Float,
}

impl Sphere {
    pub fn new(radius: Float) -> Self {
        Sphere {
            isometry: Isometry3::identity(),
            radius,
        }
    }

    /// Returns contact point if in contact
    pub fn contact_halfspace(&self, halfspace: &HalfSpace) -> Option<Vector3<Float>> {
        let center = &self.isometry.translation.vector;
        let distance = (center - halfspace.point).dot(&halfspace.normal);
        if distance <= self.radius {
            return Some(center - distance * *halfspace.normal);
        }
        None
    }
}

#[cfg(test)]
mod sphere_tests {
    use na::{vector, UnitQuaternion, UnitVector3};

    use crate::{
        assert_close, assert_vec_close,
        contact::HalfSpace,
        helpers::build_sphere,
        integrators::Integrator,
        joint::{JointPosition, JointTorque},
        simulate::step,
        spatial::{pose::Pose, spatial_vector::SpatialVector},
        types::Float,
    };

    #[test]
    pub fn sphere_hit_ground() {
        // Arrange
        let mut state = build_sphere(1.0, 1.0);

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0., 0., 1.5],
        })];
        state.update_q(&q_init);

        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let pose = state.poses()[0];
        assert_vec_close!(pose.translation, vec![0., 0., 1.], 1e-3);
    }

    #[test]
    pub fn sphere_roll_forward() {
        // Arrange
        let m = 1.0;
        let r = 0.5;
        let mut state = build_sphere(m, r);

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0., 0., 1.0 * r],
        })];
        state.update_q(&q_init);

        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        let tau = 0.1;
        for _s in 0..num_steps {
            let torque = vec![JointTorque::Spatial(SpatialVector {
                angular: vector![0., tau, 0.],
                linear: vector![0., 0., 0.],
            })];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let pose = state.poses()[0];
        let euler_angles = pose.rotation.euler_angles();
        assert_eq!(euler_angles.0, 0.);
        assert_eq!(euler_angles.2, 0.);

        let translation = pose.translation;
        assert_eq!(translation[1], 0.);
        assert_eq!(translation[2], r);

        // distance = radius * angle
        assert_close!(r * euler_angles.1, translation[0], 1e-4);
    }
}
