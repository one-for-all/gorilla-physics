use crate::{
    joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
};

use super::Controller;

pub struct PusherController {}

impl Controller for PusherController {
    fn control(&mut self, state: &mut MechanismState) -> Vec<JointTorque> {
        let q_pusher = state.q[1].float();
        let v_pusher = state.v[1].float();
        let q_pusher_desired = 3.0;
        let q_d = q_pusher_desired - q_pusher;
        let v_target = q_d.signum() * 10.0 * q_d.abs().min(0.1);
        let tau_pusher = 100.0 * (v_target - v_pusher);

        vec![
            JointTorque::Float(0.0),
            JointTorque::Float(tau_pusher),
            JointTorque::Spatial(SpatialVector::zero()),
        ]
    }
}

#[cfg(test)]
mod pusher_controller_tests {
    use na::Vector3;

    use crate::{
        assert_close, contact::HalfSpace, helpers::build_pusher, integrators::Integrator,
        simulate::step,
    };

    use super::*;

    #[test]
    fn robot_pusher() {
        // Arrange
        let mut state = build_pusher();
        let ground = HalfSpace::new(Vector3::z_axis(), 0.0);
        state.add_halfspace(&ground);

        let mut controller = PusherController {};

        // Act
        let final_time = 10.0;
        let dt = 1.0 / 1000.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = controller.control(&mut state);
            let (q, v) = step(&mut state, dt, &torque, &Integrator::SemiImplicitEuler);
        }

        // Assert
        let box_pose = state.poses()[2];
        assert!(box_pose.translation.x > 5.0);
        assert_close!(box_pose.translation.y, 0.0, 1e-2);
        assert_close!(box_pose.translation.z, 0.25, 1e-2);
        assert_close!(box_pose.rotation.angle(), 0.0, 1e-2);
    }
}
