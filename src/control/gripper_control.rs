use crate::{
    joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
};

use super::{ControlInput, Controller};

pub struct GripperManualController {}

impl Controller for GripperManualController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let input = input.unwrap();
        let up = input.floats[0];
        let close = input.floats[1];

        let lift_target = {
            if up > 0.0 {
                0.7 // drop the arm
            } else {
                0.0 // raise the arm
            }
        };

        let gripper_target = {
            if close > 0.0 {
                -0.2 // close the gripper
            } else {
                0.0 // open the gripper
            }
        };

        let q_lift = state.q[0].float();
        let v_lift = state.v[0].float();
        let tau_lift = 100.0 * (lift_target - q_lift) + 20.0 * (-v_lift);

        let q_gripper_left = state.q[1].float();
        let v_gripper_left = state.v[1].float();
        let tau_gripper_left = 100.0 * (gripper_target - q_gripper_left) + 10.0 * (-v_gripper_left);

        let q_gripper_right = state.q[2].float();
        let v_gripper_right = state.v[2].float();
        let tau_gripper_right =
            100.0 * (gripper_target - q_gripper_right) + 10.0 * (-v_gripper_right);

        vec![
            JointTorque::Float(tau_lift),
            JointTorque::Float(tau_gripper_left),
            JointTorque::Float(tau_gripper_right),
            JointTorque::Spatial(SpatialVector::zero()),
        ]
    }
}
