//! Energy-based control methods
use na::{ComplexField, Vector, Vector3};
use web_sys::console;

use crate::{
    energy::hopper_energy,
    joint::{JointPosition, JointTorque},
    mechanism::MechanismState,
    spatial_vector::SpatialVector,
    transform::compute_bodies_to_root,
    types::Float,
    util::console_log,
    GRAVITY, PI,
};

/// TODO: Add spring elements to mechanism state
/// Force exerted by a spring
/// F = -k * x
pub fn spring_force(l_rest: Float, l: Float, k: Float) -> Float {
    -k * (l - l_rest)
}

/// TODO: Add mechanical stop to mechanism state
/// Force exerted by a mechanical stop, i.e. very stiff spring w/ damping
/// F = -k * x - b * v
pub fn mechanical_stop(l_rest: Float, l: Float, v: Float, k: Float, b: Float) -> Float {
    -k * (l - l_rest) - b * v
}

pub trait Controller {
    fn control(&mut self, state: &mut MechanismState) -> Vec<JointTorque>;
}

pub struct Hopper1DController {
    pub k_spring: Float,   // leg-foot spring constant
    pub h_setpoint: Float, // target height

    pub leg_length_setpoint: Float, // target body-leg length
    pub v_vertical_prev: Float,     // vertical speed at last time step

    pub body_leg_length: Float, // default length from body to leg
    pub leg_foot_length: Float, // default length from leg to foot
}

impl Controller for Hopper1DController {
    /// Control the hopper to jump to certain vertical height
    /// Ref: Hopping in Legged Systems-Modeling and Simulation for the Two-Dimensional One-Legged Case
    /// Marc Raibert, 1984
    fn control(&mut self, state: &mut MechanismState) -> Vec<JointTorque> {
        let mut tau = vec![JointTorque::Spatial(SpatialVector::zero())]; // first floating joint unactuated

        let q1 = state.q[1].float();
        let v1 = state.v[1].float();
        let q_foot = state.q[2].float();
        let v_foot = state.v[2].float();

        // Energy-based control of vertical height
        let v_vertical = state.v[0].spatial().linear.z;
        let m_body = state.bodies[0].inertia.mass;
        let m_leg = state.bodies[1].inertia.mass;
        let m_foot = state.bodies[2].inertia.mass;
        if self.v_vertical_prev < 0.0 && v_vertical > 0.0 {
            // Bottom: The moment in stance when the body has minimum altitude
            // and vertical motion of the body changes from downward to upward.
            let E = hopper_energy(state, q_foot, &self.k_spring);
            let h_setpoint = self.h_setpoint;
            let E_target = GRAVITY
                * (m_body * h_setpoint
                    + m_leg * (h_setpoint - self.body_leg_length)
                    + m_foot * (h_setpoint - self.body_leg_length - self.leg_foot_length));
            let dE = E_target - E;
            self.leg_length_setpoint = q_foot + (q_foot * q_foot + 2.0 * dE / self.k_spring).sqrt();
        } else if self.v_vertical_prev > 0.0 && v_vertical < 0.0 {
            // Top: The moment in flight when the body has peak altitude and
            // vertical motion changes from upward to downward.
            self.leg_length_setpoint = 0.0;
        }

        // PID leg length control
        let kp = 2000.0; // try to approximate ideal length control
        let kd = 100.0;
        let ki = 100.0;

        let p_term = kp * (self.leg_length_setpoint - q1);
        let d_term = kd * (0.0 - v1);
        let tau1 = p_term + d_term;

        // Spring simulation
        let l_rest = 0.0;
        let tau_foot = {
            if *q_foot < l_rest {
                spring_force(l_rest, *q_foot, self.k_spring) // force exerted by spring
            } else {
                let k_stop = 1e5;
                let b_stop = 125.0;
                mechanical_stop(l_rest, *q_foot, *v_foot, k_stop, b_stop)
            }
        };

        tau.push(JointTorque::Float(tau1 - tau_foot)); // control + reaction force from spring/stop
        tau.push(JointTorque::Float(tau_foot));

        self.v_vertical_prev = v_vertical;
        tau
    }
}


        self.v_vertical_prev = v_vertical;
        tau
    }
}
