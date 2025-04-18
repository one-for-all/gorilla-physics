//! Energy-based control methods

use crate::{
    energy::hopper_energy, joint::JointTorque, mechanism::MechanismState,
    spatial::spatial_vector::SpatialVector, types::Float, GRAVITY,
};

use super::Controller;

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

pub struct Hopper2DController {
    pub k_spring: Float,   // leg-foot spring constant
    pub h_setpoint: Float, // target height

    pub q_hip_setpoint: Float,    // target hip joint value
    pub q_piston_setpoint: Float, // target hip-leg length
    pub theta1_setpoint: Float,   // target theta 1 value
    pub v_vertical_prev: Float,   // vertical speed at last time step
    pub in_contact_prev: bool,    // whether hopper was in contact at last time step

    pub body_hip_length: Float,   // default length from body to hip
    pub hip_piston_length: Float, // default length from hip to leg
    pub piston_leg_length: Float, // default length from leg to foot
    pub l_leg: Float,             // leg length
}

impl Hopper2DController {
    pub fn new(
        h_setpoint: Float,
        body_hip_length: Float,
        hip_piston_length: Float,
        piston_leg_length: Float,
        l_leg: Float,
        k_spring: Float,
    ) -> Self {
        Hopper2DController {
            k_spring,
            h_setpoint,
            q_hip_setpoint: 0.0,
            q_piston_setpoint: 0.0,
            theta1_setpoint: 0.0,
            v_vertical_prev: 0.0,
            in_contact_prev: false,
            body_hip_length,
            hip_piston_length,
            piston_leg_length,
            l_leg,
        }
    }
}

impl Controller for Hopper2DController {
    fn control(&mut self, state: &mut MechanismState) -> Vec<JointTorque> {
        let mut tau = vec![JointTorque::Spatial(SpatialVector::zero())]; // first floating joint unactuated

        let q = state.q.clone();
        let v = state.v.clone();

        let q_body = q[0].pose();
        let q_hip = q[1].float();
        let q_piston = q[2].float();
        let q_leg = q[3].float();
        let v_body = v[0].spatial();
        let v_hip = v[1].float();
        let v_piston = v[2].float();
        let v_leg = v[3].float();

        // Energy-based control of vertical height
        let v_vertical = state.v[0].spatial().linear.z;
        let m_body = state.bodies[0].inertia.mass;
        let m_hip = state.bodies[1].inertia.mass;
        let m_piston = state.bodies[2].inertia.mass;
        let m_leg = state.bodies[3].inertia.mass;
        if self.v_vertical_prev < 0.0 && v_vertical > 0.0 {
            // Bottom: The moment in stance when the body has minimum altitude
            // and vertical motion of the body changes from downward to upward.
            let E = hopper_energy(state, q_leg, &self.k_spring);
            let h_setpoint = self.h_setpoint;
            let E_target = GRAVITY
                * (m_body * h_setpoint
                    + m_hip * (h_setpoint - self.body_hip_length)
                    + m_piston * (h_setpoint - self.body_hip_length - self.hip_piston_length)
                    + m_leg
                        * (h_setpoint
                            - self.body_hip_length
                            - self.hip_piston_length
                            - self.piston_leg_length
                            + self.l_leg / 2.0));
            let dE = E_target - E;
            self.q_piston_setpoint = q_leg + (q_leg * q_leg + 2.0 * dE / self.k_spring).sqrt();
        } else if self.v_vertical_prev > 0.0 && v_vertical < 0.0 {
            // Top: The moment in flight when the body has peak altitude and
            // vertical motion changes from upward to downward.
            self.q_piston_setpoint = 0.0;
        }

        // PID leg length control
        let kp = 2e4; // try to approximate ideal length control
        let kd = 1e3;

        let p_term = kp * (self.q_piston_setpoint - q_piston);
        let d_term = kd * (0.0 - v_piston);
        let tau_piston = p_term + d_term;

        // Spring simulation
        let l_rest = 0.0;
        let tau_leg = {
            if *q_leg < l_rest {
                spring_force(l_rest, *q_leg, self.k_spring) // force exerted by spring
            } else {
                let k_stop = 1e5;
                let b_stop = 125.0;
                mechanical_stop(l_rest, *q_leg, *v_leg, k_stop, b_stop)
            }
        };

        // TODO: Fix the hip angle during stance
        let mut tau_hip = 0.0;

        // TODO: Foot Placement for velocity and attitude control
        // Note: important. Compute body linear velocity in world frame
        tau.push(JointTorque::Float(tau_hip));
        tau.push(JointTorque::Float(tau_piston - tau_leg)); // control + reaction force from spring
        tau.push(JointTorque::Float(tau_leg));

        self.v_vertical_prev = v_vertical;
        tau
    }
}
