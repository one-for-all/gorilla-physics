use crate::{
    joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
    types::Float,
};

pub struct SLIPController {
    pub k_q: Float,
    pub speed_bound: Float,

    pub k_v_p: Float, // proportional gain for speed control
    pub k_v_i: Float, // integral gain for speed control
    pub k_v_d: Float, // derivative gain for speed control

    pub v_integral: Float,
    pub v_diff_prev: Float,
}

impl SLIPController {
    pub fn new(k_q: Float, speed_bound: Float, k_v_p: Float, k_v_i: Float, k_v_d: Float) -> Self {
        Self {
            k_q,
            speed_bound,
            k_v_p,
            k_v_i,
            k_v_d,
            v_integral: 0.0,
            v_diff_prev: 0.0,
        }
    }

    /// Control a SLIP model to reach position x, and stay there
    /// Returns the command touch-down angle
    pub fn control_to_pos(&mut self, state: &MechanismState, x_target: Float) -> Float {
        let x = state.q[0].pose().translation.x;

        // speed proportional to distance, upper-bounded
        // v = k * dx
        let mut v_x_target = -self.k_q * (x - x_target);
        if v_x_target.abs() > self.speed_bound {
            v_x_target = v_x_target.signum() * self.speed_bound;
        }

        self.control_to_velocity(state, v_x_target)
    }

    /// Control a SLIP model to maintain targe velocity
    /// Returns the command touch-down angle
    pub fn control_to_velocity(&mut self, state: &MechanismState, v_x_target: Float) -> Float {
        let v_x = (state.q[0].pose().rotation * state.v[0].spatial().linear).x;
        let v_diff = v_x - v_x_target;
        self.v_integral += v_diff;

        // PID control on velocity
        let degree = self.k_v_p * v_diff
            + self.k_v_i * self.v_integral
            + self.k_v_d * (v_diff - self.v_diff_prev);

        self.v_integral += 0.0;
        self.v_diff_prev = v_diff;

        Float::to_radians(degree)
    }
}

#[cfg(test)]
mod SLIP_control_tests {

    use na::{vector, UnitVector3, Vector3};

    use crate::{
        contact::HalfSpace, helpers::build_SLIP, integrators::Integrator, simulate::step,
        util::assert_close,
    };

    use super::*;

    #[test]
    fn position_control() {
        // Arrange
        let m = 0.54;
        let r = 1.0; // set r to be relatively large to make the effect of angular torque small
                     // theoretically there should be no angular torque. here it is due
                     // to simulation numerical error.
        let l_rest = 0.2;
        let init_angle: Float = Float::to_radians(0.0);
        let k_spring = 2000.0;

        let mut state = build_SLIP(m, r, l_rest, init_angle, k_spring);

        let h_ground = -0.5;
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), h_ground));

        let mut controller = SLIPController::new(
            1.0,  // k_pos
            0.5,  // speed bound
            20.0, // k_v proportional
            0.0,  // k_v integral
            0.0,  // k_v derivative
        );

        // Act
        let x_target = 2.0;
        let mut v_z_prev = 0.0;

        let final_time = 10.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::SemiImplicitEuler);

            let rot = state.q[0].pose().rotation;
            let v_linear = rot * state.v[0].spatial().linear;
            let v_z = v_linear.z;

            if v_z_prev >= 0.0 && v_z < 0.0 {
                // Apex
                let angle = controller.control_to_pos(&state, x_target);
                let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);
                state.bodies[0].spring_contacts[0].direction = direction;
                state.bodies[0].spring_contacts[0].l_rest = l_rest;
            }

            v_z_prev = v_z;
        }

        // Assert
        let x = state.q[0].pose().translation.x;
        assert_close(x.abs(), x_target, 1e-3);
    }

    #[test]
    fn speed_control() {
        // Arrange
        let m = 0.54;
        let r = 1.0; // set r to be relatively large to make the effect of angular torque small
                     // theoretically there should be no angular torque. here it is due
                     // to simulation numerical error.
        let l_rest = 0.2;
        let init_angle: Float = Float::to_radians(0.0);
        let k_spring = 2000.0;

        let mut state = build_SLIP(m, r, l_rest, init_angle, k_spring);

        let h_ground = -0.5;
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), h_ground));

        let mut controller = SLIPController::new(
            1.0,  // k_pos
            0.5,  // speed bound
            11.0, // k_v proportional
            1.5,  // k_v integral
            1.5,  // k_v derivative
        );

        // Act
        let v_x_target = 0.3;
        let mut v_z_prev = 0.0;

        let final_time = 15.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        let mut v_x = vec![];
        for _ in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::SemiImplicitEuler);

            let rot = state.q[0].pose().rotation;
            let v_linear = rot * state.v[0].spatial().linear;
            let v_z = v_linear.z;

            if v_z_prev >= 0.0 && v_z < 0.0 {
                // Apex
                let angle = controller.control_to_velocity(&state, v_x_target);
                let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);
                state.bodies[0].spring_contacts[0].direction = direction;
                state.bodies[0].spring_contacts[0].l_rest = l_rest;

                v_x.push(v_linear.x);
            }

            v_z_prev = v_z;
        }

        // Assert
        assert_close(v_x[v_x.len() - 1], v_x_target, 1e-3);
        assert_close(v_x[v_x.len() - 2], v_x_target, 1e-3);
        assert_close(v_x[v_x.len() - 3], v_x_target, 1e-3);
    }
}

/// Controller for Actuated Lossy Spring-Loaded Inverted Pendulum model
pub struct ALSLIPController {
    pub k_spring: Float,
    pub l_rest_spring: Float,
}

impl ALSLIPController {
    pub fn new(k_spring: Float, l_rest_spring: Float) -> Self {
        ALSLIPController {
            k_spring,
            l_rest_spring,
        }
    }

    /// TODO: control to velocity/position
    pub fn control(&mut self, _state: &mut MechanismState) -> Vec<JointTorque> {
        let mut tau = vec![JointTorque::Spatial(SpatialVector::zero())]; // first floating joint unactuated
        tau.push(JointTorque::Float(0.0));
        tau.push(JointTorque::Float(0.0));

        tau
    }
}
