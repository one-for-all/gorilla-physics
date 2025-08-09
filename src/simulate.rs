use std::collections::HashMap;
use std::f64::INFINITY;

use itertools::izip;
use na::DMatrix;
use na::Matrix3xX;
use na::Matrix6xX;
use na::UnitVector3;
use na::Vector3;

use crate::dynamics::addPrismaticJointSpringForce;
use crate::dynamics::build_constraint_jacobians;
use crate::dynamics::build_jacobian_blocks;
use crate::dynamics::collision_detection;
use crate::dynamics::compose_contact_jacobian;
use crate::dynamics::dynamics_bias;
use crate::dynamics::dynamics_quantities;
use crate::dynamics::dynamics_solve;
use crate::dynamics::solve_constraint_and_contact;
use crate::flog;
use crate::integrators::ccd_velocity_stepping;
use crate::integrators::compute_new_q;
use crate::integrators::runge_kutta_2;
use crate::integrators::runge_kutta_4;
use crate::integrators::semi_implicit_euler;
use crate::integrators::velocity_stepping;
use crate::integrators::Integrator;
use crate::joint::JointTorque;
use crate::joint::JointVelocity;
use crate::joint::ToFloatDVec;
use crate::rigid_body::CollisionGeometry;
use crate::spatial::spatial_vector::SpatialVector;
use crate::spatial::twist::compute_twists_wrt_world;
use crate::spatial::twist::Twist;
use crate::{joint::JointPosition, mechanism::MechanismState, types::Float};
pub struct DynamicsResult {}

impl DynamicsResult {
    pub fn new() -> Self {
        DynamicsResult {}
    }
}

/// Step the mechanism state forward by dt seconds.
pub fn step(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
    integrator: &Integrator,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    // Fill tau with zero torques, if it is empty
    let tau = {
        if tau.is_empty() {
            &state
                .v
                .iter()
                .map(|v| match v {
                    JointVelocity::Float(_) => JointTorque::Float(0.0),
                    JointVelocity::Spatial(_) => JointTorque::Spatial(SpatialVector::zero()),
                    JointVelocity::None => JointTorque::None,
                })
                .collect::<Vec<JointTorque>>()
        } else {
            assert_eq!(
                tau.len(),
                state.v.len(),
                "joint torques vector τ length {}  and Joint velocity vector v length {} differ!",
                tau.len(),
                state.v.len()
            );
            tau
        }
    };

    match integrator {
        Integrator::SemiImplicitEuler => {
            let (new_q, new_v) = semi_implicit_euler(state, dt, &tau);
            state.update(&new_q, &new_v);
            (new_q, new_v)
        }
        Integrator::RungeKutta2 => {
            assert!(
                !state.has_spring_contacts(),
                "Cannot use Runge-Kutta on state with spring contacts"
            );
            let (new_q, new_v) = runge_kutta_2(state, dt, &tau);
            state.update(&new_q, &new_v);
            (new_q, new_v)
        }
        Integrator::RungeKutta4 => {
            assert!(
                !state.has_spring_contacts(),
                "Cannot use Runge-Kutta on state with spring contacts"
            );
            let (new_q, new_v) = runge_kutta_4(state, dt, &tau);
            state.update(&new_q, &new_v);
            (new_q, new_v)
        }
        Integrator::VelocityStepping => {
            let (new_q, new_v) = velocity_stepping(state, dt, &tau);
            state.update(&new_q, &new_v);
            (new_q, new_v)
        }
        Integrator::CCDVelocityStepping => {
            return ccd_velocity_stepping(state, dt, &tau);
        }
    }
}

/// Simulate the mechanism state from 0 to final_time with a time step of dt.
/// Returns the joint configurations at each time step.
pub fn simulate<F>(
    state: &mut MechanismState,
    final_time: Float,
    dt: Float,
    control_fn: F,
    integrator: &Integrator,
) -> (Vec<Vec<JointPosition>>, Vec<Vec<JointVelocity>>)
where
    F: Fn(&MechanismState) -> Vec<JointTorque>,
{
    let mut t = 0.0;
    let mut qs: Vec<Vec<JointPosition>> = vec![];
    let mut vs: Vec<Vec<JointVelocity>> = vec![];
    qs.push(state.q.clone());
    vs.push(state.v.clone());
    while t < final_time {
        let tau = control_fn(state);
        let (q, v) = step(state, dt, &tau, integrator);
        qs.push(q);
        vs.push(v);

        t += dt;
    }

    (qs, vs)
}

#[cfg(test)]
mod simulate_tests {
    use crate::joint::ToJointPositionVec;
    use crate::joint::{ToFloatDVec, ToJointTorqueVec, ToJointVelocityVec};
    use crate::{assert_close, assert_vec_close};
    use crate::{
        helpers::{build_cart, build_cart_pole},
        util::assert_dvec_close,
        GRAVITY, PI,
    };

    use super::*;
    use na::{Isometry3, Vector3};
    use nalgebra::{vector, Matrix3};

    #[test]
    fn simulate_horizontal_right_rod() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Isometry3::identity(); // transformation from rod to world frame
        let axis = Vector3::y_axis(); // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        let initial_energy = 0.0 + 0.0; // E = PE + KE, both are zero at the start.

        // Act
        let final_time = 10.0;
        let dt = 0.001;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![0.0].to_joint_torque_vec(),
            &Integrator::SemiImplicitEuler,
        );

        // Assert
        let q_max = qs
            .iter()
            .map(|q| q[0].float().clone())
            .fold(Float::NEG_INFINITY, Float::max);
        assert_close!(q_max, PI, 1e-2); // Check highest point of swing

        let q_final = qs[qs.len() - 1][0].float();
        let v_final = vs[vs.len() - 1][0].float();

        let potential_energy = m * GRAVITY * l / 2.0 * (-q_final.sin()); // mgh
        let kinetic_energy = 0.5 * (m * l * l / 3.0) * v_final * v_final; // 1/2 I ω^2
        assert_close!(initial_energy, potential_energy + kinetic_energy, 1e-1);
        // Sanity check that energy is conserved. Not exact due to numerical integration.
        // Note: this is potentially flaky test depending on parameters
    }

    #[test]
    fn cart() {
        // Arrange
        let m = 3.0; // mass of cart
        let l = 5.0; // length of cart

        let moment_x = 0.0;
        let moment_y = m * l * l / 12.0;
        let moment_z = m * l * l / 12.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0.0, 0.0, 0.0];
        let axis = vector![1.0, 0.0, 0.0];

        let mut state = build_cart(&m, &moment, &cross_part, &axis);

        let q_init = vec![2.0].to_joint_pos_vec();
        let v_init = vec![-1.0].to_joint_vel_vec();
        state.update(&q_init, &v_init);

        // Act
        let final_time = 10.0;
        let dt = 0.02;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![0.0].to_joint_torque_vec(),
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = &qs[qs.len() - 1].to_float_dvec();
        let v_final = &vs[vs.len() - 1].to_float_dvec();

        let q_calc = q_init.to_float_dvec() + v_init.to_float_dvec() * final_time;
        assert_vec_close!(q_final, &q_calc, 2e-2);
        // assert_vec_close!(q_final, &q_calc, 1e-5); // TODO(f64): needs higher tolerance w/ f64
        assert_vec_close!(v_final, &v_init.to_float_dvec(), 1e-6);
    }

    /// A cart pole system where a simple pendulum is dangled from a cart
    #[test]
    fn cart_pole() {
        // Arrange
        let m_cart = 3.0;
        let l_cart = 1.0;
        let moment_x = 0.0;
        let moment_y = m_cart * l_cart * l_cart / 12.0;
        let moment_z = m_cart * l_cart * l_cart / 12.0;
        let moment_cart = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_cart = vector![0.0, 0.0, 0.0];

        let m_pole = 5.0;
        let l_pole = 7.0;
        let moment_x = m_pole * l_pole * l_pole;
        let moment_y = m_pole * l_pole * l_pole;
        let moment_z = 0.0;
        let moment_pole = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_pole = vector![0.0, 0.0, -l_pole * m_pole];
        let axis_pole = Vector3::y_axis();

        let mut state = build_cart_pole(
            &m_cart,
            &m_pole,
            &moment_cart,
            &moment_pole,
            &cross_part_cart,
            &cross_part_pole,
            axis_pole,
        );

        let F_cart = 1.0; // force to be exerted on the cart
        let acc = F_cart / (m_cart + m_pole);
        let pole_theta_expected = acc.atan2(GRAVITY); // steady state pole joint angle

        let q_init = vec![0.0, pole_theta_expected].to_joint_pos_vec(); // set to steady state
        let v_init = vec![0.0, 0.0].to_joint_vel_vec();
        state.update(&q_init, &v_init);

        // Act
        let final_time = 20.0;
        let dt = 1e-2;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![F_cart, 0.0].to_joint_torque_vec(),
            &Integrator::RungeKutta4,
        );

        // Assert
        let v_final = vs[vs.len() - 1][0].float();
        assert_close!((v_final - acc * final_time).abs(), 0., 1e-4);

        let pole_theta_final = qs[qs.len() - 1][1].float();
        assert!((pole_theta_final - pole_theta_expected).abs() < 1e-5);
    }
}
