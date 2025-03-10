use itertools::izip;
use na::UnitQuaternion;

use crate::{
    dynamics::dynamics,
    joint::{JointAcceleration, JointPosition, JointTorque, JointVelocity},
    mechanism::MechanismState,
    pose::Pose,
    types::Float,
    util::quaternion_derivative,
};

pub enum Integrator {
    SemiImplicitEuler,
    RungeKutta2,
    RungeKutta4,
}

pub fn semi_implicit_euler(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let vdot = dynamics(state, &tau);

    // Semi-implicit Euler integration
    // Note: this actually turns out to be energy conserving for Hamiltonian systems,
    // informally meaning systems that are not subject to velocity-dependent
    // forces. E.g. single pendulum
    //
    // Ref: Drake Doc, https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_semi_explicit_euler_integrator.html
    semi_implicit_euler_step(&state.q, &state.v, &vdot, dt)
}

pub fn runge_kutta_2(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let q0 = state.q.clone();
    let v0 = state.v.clone();

    let f1 = dynamics(state, &tau);
    let (q1, v1) = euler_step(&q0, &v0, &f1, dt / 2.0);

    state.update(&q1, &v1);
    let f2 = dynamics(state, &tau);
    let (q2, v2) = euler_step(&q0, &v0, &f2, dt);

    (q2, v2)
}

pub fn runge_kutta_4(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let q0 = state.q.clone();
    let v0 = state.v.clone();

    let f1 = dynamics(state, &tau);

    let (q1, v1) = euler_step(&q0, &v0, &f1, dt / 2.0);
    state.update(&q1, &v1);
    let f2 = dynamics(state, &tau);

    let (q2, v2) = euler_step(&q0, &v0, &f2, dt / 2.0);
    state.update(&q2, &v2);
    let f3 = dynamics(state, &tau);

    let (q3, v3) = euler_step(&q0, &v0, &f3, dt);
    state.update(&q3, &v3);
    let f4 = dynamics(state, &tau);

    let mut f_final = vec![];
    for (f1i, f2i, f3i, f4i) in izip!(f1.iter(), f2.iter(), f3.iter(), f4.iter()) {
        f_final.push((*f1i + f2i * 2.0 + f3i * 2.0 + *f4i) / 6.0);
    }

    let (q_final, v_final) = euler_step(&q0, &v0, &f_final, dt);

    (q_final, v_final)
}

/// Euler integration step:
///     v(k+1) = v(k) + dt * vdot
///     q(k+1) = q(k) + dt * v(k)
pub fn euler_step(
    q: &Vec<JointPosition>,
    v: &Vec<JointVelocity>,
    vdot: &Vec<JointAcceleration>,
    dt: Float,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let mut new_v = vec![];
    let mut new_q = vec![];
    for (qi, vi, vdot) in izip!(q.iter(), v.iter(), vdot.iter()) {
        match qi {
            JointPosition::Float(qi) => {
                let vdot = vdot.float();
                let new_vi = vi.float() + vdot * dt;
                let new_qi = qi + vi.float() * dt;
                new_v.push(JointVelocity::Float(new_vi));
                new_q.push(JointPosition::Float(new_qi));
            }
            JointPosition::Pose(qi) => {
                let vdot = vdot.spatial();
                let new_vi = vi.spatial() + &(vdot * dt);

                let quaternion_dot = quaternion_derivative(&qi.rotation, &vi.spatial().angular);
                let translation_dot = qi.rotation * vi.spatial().linear;
                let new_qi = Pose {
                    translation: qi.translation + translation_dot * dt,
                    rotation: UnitQuaternion::from_quaternion(
                        qi.rotation.quaternion() + quaternion_dot * dt,
                    ),
                };

                new_v.push(JointVelocity::Spatial(new_vi));
                new_q.push(JointPosition::Pose(new_qi));
            }
        }
    }

    (new_q, new_v)
}

/// Semi-Implicit Euler integration step:
///     v(k+1) = v(k) + dt * vdot
///     q(k+1) = q(k) + dt * v(k+1)
pub fn semi_implicit_euler_step(
    q: &Vec<JointPosition>,
    v: &Vec<JointVelocity>,
    vdot: &Vec<JointAcceleration>,
    dt: Float,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let mut new_v = vec![];
    let mut new_q = vec![];
    for (qi, vi, vdot) in izip!(q.iter(), v.iter(), vdot.iter()) {
        match qi {
            JointPosition::Float(qi) => {
                let vdot = vdot.float();
                let new_vi = vi.float() + vdot * dt;
                new_v.push(JointVelocity::Float(new_vi));
                new_q.push(JointPosition::Float(qi + new_vi * dt));
            }
            JointPosition::Pose(qi) => {
                let vdot = vdot.spatial();
                let new_vi = vi.spatial() + &(vdot * dt);

                let quaternion_dot = quaternion_derivative(&qi.rotation, &new_vi.angular);
                let translation_dot = qi.rotation * new_vi.linear;
                let new_qi = Pose {
                    translation: qi.translation + translation_dot * dt,
                    rotation: UnitQuaternion::from_quaternion(
                        qi.rotation.quaternion() + quaternion_dot * dt,
                    ),
                };

                new_v.push(JointVelocity::Spatial(new_vi));
                new_q.push(JointPosition::Pose(new_qi));
            }
        }
    }

    (new_q, new_v)
}
