use std::collections::HashMap;

use itertools::izip;
use na::{UnitQuaternion, UnitVector3, Vector3};

use crate::{
    dynamics::{dynamics_continuous, dynamics_discrete},
    joint::{JointAcceleration, JointPosition, JointTorque, JointVelocity},
    mechanism::MechanismState,
    rigid_body::CollisionGeometry,
    spatial::{
        pose::Pose,
        twist::{compute_twists_wrt_world, Twist},
    },
    types::Float,
    util::quaternion_derivative,
};

pub enum Integrator {
    SemiImplicitEuler,
    RungeKutta2,
    RungeKutta4,
    VelocityStepping,
    CCDVelocityStepping,
}

pub fn semi_implicit_euler(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let vdot = dynamics_continuous(state, &tau);

    // Semi-implicit Euler integration
    // Note: this actually turns out to be energy conserving for Hamiltonian systems,
    // informally meaning systems that are not subject to velocity-dependent
    // forces. E.g. single pendulum
    //
    // Ref: Drake Doc, https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_semi_explicit_euler_integrator.html
    semi_implicit_euler_step(&state.q, &state.v, &vdot, dt)
}

/// Given state at current time-step, compute the state at next time-step.
pub fn velocity_stepping(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let (v_next, _contacts) = dynamics_discrete(state, &tau, dt, None);

    (compute_new_q(&state.q, &v_next, dt), v_next)
}

pub fn ccd_velocity_stepping(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let mut t_remain = dt;
    // expected contact from CCD step where time progressed to when this contact would happen
    let mut expected_contact: Option<(UnitVector3<Float>, Vector3<Float>, usize, Option<usize>)> =
        None;

    loop {
        let (v_new, contacts) = dynamics_discrete(state, &tau, t_remain, expected_contact);

        let joint_new_twists: Vec<Twist> = izip!(state.treejoints.iter(), v_new.iter())
            .map(|(joint, v)| Twist::new(&joint, v))
            .collect();

        let bodies_to_root = state.get_bodies_to_root();

        let new_twists = compute_twists_wrt_world(state, &bodies_to_root, &joint_new_twists);

        // Continuous collision detection (CCD) step
        let mut earliest_collision_t = Float::INFINITY;
        let mut earliest_collision: Option<(
            UnitVector3<Float>,
            Vector3<Float>,
            usize,
            Option<usize>,
        )> = None;
        for (bodyid, body) in izip!(state.treejointids.iter(), state.bodies.iter()) {
            if let Some(collider) = &body.collider {
                if !collider.enabled {
                    continue;
                }
                match &collider.geometry {
                    CollisionGeometry::Sphere(sphere) => {
                        // compute linear velocity of the body center
                        let twist = &new_twists[*bodyid];
                        let center = sphere.center();
                        let v = twist.linear + twist.angular.cross(&center);

                        // TODO: need to take into account that different halfspaces can give different constraints
                        // No check if this body pair's contact has already been accounted for in contact contraints
                        if contacts
                            .iter()
                            .any(|(_, _, contact_bodyid, contact_other_bodyid)| {
                                contact_bodyid == bodyid && contact_other_bodyid.is_none()
                            })
                        {
                            continue;
                        }

                        for halfspace in &state.halfspaces {
                            let n = halfspace.normal;
                            let v_dot_n = v.dot(&n);

                            // moving in parallel, early exit to avoid division by zero
                            if v_dot_n == 0. {
                                continue;
                            }

                            // if already in contact, should have already been handled in discrete collision checking step
                            let distance = sphere.distance_halfspace(halfspace);
                            if distance <= 0. {
                                continue;
                            }

                            // compute collision time
                            let t = distance / -v_dot_n;
                            if t < earliest_collision_t && t > 0. {
                                earliest_collision_t = t;

                                // compute expected contact point
                                let contact_point =
                                    sphere.center() + t * v - n.scale(sphere.radius);
                                earliest_collision = Some((n, contact_point, *bodyid, None))
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        expected_contact = earliest_collision;

        let q_new;
        if earliest_collision_t < t_remain {
            q_new = compute_new_q(&state.q, &v_new, earliest_collision_t);
            state.update(&q_new, &v_new);
            t_remain -= earliest_collision_t;
        } else {
            q_new = compute_new_q(&state.q, &v_new, t_remain);
            state.update(&q_new, &v_new);

            return (q_new, v_new);
        }
    }
}

pub fn runge_kutta_2(
    state: &mut MechanismState,
    dt: Float,
    tau: &Vec<JointTorque>,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let q0 = state.q.clone();
    let v0 = state.v.clone();

    let f1 = dynamics_continuous(state, &tau);
    let (q1, v1) = euler_step(&q0, &v0, &f1, dt / 2.0);

    state.update(&q1, &v1);
    let f2 = dynamics_continuous(state, &tau);
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

    let f1 = dynamics_continuous(state, &tau);

    let (q1, v1) = euler_step(&q0, &v0, &f1, dt / 2.0);
    state.update(&q1, &v1);
    let f2 = dynamics_continuous(state, &tau);

    let (q2, v2) = euler_step(&q0, &v0, &f2, dt / 2.0);
    state.update(&q2, &v2);
    let f3 = dynamics_continuous(state, &tau);

    let (q3, v3) = euler_step(&q0, &v0, &f3, dt);
    state.update(&q3, &v3);
    let f4 = dynamics_continuous(state, &tau);

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
fn euler_step(
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
            JointPosition::None => {
                new_v.push(JointVelocity::None);
                new_q.push(JointPosition::None);
            }
        }
    }

    (new_q, new_v)
}

/// Semi-Implicit Euler integration step:
///     v(k+1) = v(k) + dt * vdot
///     q(k+1) = q(k) + dt * v(k+1)
fn semi_implicit_euler_step(
    q: &Vec<JointPosition>,
    v: &Vec<JointVelocity>,
    vdot: &Vec<JointAcceleration>,
    dt: Float,
) -> (Vec<JointPosition>, Vec<JointVelocity>) {
    let new_v: Vec<JointVelocity> = izip!(v.iter(), vdot.iter())
        .map(|(vi, vdoti)| match vi {
            JointVelocity::Float(vi) => JointVelocity::Float(vi + vdoti.float() * dt),
            JointVelocity::Spatial(vi) => JointVelocity::Spatial(vi + &(vdoti.spatial() * dt)),
            JointVelocity::None => JointVelocity::None,
        })
        .collect();

    let new_q = compute_new_q(q, &new_v, dt);
    (new_q, new_v)
}

/// Compute the next q from current q and updated v
///    q(k+1) = q(k) + dt * v(k+1)
pub fn compute_new_q(
    current_q: &Vec<JointPosition>,
    new_v: &Vec<JointVelocity>,
    dt: Float,
) -> Vec<JointPosition> {
    izip!(current_q.iter(), new_v.iter())
        .map(|(qi, new_vi)| match qi {
            JointPosition::Float(qi) => JointPosition::Float(qi + new_vi.float() * dt),
            JointPosition::Pose(qi) => {
                let new_vi = new_vi.spatial();
                let quaternion_dot = quaternion_derivative(&qi.rotation, &new_vi.angular);
                let translation_dot = qi.rotation * new_vi.linear;
                let new_qi = Pose {
                    translation: qi.translation + translation_dot * dt,
                    rotation: UnitQuaternion::from_quaternion(
                        qi.rotation.quaternion() + quaternion_dot * dt,
                    ),
                };
                JointPosition::Pose(new_qi)
            }
            JointPosition::None => JointPosition::None,
        })
        .collect()
}

#[cfg(test)]
mod integrators_tests {

    use na::vector;
    use na::UnitVector3;
    use na::Vector3;

    use crate::assert_close;
    use crate::assert_vec_close;
    use crate::contact::HalfSpace;
    use crate::control::pusher_control::PusherController;
    use crate::control::Controller;
    use crate::helpers::build_pusher;
    use crate::helpers::build_quadruped;
    use crate::simulate::step;
    use crate::{helpers::build_cube, spatial::spatial_vector::SpatialVector, PI};

    use super::*;

    /// Shoot a box from mid-air, and verify that it hits ground and comes to rest.
    #[test]
    fn velocity_stepping_shoot_box() {
        // Arrange
        let l = 1.0;
        let mut state = build_cube(1.0, l);

        let rot_init = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 2.0);
        let q_init = vec![JointPosition::Pose(Pose {
            rotation: rot_init,
            translation: vector![0.0, 0.0, 1.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0., 0., 0.],
            linear: rot_init.inverse() * vector![1.0, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        let angle = Float::to_radians(-10.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let pose = state.poses()[0];

        assert_close!(pose.translation.dot(&normal), l / 2.0, 1e-2);
        assert_close!(pose.translation.y, 0.0, 1e-5);

        let velocity = state.v[0].spatial(); // final velocity in body frame
        assert_vec_close!(velocity.angular, Vector3::<Float>::zeros(), 1e-5);
        assert_vec_close!(velocity.linear, Vector3::<Float>::zeros(), 1e-5);
    }

    // Drop a quadruped (i.e. multi-link floating base model) on the ground
    #[test]
    fn velocity_stepping_drop_quadruped() {
        // Arrange
        let mut state = build_quadruped();
        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0., 0., 5.0],
            }),
        );
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.0));

        // Act
        let final_time = 3.0;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_q, _v) = step(&mut state, dt, &vec![], &Integrator::VelocityStepping);
        }

        // Assert
        let body_pose = state.poses()[0];
        let tol = 1e-3;
        assert!(body_pose.translation.z > 0.0 - tol);

        let body_vel = state.v[0].spatial();
        assert_vec_close!(body_vel.linear, Vector3::<Float>::zeros(), 1e-3);
        assert_vec_close!(body_vel.angular, Vector3::<Float>::zeros(), 1e-3);
    }

    #[test]
    fn velocity_stepping_box_pusher() {
        // Arrange
        let mut state = build_pusher();
        let ground = HalfSpace::new(Vector3::z_axis(), 0.0);
        state.add_halfspace(ground);

        let mut controller = PusherController {};

        // Act
        let final_time = 5.0;
        let dt = 1.0 / 120.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = controller.control(&mut state, None);
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let box_pose = state.poses()[2];
        assert!(
            box_pose.translation.x > 3.0,
            "x: {}",
            box_pose.translation.x
        ); // TODO(mu): needs friction mu to be < 1.0
        assert_close!(box_pose.translation.y, 0.0, 0.2);
        assert_close!(box_pose.translation.z, 0.25, 1e-2);
    }
}
