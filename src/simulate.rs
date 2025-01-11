use crate::{
    dynamics::{dynamics, newton_euler},
    mechanism::MechanismState,
    spatial_acceleration::SpatialAcceleration,
    spatial_force::compute_torques,
    transform::{compute_bodies_to_root, Transform3D},
    twist::{compute_twists_wrt_world, Twist},
    types::Float,
};
use itertools::izip;
use na::{dvector, DVector};
use std::collections::HashMap;

pub struct DynamicsResult {}

impl DynamicsResult {
    pub fn new() -> Self {
        DynamicsResult {}
    }
}

pub fn spatial_accelerations(
    state: &mut MechanismState,
    vdot: &DVector<Float>,
) -> (
    HashMap<u32, SpatialAcceleration>,
    HashMap<u32, Transform3D>,
    HashMap<u32, Twist>,
) {
    let joints = &state.treejoints;
    let jointids = &state.treejointids;
    let qs = &state.q;
    let vs = &state.v;
    let vdots = vdot;

    let rootid = 0; // root has body id 0 by convention

    // Compute the body to root frame transform for each body
    let bodies_to_root = compute_bodies_to_root(state);

    // Compute the twist of the each body with respect to the world frame
    let twists = compute_twists_wrt_world(state, &bodies_to_root);

    // Compute the joint spatial accelerations of each body expressed in body frame
    let mut joint_accels: HashMap<u32, SpatialAcceleration> = HashMap::new();
    for (jointid, joint, _q, _v, vdot) in izip!(
        jointids.iter(),
        joints.iter(),
        qs.iter(),
        vs.iter(),
        vdots.iter()
    ) {
        let bodyid = jointid;
        joint_accels.insert(*bodyid, joint.spatial_acceleration(vdot));
    }

    // Compute the spatial acceleration of each body wrt. world frame, expressed
    // in world frame
    let mut accels: HashMap<u32, SpatialAcceleration> = HashMap::new();
    accels.insert(
        rootid,
        SpatialAcceleration::inv_gravitational_spatial_acceleration(),
    ); // simulates the effect of gravity
    for jointid in jointids.iter() {
        let parentbodyid = jointid - 1;
        let bodyid = jointid;
        let parent_acc = accels.get(&parentbodyid).unwrap();
        let parent_twist = twists.get(&parentbodyid).unwrap();
        let body_twist = twists.get(bodyid).unwrap();

        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let joint_acc = joint_accels.get(bodyid).unwrap().transform(body_to_root);

        let body_acc = &(parent_acc + &joint_acc) + &parent_twist.cross(body_twist);
        accels.insert(*bodyid, body_acc);
    }

    (accels, bodies_to_root, twists)
}

/// Do inverse dynamics, i.e. compute τ in the unconstrained joint-space
/// equations of motion
///
/// M(q) vdot + c(q, v) = τ
///
/// given joint configuration vector q, joint velocity vector v, joint
/// acceleration vector vdot.
///
/// This method implements the recursive Newton-Euler algorithm.
pub fn inverse_dynamics(state: &mut MechanismState, vdot: &DVector<Float>) -> DVector<Float> {
    let (accels, bodies_to_root, twists) = spatial_accelerations(state, &vdot);
    let wrenches = newton_euler(&state, &accels, &bodies_to_root, &twists);
    let torquesout = compute_torques(state, &wrenches, &bodies_to_root);

    torquesout
}

/// Step the mechanism state forward by dt seconds.
pub fn step(
    state: &mut MechanismState,
    dt: Float,
    tau: &DVector<Float>,
) -> (DVector<Float>, DVector<Float>) {
    let vdot = dynamics(state, tau);

    // Semi-implicit Euler integration
    // Note: this actually turns out to be stable for pendulum system
    let v = state.v.clone() + vdot * dt;
    let q = state.q.clone() + v.clone() * dt;

    state.update(&q, &v);
    (q, v)
}

// pub fn integrate(result: &DynamicsResult, state: &MechanismState, final_time: Float, dt: Float) {
//     let mut t = 0.0;
//     while t < final_time {
//         step(&result, &state, t, dt);
//         t += dt;
//     }
// }

/// Simulate the mechanism state from 0 to final_time with a time step of dt.
/// Returns the joint configurations at each time step.
pub fn simulate<F>(
    state: &mut MechanismState,
    final_time: Float,
    dt: Float,
    control_fn: F,
) -> (DVector<DVector<Float>>, DVector<DVector<Float>>)
where
    F: Fn(&MechanismState) -> DVector<Float>,
{
    let mut t = 0.0;
    let mut qs: DVector<DVector<Float>> = dvector![];
    let mut vs: DVector<DVector<Float>> = dvector![];
    qs.extend([state.q.clone()]);
    vs.extend([state.v.clone()]);
    while t < final_time {
        let tau = control_fn(state);
        let (q, v) = step(state, dt, &tau);
        qs.extend([q]);
        vs.extend([v]);

        t += dt;
    }

    (qs, vs)
}

#[cfg(test)]
mod simulate_tests {
    use crate::{
        helpers::{build_cart, build_cart_pole},
        util::assert_close,
        GRAVITY, PI,
    };

    use super::*;
    use na::Matrix4;
    use nalgebra::{dvector, vector, Matrix3};

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

        let rod_to_world = Matrix4::identity(); // transformation from rod to world frame
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        let initial_energy = 0.0 + 0.0; // E = PE + KE, both are zero at the start.

        // Act
        let final_time = 10.0;
        let dt = 0.02;
        let (qs, vs) = simulate(&mut state, final_time, dt, |_state| dvector![0.0]);

        // Assert
        let q_max = qs
            .iter()
            .map(|q| q[0])
            .fold(Float::NEG_INFINITY, Float::max);
        assert!((q_max - PI).abs() < 1e-3); // Check highest point of swing

        let q_final = qs[qs.len() - 1][0];
        let v_final = vs[vs.len() - 1][0];

        let potential_energy = m * GRAVITY * l / 2.0 * (-q_final.sin()); // mgh
        let kinetic_energy = 0.5 * (m * l * l / 3.0) * v_final * v_final; // 1/2 I ω^2
        assert!((initial_energy - (potential_energy + kinetic_energy)).abs() < 1.0);
        // Sanity check that energy is conserved. Not exact due to numerical integration.
        // Note: this is potentially flaky test depending on parameters
    }

    #[test]
    fn inverse_dynamics_upright_rod() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 3.0; // Length of rod
        let moment_x = 1.0 / 3.0 * m * l * l;
        let moment_y = moment_x;
        let moment_z = 0.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0.0, 0.0, l / 2.0];

        let rod_to_world = Matrix4::identity(); // transformation from rod to world frame
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        let vdot = dvector![-1.0]; // acceleration around -y axis

        // Act
        let torquesout = inverse_dynamics(&mut state, &vdot);

        // Assert
        assert_eq!(torquesout, dvector![-1.0 / 3.0 * m * l * l]);
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

        let mut state = build_cart(&m, &moment, &cross_part);

        let q_init = dvector![2.0];
        let v_init = dvector![-1.0];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 10.0;
        let dt = 0.02;
        let (qs, vs) = simulate(&mut state, final_time, dt, |_state| dvector![0.0]);

        // Assert
        let q_final = &qs[qs.len() - 1];
        let v_final = &vs[vs.len() - 1];

        let q_calc = q_init + v_init.clone() * final_time;
        assert_close(q_final, &q_calc, 1e-4);
        assert_eq!(*v_final, v_init);
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

        let mut state = build_cart_pole(
            &m_cart,
            &m_pole,
            &moment_cart,
            &moment_pole,
            &cross_part_cart,
            &cross_part_pole,
        );

        let F_cart = 1.0; // force to be exerted on the cart
        let acc = F_cart / (m_cart + m_pole);
        let pole_theta_expected = acc.atan2(GRAVITY); // steady state pole joint angle

        let q_init = dvector![0.0, pole_theta_expected]; // set to steady state
        let v_init = dvector![0.0, 0.0];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 20.0;
        let dt = 0.02;
        let (qs, vs) = simulate(&mut state, final_time, dt, |_state| dvector![F_cart, 0.0]);

        // Assert
        let v_final = vs[vs.len() - 1][0];
        assert!((v_final - acc * final_time).abs() < 1e-5);

        let pole_theta_final = qs[qs.len() - 1][1];
        assert!((pole_theta_final - pole_theta_expected).abs() < 1e-5);
    }
}
