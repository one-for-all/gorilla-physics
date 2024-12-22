use crate::{
    inertia::compute_inertias,
    mechanism::mass_matrix,
    spatial_force::compute_torques,
    spatial_force::Wrench,
    transform::{compute_bodies_to_root, Transform3D},
    twist::{compute_twists_wrt_world, Twist},
    types::Float,
    util::mul_inertia,
    GRAVITY,
};
use itertools::izip;
use na::{DMatrix, DVector};
use nalgebra::Vector3;
use std::collections::HashMap;

use crate::{mechanism::MechanismState, spatial_acceleration::SpatialAcceleration};

/// Apply the Newton-Euler equation to compute the wrench to move each body at
/// given acceleration and velocity:
///     f_i = I_i * a_i + v_i \dualcross I_i * v_i
///
/// Reference: Table 5.1 in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn newton_euler(
    state: &MechanismState,
    accels: &HashMap<u32, SpatialAcceleration>,
    bodies_to_root: &HashMap<u32, Transform3D>,
    twists: &HashMap<u32, Twist>,
) -> HashMap<u32, Wrench> {
    // Compute the body inertias wrt. world frame
    let inertias = compute_inertias(state, bodies_to_root);

    // Compute the wrenches at each joint for each body expressed in world frame
    let mut wrenches: HashMap<u32, Wrench> = HashMap::new();
    for jointid in state.treejointids.iter() {
        let bodyid = jointid;
        let I = inertias.get(bodyid).unwrap();
        let T = twists.get(bodyid).unwrap();
        let Tdot = accels.get(bodyid).unwrap();

        if T.frame != Tdot.frame {
            panic!(
                "T frame {}  is not equal to Tdot frame {} !",
                T.frame, Tdot.frame
            );
        }

        if T.body != Tdot.body || T.base != Tdot.base {
            panic!("T and Tdot body/base do not match!");
        }

        let (mut ang, mut lin) = mul_inertia(
            &I.moment,
            &I.cross_part,
            I.mass,
            &Tdot.angular,
            &Tdot.linear,
        );
        let (angular_momentum, linear_momentum) =
            mul_inertia(&I.moment, &I.cross_part, I.mass, &T.angular, &T.linear);

        ang += T.angular.cross(&angular_momentum) + T.linear.cross(&linear_momentum);
        lin += T.angular.cross(&linear_momentum);
        wrenches.insert(
            *bodyid,
            Wrench {
                frame: T.frame.clone(),
                angular: ang,
                linear: lin,
            },
        );
    }
    wrenches
}

/// Compute the bias acceleration for each body in world frame.
/// Bias acceleration is the acceleration that the body would have under no
/// external force.
///
/// Here, we add the inv gravity acceleration term to simulate gravity.
/// Imagine the whole system is in a elevator accelerating upwards at 9.81 m/s^2.
pub fn bias_accelerations(state: &MechanismState) -> HashMap<u32, SpatialAcceleration> {
    let mut bias_accels = HashMap::new();
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let bodyid = jointid;
        let body_name = &joint.transform.from;
        let inv_gravity_accel = SpatialAcceleration {
            body: body_name.clone(),
            base: "world".to_string(),
            frame: "world".to_string(),
            angular: Vector3::zeros(),
            linear: Vector3::new(0.0, 0.0, GRAVITY),
        };
        bias_accels.insert(*bodyid, inv_gravity_accel);
    }

    bias_accels
}

/// Compute the 'dynamics bias term', i.e. the term
///     c(q, v)
/// in the unconstrained joint-space equations of motion
///     M(q) vdot + c(q, v) = τ
/// given joint configuration vector q, joint velocity vector v, joint
/// acceleration vector vdot.
pub fn dynamics_bias(
    state: &MechanismState,
    bodies_to_root: &HashMap<u32, Transform3D>,
) -> DVector<Float> {
    let bias_accels = bias_accelerations(&state);

    // Compute the twist of the each body with respect to the world frame
    let twists = compute_twists_wrt_world(state, &bodies_to_root);

    let wrenches = newton_euler(&state, &bias_accels, &bodies_to_root, &twists);

    let torques = compute_torques(state, &wrenches, &bodies_to_root);

    torques
}

/// Solves the dynamics equation:
/// M(q) vdot + c(q, v) = 0
pub fn dynamics_solve(
    mass_matrix: &DMatrix<Float>,
    dynamics_bias: &DVector<Float>,
) -> DVector<Float> {
    // Convert lower-triangular matrix to full symmetric matrix
    let mut full_mass_matrix = mass_matrix.clone();
    for i in 0..mass_matrix.nrows() {
        for j in (i + 1)..mass_matrix.nrows() {
            full_mass_matrix[(i, j)] = mass_matrix[(j, i)];
        }
    }

    if let Some(vdot) = full_mass_matrix.clone().lu().solve(&(-dynamics_bias)) {
        vdot
    } else {
        panic!(
            r#"Failed to solve for vdot in M(q) vdot + c(q, v) = 0
        where M = {}, 
              c = {}
        "#,
            full_mass_matrix, dynamics_bias
        )
    }
}

/// Compute the joint acceleration vector vdot vdot that satisfies the joint-space
/// equations of motion:
///     M(q)vdot + c(q, v) = τ
/// given joint configuration vector q, joint velocity vector v, and joint
/// torques τ which is 0 for now.
pub fn dynamics(state: &MechanismState) -> DVector<Float> {
    // Compute the body to root frame transform for each body
    let bodies_to_root = compute_bodies_to_root(state);

    let dynamics_bias = dynamics_bias(state, &bodies_to_root); // c(q, v)
    let mass_matrix = mass_matrix(state, &bodies_to_root);

    let vdot = dynamics_solve(&mass_matrix, &dynamics_bias);
    vdot
}

#[cfg(test)]
mod dynamics_tests {
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{test_helpers::build_rod_pendulum, GRAVITY, PI};

    use super::*;

    #[test]
    fn dynamics_horizontal_right_rod() {
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

        let state =
            crate::test_helpers::build_rod_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state);

        // Assert
        assert_eq!(joint_accels, dvector![3.0 * GRAVITY / (2.0 * l)]);
    }

    /// world frame ^z       rod/joint frame ^y
    ///             |  / y                   |
    ///             | /                      |
    ///             |/                       |
    ///             +----> x                 +---->x
    #[test]
    fn dynamics_horizontal_right_rod_rotated_frame() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Transform3D::rot_x(PI / 2.0);
        let axis = vector![0.0, 0.0, 1.0];

        let state = build_rod_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state);

        // Assert
        assert_eq!(joint_accels, dvector![-3.0 * GRAVITY / (2.0 * l)]);
    }

    /// world frame ^z       rod/joint frame ^y
    ///             |  / y                   |
    ///             | /                      |
    ///             |/                       |
    ///             +----> x                 +---->x
    /// separated by some distance in x
    #[test]
    fn dynamics_horizontal_right_rod_rotated_moved_rod() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod
        let d: Float = 11.0;

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Transform3D::move_x(d) * Transform3D::rot_x(PI / 2.0);
        let axis = vector![0.0, 0.0, 1.0];

        let state = build_rod_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state);

        // Assert
        let error = (joint_accels - dvector![-3.0 * GRAVITY / (2.0 * l)]).abs();
        assert!(error < dvector![1e-6], "\nError too large: {:#?}", error);
        // Note: Needs to check for close equality because of numerical error
        // TODO: Handle this such that can check for exact equality?
        // assert_eq!(joint_accels, dvector![-3.0 * GRAVITY / (2.0 * l)]);
    }
}
