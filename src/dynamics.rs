use crate::{
    inertia::compute_inertias,
    mechanism::mass_matrix,
    spatial_force::{compute_torques, Wrench},
    transform::{compute_bodies_to_root, Transform3D},
    twist::{compute_joint_twists, compute_twists_wrt_world, Twist},
    types::Float,
    util::{mul_inertia, se3_commutator},
    GRAVITY,
};
use itertools::izip;
use na::{zero, DMatrix, DVector};
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

/// Compute the coriolis bias acceleration that results from the body velocity
/// and joint velocity, expressed in body frame.
/// Reference: Chapter 5.3 The Recursive Newton-Euler Algorithm in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn coriolis_accel(body_twist: &Twist, joint_twist: &Twist) -> SpatialAcceleration {
    if body_twist.frame != joint_twist.frame {
        panic!(
            "body_twist frame {}  is not equal to joint_twist frame {} !",
            body_twist.frame, joint_twist.frame
        );
    }
    if body_twist.body != joint_twist.body {
        panic!(
            "body_twist body frame {}  is not equal to joint_twist body frame {} !",
            body_twist.body, joint_twist.body
        )
    }
    if body_twist.base != "world" {
        panic!(
            "body_twist base frame {}  is not equal to world frame {} !",
            body_twist.base, "world"
        )
    }

    let (ang, lin) = se3_commutator(
        &body_twist.angular,
        &body_twist.linear,
        &joint_twist.angular,
        &joint_twist.linear,
    );

    SpatialAcceleration {
        body: body_twist.body.clone(),
        base: body_twist.base.clone(),
        frame: body_twist.frame.clone(),
        angular: ang,
        linear: lin,
    }
}

/// Compute the coriolis bias acceleration for all bodies.
/// Reference: Chapter 5.3 The Recursive Newton-Euler Algorithm in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn compute_coriolis_bias_accelerations(
    state: &MechanismState,
    bodies_to_root: &HashMap<u32, Transform3D>,
    twists: &HashMap<u32, Twist>,
    joint_twists: &HashMap<u32, Twist>,
) -> HashMap<u32, SpatialAcceleration> {
    let mut coriolis_bias_accels = HashMap::new();
    let rootid = 0;
    coriolis_bias_accels.insert(
        rootid,
        SpatialAcceleration {
            body: "world".to_string(),
            base: "world".to_string(),
            frame: "world".to_string(),
            angular: zero(),
            linear: zero(),
        },
    );
    for jointid in state.treejointids.iter() {
        let bodyid = jointid;
        let parentid = bodyid - 1;

        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let root_to_body = body_to_root.inv();

        // body twist wrt. world expressed in body frame
        let body_twist = twists.get(bodyid).unwrap().transform(&root_to_body);
        // joint twist expressed in body frame
        let joint_twist = joint_twists.get(bodyid).unwrap();
        let coriolis_accel = coriolis_accel(&body_twist, joint_twist).transform(body_to_root);

        let parent_bias_accel = coriolis_bias_accels.get(&parentid).unwrap();

        // rename the body frame so that it can be added to additionally
        // computed coriolis accel
        let parent_bias_accel = SpatialAcceleration {
            body: coriolis_accel.body.clone(), // rename to current body frame
            base: parent_bias_accel.base.clone(),
            frame: parent_bias_accel.frame.clone(),
            angular: parent_bias_accel.angular,
            linear: parent_bias_accel.linear,
        };
        coriolis_bias_accels.insert(*bodyid, &parent_bias_accel + &coriolis_accel);
        // Note: joint bias is zero for all of our current joint types, since
        // the apparent derivative of motion subspace S is zero.
        // Therefore, we don't add it here.
    }

    coriolis_bias_accels
}

/// Compute the bias acceleration for each body in world frame.
/// Bias acceleration is the acceleration that the body would have under no
/// external force.
///
/// Here, we add the inv gravity acceleration term to simulate gravity.
/// Imagine the whole system is in a elevator accelerating upwards at 9.81 m/s^2.
pub fn bias_accelerations(
    state: &MechanismState,
    bodies_to_root: &HashMap<u32, Transform3D>,
    twists: &HashMap<u32, Twist>,
    joint_twists: &HashMap<u32, Twist>,
) -> HashMap<u32, SpatialAcceleration> {
    let mut bias_accels = HashMap::new();
    let coriolis_bias_accels =
        compute_coriolis_bias_accelerations(state, bodies_to_root, twists, joint_twists);
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let bodyid = jointid;
        let body_name = &joint.transform().from;
        let inv_gravity_accel = SpatialAcceleration {
            body: body_name.clone(),
            base: "world".to_string(),
            frame: "world".to_string(),
            angular: Vector3::zeros(),
            linear: Vector3::new(0.0, 0.0, GRAVITY),
        };

        bias_accels.insert(*bodyid, &inv_gravity_accel + &coriolis_bias_accels[bodyid]);
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
    // Compute the twist of each joint
    let joint_twists = compute_joint_twists(state);

    // Compute the twist of the each body with respect to the world frame
    let twists = compute_twists_wrt_world(state, &bodies_to_root, &joint_twists);

    let bias_accels = bias_accelerations(&state, bodies_to_root, &twists, &joint_twists);

    let wrenches = newton_euler(&state, &bias_accels, &bodies_to_root, &twists);

    let torques = compute_torques(state, &wrenches, &bodies_to_root);

    torques
}

/// Solves the dynamics equation:
/// M(q) vdot + c(q, v) = τ
///
/// Note: mass_matrix is lower-triangular matrix
pub fn dynamics_solve(
    mass_matrix: &DMatrix<Float>,
    dynamics_bias: &DVector<Float>,
    tau: &DVector<Float>,
) -> DVector<Float> {
    // Convert lower-triangular matrix to full symmetric matrix M
    let mut M = mass_matrix.clone();
    for i in 0..mass_matrix.nrows() {
        for j in (i + 1)..mass_matrix.nrows() {
            M[(i, j)] = mass_matrix[(j, i)];
        }
    }

    // dynamics bias term
    let c = dynamics_bias;

    if let Some(vdot) = M.clone().lu().solve(&(tau - c)) {
        vdot
    } else {
        panic!(
            r#"Failed to solve for vdot in M(q) vdot + c(q, v) = 0
        where M = {}, 
              c = {}
        "#,
            M, dynamics_bias
        )
    }
}

/// Compute the joint acceleration vector vdot vdot that satisfies the joint-space
/// equations of motion:
///     M(q)vdot + c(q, v) = τ
/// given joint configuration vector q, joint velocity vector v, and joint
/// torques τ.
pub fn dynamics(state: &MechanismState, tau: &DVector<Float>) -> DVector<Float> {
    if state.v.len() != tau.len() {
        panic!(
            "Joint velocity vector v length {} and joint torques vector τ length {} differ!",
            state.v.len(),
            tau.len()
        );
    }

    // Compute the body to root frame transform for each body
    let bodies_to_root = compute_bodies_to_root(state);

    let dynamics_bias = dynamics_bias(state, &bodies_to_root); // c(q, v)
    let mass_matrix = mass_matrix(state, &bodies_to_root);

    let vdot = dynamics_solve(&mass_matrix, &dynamics_bias, tau);
    vdot
}

#[cfg(test)]
mod dynamics_tests {
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{
        double_pendulum::SimpleDoublePendulum,
        helpers::{build_double_pendulum, build_pendulum},
        inertia::SpatialInertia,
        joint::{revolute::RevoluteJoint, Joint},
        rigid_body::RigidBody,
        util::assert_close,
        GRAVITY, PI,
    };

    use super::*;

    #[test]
    fn dynamics_rod_pendulum_horizontal() {
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

        let state = crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state, &dvector![0.0]);

        // Assert
        assert_eq!(joint_accels, dvector![3.0 * GRAVITY / (2.0 * l)]);
    }

    /// world frame ^z       rod/joint frame ^y
    ///             |  / y                   |
    ///             | /                      |
    ///             |/                       |
    ///             +----> x                 +---->x
    #[test]
    fn dynamics_rod_pendulum_horizontal_rotated_frame() {
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

        let state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state, &dvector![0.0]);

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
    fn dynamics_rod_pendulum_horizontal_moved_frame() {
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

        let state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state, &dvector![0.0]);

        // Assert
        let error = (joint_accels - dvector![-3.0 * GRAVITY / (2.0 * l)]).abs();
        assert!(error < dvector![1e-6], "\nError too large: {:#?}", error);
        // Note: Needs to check for close equality because of numerical error
        // TODO: Handle this such that can check for exact equality?
        // assert_eq!(joint_accels, dvector![-3.0 * GRAVITY / (2.0 * l)]);
    }

    /// Apply a torque that should hold the rod at horizontal position
    #[test]
    fn dynamics_hold_horizontal_rod_pendulum() {
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

        let state = crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let torque = -m * GRAVITY * l / 2.0;
        let joint_accels = dynamics(&state, &dvector![torque]);

        // Assert
        let error = (joint_accels - dvector![0.0]).abs();
        assert!(
            error < dvector![1e-6],
            "\njoint accel error too large: {:#?}",
            error
        );
    }

    /// A pendulum of point mass
    #[test]
    fn dynamics_simple_pendulum_horizontal() {
        // Arrange
        let m = 5.0;
        let l: Float = 7.0;

        let moment_x = 0.0;
        let moment_y = m * l * l;
        let moment_z = m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l, 0., 0.];

        let rod_to_world = Matrix4::identity();
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let state = crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        // Act
        let joint_accels = dynamics(&state, &dvector![0.0]);

        // Assert
        assert_eq!(joint_accels, dvector![GRAVITY / l]);
    }

    /// Release a double pendulum (point masses on massless rods) from
    /// horizontal position.
    #[test]
    fn dynamics_double_pendulum_horizontal() {
        // Arrange
        let m = 5.0;
        let l: Float = 7.0;

        let moment_x = 0.0;
        let moment_y = m * l * l;
        let moment_z = m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l, 0., 0.];

        let rod1_to_world = Matrix4::identity();
        let rod2_to_rod1 = Transform3D::move_x(l);
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let state = MechanismState {
            treejoints: dvector![
                Joint::RevoluteJoint(RevoluteJoint {
                    init_mat: rod1_to_world,
                    transform: Transform3D::new("rod1", "world", &rod1_to_world),
                    axis: axis.clone(),
                }),
                Joint::RevoluteJoint(RevoluteJoint {
                    init_mat: rod2_to_rod1,
                    transform: Transform3D::new("rod2", "rod1", &rod2_to_rod1),
                    axis: axis.clone(),
                })
            ],
            treejointids: dvector![1, 2],
            bodies: dvector![
                RigidBody {
                    inertia: SpatialInertia {
                        frame: "rod1".to_string(),
                        moment: moment.clone(),
                        cross_part: cross_part.clone(),
                        mass: m,
                    }
                },
                RigidBody {
                    inertia: SpatialInertia {
                        frame: "rod2".to_string(),
                        moment: moment.clone(),
                        cross_part: cross_part.clone(),
                        mass: m,
                    }
                }
            ],
            q: dvector![0.0, 0.0],
            v: dvector![0.0, 0.0],
        };

        // Act
        let joint_accels = dynamics(&state, &dvector![0.0, 0.0]);

        // Assert
        assert_close(&joint_accels, &dvector![GRAVITY / l, -GRAVITY / l], 1e-6);
    }

    #[test]
    fn double_pendulum_dynamics() {
        // Arrange
        let m = 3.0;
        let l: Float = 5.0;

        let moment_x = m * l * l;
        let moment_y = m * l * l;
        let moment_z = 0.;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0., 0., -m * l];

        let rod1_to_world = Matrix4::identity();
        let rod2_to_rod1 = Transform3D::move_z(-l);
        let axis = vector![0.0, 1.0, 0.0];

        let mut state = build_double_pendulum(
            &m,
            &moment,
            &cross_part,
            &rod1_to_world,
            &rod2_to_rod1,
            &axis,
        );

        let q1 = 3.0;
        let q2 = 5.0;
        let q1dot = 3.0;
        let q2dot = 5.0;
        let q_init = dvector![q1, q2];
        let v_init = dvector![q1dot, q2dot];
        state.update(&q_init, &v_init);

        // Act
        let vdot = dynamics(&state, &dvector![0.0, 0.0]);

        // Assert
        let double_pendulum = SimpleDoublePendulum::new(m, m, l, l, q1, q2, q1dot, q2dot);
        let vdot_ref = DVector::from_column_slice(double_pendulum.dynamics().as_slice());
        assert_close(&vdot, &vdot_ref, 1e-5);
    }
}
