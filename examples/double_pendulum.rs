use gorilla_physics::joint::ToJointPositionVec;
use gorilla_physics::joint::ToJointVelocityVec;
use gorilla_physics::transform::Matrix4Ext;
use gorilla_physics::{
    dynamics::bias_accelerations,
    helpers::build_double_pendulum,
    transform::{compute_bodies_to_root, Transform3D},
    twist::{compute_joint_twists, compute_twists_wrt_world},
    types::Float,
};
use nalgebra::{vector, Matrix3, Matrix4};

/// Verify bias_acceleration function by looking at its result on a double-pendulum
pub fn main() {
    let m = 1.0;
    let l: Float = 1.0;

    let moment_x = m * l * l;
    let moment_y = m * l * l;
    let moment_z = 0.;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., 0., -m * l];

    let rod1_to_world = Matrix4::identity();
    let rod2_to_rod1 = Matrix4::<Float>::move_z(-l);
    let axis = vector![0.0, 1.0, 0.0];

    let mut state = build_double_pendulum(
        &m,
        &moment,
        &cross_part,
        &rod1_to_world,
        &rod2_to_rod1,
        &axis,
    );

    let q_init = vec![1., 1.].to_joint_pos_vec();
    let v_init = vec![1., 1.].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    let bodies_to_root = compute_bodies_to_root(&state); // Compute the twist of each joint
    let joint_twists = compute_joint_twists(&state);
    let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);
    println!(
        "{:#?}",
        bias_accelerations(&state, &bodies_to_root, &twists, &joint_twists)
    );
}
