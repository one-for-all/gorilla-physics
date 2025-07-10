use na::{vector, Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

use crate::{
    collision::mesh::Mesh,
    inertia::SpatialInertia,
    joint::{fixed::FixedJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::transform::Transform3D,
    WORLD_FRAME,
};

pub mod navbot;

fn build_so101_base_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.147;
    let com = vector![0.0137179, -5.19711e-05, 0.0334843];
    let ixx = 0.000114686;
    let ixy = -4.59787e-07;
    let ixz = 4.97151e-06;
    let iyy = 0.000136117;
    let iyz = 9.75275e-08;
    let izz = 0.000130364;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(-0.00636471, -8.97657e-09, -0.0024),
        UnitQuaternion::from_euler_angles(1.5708, -2.78073e-29, 1.5708),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

fn build_so101_shoulder_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.100006;
    let com = vector![-0.0307604, -1.66727e-05, -0.0252713];
    let ixx = 8.3759e-05;
    let ixy = 7.55525e-08;
    let ixz = -1.16342e-06;
    let iyy = 8.10403e-05;
    let iyz = 1.54663e-07;
    let izz = 2.39783e-05;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(0.0122008, 2.22413e-05, 0.0464),
        UnitQuaternion::from_euler_angles(-1.5708, 2.35221e-33, 0.),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

fn build_so101_upper_arm_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.103;
    let com = vector![-0.0898471, -0.00838224, 0.0184089];
    let ixx = 4.08002e-05;
    let ixy = -1.97819e-05;
    let ixz = -4.03016e-08;
    let iyy = 0.000147318;
    let iyz = 8.97326e-09;
    let izz = 0.000142487;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(-0.065085, 0.012, 0.0182),
        UnitQuaternion::from_euler_angles(3.14159, -0., -1.30911e-30),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

fn build_so101_lower_arm_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.104;
    let com = vector![-0.0980701, 0.00324376, 0.0182831];
    let ixx = 2.87438e-05;
    let ixy = 7.41152e-06;
    let ixz = 1.26409e-06;
    let iyy = 0.000159844;
    let iyz = -4.90188e-08;
    let izz = 0.00014529;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(-0.0648499, -0.032, 0.0182),
        UnitQuaternion::from_euler_angles(3.14159, -0., 6.67202e-31),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

fn build_so101_wrist_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.079;
    let com = vector![-0.000103312, -0.0386143, 0.0281156];
    let ixx = 3.68263e-05;
    let ixy = 1.7893e-08;
    let ixz = -5.28128e-08;
    let iyy = 2.5391e-05;
    let iyz = 3.6412e-06;
    let izz = 2.1e-05;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(0., -0.028, 0.0181),
        UnitQuaternion::from_euler_angles(-1.5708, -1.5708, 0.),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

fn build_so101_gripper_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.087;
    let com = vector![0.000213627, 0.000245138, -0.025187];
    let ixx = 2.75087e-05;
    let ixy = -3.35241e-07;
    let ixz = -5.7352e-06;
    let iyy = 4.33657e-05;
    let iyz = -5.17847e-08;
    let izz = 3.45059e-05;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(8.32667e-17, -0.000218214, 0.000949706),
        UnitQuaternion::from_euler_angles(-3.14159, -5.55112e-17, 0.),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

fn build_so101_jaw_body(mut mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.012;
    let com = vector![-0.00157495, -0.0300244, 0.0192755];
    let ixx = 6.61427e-06;
    let ixy = -3.19807e-07;
    let ixz = -5.90717e-09;
    let iyy = 1.89032e-06;
    let iyz = -1.09945e-07;
    let izz = 5.28738e-06;

    // inertia moment matrix about the center-of-mass
    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz, 
        ixy, iyy, iyz, 
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    // Update mesh initial pose relative to body frame
    let iso = Isometry3::from_parts(
        Translation3::new(-5.55112e-17, -5.55112e-17, 0.0189),
        UnitQuaternion::from_euler_angles(9.53145e-17, 6.93889e-18, 1.24077e-24),
    );
    mesh.update_base_isometry(&iso);

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

/// Build SO-Arm 101 from its URDF description
/// Ref: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
pub fn build_so101(
    base_mesh: Mesh,
    shoulder_mesh: Mesh,
    upper_arm_mesh: Mesh,
    lower_arm_mesh: Mesh,
    wrist_mesh: Mesh,
    gripper_mesh: Mesh,
    jaw_mesh: Mesh,
) -> MechanismState {
    let base_frame = "base";
    let base_body = build_so101_base_body(base_mesh, base_frame);
    let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

    let shoulder_frame = "shoulder";
    let shoulder_body = build_so101_shoulder_body(shoulder_mesh, shoulder_frame);
    let shoulder_to_base = Transform3D::new_xyz_rpy(
        shoulder_frame,
        base_frame,
        &vec![0.0388353, -8.97657e-09, 0.0624],
        &vec![3.14159, 4.18253e-17, -3.14159],
    );

    let upper_arm_frame = "upper_arm";
    let upper_arm_body = build_so101_upper_arm_body(upper_arm_mesh, upper_arm_frame);
    let upper_arm_to_shoulder = Transform3D::new_xyz_rpy(
        upper_arm_frame,
        shoulder_frame,
        &vec![-0.0303992, -0.0182778, -0.054],
        &vec![-1.5708, -1.5708, 0.],
    );

    let lower_arm_frame = "lower_arm";
    let lower_arm_body = build_so101_lower_arm_body(lower_arm_mesh, lower_arm_frame);
    let lower_arm_to_upper_arm = Transform3D::new_xyz_rpy(
        lower_arm_frame,
        upper_arm_frame,
        &vec![-0.11257, -0.028, 1.73763e-16],
        &vec![-3.63608e-16, 8.74301e-16, 1.5708],
    );

    let wrist_frame = "wrist";
    let wrist_body = build_so101_wrist_body(wrist_mesh, wrist_frame);
    let wrist_to_lower_arm = Transform3D::new_xyz_rpy(
        wrist_frame,
        lower_arm_frame,
        &vec![-0.1349, 0.0052, 3.62355e-17],
        &vec![4.02456e-15, 8.67362e-16, -1.5708],
    );

    let gripper_frame = "gripper";
    let gripper_body = build_so101_gripper_body(gripper_mesh, gripper_frame);
    let gripper_to_wrist = Transform3D::new_xyz_rpy(
        gripper_frame,
        wrist_frame,
        &vec![5.55112e-17, -0.0611, 0.0181],
        &vec![1.5708, 0.0486795, 3.14159],
    );

    let jaw_frame = "jaw";
    let jaw_body = build_so101_jaw_body(jaw_mesh, jaw_frame);
    let jaw_to_gripper = Transform3D::new_xyz_rpy(
        jaw_frame,
        gripper_frame,
        &vec![0.0202, 0.0188, -0.0234],
        &vec![1.5708, -5.24284e-08, -1.41553e-15],
    );

    let treejoints = vec![
        Joint::FixedJoint(FixedJoint::new(base_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(shoulder_to_base, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(upper_arm_to_shoulder, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(
            lower_arm_to_upper_arm,
            Vector3::z_axis(),
        )),
        Joint::RevoluteJoint(RevoluteJoint::new(wrist_to_lower_arm, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(gripper_to_wrist, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(jaw_to_gripper, Vector3::z_axis())),
    ];
    let bodies = vec![
        base_body,
        shoulder_body,
        upper_arm_body,
        lower_arm_body,
        wrist_body,
        gripper_body,
        jaw_body,
    ];
    MechanismState::new(treejoints, bodies)
}

#[cfg(test)]
mod so101_tests {
    use crate::{
        collision::mesh::Mesh,
        control::{so101_control::SO101PositionController, Controller},
        plot::plot,
        simulate::step,
        util::read_file,
    };

    use super::build_so101;

    #[test]
    // #[ignore] // TODO: complete this test
    fn so101() {
        // Arrange
        let base_buf = read_file("data/so101/base_so101_v2.obj");
        let base_mesh = Mesh::new_from_obj(&base_buf);

        let shoulder_buf = read_file("data/so101/rotation_pitch_so101_v1.obj");
        let shoulder_mesh = Mesh::new_from_obj(&shoulder_buf);

        let upper_arm_buf = read_file("data/so101/upper_arm_so101_v1.obj");
        let upper_arm_mesh = Mesh::new_from_obj(&upper_arm_buf);

        let lower_arm_buf = read_file("data/so101/under_arm_so101_v1.obj");
        let lower_arm_mesh = Mesh::new_from_obj(&lower_arm_buf);

        let wrist_buf = read_file("data/so101/wrist_roll_pitch_so101_v2.obj");
        let wrist_mesh = Mesh::new_from_obj(&wrist_buf);

        let gripper_buf = read_file("data/so101/wrist_roll_follower_so101_v1.obj");
        let gripper_mesh = Mesh::new_from_obj(&gripper_buf);

        let jaw_buf = read_file("data/so101/moving_jaw_so101_v1.obj");
        let jaw_mesh = Mesh::new_from_obj(&jaw_buf);

        let mut state = build_so101(
            base_mesh,
            shoulder_mesh,
            upper_arm_mesh,
            lower_arm_mesh,
            wrist_mesh,
            gripper_mesh,
            jaw_mesh,
        );

        // Act
        let mut data = vec![];
        let final_time = 2.0;
        let dt = 1.0 / (100.0 * 60.0);
        let num_steps = (final_time / dt) as usize;
        let mut controller = SO101PositionController {};
        for _s in 0..num_steps {
            let tau = controller.control(&mut state, None);
            let (_q, _v) = step(
                &mut state,
                dt,
                &tau,
                &crate::integrators::Integrator::VelocityStepping,
            );

            data.push(*state.q[5].float());
        }

        // Assert
        plot(&data, final_time, dt, num_steps, "so101");
        println!("final q: {:#?}", state.q);
    }
}
