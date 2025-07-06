use na::{vector, Isometry3, Matrix3, Translation3, UnitQuaternion};

use crate::{
    collision::mesh::Mesh,
    inertia::SpatialInertia,
    joint::{fixed::FixedJoint, floating::FloatingJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::transform::Transform3D,
    types::Float,
    WORLD_FRAME,
};

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

/// Build SO-Arm 101 from its URDF description
/// Ref: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
pub fn build_so101(base_mesh: Mesh, shoulder_mesh: Mesh) -> MechanismState {
    let base_frame = "base";
    let base_body = build_so101_base_body(base_mesh, base_frame);
    let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

    let shoulder_frame = "shoulder";
    let shoulder_body = build_so101_shoulder_body(shoulder_mesh, shoulder_frame);
    let shoulder_to_base = Transform3D::new_xyz_rpy(
        shoulder_frame,
        base_frame,
        &vec![0.0388353, -8.97657e-09, 0.0624],
        &vec![3.14159, 4.18253e-17, -3.1415], // TODO: -3.14159 vs -3.1415 makes huge difference. Fix it.
    );

    let treejoints = vec![
        Joint::FixedJoint(FixedJoint::new(base_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(shoulder_to_base, vector![0., 0., 1.])),
    ];
    let bodies = vec![base_body, shoulder_body];
    MechanismState::new(treejoints, bodies)
}

#[cfg(test)]
mod so101_tests {
    use crate::{collision::mesh::Mesh, simulate::step, util::read_file};

    use super::build_so101;

    #[test]
    #[ignore] // TODO: complete this test
    fn so101() {
        // Arrange
        let base_buf = read_file("data/so101/base_so101_v2.obj");
        let base_mesh = Mesh::new_from_obj(&base_buf);

        let shoulder_buf = read_file("data/so101/rotation_pitch_so101_v1.obj");
        let shoulder_mesh = Mesh::new_from_obj(&shoulder_buf);

        let mut state = build_so101(base_mesh, shoulder_mesh);

        // Act
        let dt = 1.0 / 60.0;
        for _s in 0..50 {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );
            println!(
                "vertex: {:?}",
                state.bodies[1]
                    .collider
                    .as_ref()
                    .unwrap()
                    .geometry
                    .mesh()
                    .vertices[1000]
            );
        }

        // Assert
    }
}
