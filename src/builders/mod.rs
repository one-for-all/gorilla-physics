use na::{vector, Isometry3, Matrix3, Translation3, UnitQuaternion};

use crate::{
    collision::mesh::Mesh,
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::transform::Transform3D,
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

    RigidBody::new_mesh(mesh, spatial_inertia)
}

/// Build SO-Arm 101 from its URDF description
/// Ref: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
pub fn build_so101(base_mesh: Mesh) -> MechanismState {
    let base_frame = "base";
    let base_body = build_so101_base_body(base_mesh, base_frame);
    let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(base_to_world))];
    let bodies = vec![base_body];
    MechanismState::new(treejoints, bodies)
}
