use na::{vector, Isometry3, Matrix3, Transform, Translation3, UnitQuaternion, Vector3};

use crate::{
    collision::{mesh::Mesh, sphere::Sphere},
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::{Collider, RigidBody},
    spatial::transform::Transform3D,
    WORLD_FRAME,
};

fn build_navbot_motor_body(mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.041;
    let r = 0.01385;
    let d = 0.02;

    let ixx = m * (3.0 * r * r + d * d) / 12.0;
    let izz = ixx;
    let iyy = m * r * r / 2.0;
    let moment = Matrix3::from_diagonal(&vector![ixx, iyy, izz]);

    let cross_part = vector![0., 0., 0.];
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    let collider = Collider::new_sphere(Sphere::new(r));

    RigidBody::new_collider_and_visual(collider, mesh, spatial_inertia)
}

pub fn build_navbot_motor(mesh: Mesh) -> MechanismState {
    let frame = "motor";
    let body = build_navbot_motor_body(mesh, frame);

    let body_to_world = Transform3D::identity(frame, WORLD_FRAME);

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(body_to_world))];
    let bodies = vec![body];
    MechanismState::new(treejoints, bodies)
}

pub fn build_balancing_bot() -> MechanismState {
    let body_frame = "body";
    let w_body = 0.06;
    let d_body = 0.05;
    let h_body = 0.025;
    let m_body = 0.5;
    let mut body = RigidBody::new_cuboid(m_body, w_body, d_body, h_body, body_frame);
    body.add_cuboid_contacts(w_body, d_body, h_body);

    let body_to_world = Transform3D::identity(body_frame, WORLD_FRAME);

    let m_wheel = 0.05;
    let r_wheel = 0.02;
    let h_offset = -2.0 * h_body;
    let wheel_left_frame = "wheel_left";
    let wheel_left = RigidBody::new_sphere(m_wheel, r_wheel, wheel_left_frame);
    let wheel_left_to_body =
        Transform3D::move_xyz(wheel_left_frame, body_frame, -w_body / 2.0, 0., h_offset);

    let wheel_right_frame = "wheel_right";
    let wheel_right = RigidBody::new_sphere(m_wheel, r_wheel, wheel_right_frame);
    let wheel_right_to_body =
        Transform3D::move_xyz(wheel_right_frame, body_frame, w_body / 2.0, 0., h_offset);

    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(wheel_left_to_body, Vector3::x_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(wheel_right_to_body, Vector3::x_axis())),
    ];
    let bodies = vec![body, wheel_left, wheel_right];
    MechanismState::new(treejoints, bodies)
}

pub struct NavbotMeshes {
    pub esp32pcb: Option<Mesh>,
    pub top_plate: Option<Mesh>,
}

impl NavbotMeshes {
    pub fn new() -> Self {
        NavbotMeshes {
            esp32pcb: None,
            top_plate: None,
        }
    }
}

fn build_navbot_base(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let w = 0.064;
    let h = 0.028;
    let d = 0.054;
    let m = 0.1;
    let moment_x = m * (h * h + d * d) / 3.0;
    let moment_y = m * (4. * h * h + w * w) / 12.0;
    let moment_z = m * (4. * d * d + w * w) / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., -h / 2.0, d / 2.0];

    let mut body = RigidBody::new(SpatialInertia::new(moment, cross_part, m, frame));

    let mut mesh = meshes.esp32pcb.take().expect("Navbot has no esp32pcb mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(0.000112254, 0.0303721, -0.00163541),
        UnitQuaternion::from_euler_angles(1.5708, -1.74345e-08, 1.5708),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    let mut mesh = meshes
        .top_plate
        .take()
        .expect("Navbot has no top plate mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(0.000112254, 0.0481221, 0.00957459),
        UnitQuaternion::from_euler_angles(-1.5708, -1.23575e-09, 9.92151e-09),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    body
}

pub fn build_navbot(mut meshes: NavbotMeshes) -> MechanismState {
    let base_frame = "base";
    let base = build_navbot_base(&mut meshes, base_frame);
    let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

    let bodies = vec![base];
    let treejoints = vec![Joint::RevoluteJoint(RevoluteJoint::new(
        base_to_world,
        Vector3::z_axis(),
    ))];

    MechanismState::new(treejoints, bodies)
}
