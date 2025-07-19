use na::{vector, Isometry3, Matrix3, Rotation3, Transform, Translation3, UnitQuaternion, Vector3};

use crate::{
    collision::{mesh::Mesh, sphere::Sphere},
    inertia::SpatialInertia,
    joint::{
        constraint_revolute::RevoluteConstraintJoint, floating::FloatingJoint,
        revolute::RevoluteJoint, Joint,
    },
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
    pub side_plate_left: Option<Mesh>,
    pub side_plate_right: Option<Mesh>,

    pub leg_inner_left: Option<Mesh>,
    pub leg_outer_left: Option<Mesh>,

    pub foot_pin_left: Option<Mesh>,
    pub foot_motor_left: Option<Mesh>,
    pub foot_encoder_left: Option<Mesh>,
    pub foot_plate_left: Option<Mesh>,

    pub link_plate_left: Option<Mesh>,
}

impl NavbotMeshes {
    pub fn new() -> Self {
        NavbotMeshes {
            esp32pcb: None,
            top_plate: None,
            side_plate_left: None,
            side_plate_right: None,

            leg_inner_left: None,
            leg_outer_left: None,

            foot_pin_left: None,
            foot_motor_left: None,
            foot_encoder_left: None,
            foot_plate_left: None,

            link_plate_left: None,
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

    let mut mesh = meshes
        .side_plate_left
        .take()
        .expect("Navbot has no side plate left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(-0.0318877, 0.0253721, -0.0176354),
        UnitQuaternion::from_euler_angles(1.5708, 1.5708, 0.),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    let mut mesh = meshes
        .side_plate_right
        .take()
        .expect("Navbot has no side plate right mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(0.0321123, 0.0253721, -0.0176354),
        UnitQuaternion::from_euler_angles(-1.5708, 1.5708, 0.),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    body
}

fn build_navbot_leg_left(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.00775586;
    let com = vector![-0.0134676, -0.000849763, -0.0089392];
    let ixx = 4.45178e-07;
    let ixy = -2.74989e-08;
    let ixz = 2.09677e-08;
    let iyy = 2.67121e-06;
    let iyz = 4.57695e-09;
    let izz = 2.81454e-06;

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
    let mut body = RigidBody::new(spatial_inertia);

    let mut mesh = meshes
        .leg_inner_left
        .take()
        .expect("Navbot has no leg inner left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(-3.46945e-18, 4.33681e-19, -0.004),
        UnitQuaternion::from_euler_angles(-1.5708, -1.43064e-25, -1.50654e-17),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    let mut mesh = meshes
        .leg_outer_left
        .take()
        .expect("Navbot has no leg outer left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(-6.93889e-18, 4.33681e-18, -0.0105),
        UnitQuaternion::from_euler_angles(-1.5708, -1.66533e-16, -2.66594e-16),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    body
}

fn build_navbot_foot_left(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.0379925;
    let com = vector![0.0351141, 0.0277485, -0.000900737];
    let ixx = 4.76231e-06;
    let ixy = -3.17037e-06;
    let ixz = -1.41079e-07;
    let iyy = 6.22648e-06;
    let iyz = -1.19066e-07;
    let izz = 9.12834e-06;

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
    let mut body = RigidBody::new(spatial_inertia);

    let mut mesh = meshes
        .foot_pin_left
        .take()
        .expect("Navbot has no foot pin left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(-1.38778e-17, -1.73472e-18, 0.004),
        UnitQuaternion::from_euler_angles(-1.5708, 2.76905e-16, -0.701537),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    let mut mesh = meshes
        .foot_motor_left
        .take()
        .expect("Navbot has no foot motor left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(0.0392041, 0.0310329, 0.003525),
        UnitQuaternion::from_euler_angles(-1.5708, 2.20279e-16, -0.116852),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    let mut mesh = meshes
        .foot_encoder_left
        .take()
        .expect("Navbot has no foot encoder left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(0.039213, 0.0310399, 0.013),
        UnitQuaternion::from_euler_angles(1.5708, 1.87067e-16, -0.90225),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    let mut mesh = meshes
        .foot_plate_left
        .take()
        .expect("Navbot has no foot plate left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(0.0680493, 0.027659, 0.01675),
        UnitQuaternion::from_euler_angles(-1.5708, 1.65883e-16, -0.701537),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    body
}

fn build_navbot_link_left(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.00130815;
    let com = vector![0.00234762, -0.0243892, -0.001];
    let ixx = 4.47598e-07;
    let ixy = 1.03315e-08;
    let ixz = -9.14854e-31;
    let iyy = 1.13491e-08;
    let iyz = 2.41596e-31;
    let izz = 4.58075e-07;

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
    let mut body = RigidBody::new(spatial_inertia);

    let mut mesh = meshes
        .link_plate_left
        .take()
        .expect("Navbot has no link plate left mesh");
    let iso = Isometry3::from_parts(
        Translation3::new(-3.46945e-18, -0.052086, -0.002),
        UnitQuaternion::from_euler_angles(1.5708, 5.69618e-25, 5.25003e-19),
    );
    mesh.update_base_isometry(&iso);
    body.add_visual_mesh(mesh);

    body
}

pub fn build_navbot(mut meshes: NavbotMeshes) -> MechanismState {
    let base_frame = "base";
    let base = build_navbot_base(&mut meshes, base_frame);
    let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

    let leg_left_frame = "leg_left";
    let leg_left = build_navbot_leg_left(&mut meshes, leg_left_frame);
    let leg_left_to_base = Transform3D::new_xyz_rpy(
        leg_left_frame,
        base_frame,
        &vec![-0.0299877, 0.0274141, -0.0126354],
        &vec![-1.5708, -0.3074, -1.5708],
    );

    let foot_left_frame = "foot_left";
    let foot_left = build_navbot_foot_left(&mut meshes, foot_left_frame);
    let foot_left_to_leg_left = Transform3D::new_xyz_rpy(
        foot_left_frame,
        leg_left_frame,
        &vec![-0.052, 0.003, -0.0065],
        &vec![-1.98635e-26, -1.43064e-25, -1.50654e-17],
    );

    let link_left_frame = "link_left";
    let link_left = build_navbot_link_left(&mut meshes, link_left_frame);
    let link_left_to_base = Transform3D::new_xyz_rpy(
        link_left_frame,
        base_frame,
        &vec![-0.0405877, 0.0451087, 0.00357876],
        &vec![1.5708, -0.943886, 1.5708],
    );

    let bodies = vec![base, leg_left, foot_left, link_left];
    let treejoints = vec![
        Joint::RevoluteJoint(RevoluteJoint::new(base_to_world, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(leg_left_to_base, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(foot_left_to_leg_left, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(link_left_to_base, Vector3::z_axis())),
    ];

    // TODO: make this a cylindrical constraint to avoid over-constraining
    let link_left_to_foot_left = RevoluteConstraintJoint::new(
        link_left_frame,
        Isometry3::from_parts(
            Translation3::new(-3.46945e-18, -0.052086, 0.),
            UnitQuaternion::from_euler_angles(-1.97267e-25, 5.69618e-25, 5.25003e-19),
        ),
        foot_left_frame,
        Isometry3::from_parts(
            Translation3::new(-0.00940899, -0.00744789, -0.0042),
            UnitQuaternion::from_euler_angles(4.56749e-16, 1.65883e-16, -0.701537),
        ),
        Vector3::z_axis(),
    );
    let constraints = vec![link_left_to_foot_left];

    MechanismState::new_with_constraint(treejoints, bodies, constraints)
}
