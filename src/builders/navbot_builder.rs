use na::{vector, Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

use crate::{
    collision::{mesh::Mesh, sphere::Sphere},
    flog,
    inertia::SpatialInertia,
    joint::{
        constraint_revolute::RevoluteConstraintJoint,
        cylindrical_constraint::{Constraint, CylindricalConstraintJoint},
        floating::FloatingJoint,
        revolute::RevoluteJoint,
        Joint,
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
    pub base: Option<Mesh>,
    pub leg_left: Option<Mesh>,
    pub foot_left: Option<Mesh>,
    pub link_left: Option<Mesh>,
    pub wheel_left: Option<Mesh>,
    pub leg_right: Option<Mesh>,
    pub foot_right: Option<Mesh>,
    pub link_right: Option<Mesh>,
    pub wheel_right: Option<Mesh>,
}

impl NavbotMeshes {
    pub fn new() -> Self {
        NavbotMeshes {
            base: None,
            leg_left: None,
            foot_left: None,
            link_left: None,
            wheel_left: None,
            leg_right: None,
            foot_right: None,
            link_right: None,
            wheel_right: None,
        }
    }
}

fn build_navbot_base(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.0414438;
    let com = vector![0.000111698, 0.0256898, -0.00962915];
    let ixx = 2.00397e-05;
    let ixy = 1.88378e-10;
    let ixz = 4.17829e-10;
    let iyy = 2.61065e-05;
    let iyz = -2.16605e-06;
    let izz = 3.3757e-05;

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

    if let Some(mut mesh) = meshes.base.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(0.000111698, 0.0256898, -0.00962915),
            UnitQuaternion::from_euler_angles(0., 0., 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

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

    if let Some(mut mesh) = meshes.leg_left.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(-0.0134676, -0.000849763, -0.0089392),
            UnitQuaternion::from_euler_angles(1.87857, -1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    body
}

fn build_navbot_foot_left(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.0379925;
    let com = vector![0.03509, 0.027779, -0.000900737];
    let ixx = 4.76781e-06;
    let ixy = -3.17164e-06;
    let ixz = -1.40975e-07;
    let iyy = 6.22098e-06;
    let iyz = -1.19188e-07;
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

    if let Some(mut mesh) = meshes.foot_left.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(0.03509, 0.027779, -0.000900737),
            UnitQuaternion::from_euler_angles(1.87857, -1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    body
}

fn build_navbot_link_left(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.00242268;
    let com = vector![0.00126762, -0.02515, -0.00146631];
    let ixx = 1.21113e-06;
    let ixy = 7.99494e-09;
    let ixz = -1.43206e-09;
    let iyy = 2.07936e-08;
    let iyz = -1.00885e-09;
    let izz = 1.22447e-06;

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

    if let Some(mut mesh) = meshes.link_left.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(0.00126762, -0.02515, -0.00146631),
            UnitQuaternion::from_euler_angles(-2.51439, -1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    body
}

fn build_navbot_wheel_left(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.0155748;
    let com = vector![4.83102e-08, -1.61747e-09, -0.00780743];
    let ixx = 1.75465e-06;
    let ixy = -3.92314e-13;
    let ixz = 3.65986e-12;
    let iyy = 1.75464e-06;
    let iyz = -1.18704e-13;
    let izz = 2.81671e-06;

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

    if let Some(mut mesh) = meshes.wheel_left.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(4.83102e-08, -1.61747e-09, -0.00780743),
            UnitQuaternion::from_euler_angles(0.992489, -1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    let mut sphere = Sphere::new(0.02);
    sphere.base_translation = com;
    body.add_sphere_collider(sphere);

    body
}

fn build_navbot_leg_right(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.00775582;
    let com = vector![-0.0134676, 0.000849731, -0.0089392];
    let ixx = 4.45176e-07;
    let ixy = 2.74979e-08;
    let ixz = 2.09678e-08;
    let iyy = 2.67121e-06;
    let iyz = -4.57706e-09;
    let izz = 2.81453e-06;

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

    if let Some(mut mesh) = meshes.leg_right.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(-0.0134676, 0.000849731, -0.0089392),
            UnitQuaternion::from_euler_angles(-1.26301, 1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    body
}

fn build_navbot_foot_right(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.0379925;
    let com = vector![0.00891353, -0.043858, -0.000900737];
    let ixx = 8.49931e-06;
    let ixy = 1.24815e-06;
    let ixz = -3.10263e-08;
    let iyy = 2.48948e-06;
    let iyz = 1.81978e-07;
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

    if let Some(mut mesh) = meshes.foot_right.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(0.00891353, -0.043858, -0.000900737),
            UnitQuaternion::from_euler_angles(-0.562376, 1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    body
}

fn build_navbot_link_right(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.00242268;
    let com = vector![0.00126762, 0.02515, -0.00146631];
    let ixx = 1.21113e-06;
    let ixy = 7.99494e-09;
    let ixz = -1.43206e-09;
    let iyy = 2.07936e-08;
    let iyz = -1.00885e-09;
    let izz = 1.22447e-06;

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

    if let Some(mut mesh) = meshes.link_right.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(0.00126762, 0.02515, -0.00146631),
            UnitQuaternion::from_euler_angles(0.627216, 1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    body
}

fn build_navbot_wheel_right(meshes: &mut NavbotMeshes, frame: &str) -> RigidBody {
    let m = 0.0155748;
    let com = vector![-1.61747e-09, -4.83102e-08, -0.00780743];
    let ixx = 1.75464e-06;
    let ixy = 3.92314e-13;
    let ixz = -1.18704e-13;
    let iyy = 1.75465e-06;
    let iyz = -3.65986e-12;
    let izz = 2.81671e-06;

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

    if let Some(mut mesh) = meshes.wheel_right.take() {
        let iso = Isometry3::from_parts(
            Translation3::new(-1.61747e-09, -4.83102e-08, -0.00780743),
            UnitQuaternion::from_euler_angles(-2.40559, 1.5708, 0.),
        );
        mesh.update_base_isometry(&iso);
        body.add_visual_mesh(mesh);
    }

    let mut sphere = Sphere::new(0.02);
    sphere.base_translation = com;
    body.add_sphere_collider(sphere);

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
        &vec![-1.5708, -0.307769, -1.5708],
    );

    let foot_left_frame = "foot_left";
    let foot_left = build_navbot_foot_left(&mut meshes, foot_left_frame);
    let foot_left_to_leg_left = Transform3D::new_xyz_rpy(
        foot_left_frame,
        leg_left_frame,
        &vec![-0.052, 0.003, -0.0065],
        &vec![4.51632e-25, 1.59286e-24, 3.23144e-17],
    );

    let link_left_frame = "link_left";
    let link_left = build_navbot_link_left(&mut meshes, link_left_frame);
    let link_left_to_base = Transform3D::new_xyz_rpy(
        link_left_frame,
        base_frame,
        &vec![-0.0406877, 0.0451087, 0.00357876],
        &vec![1.5708, -0.943592, 1.5708],
    );

    let wheel_left_frame = "wheel_left";
    let wheel_left = build_navbot_wheel_left(&mut meshes, wheel_left_frame);
    let wheel_left_to_foot_left = Transform3D::new_xyz_rpy(
        wheel_left_frame,
        foot_left_frame,
        &vec![0.0391772, 0.0310668, -0.00035],
        &vec![8.25667e-17, -3.43754e-16, 0.886077],
    );

    let leg_right_frame = "leg_right";
    let leg_right = build_navbot_leg_right(&mut meshes, leg_right_frame);
    let leg_right_to_base = Transform3D::new_xyz_rpy(
        leg_right_frame,
        base_frame,
        &vec![0.0302123, 0.0274141, -0.0126354],
        &vec![1.5708, -0.307784, -1.5708],
    );

    let foot_right_frame = "foot_right";
    let foot_right = build_navbot_foot_right(&mut meshes, foot_right_frame);
    let foot_right_to_leg_right = Transform3D::new_xyz_rpy(
        foot_right_frame,
        leg_right_frame,
        &vec![-0.052, -0.003, -0.0065],
        &vec![-3.24841e-15, -1.09622e-15, 0.700637],
    );

    let link_right_frame = "link_right";
    let link_right = build_navbot_link_right(&mut meshes, link_right_frame);
    let link_right_to_base = Transform3D::new_xyz_rpy(
        link_right_frame,
        base_frame,
        &vec![0.0409123, 0.0451087, 0.00357876],
        &vec![-1.5708, -0.943581, 1.5708],
    );

    let wheel_right_frame = "wheel_right";
    let wheel_right = build_navbot_wheel_right(&mut meshes, wheel_right_frame);
    let wheel_right_to_foot_right = Transform3D::new_xyz_rpy(
        wheel_right_frame,
        foot_right_frame,
        &vec![0.00991772, -0.0490065, -0.00035],
        &vec![-4.01485e-15, 1.79841e-15, -1.84321],
    );

    let bodies = vec![
        base,
        leg_left,
        foot_left,
        link_left,
        wheel_left,
        leg_right,
        foot_right,
        link_right,
        wheel_right,
    ];
    let treejoints = vec![
        Joint::RevoluteJoint(RevoluteJoint::new(base_to_world, Vector3::z_axis())),
        // Joint::FloatingJoint(FloatingJoint::new(base_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(leg_left_to_base, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(foot_left_to_leg_left, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(link_left_to_base, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(
            wheel_left_to_foot_left,
            Vector3::z_axis(),
        )),
        Joint::RevoluteJoint(RevoluteJoint::new(leg_right_to_base, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(
            foot_right_to_leg_right,
            Vector3::z_axis(),
        )),
        Joint::RevoluteJoint(RevoluteJoint::new(link_right_to_base, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(
            wheel_right_to_foot_right,
            Vector3::z_axis(),
        )),
    ];

    let link_left_to_foot_left = RevoluteConstraintJoint::new(
        link_left_frame,
        Isometry3::from_parts(
            Translation3::new(6.93889e-18, -0.052086, 0.),
            UnitQuaternion::from_euler_angles(-1.44955e-25, 4.35151e-25, -4.26445e-18),
        ),
        foot_left_frame,
        Isometry3::from_parts(
            Translation3::new(-0.00940253, -0.00745604, -0.0042),
            UnitQuaternion::from_euler_angles(2.02194e-16, -1.32857e-16, -0.700671),
        ),
        Vector3::z_axis(),
    );
    let link_right_to_foot_right = RevoluteConstraintJoint::new(
        link_right_frame,
        Isometry3::from_parts(
            Translation3::new(3.46945e-18, 0.052086, 0.),
            UnitQuaternion::from_euler_angles(1.77223e-24, -1.82722e-25, -2.02994e-17),
        ),
        foot_right_frame,
        Isometry3::from_parts(
            Translation3::new(-0.00238025, 0.0117616, -0.0042),
            UnitQuaternion::from_euler_angles(-5.82594e-25, 4.12715e-26, 6.18256e-17),
        ),
        Vector3::z_axis(),
    );

    let constraints = vec![
        Constraint::Revolute(link_left_to_foot_left),
        Constraint::Revolute(link_right_to_foot_right),
    ];

    MechanismState::new_with_constraint(treejoints, bodies, constraints)
}
