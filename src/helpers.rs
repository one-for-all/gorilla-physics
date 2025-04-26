use crate::collision::cuboid::Cuboid;
use crate::contact::ContactPoint;
use crate::contact::SpringContact;
use crate::joint::floating::FloatingJoint;
use crate::joint::prismatic::JointSpring;
use crate::spatial::transform::Matrix4Ext;
use crate::PI;
use crate::WORLD_FRAME;
use crate::{
    inertia::SpatialInertia,
    joint::{prismatic::PrismaticJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::transform::Transform3D,
    types::Float,
};
use na::zero;
use na::Rotation3;
use na::UnitVector3;
use na::{vector, Matrix3, Matrix4, Vector3};

/// Build a mechanism state of a pendulum
pub fn build_pendulum(
    mass: &Float,
    moment: &Matrix3<Float>,
    cross_part: &Vector3<Float>,
    rod_to_world: &Matrix4<Float>,
    axis: &Vector3<Float>,
) -> MechanismState {
    let rod_frame = "rod";
    let world_frame = "world";
    let rod_to_world = Transform3D::new(rod_frame, world_frame, &rod_to_world);

    let rod = RigidBody::new(SpatialInertia {
        frame: rod_frame.to_string(),
        moment: moment.clone(),
        cross_part: cross_part.clone(),
        mass: mass.clone(),
    });

    let treejoints = vec![Joint::RevoluteJoint(RevoluteJoint {
        init_mat: rod_to_world.mat.clone(),
        transform: rod_to_world,
        axis: axis.clone(),
    })];
    let bodies = vec![rod];
    let state = MechanismState::new(treejoints, bodies);

    state
}

/// Build a mechanism state of a double pendulum
pub fn build_double_pendulum(
    mass: &Float,
    moment: &Matrix3<Float>,
    cross_part: &Vector3<Float>,
    rod1_to_world: &Matrix4<Float>,
    rod2_to_rod1: &Matrix4<Float>,
    axis: &Vector3<Float>,
) -> MechanismState {
    let rod1_frame = "rod1";
    let rod2_frame = "rod2";
    let world_frame = "world";

    let rod1_to_world = Transform3D::new(rod1_frame, world_frame, &rod1_to_world);
    let rod2_to_rod1 = Transform3D::new(rod2_frame, rod1_frame, &rod2_to_rod1);

    let treejoints = vec![
        Joint::RevoluteJoint(RevoluteJoint {
            init_mat: rod1_to_world.mat.clone(),
            transform: rod1_to_world,
            axis: axis.clone(),
        }),
        Joint::RevoluteJoint(RevoluteJoint {
            init_mat: rod2_to_rod1.mat.clone(),
            transform: rod2_to_rod1,
            axis: axis.clone(),
        }),
    ];
    let bodies = vec![
        RigidBody::new(SpatialInertia {
            frame: rod1_frame.to_string(),
            moment: moment.clone(),
            cross_part: cross_part.clone(),
            mass: mass.clone(),
        }),
        RigidBody::new(SpatialInertia {
            frame: rod2_frame.to_string(),
            moment: moment.clone(),
            cross_part: cross_part.clone(),
            mass: mass.clone(),
        }),
    ];
    let state = MechanismState::new(treejoints, bodies);
    state
}

/// Build the mechanism state of a cart system
pub fn build_cart(
    mass: &Float,
    moment: &Matrix3<Float>,
    cross_part: &Vector3<Float>,
    axis: &Vector3<Float>,
) -> MechanismState {
    let cart_frame = "cart";
    let world_frame = "world";

    let cart_to_world = Transform3D::new(cart_frame, world_frame, &Matrix4::identity());

    let treejoints = vec![Joint::PrismaticJoint(PrismaticJoint::new(
        cart_to_world,
        *axis,
    ))];
    let bodies = vec![RigidBody::new(SpatialInertia {
        frame: cart_frame.to_string(),
        moment: *moment,
        cross_part: *cross_part,
        mass: *mass,
    })];
    let state = MechanismState::new(treejoints, bodies);
    state
}

/// Build the mechanism state of a cart pole system
pub fn build_cart_pole(
    mass_cart: &Float,
    mass_pole: &Float,
    moment_cart: &Matrix3<Float>,
    moment_pole: &Matrix3<Float>,
    cross_part_cart: &Vector3<Float>,
    cross_part_pole: &Vector3<Float>,
    axis_pole: &Vector3<Float>,
) -> MechanismState {
    let world_frame = "world";
    let cart_frame = "cart";
    let pole_frame = "pole";

    let cart_to_world = Transform3D::new(cart_frame, world_frame, &Matrix4::identity());
    let axis_cart = vector![1.0, 0.0, 0.0];

    let pole_to_cart = Transform3D::new(pole_frame, cart_frame, &Matrix4::identity());

    let treejoints = vec![
        Joint::PrismaticJoint(PrismaticJoint::new(cart_to_world, axis_cart)),
        Joint::RevoluteJoint(RevoluteJoint {
            init_mat: pole_to_cart.mat.clone(),
            transform: pole_to_cart,
            axis: *axis_pole,
        }),
    ];
    let bodies = vec![
        RigidBody::new(SpatialInertia {
            frame: cart_frame.to_string(),
            moment: *moment_cart,
            cross_part: *cross_part_cart,
            mass: *mass_cart,
        }),
        RigidBody::new(SpatialInertia {
            frame: pole_frame.to_string(),
            moment: *moment_pole,
            cross_part: *cross_part_pole,
            mass: *mass_pole,
        }),
    ];
    let state = MechanismState::new(treejoints, bodies);

    state
}

pub fn build_cube(mass: Float, length: Float) -> MechanismState {
    let m = mass;
    let l = length;

    let cube_frame = "cube";
    let cube_to_world = Transform3D::identity(&cube_frame, WORLD_FRAME);
    let cube = RigidBody::new_cube(m, l, cube_frame);

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(cube_to_world))];
    let bodies = vec![cube];
    let mut state = MechanismState::new(treejoints, bodies);

    add_cube_contacts(&mut state, &cube_frame, l);

    state
}

pub fn build_rimless_wheel(
    m_body: Float,
    r_body: Float,
    l: Float,
    n_foot: usize,
) -> MechanismState {
    let moment_x = 2.0 / 5.0 * m_body * r_body * r_body;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let world_frame = "world";
    let body_to_world = Transform3D::identity(&body_frame, &world_frame);

    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        mass: m_body,
        moment,
        cross_part,
    });

    let alpha = 2.0 * PI / n_foot as Float / 2.0;

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(body_to_world))];
    let bodies = vec![body];

    let mut state = MechanismState::new(treejoints, bodies);
    for i in 0..n_foot {
        let rotation = Rotation3::from_axis_angle(&Vector3::y_axis(), i as Float * 2.0 * alpha);
        let location = rotation * Vector3::new(0., 0., -l);
        state.add_contact_point(&ContactPoint::new(body_frame, location));
    }

    state
}

pub fn build_2d_hopper(
    m_body: Float,
    w_body: Float,
    h_body: Float,
    m_hip: Float,
    r_hip: Float,
    body_hip_length: Float,
    m_piston: Float,
    r_piston: Float,
    hip_piston_length: Float,
    m_leg: Float,
    l_leg: Float,
    piston_leg_length: Float,
) -> MechanismState {
    // Create hopper body
    let moment_x = (w_body * w_body + h_body * h_body) * m_body / 12.0;
    let moment_y = (w_body * w_body + h_body * h_body) * m_body / 12.0;
    let moment_z = (w_body * w_body + w_body * w_body) * m_body / 12.0;
    let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_body = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let world_frame = "world";
    let body_to_world = Transform3D::identity(&body_frame, &world_frame);
    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        moment: moment_body,
        cross_part: cross_part_body,
        mass: m_body,
    });

    // Create hopper hip
    let moment_x = 2.0 / 5.0 * m_hip * r_hip * r_hip;
    let moment_hip = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_hip = vector![0.0, 0.0, 0.0];
    let axis_hip = vector![0.0, 1.0, 0.0];

    let hip_frame = "hip";
    let hip_to_body = Transform3D {
        from: hip_frame.to_string(),
        to: body_frame.to_string(),
        mat: Matrix4::<Float>::move_z(-body_hip_length),
    };
    let hip = RigidBody::new(SpatialInertia {
        frame: hip_frame.to_string(),
        moment: moment_hip,
        cross_part: cross_part_hip,
        mass: m_hip,
    });

    // Create hopper piston
    let moment_x = 2.0 / 5.0 * m_piston * r_piston * r_piston;
    let moment_piston = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_piston = vector![0.0, 0.0, 0.0];
    let axis_piston = vector![0.0, 0.0, -1.0];

    let piston_frame = "piston";
    let piston_to_hip = Transform3D {
        from: piston_frame.to_string(),
        to: hip_frame.to_string(),
        mat: Matrix4::<Float>::move_z(-hip_piston_length),
    };
    let piston = RigidBody::new(SpatialInertia {
        frame: piston_frame.to_string(),
        moment: moment_piston,
        cross_part: cross_part_piston,
        mass: m_piston,
    });

    // Create hopper leg
    let moment_x = 1.0 / 3.0 * m_leg * l_leg * l_leg;
    let moment_y = 1.0 / 3.0 * m_leg * l_leg * l_leg;
    let moment_z = 0.0;
    let moment_leg = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_leg = vector![0.0, 0.0, m_leg * l_leg / 2.0];
    let axis_leg = vector![0.0, 0.0, -1.0];

    let leg_frame = "leg";
    let leg_to_piston = Transform3D {
        from: leg_frame.to_string(),
        to: piston_frame.to_string(),
        mat: Matrix4::<Float>::move_z(-piston_leg_length),
    };
    let leg = RigidBody::new(SpatialInertia {
        frame: leg_frame.to_string(),
        moment: moment_leg,
        cross_part: cross_part_leg,
        mass: m_leg,
    });

    // Create the hopper
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint {
            init_mat: body_to_world.mat.clone(),
            transform: body_to_world,
        }),
        Joint::RevoluteJoint(RevoluteJoint {
            init_mat: hip_to_body.mat.clone(),
            transform: hip_to_body,
            axis: axis_hip,
        }),
        Joint::PrismaticJoint(PrismaticJoint::new(piston_to_hip, axis_piston)),
        Joint::PrismaticJoint(PrismaticJoint::new(leg_to_piston, axis_leg)),
    ];

    let bodies = vec![body, hip, piston, leg];
    let mut state = MechanismState::new(treejoints, bodies);

    state.add_contact_point(&ContactPoint::new(leg_frame, vector![0., 0., 0.]));

    state
}

pub fn build_SLIP(
    m: Float,
    r: Float,
    l_rest: Float,
    angle: Float,
    k_spring: Float,
) -> MechanismState {
    let moment_x = 2.0 / 5.0 * m * r * r;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let body_to_world = Transform3D::identity(&body_frame, WORLD_FRAME);

    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        moment,
        cross_part,
        mass: m,
    });

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint {
        init_mat: body_to_world.mat.clone(),
        transform: body_to_world,
    })];
    let bodies = vec![body];
    let mut state = MechanismState::new(treejoints, bodies);

    let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);
    state.add_spring_contact(&SpringContact::new(body_frame, l_rest, direction, k_spring));

    state
}

/// Build a hip-actuated hopper
///    base
///  ( hip )
///     |
///     |
///    foot
pub fn build_hopper(
    m_foot: Float,
    r_foot: Float,
    m_hip: Float,
    r_hip: Float,
    m_body: Float,
    r_body: Float,
    l_foot_to_hip: Float,
) -> MechanismState {
    let foot_frame = "foot";
    let foot = RigidBody::new_sphere(m_foot, r_foot, &foot_frame);
    let foot_to_world = Transform3D::identity(&foot_frame, WORLD_FRAME);

    let hip_frame = "hip";
    let hip = RigidBody::new_sphere(m_hip, r_hip, &hip_frame);
    let hip_to_foot = Transform3D::move_z(&hip_frame, &foot_frame, l_foot_to_hip);

    let body_frame = "body";
    let body = RigidBody::new_sphere(m_body, r_body, &body_frame);
    let body_to_hip = Transform3D::identity(&body_frame, &hip_frame);
    let hip_axis = vector![0., 1., 0.];

    let leg_axis = vector![0., 0., 1.0];
    let leg_spring = JointSpring { k: 1e3, l: 0.0 };

    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(foot_to_world)),
        Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
            hip_to_foot,
            leg_axis,
            leg_spring,
        )),
        Joint::RevoluteJoint(RevoluteJoint::new(body_to_hip, hip_axis)),
    ];
    let bodies = vec![foot, hip, body];
    let mut state = MechanismState::new(treejoints, bodies);

    // TODO: revisit hopper control tests with lower k
    state.add_contact_point(&ContactPoint::new_with_k(foot_frame, zero(), 75e3));

    state
}

pub fn add_cube_contacts(state: &mut MechanismState, frame: &str, l: Float) {
    // Contact points on the bottom face
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![l / 2.0, l / 2.0, -l / 2.0],
    ));
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![l / 2.0, -l / 2.0, -l / 2.0],
    ));
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![-l / 2.0, l / 2.0, -l / 2.0],
    ));
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![-l / 2.0, -l / 2.0, -l / 2.0],
    ));

    // Contact Points on the top face
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![l / 2.0, l / 2.0, l / 2.0],
    ));
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![l / 2.0, -l / 2.0, l / 2.0],
    ));
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![-l / 2.0, l / 2.0, l / 2.0],
    ));
    state.add_contact_point(&ContactPoint::new(
        frame,
        vector![-l / 2.0, -l / 2.0, l / 2.0],
    ));
}

pub fn build_quadruped() -> MechanismState {
    // Create body
    let m_body = 5.0;
    let w_body = 1.5;
    let d_body = 0.5;
    let h_body = 0.5;
    let moment_x = (d_body * d_body + h_body + h_body) * m_body / 12.0;
    let moment_y = (w_body * w_body + h_body + h_body) * m_body / 12.0;
    let moment_z = (w_body * w_body + d_body * d_body) * m_body / 12.0;
    let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_body = vector![0.0, 0.0, 0.0];

    let l_leg = 1.0;

    let body_frame = "body";
    let body_to_world = Transform3D::identity(&body_frame, WORLD_FRAME);
    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        moment: moment_body,
        cross_part: cross_part_body,
        mass: m_body,
    });

    let m_hip = 0.5;
    let l_hip = 0.2;
    let hip_axis = vector![0., -1., 0.];
    let m_knee = 0.5;
    let l_knee = 0.2;
    let knee_axis = vector![0., -1., 0.];

    // Create front right leg
    let fr_hip_frame = "fr_hip";
    let fr_hip = RigidBody::new_cube(m_hip, l_hip, &fr_hip_frame);
    let fr_hip_to_base = Transform3D::new(
        &fr_hip_frame,
        &body_frame,
        &(Matrix4::<Float>::move_x(w_body / 2.0) * Matrix4::<Float>::move_y(-d_body / 2.0)),
    );

    let fr_knee_frame = "fr_knee";
    let fr_knee = RigidBody::new_cube(m_knee, l_knee, &fr_knee_frame);
    let fr_knee_to_hip = Transform3D::move_z(&fr_knee_frame, &fr_hip_frame, -l_leg / 2.0);

    // Create front left leg
    let fl_hip_frame = "fl_hip";
    let fl_hip = RigidBody::new_cube(m_hip, l_hip, &fl_hip_frame);
    let fl_hip_to_base = Transform3D::new(
        &fl_hip_frame,
        &body_frame,
        &(Matrix4::<Float>::move_x(w_body / 2.0) * Matrix4::<Float>::move_y(d_body / 2.0)),
    );

    let fl_knee_frame = "fl_knee";
    let fl_knee = RigidBody::new_cube(m_knee, l_knee, &fl_knee_frame);
    let fl_knee_to_hip = Transform3D::move_z(&fl_knee_frame, &fl_hip_frame, -l_leg / 2.0);

    // Create back right leg
    let br_hip_frame = "br_hip";
    let br_hip = RigidBody::new_cube(m_hip, l_hip, &br_hip_frame);
    let br_hip_to_base = Transform3D::new(
        &br_hip_frame,
        &body_frame,
        &(Matrix4::<Float>::move_x(-w_body / 2.0) * Matrix4::<Float>::move_y(-d_body / 2.0)),
    );

    let br_knee_frame = "br_knee";
    let br_knee = RigidBody::new_cube(m_knee, l_knee, &br_knee_frame);
    let br_knee_to_hip = Transform3D::move_z(&br_knee_frame, &br_hip_frame, -l_leg / 2.0);

    // Create back left leg
    let bl_hip_frame = "bl_hip";
    let bl_hip = RigidBody::new_cube(m_hip, l_hip, &bl_hip_frame);
    let bl_hip_to_base = Transform3D::new(
        &bl_hip_frame,
        &body_frame,
        &(Matrix4::<Float>::move_x(-w_body / 2.0) * Matrix4::<Float>::move_y(d_body / 2.0)),
    );

    let bl_knee_frame = "bl_knee";
    let bl_knee = RigidBody::new_cube(m_knee, l_knee, &bl_knee_frame);
    let bl_knee_to_hip = Transform3D::move_z(&bl_knee_frame, &bl_hip_frame, -l_leg / 2.0);

    let bodies = vec![
        body, fr_hip, fr_knee, fl_hip, fl_knee, br_hip, br_knee, bl_hip, bl_knee,
    ];
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(fr_hip_to_base, hip_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(fr_knee_to_hip, knee_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(fl_hip_to_base, hip_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(fl_knee_to_hip, knee_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(br_hip_to_base, hip_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(br_knee_to_hip, knee_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(bl_hip_to_base, hip_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(bl_knee_to_hip, knee_axis)),
    ];

    let mut state = MechanismState::new(treejoints, bodies);
    // hip contacts
    state.add_contact_point(&ContactPoint::new(fr_hip_frame, vector![0., 0., 0.]));
    state.add_contact_point(&ContactPoint::new(fl_hip_frame, vector![0., 0., 0.]));
    state.add_contact_point(&ContactPoint::new(br_hip_frame, vector![0., 0., 0.]));
    state.add_contact_point(&ContactPoint::new(bl_hip_frame, vector![0., 0., 0.]));

    // knee contacts
    state.add_contact_point(&ContactPoint::new(fr_knee_frame, vector![0., 0., 0.]));
    state.add_contact_point(&ContactPoint::new(fl_knee_frame, vector![0., 0., 0.]));
    state.add_contact_point(&ContactPoint::new(br_knee_frame, vector![0., 0., 0.]));
    state.add_contact_point(&ContactPoint::new(bl_knee_frame, vector![0., 0., 0.]));

    // foot contacts
    let k_foot = 10e3; // racquet ball spring constant
    state.add_contact_point(&ContactPoint::new_with_k(
        fr_knee_frame,
        vector![0., 0., -l_leg / 2.0],
        k_foot,
    ));
    state.add_contact_point(&ContactPoint::new_with_k(
        fl_knee_frame,
        vector![0., 0., -l_leg / 2.0],
        k_foot,
    ));
    state.add_contact_point(&ContactPoint::new_with_k(
        br_knee_frame,
        vector![0., 0., -l_leg / 2.0],
        k_foot,
    ));
    state.add_contact_point(&ContactPoint::new_with_k(
        bl_knee_frame,
        vector![0., 0., -l_leg / 2.0],
        k_foot,
    ));

    state
}

pub fn build_pusher() -> MechanismState {
    // Build planar pusher robot
    let base_frame = "base";
    let w_base = 0.5;
    let h_base = 0.5;
    let base_link = RigidBody::new_cuboid(1.0, w_base, 0.5, h_base, &base_frame);
    let base_to_world = Transform3D::move_xyz(&base_frame, WORLD_FRAME, 0.0, 0.0, 2.0);

    let pusher_frame = "pusher";
    let m_pusher = 1.0;
    let w_pusher = 0.2;
    let d_pusher = 0.2;
    let h_pusher = 1.75;
    let mut pusher_link =
        RigidBody::new_cuboid(m_pusher, w_pusher, d_pusher, h_pusher, &pusher_frame);
    pusher_link.add_collider(Cuboid::new_at_center(w_pusher, d_pusher, h_pusher));
    let pusher_to_base = Transform3D::move_xyz(
        &pusher_frame,
        &base_frame,
        w_base / 2.0 + w_pusher / 2.0,
        0.0,
        -h_base / 2.0 - h_pusher / 2.0,
    );

    let cube_frame = "cube";
    let l_cube = 0.5;
    let mut cube = RigidBody::new_cube(0.1, l_cube, &cube_frame);
    cube.add_collider(Cuboid::new_cube_at_center(l_cube));
    let cube_to_world = Transform3D::move_xyz(
        &cube_frame,
        WORLD_FRAME,
        w_base + w_pusher + 0.5,
        0.,
        l_cube / 2.0,
    );

    let bodies = vec![base_link, pusher_link, cube];
    let treejoints = vec![
        Joint::RevoluteJoint(RevoluteJoint::new(base_to_world, vector![0.0, 0.0, 1.0])),
        Joint::PrismaticJoint(PrismaticJoint::new(pusher_to_base, vector![1.0, 0.0, 0.0])),
        Joint::FloatingJoint(FloatingJoint::new(cube_to_world)),
    ];
    let mut state = MechanismState::new(treejoints, bodies);

    add_cube_contacts(&mut state, &cube_frame, l_cube);

    state
}

pub fn build_gripper() -> MechanismState {
    // Build gripper robot
    let lift_frame = "lift";
    let m_lift = 1.0;
    let w_lift = 0.2;
    let d_lift = 0.2;
    let h_lift = 1.0;
    let lift = RigidBody::new_cuboid(m_lift, w_lift, d_lift, h_lift, &lift_frame);
    let lift_to_world = Transform3D::move_xyz(&lift_frame, WORLD_FRAME, 0.0, 0.0, 2.0);

    let m_gripper = 0.5;
    let w_gripper = 0.1;
    let d_gripper = 0.4;
    let h_gripper = 0.4;
    let gripper_left_frame = "gripper_left";
    let mut gripper_left = RigidBody::new_cuboid(
        m_gripper,
        w_gripper,
        d_gripper,
        h_gripper,
        &gripper_left_frame,
    );
    gripper_left.add_collider(Cuboid::new_at_center(w_gripper, d_gripper, h_gripper));
    let gripper_left_to_lift = Transform3D::move_xyz(
        &gripper_left_frame,
        &lift_frame,
        -w_lift / 2.0 - w_gripper / 2.0 - 0.3,
        0.0,
        -h_lift / 2.0 - h_gripper / 2.0,
    );

    let gripper_right_frame = "gripper_right";
    let mut gripper_right = RigidBody::new_cuboid(
        m_gripper,
        w_gripper,
        d_gripper,
        h_gripper,
        &gripper_right_frame,
    );
    gripper_right.add_collider(Cuboid::new_at_center(w_gripper, d_gripper, h_gripper));
    let gripper_right_to_lift = Transform3D::move_xyz(
        &gripper_right_frame,
        &lift_frame,
        w_lift / 2.0 + w_gripper / 2.0 + 0.3,
        0.0,
        -h_lift / 2.0 - h_gripper / 2.0,
    );

    let cube_frame = "cube";
    let l_cube = 0.5;
    let mut cube = RigidBody::new_cube(0.1, l_cube, &cube_frame);
    cube.add_collider(Cuboid::new_cube_at_center(l_cube));
    let cube_to_world = Transform3D::move_xyz(&cube_frame, WORLD_FRAME, 0.0, 0., l_cube / 2.0);

    let bodies = vec![lift, gripper_left, gripper_right, cube];
    let treejoints = vec![
        Joint::PrismaticJoint(PrismaticJoint::new(lift_to_world, vector![0.0, 0.0, -1.0])),
        Joint::PrismaticJoint(PrismaticJoint::new(
            gripper_left_to_lift,
            vector![-1.0, 0.0, 0.0],
        )),
        Joint::PrismaticJoint(PrismaticJoint::new(
            gripper_right_to_lift,
            vector![1.0, 0.0, 0.0],
        )),
        Joint::FloatingJoint(FloatingJoint::new(cube_to_world)),
    ];
    let mut state = MechanismState::new(treejoints, bodies);

    add_cube_contacts(&mut state, &cube_frame, l_cube);

    state
}
