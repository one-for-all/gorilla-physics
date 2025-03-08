use crate::contact::ContactPoint;
use crate::contact::SpringContact;
use crate::joint::floating::FloatingJoint;
use crate::joint::ToJointPositionVec;
use crate::joint::ToJointVelocityVec;
use crate::transform::Matrix4Ext;
use crate::PI;
use crate::WORLD_FRAME;
use crate::{
    inertia::SpatialInertia,
    joint::{prismatic::PrismaticJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    transform::Transform3D,
    types::Float,
};
use na::Rotation3;
use na::UnitVector3;
use na::{dvector, vector, Matrix3, Matrix4, Vector3};

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

    let moment_x = m * l * l / 6.0;
    let moment_y = moment_x;
    let moment_z = moment_x;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., 0., 0.];

    let cube_frame = "cube";
    let world_frame = "world"; // TODO: make world frame a global constant
    let cube_to_world = Transform3D::identity(&cube_frame, &world_frame);
    let cube = RigidBody::new(SpatialInertia {
        frame: cube_frame.to_string(),
        moment,
        cross_part,
        mass: m,
    });

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(cube_to_world))];
    let bodies = vec![cube];
    let mut state = MechanismState::new(treejoints, bodies);

    // Contact points on the bottom face
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![l / 2.0, l / 2.0, -l / 2.0],
    });
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![l / 2.0, -l / 2.0, -l / 2.0],
    });
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![-l / 2.0, l / 2.0, -l / 2.0],
    });
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![-l / 2.0, -l / 2.0, -l / 2.0],
    });

    // Contact Points on the top face
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![l / 2.0, l / 2.0, l / 2.0],
    });
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![l / 2.0, -l / 2.0, l / 2.0],
    });
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![-l / 2.0, l / 2.0, l / 2.0],
    });
    state.add_contact_point(&ContactPoint {
        frame: cube_frame.to_string(),
        location: vector![-l / 2.0, -l / 2.0, l / 2.0],
    });

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
        state.add_contact_point(&ContactPoint {
            frame: body_frame.to_string(),
            location,
        });
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

    state.add_contact_point(&ContactPoint {
        frame: leg_frame.to_string(),
        location: vector![0., 0., 0.],
    });

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
