use crate::contact::ContactPoint;
use crate::joint::floating::FloatingJoint;
use crate::joint::ToJointPositionVec;
use crate::joint::ToJointVelocityVec;
use crate::{
    inertia::SpatialInertia,
    joint::{prismatic::PrismaticJoint, revolute::RevoluteJoint, Joint, JointPosition},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    transform::Transform3D,
    types::Float,
};
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

    let treejoints = dvector![Joint::RevoluteJoint(RevoluteJoint {
        init_mat: rod_to_world.mat.clone(),
        transform: rod_to_world,
        axis: axis.clone(),
    })];
    let bodies = dvector![rod];
    let halfspaces = dvector![];
    let state = MechanismState::new(treejoints, bodies, halfspaces);

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

    let treejoints = dvector![
        Joint::RevoluteJoint(RevoluteJoint {
            init_mat: rod1_to_world.mat.clone(),
            transform: rod1_to_world,
            axis: axis.clone(),
        }),
        Joint::RevoluteJoint(RevoluteJoint {
            init_mat: rod2_to_rod1.mat.clone(),
            transform: rod2_to_rod1,
            axis: axis.clone(),
        })
    ];
    let bodies = dvector![
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
        })
    ];
    let halfspaces = dvector![];
    let state = MechanismState::new(treejoints, bodies, halfspaces);
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

    let treejoints = dvector![Joint::PrismaticJoint(PrismaticJoint {
        init_mat: cart_to_world.mat.clone(),
        transform: cart_to_world,
        axis: *axis
    })];
    let bodies = dvector![RigidBody::new(SpatialInertia {
        frame: cart_frame.to_string(),
        moment: *moment,
        cross_part: *cross_part,
        mass: *mass
    })];
    let halfspaces = dvector![];
    let state = MechanismState::new(treejoints, bodies, halfspaces);
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

    let state = MechanismState {
        treejoints: dvector![
            Joint::PrismaticJoint(PrismaticJoint {
                init_mat: cart_to_world.mat.clone(),
                transform: cart_to_world,
                axis: axis_cart
            }),
            Joint::RevoluteJoint(RevoluteJoint {
                init_mat: pole_to_cart.mat.clone(),
                transform: pole_to_cart,
                axis: *axis_pole
            })
        ],
        treejointids: dvector![1, 2],
        bodies: dvector![
            RigidBody::new(SpatialInertia {
                frame: cart_frame.to_string(),
                moment: *moment_cart,
                cross_part: *cross_part_cart,
                mass: *mass_cart
            }),
            RigidBody::new(SpatialInertia {
                frame: pole_frame.to_string(),
                moment: *moment_pole,
                cross_part: *cross_part_pole,
                mass: *mass_pole
            })
        ],
        q: vec![0.0, 0.0].to_joint_pos_vec(),
        v: vec![0.0, 0.0].to_joint_vel_vec(),
        halfspaces: dvector![],
    };

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

    let treejoints = dvector![Joint::FloatingJoint(FloatingJoint::new(cube_to_world))];
    let bodies = dvector![cube];
    let halfspaces = dvector![];
    let mut state = MechanismState::new(treejoints, bodies, halfspaces);

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
