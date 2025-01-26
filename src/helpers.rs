use na::{dvector, vector, Matrix3, Matrix4, Vector3};

use crate::{
    inertia::SpatialInertia,
    joint::{prismatic::PrismaticJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    transform::Transform3D,
    types::Float,
};

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

    let state = MechanismState {
        treejoints: dvector![Joint::RevoluteJoint(RevoluteJoint {
            init_mat: rod_to_world.mat.clone(),
            transform: rod_to_world,
            axis: axis.clone(),
        })],
        treejointids: dvector![1],
        bodies: dvector![rod],
        q: dvector![0.0],
        v: dvector![0.0],
        halfspaces: dvector![],
    };

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

    let state = MechanismState {
        treejoints: dvector![
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
        ],
        treejointids: dvector![1, 2],
        bodies: dvector![
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
        ],
        q: dvector![0.0, 0.0],
        v: dvector![0.0, 0.0],
        halfspaces: dvector![],
    };

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

    let state = MechanismState {
        treejoints: dvector![Joint::PrismaticJoint(PrismaticJoint {
            init_mat: cart_to_world.mat.clone(),
            transform: cart_to_world,
            axis: *axis
        })],
        treejointids: dvector![1],
        bodies: dvector![RigidBody::new(SpatialInertia {
            frame: cart_frame.to_string(),
            moment: *moment,
            cross_part: *cross_part,
            mass: *mass
        })],
        q: dvector![0.0],
        v: dvector![0.0],
        halfspaces: dvector![],
    };
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
        q: dvector![0.0, 0.0],
        v: dvector![0.0, 0.0],
        halfspaces: dvector![],
    };

    state
}
