use crate::{
    contact::ContactPoint,
    inertia::SpatialInertia,
    joint::{
        floating::FloatingJoint, prismatic::PrismaticJoint, revolute::RevoluteJoint, Joint,
        JointPosition, JointVelocity,
    },
    mechanism::MechanismState,
    pose::Pose,
    rigid_body::RigidBody,
    spatial_vector::SpatialVector,
    transform::Transform3D,
    types::Float,
};

use super::InterfaceMechanismState;
use na::{dvector, zero, UnitQuaternion, Vector3};
use nalgebra::{vector, Matrix3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn create1DHopper(
    w_body: Float,
    h_body: Float,
    r_leg: Float,
    r_foot: Float,
    body_leg_length: Float,
    leg_foot_length: Float,
) -> InterfaceMechanismState {
    // Create hopper body
    let m_body = 10.0;

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

    // Create hopper leg
    let m_leg = 1.0;

    let moment_x = 2.0 / 5.0 * m_leg * r_leg * r_leg;
    let moment_y = 2.0 / 5.0 * m_leg * r_leg * r_leg;
    let moment_z = 2.0 / 5.0 * m_leg * r_leg * r_leg;
    let moment_leg = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_leg = vector![0.0, 0.0, 0.0];
    let axis_leg = vector![0.0, 0.0, -1.0];

    let leg_frame = "leg";
    let leg_to_body = Transform3D {
        from: leg_frame.to_string(),
        to: body_frame.to_string(),
        mat: Transform3D::move_z(-body_leg_length),
    };
    let leg = RigidBody::new(SpatialInertia {
        frame: leg_frame.to_string(),
        moment: moment_leg,
        cross_part: cross_part_leg,
        mass: m_leg,
    });

    // Create hopper foot
    let m_foot = 1.0;

    let moment_x = 2.0 / 5.0 * m_foot * r_foot * r_foot;
    let moment_y = 2.0 / 5.0 * m_foot * r_foot * r_foot;
    let moment_z = 2.0 / 5.0 * m_foot * r_foot * r_foot;
    let moment_foot = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_foot = vector![0.0, 0.0, 0.0];
    let axis_foot = vector![0.0, 0.0, -1.0];

    let foot_frame = "foot";
    let foot_to_leg = Transform3D {
        from: foot_frame.to_string(),
        to: leg_frame.to_string(),
        mat: Transform3D::move_z(-leg_foot_length),
    };
    let foot = RigidBody::new(SpatialInertia {
        frame: foot_frame.to_string(),
        moment: moment_foot,
        cross_part: cross_part_foot,
        mass: m_foot,
    });

    // Create the hopper
    let treejoints = dvector![
        Joint::FloatingJoint(FloatingJoint {
            init_mat: body_to_world.mat.clone(),
            transform: body_to_world,
        }),
        Joint::PrismaticJoint(PrismaticJoint {
            init_mat: leg_to_body.mat.clone(),
            transform: leg_to_body,
            axis: axis_leg
        }),
        Joint::PrismaticJoint(PrismaticJoint {
            init_mat: foot_to_leg.mat.clone(),
            transform: foot_to_leg,
            axis: axis_foot
        })
    ];
    let bodies = dvector![body, leg, foot];
    let halfspaces = dvector![];
    let mut state = MechanismState::new(treejoints, bodies, halfspaces);

    state.add_contact_point(&ContactPoint {
        frame: body_frame.to_string(),
        location: vector![0., 0., 0.],
    });

    state.add_contact_point(&ContactPoint {
        frame: leg_frame.to_string(),
        location: vector![0., 0., 0.],
    });

    state.add_contact_point(&ContactPoint {
        frame: foot_frame.to_string(),
        location: vector![0., 0., 0.],
    });

    InterfaceMechanismState { inner: state }
}
}
