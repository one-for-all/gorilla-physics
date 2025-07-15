use crate::{
    contact::ContactPoint,
    helpers::{build_2d_hopper, build_hopper},
    inertia::SpatialInertia,
    joint::{
        floating::FloatingJoint,
        prismatic::{JointSpring, PrismaticJoint},
        revolute::RevoluteJoint,
        Joint, JointPosition, JointVelocity,
    },
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::pose::Pose,
    spatial::spatial_vector::SpatialVector,
    spatial::transform::Transform3D,
    types::Float,
    WORLD_FRAME,
};

use super::InterfaceMechanismState;
use na::{zero, Isometry3, UnitQuaternion, Vector3};
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
        iso: Isometry3::translation(0., 0., -body_leg_length),
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
        iso: Isometry3::translation(0., 0., -leg_foot_length),
    };
    let foot = RigidBody::new(SpatialInertia {
        frame: foot_frame.to_string(),
        moment: moment_foot,
        cross_part: cross_part_foot,
        mass: m_foot,
    });

    // Create the hopper
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint {
            init_iso: body_to_world.iso,
            transform: body_to_world,
        }),
        Joint::PrismaticJoint(PrismaticJoint::new(leg_to_body, axis_leg)),
        Joint::PrismaticJoint(PrismaticJoint::new(foot_to_leg, axis_foot)),
    ];
    let bodies = vec![body, leg, foot];
    let mut state = MechanismState::new(treejoints, bodies);

    state.add_contact_point(ContactPoint::new(body_frame, vector![0., 0., 0.]));

    state.add_contact_point(ContactPoint::new(leg_frame, vector![0., 0., 0.]));

    state.add_contact_point(ContactPoint::new(foot_frame, vector![0., 0., 0.]));

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn create2DHopper(
    w_body: Float,
    h_body: Float,
    r_hip: Float,
    body_hip_length: Float,
    r_piston: Float,
    hip_piston_length: Float,
    l_leg: Float,
    piston_leg_length: Float,
) -> InterfaceMechanismState {
    let m_body = 10.0;
    let m_hip = 0.1;
    let m_piston = 0.1;
    let m_leg = 1.0;

    let mut state = build_2d_hopper(
        m_body,
        w_body,
        h_body,
        m_hip,
        r_hip,
        body_hip_length,
        m_piston,
        r_piston,
        hip_piston_length,
        m_leg,
        l_leg,
        piston_leg_length,
    );

    let theta2 = -0.05;
    let theta1 = 0.0;
    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), theta2),
            translation: zero(),
        }),
        JointPosition::Float(theta1 - theta2),
        JointPosition::Float(0.0),
        JointPosition::Float(0.0),
    ];
    let v_init = vec![
        JointVelocity::Spatial(SpatialVector {
            angular: zero(),
            linear: zero(),
        }),
        JointVelocity::Float(0.0),
        JointVelocity::Float(0.0),
        JointVelocity::Float(0.0),
    ];
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn createActuatedAnkleHopper() -> InterfaceMechanismState {
    let m_foot = 0.5;
    let r_foot = 1.0;
    let m_ankle = 0.5;
    let r_ankle = 1.0;
    let m_body = 10.0;
    let r_body = 1.0;
    let l_ankle_to_body = 1.0;

    let foot_frame = "foot";
    let foot = RigidBody::new_sphere(m_foot, r_foot, &foot_frame);
    let foot_to_world = Transform3D::identity(&foot_frame, WORLD_FRAME);

    let ankle_frame = "ankle";
    let ankle = RigidBody::new_sphere(m_ankle, r_ankle, &ankle_frame);
    let ankle_to_foot = Transform3D::identity(&ankle_frame, &foot_frame);
    let ankle_axis = -Vector3::y_axis();

    let body_frame = "body";
    let body = RigidBody::new_sphere(m_body, r_body, &body_frame);
    let body_to_ankle = Transform3D::move_z(&body_frame, &ankle_frame, l_ankle_to_body);

    let leg_axis = vector![0., 0., 1.0];
    let leg_spring = JointSpring { k: 1e3, l: 0.0 };

    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(foot_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(ankle_to_foot, ankle_axis)),
        Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
            body_to_ankle,
            leg_axis,
            leg_spring,
        )),
    ];
    let bodies = vec![foot, ankle, body];
    let mut state = MechanismState::new(treejoints, bodies);

    state.add_contact_point(ContactPoint::new(foot_frame, zero()));

    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0., 0., 0.5],
        }),
    );
    state.set_joint_v(
        1,
        JointVelocity::Spatial(SpatialVector {
            angular: zero(),
            linear: vector![0.1, 0.0, 0.0],
        }),
    );

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn createActuatedHipHopper() -> InterfaceMechanismState {
    let m_foot = 1.0;
    let r_foot = 1.0;
    let m_hip = 0.5;
    let r_hip = 1.0;
    let m_body = 9.5;
    let r_body = 1.0;
    let l_foot_to_hip = 1.0;

    let mut state = build_hopper(m_foot, r_foot, m_hip, r_hip, m_body, r_body, l_foot_to_hip);

    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0., 0.5],
        }),
    );
    state.set_joint_q(3, JointPosition::Float(0.0));
    state.set_joint_v(
        1,
        JointVelocity::Spatial(SpatialVector {
            angular: zero(),
            linear: vector![0.0, 0.0, 0.0],
        }),
    );

    InterfaceMechanismState { inner: state }
}
