use na::{vector, Matrix3, Matrix4, UnitQuaternion};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    contact::ContactPoint,
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, revolute::RevoluteJoint, Joint, JointPosition},
    mechanism::MechanismState,
    pose::Pose,
    rigid_body::RigidBody,
    transform::{Matrix4Ext, Transform3D},
    types::Float,
    PI, WORLD_FRAME,
};

use super::InterfaceMechanismState;

#[wasm_bindgen]
pub fn createQuadruped() -> InterfaceMechanismState {
    // Create body
    let m_body = 10.0;
    let w_body = 1.5;
    let d_body = 0.5;
    let h_body = 0.5;
    let moment_x = (d_body * d_body + h_body + h_body) * m_body / 12.0;
    let moment_y = (w_body * w_body + h_body + h_body) * m_body / 12.0;
    let moment_z = (w_body * w_body + d_body * d_body) * m_body / 12.0;
    let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_body = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let body_to_world = Transform3D::identity(&body_frame, WORLD_FRAME);
    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        moment: moment_body,
        cross_part: cross_part_body,
        mass: m_body,
    });

    let l_leg = 1.0;
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
    let k_foot = 5000.0; // racquet ball spring constant
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

    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0., 0., 0.0 * l_leg],
        }),
    );

    let mut i = 1;
    while i < 9 {
        state.set_joint_q(i + 1, JointPosition::Float(-PI / 2.0));
        state.set_joint_q(i + 2, JointPosition::Float(PI));

        i += 2;
    }

    InterfaceMechanismState { inner: state }
}
