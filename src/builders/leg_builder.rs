use na::{vector, UnitVector3, Vector3};

use crate::{
    joint::Joint, mechanism::MechanismState, rigid_body::RigidBody,
    spatial::transform::Transform3D, types::Float, WORLD_FRAME,
};

pub fn build_leg() -> MechanismState {
    let l1 = 0.05;
    let l2 = 0.2;

    let m_thigh = 0.1;
    let w_thigh = l1;
    let d_thigh = l1;
    let h_thigh = l2;
    // let thigh_axis: UnitVector3<Float> = Vector3::x_axis();
    let thigh_com = vector![0., 0., -h_thigh / 2.];
    let thigh_left_frame = "thigh_left";
    let mut thigh_left = RigidBody::new_cuboid_at(
        &thigh_com,
        m_thigh,
        w_thigh,
        d_thigh,
        h_thigh,
        thigh_left_frame,
    );
    thigh_left.add_cuboid_contacts_with(&thigh_com, w_thigh, d_thigh, h_thigh);
    let thigh_left_to_world = Transform3D::identity(thigh_left_frame, WORLD_FRAME);

    let m_calf = 0.1;
    let w_calf = l1;
    let d_calf = l1;
    let h_calf = l2;
    let calf_axis = Vector3::x_axis();
    let calf_com = vector![0., 0., -h_calf / 2.];
    let calf_left_frame = "calf_left";
    let mut calf_left =
        RigidBody::new_cuboid_at(&calf_com, m_calf, w_calf, d_calf, h_calf, calf_left_frame);
    calf_left.add_cuboid_contacts_with(&calf_com, w_calf, d_calf, h_calf);
    let calf_left_to_thigh_left = Transform3D::move_z(calf_left_frame, thigh_left_frame, -h_thigh);

    let m_foot = 0.1;
    let w_foot = l1;
    let d_foot = l2;
    let h_foot = l1;
    let foot_axis = Vector3::x_axis();
    let foot_left_frame = "foot_left";
    let mut foot_left = RigidBody::new_cuboid(m_foot, w_foot, d_foot, h_foot, foot_left_frame);
    foot_left.add_cuboid_contacts(w_foot, d_foot, h_foot);
    let foot_left_to_calf_left = Transform3D::move_z(foot_left_frame, calf_left_frame, -h_calf);

    let treejoints = vec![
        Joint::new_floating(thigh_left_to_world),
        Joint::new_revolute(calf_left_to_thigh_left, calf_axis),
        Joint::new_revolute(foot_left_to_calf_left, foot_axis),
    ];
    let bodies = vec![thigh_left, calf_left, foot_left];

    MechanismState::new(treejoints, bodies)
}

pub fn build_leg_from_foot() -> MechanismState {
    let l1 = 0.05;
    let l2 = 0.2;

    let m_foot = 0.1;
    let w_foot = 0.1;
    let d_foot = l2;
    let h_foot = l1;
    let foot_left_frame = "foot_left";
    let mut foot_left = RigidBody::new_cuboid(m_foot, w_foot, d_foot, h_foot, foot_left_frame);
    foot_left.add_cuboid_contacts(w_foot, d_foot, h_foot);
    let foot_left_to_world = Transform3D::identity(foot_left_frame, WORLD_FRAME);

    let m_calf = 0.1;
    let w_calf = l1;
    let d_calf = l1;
    let h_calf = l2;
    let calf_axis = Vector3::x_axis();
    let calf_com = vector![0., 0., h_calf / 2.];
    let calf_left_frame = "calf_left";
    let mut calf_left =
        RigidBody::new_cuboid_at(&calf_com, m_calf, w_calf, d_calf, h_calf, calf_left_frame);
    calf_left.add_cuboid_contacts_with(&calf_com, w_calf, d_calf, h_calf);
    let calf_left_to_foot_left = Transform3D::identity(calf_left_frame, foot_left_frame);

    let m_thigh = 0.1;
    let w_thigh = l1;
    let d_thigh = l1;
    let h_thigh = l2;
    let thigh_axis: UnitVector3<Float> = Vector3::x_axis();
    let thigh_com = vector![0., 0., h_thigh / 2.];
    let thigh_left_frame = "thigh_left";
    let mut thigh_left = RigidBody::new_cuboid_at(
        &thigh_com,
        m_thigh,
        w_thigh,
        d_thigh,
        h_thigh,
        thigh_left_frame,
    );
    thigh_left.add_cuboid_contacts_with(&thigh_com, w_thigh, d_thigh, h_thigh);
    let thigh_left_to_calf_left = Transform3D::move_z(thigh_left_frame, calf_left_frame, h_calf);

    let m_hip = 0.1;
    let w_hip = l2;
    let d_hip = l1;
    let h_hip = l1;
    let hip_axis: UnitVector3<Float> = Vector3::x_axis();
    let hip_left_frame = "hip_left";
    let hip_left_com = vector![-w_hip / 2., 0., 0.]; // vector![0., 0., 0.]; //
    let hip_left =
        RigidBody::new_cuboid_at(&hip_left_com, m_hip, w_hip, d_hip, h_hip, hip_left_frame);
    let hip_left_to_thigh_left = Transform3D::move_z(hip_left_frame, thigh_left_frame, h_thigh);

    let treejoints = vec![
        Joint::new_floating(foot_left_to_world),
        // Joint::new_fixed(foot_left_to_world),
        Joint::new_revolute(calf_left_to_foot_left, calf_axis),
        Joint::new_revolute(thigh_left_to_calf_left, thigh_axis),
        Joint::new_revolute(hip_left_to_thigh_left, hip_axis),
    ];
    let bodies = vec![foot_left, calf_left, thigh_left, hip_left];

    MechanismState::new(treejoints, bodies)
}
