use na::{vector, UnitVector3, Vector3};

use crate::{
    joint::{fixed::FixedJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::transform::Transform3D,
    types::Float,
    WORLD_FRAME,
};

pub fn build_biped() -> MechanismState {
    let l1 = 0.05;
    let l2 = 0.2;

    let m_base = 0.1;
    let w_base = l1;
    let d_base = l1;
    let h_base = l2;
    let base_frame = "base";
    let base = RigidBody::new_cuboid(m_base, w_base, d_base, h_base, base_frame);
    let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

    let m_pelvis = 0.1;
    let w_pelvis = l1;
    let d_pelvis = l1;
    let h_pelvis = l2;
    let pelvis_axis = Vector3::z_axis();
    let pelvis_left_frame = "pelvis_left";
    let pelvis_com = vector![0., 0., -h_pelvis / 2.];
    let pelvis_left = RigidBody::new_cuboid_at(
        pelvis_com,
        m_pelvis,
        w_pelvis,
        d_pelvis,
        h_pelvis,
        pelvis_left_frame,
    );
    let pelvis_left_to_base = Transform3D::move_xyz(
        pelvis_left_frame,
        base_frame,
        (w_base + w_pelvis) / 2.0,
        0.,
        0.,
    );

    let m_hip = 0.1;
    let w_hip = l2;
    let d_hip = l1;
    let h_hip = l1;
    let hip_axis: UnitVector3<Float> = -Vector3::y_axis();
    let hip_left_frame = "hip_left";
    let hip_left_com = vector![w_hip / 2., 0., 0.];
    let hip_left =
        RigidBody::new_cuboid_at(hip_left_com, m_hip, w_hip, d_hip, h_hip, hip_left_frame);
    let hip_left_to_pelvis_left =
        Transform3D::move_xyz(hip_left_frame, pelvis_left_frame, 0., 0., -h_pelvis);

    let m_thigh = 0.1;
    let w_thigh = l1;
    let d_thigh = l1;
    let h_thigh = l2;
    let thigh_axis: UnitVector3<Float> = Vector3::x_axis();
    let thigh_com = vector![0., 0., -h_thigh / 2.];
    let thigh_left_frame = "thigh_left";
    let thigh_left = RigidBody::new_cuboid_at(
        thigh_com,
        m_thigh,
        w_thigh,
        d_thigh,
        h_thigh,
        thigh_left_frame,
    );
    let thigh_left_to_hip_left = Transform3D::move_x(thigh_left_frame, hip_left_frame, w_hip);

    let m_calf = 0.1;
    let w_calf = l1;
    let d_calf = l1;
    let h_calf = l2;
    let calf_axis = Vector3::x_axis();
    let calf_com = vector![0., 0., -h_calf / 2.];
    let calf_left_frame = "calf_left";
    let calf_left =
        RigidBody::new_cuboid_at(calf_com, m_calf, w_calf, d_calf, h_calf, calf_left_frame);
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
        Joint::FixedJoint(FixedJoint::new(base_to_world)),
        Joint::new_revolute(pelvis_left_to_base, pelvis_axis),
        Joint::new_revolute(hip_left_to_pelvis_left, hip_axis),
        Joint::new_revolute(thigh_left_to_hip_left, thigh_axis),
        Joint::new_revolute(calf_left_to_thigh_left, calf_axis),
        Joint::new_revolute(foot_left_to_calf_left, foot_axis),
    ];
    let bodies = vec![
        base,
        pelvis_left,
        hip_left,
        thigh_left,
        calf_left,
        foot_left,
    ];
    MechanismState::new(treejoints, bodies)
}

#[cfg(test)]
mod biped_builder_tests {
    use crate::{
        builders::biped_builder::build_biped,
        joint::{JointPosition, JointVelocity},
        plot::plot,
        simulate::step,
        PI,
    };

    #[test]
    #[ignore = "not completed"]
    fn rotate() {
        // Arrange
        let mut state = build_biped();

        state.set_joint_v(2, JointVelocity::Float(0.5));
        state.set_joint_q(3, JointPosition::Float(-PI / 2.));

        // Act
        let mut data = vec![];
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );

            let hip_pose = state.poses()[2];
            data.push(hip_pose.rotation.axis_angle().unwrap().1);
        }

        // Assert
        plot(&data, final_time, dt, num_steps, "biped");
    }
}
