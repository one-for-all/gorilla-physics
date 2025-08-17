use na::{vector, UnitVector3, Vector3};

use crate::{
    joint::{fixed::FixedJoint, floating::FloatingJoint, revolute::RevoluteJoint, Joint},
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
    let hip_com = vector![w_hip / 2., 0., 0.];
    let hip_left = RigidBody::new_cuboid_at(hip_com, m_hip, w_hip, d_hip, h_hip, hip_left_frame);
    let hip_left_to_pelvis_left =
        Transform3D::move_xyz(hip_left_frame, pelvis_left_frame, 0., 0., -h_pelvis);

    let treejoints = vec![
        Joint::FixedJoint(FixedJoint::new(base_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(pelvis_left_to_base, pelvis_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(hip_left_to_pelvis_left, hip_axis)),
    ];
    let bodies = vec![base, pelvis_left, hip_left];
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
