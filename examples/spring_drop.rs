use gorilla_physics::contact::{ContactPoint, HalfSpace};
use gorilla_physics::joint::floating::FloatingJoint;
use gorilla_physics::joint::prismatic::{JointSpring, PrismaticJoint};
use gorilla_physics::joint::{Joint, JointVelocity};
use gorilla_physics::mechanism::MechanismState;
use gorilla_physics::plot::plot;
use gorilla_physics::rigid_body::RigidBody;
use gorilla_physics::simulate::step;
use gorilla_physics::spatial::transform::Matrix4Ext;
use gorilla_physics::spatial::transform::Transform3D;
use gorilla_physics::WORLD_FRAME;
use gorilla_physics::{spatial::spatial_vector::SpatialVector, types::Float};
use nalgebra::{vector, zero, Matrix4, Vector3};

/// Simulate a spring connected by two spheres dropping to ground
pub fn main() {
    let m_body = 1.0;
    let r_body = 1.0;
    let m_foot = 1.0;
    let r_foot = 1.0;
    let l_leg = 1.0;
    let l_spring_rest = 0.0;
    let k_spring = 100.0;

    let body_frame = "body";
    let body = RigidBody::new_sphere(m_body, r_body, &body_frame);
    let body_to_world = Transform3D::identity(body_frame, WORLD_FRAME);

    let foot_frame = "foot";
    let foot = RigidBody::new_sphere(m_foot, r_foot, &foot_frame);
    let foot_to_body = Transform3D {
        from: foot_frame.to_string(),
        to: body_frame.to_string(),
        mat: Matrix4::<Float>::move_z(-l_leg),
    };
    let axis_leg = vector![0.0, 0.0, -1.0];

    let spring = JointSpring {
        k: k_spring,
        l: l_spring_rest,
    };
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
        Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
            foot_to_body,
            axis_leg,
            spring,
        )),
    ];
    let bodies = vec![body, foot];
    let mut state = MechanismState::new(treejoints, bodies);
    state.add_contact_point(ContactPoint::new(foot_frame, Vector3::zeros()));

    let h_ground = -2.0;
    let alpha = 1.0;
    let mu = 0.0;
    let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
    state.add_halfspace(&ground);

    state.set_joint_v(
        1,
        JointVelocity::Spatial(SpatialVector {
            angular: zero(),
            linear: vector![0.1, 0.0, 0.0],
        }),
    );

    // Act
    let final_time = 2.5;
    let dt = 1e-3;
    let num_steps = (final_time / dt) as usize;
    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut prev_v_spring = 0.0;
    for s in 0..num_steps {
        let torque = vec![];
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        let v_spring = v[1].float();
        if prev_v_spring < 0.0 && *v_spring >= 0.0 {
            println!("bottom");
        }
        prev_v_spring = *v_spring;

        let pose_foot = &state.poses()[1];
        data1.push(pose_foot.translation.z);
    }

    plot(&data1, final_time, dt, num_steps, "spring drop");
}
