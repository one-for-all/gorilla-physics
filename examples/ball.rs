use gorilla_physics::contact::{ContactPoint, HalfSpace};
use gorilla_physics::control::Controller;
use gorilla_physics::interface::controller::NullController;
use gorilla_physics::joint::floating::FloatingJoint;
use gorilla_physics::mechanism::MechanismState;
use gorilla_physics::plot::{plot, plot_trajectory};
use gorilla_physics::rigid_body::RigidBody;
use gorilla_physics::simulate::step;
use gorilla_physics::spatial::transform::Transform3D;
use nalgebra::{vector, UnitQuaternion, Vector3};

use gorilla_physics::joint::{Joint, JointPosition, JointVelocity};
use gorilla_physics::{spatial::pose::Pose, spatial::spatial_vector::SpatialVector, types::Float};

pub fn main() {
    // Arrange
    let m = 0.1;
    let r = 1.0;
    let v_x_init = 1.0;

    let ball_frame = "ball";
    let world_frame = "world";
    let ball_to_world = Transform3D::identity(&ball_frame, &world_frame);

    let ball = RigidBody::new_sphere(m, r, &ball_frame);

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(ball_to_world))];
    let bodies = vec![ball];
    let mut state = MechanismState::new(treejoints, bodies);

    let h_ground = -0.3;
    let alpha = 2.0; // roughly how much velocity is lost
    let mu = 1.0;
    let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
    state.add_halfspace(&ground);

    state.add_contact_point(ContactPoint::new(ball_frame, vector![0.0, 0.0, 0.0]));

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::identity(),
        translation: vector![0.0, 0.0, 0.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: vector![v_x_init, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    // Act
    let final_time = 5.0;
    let dt = 1e-3;

    let num_steps = (final_time / dt) as usize;

    let mut controller = NullController {};

    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data2: Vec<Float> = Vec::with_capacity(num_steps);

    for s in 0..num_steps {
        let torque = controller.control(&mut state, None);
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        let pose_body = &state.poses()[0];

        data1.push(pose_body.translation.x);
        data2.push(pose_body.translation.z);
    }

    plot(&data1, final_time, dt, num_steps, "plot");
    plot_trajectory(&data1, &data2, "traj");
}
