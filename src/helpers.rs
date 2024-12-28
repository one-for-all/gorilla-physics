use na::{dvector, Matrix3, Matrix4, Vector3};

use crate::{
    inertia::SpatialInertia, joint::RevoluteJoint, mechanism::MechanismState,
    rigid_body::RigidBody, transform::Transform3D, types::Float,
};

/// Build a mechanism state of a rod pendulum for testing.
pub fn build_rod_pendulum(
    mass: &Float,
    moment: &Matrix3<Float>,
    cross_part: &Vector3<Float>,
    rot_to_world: &Matrix4<Float>,
    axis: &Vector3<Float>,
) -> MechanismState {
    let rod_frame = "rod";
    let world_frame = "world";
    let rod_to_world = Transform3D::new(rod_frame, world_frame, &rot_to_world);

    let state = MechanismState {
        treejoints: dvector![RevoluteJoint {
            init_mat: rod_to_world.mat.clone(),
            transform: rod_to_world,
            axis: axis.clone(),
        }],
        treejointids: dvector![1],
        bodies: dvector![RigidBody {
            inertia: SpatialInertia {
                frame: rod_frame.to_string(),
                moment: moment.clone(),
                cross_part: cross_part.clone(),
                mass: mass.clone(),
            }
        }],
        q: dvector![0.0],
        v: dvector![0.0],
    };

    state
}
