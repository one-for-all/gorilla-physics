use na::{dvector, Matrix3, Matrix4, Vector3};

use crate::{
    inertia::SpatialInertia, joint::revolute::RevoluteJoint, mechanism::MechanismState,
    rigid_body::RigidBody, transform::Transform3D, types::Float,
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
            RevoluteJoint {
                init_mat: rod1_to_world.mat.clone(),
                transform: rod1_to_world,
                axis: axis.clone(),
            },
            RevoluteJoint {
                init_mat: rod2_to_rod1.mat.clone(),
                transform: rod2_to_rod1,
                axis: axis.clone(),
            }
        ],
        treejointids: dvector![1, 2],
        bodies: dvector![
            RigidBody {
                inertia: SpatialInertia {
                    frame: rod1_frame.to_string(),
                    moment: moment.clone(),
                    cross_part: cross_part.clone(),
                    mass: mass.clone(),
                }
            },
            RigidBody {
                inertia: SpatialInertia {
                    frame: rod2_frame.to_string(),
                    moment: moment.clone(),
                    cross_part: cross_part.clone(),
                    mass: mass.clone(),
                }
            }
        ],
        q: dvector![0.0, 0.0],
        v: dvector![0.0, 0.0],
    };

    state
}
