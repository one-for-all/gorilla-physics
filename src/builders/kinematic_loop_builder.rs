use na::{vector, Isometry3, Matrix3, Vector3};

use crate::{
    inertia::SpatialInertia,
    joint::{constraint_revolute::ConstraintRevoluteJoint, revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    spatial::transform::Transform3D,
    types::Float,
    WORLD_FRAME,
};

/// Two bars dangling straightdown, and one bar connecting these two bars. The
/// world is considered the root bar.
pub fn build_four_bar_linkage(m: Float, m_bar3: Float) -> MechanismState {
    let l = 1.0;
    let w = 0.1;

    let angle = 0.; // -PI / 4.0;

    let bar1_frame = "bar1";
    let moment_x = m * (4. * l * l + w * w) / 12.0;
    let moment_y = m * (4. * l * l + w * w) / 12.0;
    let moment_z = m * w * w / 6.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., 0., -m * l / 2.0];
    let bar1 = RigidBody::new(SpatialInertia::new(moment, cross_part, m, bar1_frame));
    // let bar1_to_world = Transform3D::move_x(bar1_frame, WORLD_FRAME, 0.0);
    let bar1_to_world = Transform3D::new(
        bar1_frame,
        WORLD_FRAME,
        &Isometry3::rotation(Vector3::z_axis().scale(angle)),
    );

    let bar2_frame = "bar2";
    let bar2 = RigidBody::new(SpatialInertia::new(moment, cross_part, m, bar2_frame));
    // let bar2_to_world = Transform3D::move_x(bar2_frame, WORLD_FRAME, l + 0.0);
    let bar2_to_world = Transform3D::new(
        bar2_frame,
        WORLD_FRAME,
        &(Isometry3::rotation(Vector3::z_axis().scale(angle)) * Isometry3::translation(l, 0., 0.)),
    );

    let bar3_frame = "bar3";
    let moment_x = m_bar3 * w * w / 6.0;
    let moment_y = m_bar3 * (4. * l * l + w * w) / 12.0;
    let moment_z = m_bar3 * (4. * l * l + w * w) / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m_bar3 * l / 2.0, 0., 0.];
    let bar3 = RigidBody::new(SpatialInertia::new(moment, cross_part, m_bar3, bar3_frame));
    let bar3_to_bar1 = Transform3D::move_xyz(bar3_frame, bar1_frame, 0., 0., -l);

    let bodies = vec![bar1, bar2, bar3];
    let treejoints = vec![
        Joint::RevoluteJoint(RevoluteJoint::new(bar1_to_world, Vector3::y_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(bar2_to_world, Vector3::y_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(bar3_to_bar1, Vector3::y_axis())),
    ];

    let constraint_to_bar3 = Isometry3::translation(l, 0., 0.);
    let constraint_to_bar2 = Isometry3::translation(0., 0., -l);
    let constraint_joints = vec![ConstraintRevoluteJoint::new(
        bar3_frame,
        constraint_to_bar3,
        bar2_frame,
        constraint_to_bar2,
        Vector3::y_axis(),
    )];
    // let constraint_joints = vec![];

    MechanismState::new_with_constraint(treejoints, bodies, constraint_joints)
}

/// A four bar linkage on a rotatable base
pub fn build_four_bar_linkage_with_base(m: Float, m_bar3: Float) -> MechanismState {
    let l = 1.0;
    let w = 0.1;

    let base_frame = "base";
    let moment_x = m * w * w / 6.0;
    let moment_y = m * (4. * l * l + w * w) / 12.0;
    let moment_z = m * (4. * l * l + w * w) / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., 0., 0.];
    let base = RigidBody::new(SpatialInertia::new(moment, cross_part, m, base_frame));
    let base_to_world = Transform3D::move_x(base_frame, WORLD_FRAME, 0.);

    let bar1_frame = "bar1";
    let moment_x = m * (4. * l * l + w * w) / 12.0;
    let moment_y = m * (4. * l * l + w * w) / 12.0;
    let moment_z = m * w * w / 6.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., 0., -m * l / 2.0];
    let bar1 = RigidBody::new(SpatialInertia::new(moment, cross_part, m, bar1_frame));
    let bar1_to_base = Transform3D::move_x(bar1_frame, base_frame, l / 2.0);

    let bar2_frame = "bar2";
    let bar2 = RigidBody::new(SpatialInertia::new(moment, cross_part, m, bar2_frame));
    let bar2_to_base = Transform3D::move_x(bar2_frame, base_frame, l + l / 2.0);

    let bar3_frame = "bar3";
    let moment_x = m_bar3 * w * w / 6.0;
    let moment_y = m_bar3 * (4. * l * l + w * w) / 12.0;
    let moment_z = m_bar3 * (4. * l * l + w * w) / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m_bar3 * l / 2.0, 0., 0.];
    let bar3 = RigidBody::new(SpatialInertia::new(moment, cross_part, m_bar3, bar3_frame));
    let bar3_to_bar1 = Transform3D::move_xyz(bar3_frame, bar1_frame, 0., 0., -l);

    let bodies = vec![base, bar1, bar2, bar3];
    let treejoints = vec![
        Joint::RevoluteJoint(RevoluteJoint::new(base_to_world, Vector3::z_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(bar1_to_base, Vector3::y_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(bar2_to_base, Vector3::y_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(bar3_to_bar1, Vector3::y_axis())),
    ];

    let constraint_to_bar3 = Isometry3::translation(l, 0., 0.);
    let constraint_to_bar2 = Isometry3::translation(0., 0., -l);
    let constraint_joints = vec![ConstraintRevoluteJoint::new(
        bar3_frame,
        constraint_to_bar3,
        bar2_frame,
        constraint_to_bar2,
        Vector3::y_axis(),
    )];

    MechanismState::new_with_constraint(treejoints, bodies, constraint_joints)
}
