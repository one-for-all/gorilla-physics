use na::{vector, Isometry3, Matrix3, Vector3};

use crate::{
    contact::ContactPoint,
    inertia::SpatialInertia,
    joint::{
        constraint_revolute::ConstraintRevoluteJoint, floating::FloatingJoint,
        revolute::RevoluteJoint, Joint,
    },
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
    let mut bar3 = RigidBody::new(SpatialInertia::new(moment, cross_part, m_bar3, bar3_frame));
    let bar3_to_bar1 = Transform3D::move_xyz(bar3_frame, bar1_frame, 0., 0., -l);
    bar3.add_contact_point(ContactPoint::new(bar3_frame, vector![0., 0., 0.]));

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

pub fn build_mock_navbot(m: Float) -> MechanismState {
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

    let right_leg_frame = "right_leg";
    let moment_x = m * (4. * l * l + w * w) / 12.0;
    let moment_y = m * (4. * l * l + w * w) / 12.0;
    let moment_z = m * w * w / 6.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., 0., -m * l / 2.0];
    let right_leg = RigidBody::new(SpatialInertia::new(moment, cross_part, m, right_leg_frame));
    let right_leg_to_base =
        Transform3D::move_xyz(right_leg_frame, base_frame, l / 2.0, -l / 2.0, 0.);

    let right_link_frame = "right_link";
    let right_link = RigidBody::new(SpatialInertia::new(moment, cross_part, m, right_link_frame));
    let right_link_to_base =
        Transform3D::move_xyz(right_link_frame, base_frame, l / 2.0, l / 2.0, 0.);

    let left_leg_frame = "left_leg";
    let left_leg = RigidBody::new(SpatialInertia::new(moment, cross_part, m, left_leg_frame));
    let left_leg_to_base =
        Transform3D::move_xyz(left_leg_frame, base_frame, -l / 2.0, -l / 2.0, 0.);

    let left_link_frame = "left_link";
    let left_link = RigidBody::new(SpatialInertia::new(moment, cross_part, m, left_link_frame));
    let left_link_to_base =
        Transform3D::move_xyz(left_link_frame, base_frame, -l / 2.0, l / 2.0, 0.);

    let right_foot_frame = "right_foot";
    let moment_x = m * (4. * l * l + w * w) / 12.0;
    let moment_y = m * w * w / 6.0;
    let moment_z = m * (4. * l * l + w * w) / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0., m * l / 2.0, 0.];
    let right_foot = RigidBody::new(SpatialInertia::new(moment, cross_part, m, right_foot_frame));
    let right_foot_to_right_leg =
        Transform3D::move_xyz(right_foot_frame, right_leg_frame, 0., 0., -l);

    let left_foot_frame = "left_foot";
    let left_foot = RigidBody::new(SpatialInertia::new(moment, cross_part, m, left_foot_frame));
    let left_foot_to_left_leg = Transform3D::move_xyz(left_foot_frame, left_leg_frame, 0., 0., -l);

    let bodies = vec![
        base, right_leg, right_link, left_leg, left_link, right_foot, left_foot,
    ];
    let treejoints = vec![
        // Joint::RevoluteJoint(RevoluteJoint::new(base_to_world, Vector3::z_axis())),
        Joint::FloatingJoint(FloatingJoint::new(base_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(right_leg_to_base, Vector3::x_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(right_link_to_base, Vector3::x_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(left_leg_to_base, Vector3::x_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(left_link_to_base, Vector3::x_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(
            right_foot_to_right_leg,
            Vector3::x_axis(),
        )),
        Joint::RevoluteJoint(RevoluteJoint::new(left_foot_to_left_leg, Vector3::x_axis())),
    ];

    let right_foot_to_right_link = ConstraintRevoluteJoint::new(
        right_foot_frame,
        Isometry3::translation(0., l, 0.),
        right_link_frame,
        Isometry3::translation(0., 0., -l),
        Vector3::x_axis(),
    );
    let left_foot_to_left_link = ConstraintRevoluteJoint::new(
        left_foot_frame,
        Isometry3::translation(0., l, 0.),
        left_link_frame,
        Isometry3::translation(0., 0., -l),
        Vector3::x_axis(),
    );
    let constraints = vec![right_foot_to_right_link, left_foot_to_left_link];

    MechanismState::new_with_constraint(treejoints, bodies, constraints)
}
