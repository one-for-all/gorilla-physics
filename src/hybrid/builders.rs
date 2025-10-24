use na::{vector, UnitQuaternion, Vector3};

use crate::{
    collision::halfspace::HalfSpace,
    flog,
    hybrid::{
        articulated::Articulated, cloth::Cloth, control::GripperController, Deformable, Hybrid,
        Rigid,
    },
    joint::{Joint, JointVelocity},
    spatial::transform::Transform3D,
    PI, WORLD_FRAME,
};

pub fn build_claw() -> Hybrid {
    let mut state = Hybrid::empty();

    let m = 1.0;

    let l_palm = 1.0; // half-size of palm
    let h_palm = 0.1;
    let palm_frame = "palm";
    let palm = Rigid::new_cuboid(m, l_palm, l_palm, h_palm, palm_frame);
    // let palm_joint = Joint::new_prismatic(
    //     Transform3D::move_xyz(palm_frame, WORLD_FRAME, 0.5, 0.5, 1.1),
    //     Vector3::z_axis(),
    // );
    let palm_joint = Joint::new_fixed(Transform3D::move_xyz(
        palm_frame,
        WORLD_FRAME,
        0.5,
        0.5,
        1.1,
    ));

    let l_finger = 1.0;
    let w_finger = 0.1;
    let left_frame = "left finger";
    let mut left = Rigid::new_cuboid_at(
        &vector![-l_finger / 2., 0., 0.],
        m,
        l_finger,
        w_finger,
        w_finger,
        left_frame,
    );
    left.add_cuboid_at(
        &vector![-l_finger, 0., -l_finger / 2.],
        m,
        w_finger,
        w_finger,
        l_finger,
    );
    let left_joint = Joint::new_revolute(
        Transform3D::move_x(left_frame, palm_frame, -l_palm / 2.),
        Vector3::y_axis(),
    );

    let right_frame = "right finger";
    let mut right = Rigid::new_cuboid_at(
        &vector![l_finger / 2., 0., 0.],
        m,
        l_finger,
        w_finger,
        w_finger,
        right_frame,
    );
    right.add_cuboid_at(
        &vector![l_finger, 0., -l_finger / 2.],
        m,
        w_finger,
        w_finger,
        l_finger,
    );
    let right_joint = Joint::new_revolute(
        Transform3D::move_x(right_frame, palm_frame, l_palm / 2.),
        Vector3::y_axis(),
    );

    let mut articulated = Articulated::new(
        vec![palm, left, right],
        vec![palm_joint, left_joint, right_joint],
    );

    state.add_articulated(articulated);

    // state.add_deformable(Deformable::new_cube());
    state.add_deformable(Deformable::new_dense_cube(1., 2, 1e3));
    // let n_nodes = state.deformables[0].nodes.len();
    // let v = vector![3., 0., 0.];
    // let v = vec![v; n_nodes];
    // state.set_deformable_velocities(vec![v]);

    state.add_halfspace(HalfSpace::new(Vector3::z_axis(), -0.5));

    state
}

pub fn build_gripper() -> Articulated {
    let m = 10.0;

    let w_base = 0.5;
    let h_base = 0.1;
    let base_frame = "base";
    let base = Rigid::new_cuboid(m, w_base, w_base, h_base, base_frame);
    let base_joint = Joint::new_prismatic(
        Transform3D::move_xyz(base_frame, WORLD_FRAME, 0.5, 0.5, 1.2),
        Vector3::z_axis(),
    );

    let l_palm = 1.5; // half-size of palm
    let h_palm = 0.1;
    let palm_frame = "palm";
    let palm = Rigid::new_cuboid(m, l_palm, l_palm, h_palm, palm_frame);
    let palm_joint = Joint::new_prismatic(
        Transform3D::move_z(palm_frame, base_frame, -h_base),
        Vector3::x_axis(),
    );

    let l_finger = 1.0;
    let w_finger = 0.1;
    let d_finger = 0.5;
    let left_frame = "left finger";
    let left = Rigid::new_cuboid(m, w_finger, d_finger, l_finger, left_frame);
    let left_joint = Joint::new_prismatic(
        Transform3D::move_xyz(left_frame, palm_frame, -l_palm / 2., 0., -l_finger / 2.),
        Vector3::x_axis(),
    );

    let right_frame = "right finger";
    let right = Rigid::new_cuboid(m, w_finger, d_finger, l_finger, right_frame);
    let right_joint = Joint::new_prismatic(
        Transform3D::move_xyz(right_frame, palm_frame, l_palm / 2., 0., -l_finger / 2.),
        Vector3::x_axis(),
    );

    let articulated = Articulated::new(
        vec![base, palm, left, right],
        vec![base_joint, palm_joint, left_joint, right_joint],
    );
    articulated
}

pub fn build_gripper_cube() -> Hybrid {
    let mut state = Hybrid::empty();
    let gripper = build_gripper();
    state.add_articulated(gripper);
    state.set_controller(0, GripperController::new(1. / 120.));

    state.add_deformable(Deformable::new_dense_cube(1., 1, 1e3));
    state.add_halfspace(HalfSpace::new(Vector3::z_axis(), -0.5));

    state
}

pub fn build_gripper_cloth() -> Hybrid {
    let mut state = Hybrid::empty();
    let gripper = build_gripper();
    state.add_articulated(gripper);
    state.set_controller(0, GripperController::new(1. / 120.));

    state.add_cloth(Cloth::new_square(
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -PI / 2.),
        vector![0.5, 0., 0.1],
        1e3,
    ));
    state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));
    state
}

pub fn build_cube_cloth() -> Hybrid {
    let mut state = Hybrid::empty();
    let m = 1.0;
    let w = 0.5;
    let cube_frame = "cube";
    let cube = Rigid::new_cuboid(m, w, w, w, cube_frame);
    let cube_to_world = Transform3D::move_xyz(cube_frame, WORLD_FRAME, -0.6, 0.5, 0.);
    let mut articulated = Articulated::new(
        vec![cube],
        vec![Joint::new_prismatic(cube_to_world, Vector3::x_axis())],
    );
    articulated.set_joint_v(0, JointVelocity::Float(1.0));
    state.add_articulated(articulated);

    state.add_cloth(Cloth::new_square(
        UnitQuaternion::identity(),
        vector![0., 0., 0.],
        1e4,
    ));

    state.disable_gravity();

    state
}
