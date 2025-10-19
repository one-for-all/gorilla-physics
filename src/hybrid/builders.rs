use na::{vector, Vector3};

use crate::{
    flog,
    hybrid::{articulated::Articulated, Deformable, Hybrid, Rigid},
    joint::Joint,
    spatial::transform::Transform3D,
    WORLD_FRAME,
};

pub fn build_gripper() -> Hybrid {
    let mut state = Hybrid::empty();

    let m = 1.0;

    let l_palm = 1.0; // half-size of palm
    let h_palm = 0.1;
    let palm_frame = "palm";
    let palm = Rigid::new_cuboid(m, l_palm, l_palm, h_palm, palm_frame);
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

    let articulated = Articulated::new(
        vec![palm, left, right],
        vec![palm_joint, left_joint, right_joint],
    );

    state.add_articulated(articulated);

    // state.add_deformable(Deformable::new_cube());
    state.add_deformable(Deformable::new_dense_cube(1., 2));

    state
}
