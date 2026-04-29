use na::Vector3;

use crate::{
    hybrid::{articulated::Articulated, Rigid},
    joint::Joint,
    spatial::transform::Transform3D,
    types::Float,
    WORLD_FRAME,
};

impl Articulated {
    pub fn new_sphere(frame: &str, m: Float, r: Float) -> Self {
        let body = Rigid::new_sphere(m, r, frame);
        let joint = Joint::new_floating(Transform3D::identity(frame, WORLD_FRAME));
        Self::new(vec![body], vec![joint])
    }

    pub fn new_sphere_at(frame: &str, m: Float, r: Float, pos: &Vector3<Float>) -> Self {
        let body = Rigid::new_sphere(m, r, frame);
        let joint = Joint::new_floating(Transform3D::move_xyz(
            frame,
            WORLD_FRAME,
            pos.x,
            pos.y,
            pos.z,
        ));
        Self::new(vec![body], vec![joint])
    }

    pub fn new_cube_at(frame: &str, m: Float, w: Float, pos: &Vector3<Float>) -> Self {
        let body = Rigid::new_cuboid(m, w, w, w, frame);
        let joint = Joint::new_floating(Transform3D::move_xyz(
            frame,
            WORLD_FRAME,
            pos.x,
            pos.y,
            pos.z,
        ));
        Self::new(vec![body], vec![joint])
    }
}
