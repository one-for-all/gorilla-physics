use na::{vector, Matrix3, Vector3};

use crate::{
    hybrid::{articulated::Articulated, Rigid},
    inertia::SpatialInertia,
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

    /// Create an articulated with a single point at position pos, with mass m, and an assumed r for inertia calculation
    pub fn new_point_at(frame: &str, m: Float, pos: &Vector3<Float>) -> Self {
        let r = 0.1;
        let moment = 2. / 5. * m * r * r;
        let moment = Matrix3::from_diagonal_element(moment);
        let cross_part = vector![0., 0., 0.];
        let inertia = SpatialInertia::new(moment, cross_part, m, frame);

        let mut body = Rigid::new(inertia);
        body.add_point_at(&vector![0., 0., 0.]);

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
