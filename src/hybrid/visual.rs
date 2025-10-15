use crate::types::Float;

pub struct SphereGeometry {
    pub r: Float,
}

pub enum Visual {
    Sphere(SphereGeometry),
}
