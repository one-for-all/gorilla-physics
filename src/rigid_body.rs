use crate::inertia::SpatialInertia;

pub struct RigidBody {
    pub inertia: SpatialInertia,
}

impl RigidBody {
    pub fn new(inertia: SpatialInertia) -> Self {
        RigidBody { inertia }
    }

    pub fn default() -> Self {
        RigidBody {
            inertia: SpatialInertia::default(),
        }
    }
}
