use na::{dvector, DVector};

use crate::{contact::ContactPoint, inertia::SpatialInertia};

#[derive(Clone, PartialEq, Debug)]
pub struct RigidBody {
    pub inertia: SpatialInertia,
    pub contact_points: DVector<ContactPoint>,
}

impl RigidBody {
    pub fn new(inertia: SpatialInertia) -> Self {
        RigidBody {
            inertia,
            contact_points: dvector![],
        }
    }

    pub fn add_contact_point(&mut self, contact_point: &ContactPoint) {
        if self.inertia.frame != contact_point.frame {
            panic!(
                "Contact point frame {} does not match body frame {}",
                contact_point.frame, self.inertia.frame
            );
        }
        self.contact_points = self.contact_points.push(contact_point.clone());
    }
}
