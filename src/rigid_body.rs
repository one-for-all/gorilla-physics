use na::{vector, Matrix3, Vector3};

use crate::{
    collision::cuboid::Cuboid,
    contact::{ContactPoint, SpringContact},
    inertia::SpatialInertia,
    mesh::Mesh,
    types::Float,
};

#[derive(Clone, PartialEq, Debug)]
pub enum Collider {
    Mesh(Mesh),
    Cuboid(Cuboid),
}

impl Collider {
    pub fn mesh(&self) -> &Mesh {
        match self {
            Collider::Mesh(mesh) => mesh,
            _ => panic!("Collider is not a Mesh"),
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct RigidBody {
    pub inertia: SpatialInertia,
    pub contact_points: Vec<ContactPoint>,
    pub spring_contacts: Vec<SpringContact>,
    pub collider: Option<Collider>,
}

impl RigidBody {
    pub fn new(inertia: SpatialInertia) -> Self {
        RigidBody {
            inertia,
            contact_points: vec![],
            spring_contacts: vec![],
            collider: None,
        }
    }

    pub fn new_mesh(mesh: Mesh, inertia: SpatialInertia) -> Self {
        RigidBody {
            inertia,
            contact_points: vec![],
            spring_contacts: vec![],
            collider: Some(Collider::Mesh(mesh)),
        }
    }

    pub fn new_sphere(m: Float, r: Float, frame: &str) -> RigidBody {
        let moment_x = 2.0 / 5.0 * m * r * r;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
        let cross_part = Vector3::zeros();

        RigidBody::new(SpatialInertia {
            frame: frame.to_string(),
            moment,
            cross_part,
            mass: m,
        })
    }

    pub fn new_cube(m: Float, l: Float, frame: &str) -> RigidBody {
        let moment_x = m * l * l / 6.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
        let cross_part = vector![0., 0., 0.];

        RigidBody::new(SpatialInertia {
            frame: frame.to_string(),
            moment,
            cross_part,
            mass: m,
        })
    }

    pub fn new_cuboid(m: Float, w: Float, d: Float, h: Float, frame: &str) -> RigidBody {
        let moment_x = m * (d * d + h * h) / 12.0;
        let moment_y = m * (w * w + h * h) / 12.0;
        let moment_z = m * (w * w + d * d) / 12.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0., 0., 0.];
        RigidBody::new(SpatialInertia {
            frame: frame.to_string(),
            moment,
            cross_part,
            mass: m,
        })
    }

    pub fn add_collider(&mut self, collision_geometry: Cuboid) {
        self.collider = Some(Collider::Cuboid(collision_geometry));
    }

    pub fn add_contact_point(&mut self, contact_point: ContactPoint) {
        if self.inertia.frame != contact_point.frame {
            panic!(
                "Contact point frame {} does not match body frame {}",
                contact_point.frame, self.inertia.frame
            );
        }
        self.contact_points.push(contact_point);
    }

    pub fn add_spring_contact(&mut self, spring_contact: &SpringContact) {
        if self.inertia.frame != spring_contact.frame {
            panic!(
                "Spring contact point frame {} does not match body frame {}",
                spring_contact.frame, self.inertia.frame
            );
        }
        self.spring_contacts.push(spring_contact.clone());
    }
}
