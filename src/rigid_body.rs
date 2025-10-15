use na::{vector, Matrix3, Vector3};

use crate::{
    collision::{cuboid::Cuboid, mesh::Mesh, sphere::Sphere},
    contact::{ContactPoint, SpringContact},
    inertia::SpatialInertia,
    types::Float,
};

#[derive(Clone, PartialEq, Debug)]
pub struct Collider {
    pub geometry: CollisionGeometry,
    pub enabled: bool,
}

impl Collider {
    pub fn new_mesh(mesh: Mesh) -> Self {
        Collider {
            geometry: CollisionGeometry::Mesh(mesh),
            enabled: true,
        }
    }

    pub fn new_cuboid(cuboid: Cuboid) -> Self {
        Collider {
            geometry: CollisionGeometry::Cuboid(cuboid),
            enabled: true,
        }
    }

    pub fn new_sphere(sphere: Sphere) -> Self {
        Collider {
            geometry: CollisionGeometry::Sphere(sphere),
            enabled: true,
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum CollisionGeometry {
    Mesh(Mesh),
    Cuboid(Cuboid),
    Sphere(Sphere),
}

impl CollisionGeometry {
    pub fn mesh(&self) -> &Mesh {
        match self {
            CollisionGeometry::Mesh(mesh) => mesh,
            _ => panic!("Collider is not a Mesh"),
        }
    }

    pub fn sphere(&self) -> &Sphere {
        match self {
            CollisionGeometry::Sphere(sphere) => sphere,
            _ => panic!("Collider is not a sphere"),
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct RigidBody {
    pub inertia: SpatialInertia,
    pub contact_points: Vec<ContactPoint>,
    pub spring_contacts: Vec<SpringContact>,
    pub collider: Option<Collider>,
    pub visual: Vec<Mesh>,
}

impl RigidBody {
    pub fn new(inertia: SpatialInertia) -> Self {
        RigidBody {
            inertia,
            contact_points: vec![],
            spring_contacts: vec![],
            collider: None,
            visual: vec![],
        }
    }

    pub fn add_visual_mesh(&mut self, mesh: Mesh) {
        self.visual.push(mesh);
    }

    /// Creates a new rigid body with the supplied collider and visual
    pub fn new_collider_and_visual(
        collider: Collider,
        visual_mesh: Mesh,
        spatial_inertia: SpatialInertia,
    ) -> Self {
        RigidBody {
            inertia: spatial_inertia,
            contact_points: vec![],
            spring_contacts: vec![],
            collider: Some(collider),
            visual: vec![visual_mesh],
        }
    }

    pub fn new_mesh(mesh: Mesh, spatial_inertia: SpatialInertia, collision_enabled: bool) -> Self {
        let visual_mesh = mesh.clone();
        let mut collider = Collider::new_mesh(mesh);
        collider.enabled = collision_enabled;
        RigidBody {
            inertia: spatial_inertia,
            contact_points: vec![],
            spring_contacts: vec![],
            collider: Some(collider),
            visual: vec![visual_mesh],
        }
    }

    pub fn new_sphere(m: Float, r: Float, frame: &str) -> RigidBody {
        let moment_x = 2.0 / 5.0 * m * r * r;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
        let cross_part = Vector3::zeros();

        let mut body = RigidBody::new(SpatialInertia {
            frame: frame.to_string(),
            moment,
            cross_part,
            mass: m,
        });
        let collider = Collider::new_sphere(Sphere::new(r));
        body.collider = Some(collider);
        body
    }

    pub fn new_sphere_at(com: &Vector3<Float>, m: Float, r: Float, frame: &str) -> RigidBody {
        let moment = 2. / 5. * m * r * r;
        let moment_com = Matrix3::from_diagonal_element(moment);

        // generalized parallel axis theorem
        let moment =
            moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
        let cross_part = m * com;
        let inertia = SpatialInertia::new(moment, cross_part, m, frame);

        let body = RigidBody::new(inertia);

        // Add collider

        body
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

    /// Create a uniform cuboid, whose center of mass is not at the origin of frame
    pub fn new_cuboid_at(
        com: &Vector3<Float>,
        m: Float,
        w: Float,
        d: Float,
        h: Float,
        frame: &str,
    ) -> RigidBody {
        let moment_x = m * (d * d + h * h) / 12.0;
        let moment_y = m * (w * w + h * h) / 12.0;
        let moment_z = m * (w * w + d * d) / 12.0;
        let moment_com = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);

        // generalized parallel axis theorem
        let moment =
            moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
        let cross_part = m * com;
        let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

        RigidBody::new(spatial_inertia)
    }

    pub fn add_cuboid_collider(&mut self, cuboid: Cuboid) {
        self.collider = Some(Collider::new_cuboid(cuboid));
    }

    pub fn add_sphere_collider(&mut self, sphere: Sphere) {
        self.collider = Some(Collider::new_sphere(sphere));
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

    /// Add contact points on the 8 corners of a cuboid
    pub fn add_cuboid_contacts(&mut self, w: Float, d: Float, h: Float) {
        let frame = self.inertia.frame.clone();
        let frame = frame.as_str();
        self.add_contact_point(ContactPoint::new(
            frame,
            vector![-w / 2.0, d / 2.0, h / 2.0],
        ));
        self.add_contact_point(ContactPoint::new(frame, vector![w / 2.0, d / 2.0, h / 2.0]));
        self.add_contact_point(ContactPoint::new(
            frame,
            vector![-w / 2.0, -d / 2.0, h / 2.0],
        ));
        self.add_contact_point(ContactPoint::new(
            frame,
            vector![w / 2.0, -d / 2.0, h / 2.0],
        ));

        self.add_contact_point(ContactPoint::new(
            frame,
            vector![-w / 2.0, d / 2.0, -h / 2.0],
        ));
        self.add_contact_point(ContactPoint::new(
            frame,
            vector![w / 2.0, d / 2.0, -h / 2.0],
        ));
        self.add_contact_point(ContactPoint::new(
            frame,
            vector![-w / 2.0, -d / 2.0, -h / 2.0],
        ));
        self.add_contact_point(ContactPoint::new(
            frame,
            vector![w / 2.0, -d / 2.0, -h / 2.0],
        ));
    }

    pub fn add_cuboid_contacts_with(&mut self, com: &Vector3<Float>, w: Float, d: Float, h: Float) {
        let frame = self.inertia.frame.clone();
        let frame = frame.as_str();
        for i in [-1., 1.].iter() {
            for j in [-1., 1.].iter() {
                for k in [-1., 1.].iter() {
                    self.add_contact_point(ContactPoint::new(
                        frame,
                        com + vector![i * w / 2.0, j * d / 2.0, k * h / 2.0],
                    ));
                }
            }
        }
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
