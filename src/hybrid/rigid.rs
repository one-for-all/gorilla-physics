use na::{dvector, vector, DMatrix, DVector, Isometry3, Matrix3, UnitVector3, Vector3};

use crate::{
    collision::mesh::vertex_face_collision,
    flog,
    hybrid::{
        visual::{SphereGeometry, Visual},
        Deformable,
    },
    inertia::SpatialInertia,
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    types::Float,
    GRAVITY, WORLD_FRAME,
};

/// Rigid body
pub struct Rigid {
    pub inertia: SpatialInertia,
    pub pose: Pose,           // TODO: use Iso instead of Pose?
    pub twist: SpatialVector, // body velocity expressed in world frame

    pub visual: Vec<(Visual, Isometry3<Float>)>, // geometry and the isometry from geometry frame to body frame
}

impl Rigid {
    /// Assuming self.inertia is in body frame, return the inertia expressed in world frame
    pub fn inertia_in_world_frame(&self) -> SpatialInertia {
        let R = self.pose.rotation.to_rotation_matrix();
        let p = self.pose.translation;

        let inertia = &self.inertia;
        let J = inertia.moment;
        let mc = inertia.cross_part;
        let m = inertia.mass;

        let Rmc = R * mc;
        let mp = m * p;
        let mcnew = Rmc + mp;
        let X = Rmc * p.transpose();
        let Y = X + X.transpose() + mp * p.transpose();
        let Jnew = R * J * R.transpose() - Y + Y.trace() * DMatrix::identity(Y.nrows(), Y.ncols());

        SpatialInertia {
            frame: WORLD_FRAME.to_string(),
            moment: Jnew,
            cross_part: mcnew,
            mass: m,
        }
    }

    pub fn new(inertia: SpatialInertia) -> Self {
        Rigid {
            inertia,
            pose: Pose::identity(),
            twist: SpatialVector::zero(),
            visual: vec![],
        }
    }

    pub fn new_sphere() -> Self {
        let m = 1.0;
        let r = 1.0;
        let moment = 2. / 5. * m * r * r;
        let moment = Matrix3::from_diagonal_element(moment);
        let cross = Vector3::zeros();
        let inertia = SpatialInertia::new(moment, cross, m, "sphere");
        Rigid::new(inertia)
    }

    pub fn new_sphere_at(com: &Vector3<Float>, m: Float, r: Float, frame: &str) -> Self {
        let moment = 2. / 5. * m * r * r;
        let moment_com = Matrix3::from_diagonal_element(moment);

        // generalized parallel axis theorem
        let moment =
            moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
        let cross_part = m * com;
        let inertia = SpatialInertia::new(moment, cross_part, m, frame);

        let mut rigid = Rigid::new(inertia);
        let iso = Isometry3::translation(com.x, com.y, com.z);
        rigid
            .visual
            .push((Visual::Sphere(SphereGeometry { r }), iso));
        rigid
    }

    /// Create a uniform cuboid, whose center of mass is not at the origin of frame
    pub fn new_cuboid_at(
        com: &Vector3<Float>,
        m: Float,
        w: Float,
        d: Float,
        h: Float,
        frame: &str,
    ) -> Rigid {
        let moment_x = m * (d * d + h * h) / 12.0;
        let moment_y = m * (w * w + h * h) / 12.0;
        let moment_z = m * (w * w + d * d) / 12.0;
        let moment_com = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);

        // generalized parallel axis theorem
        let moment =
            moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
        let cross_part = m * com;
        let inertia = SpatialInertia::new(moment, cross_part, m, frame);

        let mut rigid = Rigid::new(inertia);
        let iso = Isometry3::translation(com.x, com.y, com.z);
        rigid.visual.push((Visual::new_cuboid(w, d, h), iso));
        rigid
    }

    /// free-motion velocity in body frame
    pub fn free_velocity(&self, _dt: Float) -> DVector<Float> {
        let mut v_free = dvector![];
        v_free.extend(self.twist.angular.iter().cloned());
        v_free.extend(self.twist.linear.iter().cloned());
        assert_eq!(v_free.len(), 6);
        v_free
    }

    /// Computes linear momentum in world frame
    pub fn linear_momentum(&self) -> Vector3<Float> {
        let linear = self.inertia.mass * self.twist.linear
            - self.inertia.cross_part.cross(&self.twist.angular);
        self.pose.rotation * linear
    }

    pub fn potential_energy(&self) -> Float {
        let R = self.pose.rotation;
        let p = self.pose.translation;
        let cross = self.inertia.cross_part; // mass * com
        let mass = self.inertia.mass;
        GRAVITY * (mass * p.z + (R * cross).z)
    }

    pub fn kinetic_energy(&self) -> Float {
        let inertia = self.inertia_in_world_frame();
        let twist = self.twist;

        let w = twist.angular;
        let v = twist.linear;
        let J = inertia.moment;
        let c = inertia.cross_part;
        let m = inertia.mass;

        (w.dot(&(J * w)) + v.dot(&(m * v + 2.0 * w.cross(&c)))) / 2.0
    }
}

/// Perform collision detection between a rigid body and a deformable
/// Returns a vec of (contact point, contact normal, deformable node weights)
/// deformable node weights is a vec of involved node indices and their weights
/// Note: contact normal point towars the rigid body
pub fn rigid_deformable_cd(
    rigid: &Rigid,
    deformable: &Deformable,
) -> Vec<(Vector3<Float>, UnitVector3<Float>, Vec<(usize, Float)>)> {
    let mut result = vec![];
    for (collider, iso_collider_to_body) in rigid.visual.iter() {
        let iso = rigid.pose.to_isometry() * iso_collider_to_body;
        let collider_pos = iso.translation.vector;

        match collider {
            Visual::Sphere(sphere) => {
                for (i_node, node_pos) in deformable.get_positions().iter().enumerate() {
                    if (node_pos - collider_pos).norm() < sphere.r {
                        let n = UnitVector3::new_normalize(collider_pos - node_pos);
                        let cp = node_pos;
                        result.push((*cp, n, vec![(i_node, 1.)]));
                    }
                }
            }
            Visual::Cuboid(cuboid) => {
                let cuboid_points = cuboid.points(&iso);
                let deformable_faces: &Vec<[usize; 3]> = &deformable.faces;
                let nodes = deformable.get_positions();
                for point in cuboid_points.iter() {
                    for face in deformable_faces.iter() {
                        let face_coords: [Vector3<Float>; 3] = face.map(|f| nodes[f]);
                        if let Some((cp, n, ws)) = vertex_face_collision(point, &face_coords, 1e-3)
                        {
                            let node_weights =
                                vec![(face[0], ws[0]), (face[1], ws[1]), (face[2], ws[2])];
                            result.push((cp, n, node_weights));
                            break; // each point can have collision w/ only one face
                        }
                    }
                }
            }
        }
    }
    result
}
