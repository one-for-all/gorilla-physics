use itertools::izip;
use na::{dvector, vector, DMatrix, DVector, Isometry3, Matrix3, UnitVector3, Vector3};

use crate::{
    collision::{
        ccd::{
            accd::edge_edge::{edge_edge_accd, edge_edge_contact},
            edge_edge::edge_edge_ccd,
        },
        mesh::vertex_face_collision,
    },
    hybrid::{
        cloth::Cloth,
        visual::{rigid_mesh::RigidMesh, vertex_rect_face_collision, SphereGeometry, Visual},
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

    pub visual: Vec<(Visual, Isometry3<Float>, Option<Vector3<Float>>)>, // geometry and the isometry from geometry frame to body frame, and RGB color
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
            .push((Visual::Sphere(SphereGeometry { r }), iso, None));
        rigid
    }

    pub fn new_sphere(m: Float, r: Float, frame: &str) -> Self {
        Self::new_sphere_at(&Vector3::zeros(), m, r, frame)
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
        let inertia = SpatialInertia::cuboid_at(com, m, w, d, h, frame);

        let mut rigid = Rigid::new(inertia);
        let iso = Isometry3::translation(com.x, com.y, com.z);
        rigid.visual.push((Visual::new_cuboid(w, d, h), iso, None));
        rigid
    }

    pub fn new_cuboid(m: Float, w: Float, d: Float, h: Float, frame: &str) -> Rigid {
        Self::new_cuboid_at(&vector![0., 0., 0.], m, w, d, h, frame)
    }

    pub fn add_cuboid_at(&mut self, com: &Vector3<Float>, m: Float, w: Float, d: Float, h: Float) {
        let inertia = SpatialInertia::cuboid_at(&com, m, w, d, h, &self.inertia.frame);
        self.inertia += &inertia;

        let iso = Isometry3::translation(com.x, com.y, com.z);
        self.visual.push((Visual::new_cuboid(w, d, h), iso, None));
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
/// Note: contact normal point outwards of the deformable
pub fn rigid_deformable_cd(
    rigid: &Rigid,
    deformable: &Deformable,
    body_twist: &SpatialVector,
    v_deformable: &DVector<Float>,
    dt: Float,
) -> Vec<(Vector3<Float>, UnitVector3<Float>, Vec<(usize, Float)>)> {
    let mut result = vec![];
    for (collider, iso_collider_to_body, _color) in rigid.visual.iter() {
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
                let nodes = deformable.get_positions();
                let deformable_faces: &Vec<[usize; 3]> = &deformable.faces;
                let deformable_face_coords: Vec<[Vector3<Float>; 3]> = deformable_faces
                    .iter()
                    .map(|f| f.map(|fi| nodes[fi]))
                    .collect();

                let deformable_edges = &deformable.edges;
                let deformable_edge_coords: Vec<[Vector3<Float>; 2]> = deformable_edges
                    .iter()
                    .map(|e| e.map(|ei| nodes[ei]))
                    .collect();

                // cube point - deformable face
                for point in cuboid_points.iter() {
                    for (face, face_coords) in
                        izip!(deformable_faces.iter(), deformable_face_coords.iter())
                    {
                        if let Some((cp, n, ws)) = vertex_face_collision(point, &face_coords, 1e-2)
                        {
                            let node_weights =
                                vec![(face[0], ws[0]), (face[1], ws[1]), (face[2], ws[2])];
                            result.push((cp, n, node_weights));
                            // break; // each point can have collision w/ only one face
                        }
                    }
                }

                // cube face - deformable point
                let cuboid_faces = cuboid.faces(&iso);
                for face in cuboid_faces.iter() {
                    for (i, point) in nodes.iter().enumerate() {
                        if let Some((cp, n)) = vertex_rect_face_collision(point, face, 1e-2) {
                            result.push((cp, -n, vec![(i, 1.)])); // reverse normal to point outwards deformable
                        }
                    }
                }

                // edge - edge
                let cuboid_edges = cuboid.edges(&iso);
                for cuboid_edge in cuboid_edges.iter() {
                    // TODO(ccd): technically the rigid body edge points undergoes twist, not just linear velocity. Here we do linear velocity for simplicity, and under small dt, they should be close.
                    // Computes the end points linear velocity
                    let v3 = body_twist.linear + body_twist.angular.cross(&cuboid_edge[0]);
                    let v4 = body_twist.linear + body_twist.angular.cross(&cuboid_edge[1]);

                    let e2 = cuboid_edge;
                    let eb0_t0 = &e2[0];
                    let eb1_t0 = &e2[1];
                    let eb0_t1 = eb0_t0 + v3 * dt;
                    let eb1_t1 = eb1_t0 + v4 * dt;

                    for (i_edge, deformable_e_coords) in deformable_edge_coords.iter().enumerate() {
                        let e1 = deformable_e_coords;

                        let edge = deformable_edges[i_edge];
                        let v1: Vector3<Float> = v_deformable.fixed_rows::<3>(edge[0] * 3).into();
                        let v2: Vector3<Float> = v_deformable.fixed_rows::<3>(edge[1] * 3).into();

                        let ea0_t0 = &e1[0];
                        let ea1_t0 = &e1[1];
                        let ea0_t1 = ea0_t0 + v1 * dt;
                        let ea1_t1 = ea1_t0 + v2 * dt;

                        let toi = edge_edge_accd(
                            ea0_t0, ea1_t0, eb0_t0, eb1_t0, &ea0_t1, &ea1_t1, &eb0_t1, &eb1_t1,
                            1e-3, 1.0,
                        );
                        if let Some(toi) = toi {
                            let (cp, n, w1s, _w2s) = edge_edge_contact(
                                ea0_t0, ea1_t0, eb0_t0, eb1_t0, &ea0_t1, &ea1_t1, &eb0_t1, &eb1_t1,
                                toi,
                            );

                            let node_weights = vec![(edge[0], w1s[0]), (edge[1], w1s[1])];
                            result.push((cp, n, node_weights));
                        }
                    }
                }
            }
            Visual::RigidMesh(_) => {
                panic!("collision detection between rigid mesh and deformable is not implemented")
            }
        }
    }
    result
}

pub fn rigid_cloth_ccd(
    rigid: &Rigid,
    cloth: &Cloth,
    body_twist: &SpatialVector,
    v_cloth: &DVector<Float>,
    dt: Float,
) -> Vec<(Vector3<Float>, UnitVector3<Float>, Vec<(usize, Float)>)> {
    let mut result = vec![];
    for (collider, iso_collider_to_body, _color) in rigid.visual.iter() {
        let iso = rigid.pose.to_isometry() * iso_collider_to_body;

        match collider {
            Visual::Sphere(_) => {}
            Visual::Cuboid(cuboid) => {
                let cuboid_points = cuboid.points(&iso);
                let nodes = cloth.get_positions();
                let cloth_faces = &cloth.faces;
                let cloth_face_coords: Vec<[Vector3<Float>; 3]> =
                    cloth_faces.iter().map(|f| f.map(|fi| nodes[fi])).collect();

                let cloth_edges = &cloth.edges;
                let cloth_edge_coords: Vec<[Vector3<Float>; 2]> =
                    cloth_edges.iter().map(|e| e.map(|ei| nodes[ei])).collect();

                // // cuboid point - cloth face ccd
                // for point in cuboid_points.iter() {
                //     for (face, face_coords) in izip!(cloth_faces.iter(), cloth_face_coords.iter()) {
                //         if let Some((cp, n, ws)) = point_face_ccd(point, face_coords, dt) {
                //             let node_weights =
                //                 vec![(face[0], ws[0]), (face[1], ws[1]), (face[2], ws[2])];
                //             result.push((cp, n, node_weights));
                //         }
                //     }
                // }

                // edge - edge ccd
                let cuboid_edges = cuboid.edges(&iso);
                for cuboid_edge in cuboid_edges.iter() {
                    // Computes the end points linear velocity
                    let v3 = body_twist.linear + body_twist.angular.cross(&cuboid_edge[0]);
                    let v4 = body_twist.linear + body_twist.angular.cross(&cuboid_edge[1]);

                    for (i_edge, cloth_e_coords) in cloth_edge_coords.iter().enumerate() {
                        let e2 = cuboid_edge;
                        let e1 = cloth_e_coords;

                        let edge = cloth_edges[i_edge];
                        let v1 = v_cloth.fixed_rows::<3>(edge[0] * 3).into();
                        let v2 = v_cloth.fixed_rows::<3>(edge[1] * 3).into();

                        // Note: if edges have passed through each other already, normal would be opposite.
                        if let Some((cp, n, ws, _)) = edge_edge_ccd(e1, e2, &v1, &v2, &v3, &v4, dt)
                        {
                            // TODO(ccd): this CCD still not fail proof.
                            let node_weights = vec![(edge[0], ws[0]), (edge[1], ws[1])];
                            result.push((cp, n, node_weights));
                        }
                    }
                }
            }
            Visual::RigidMesh(_) => {
                panic!("collision detection between rigid mesh and deformable is not implemented")
            }
        }
    }
    result
}
