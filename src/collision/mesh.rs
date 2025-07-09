use std::collections::HashMap;

use itertools::Itertools;
use na::{vector, Isometry3, Matrix2, Matrix2x1, Point3, UnitVector3, Vector3};

use crate::types::Float;

/// Mesh collider
#[derive(Clone, PartialEq, Debug)]
pub struct Mesh {
    pub base_vertices: Vec<Vector3<Float>>, // vertex positions expressed in body frame

    pub vertices: Vec<Vector3<Float>>, // vertex positions expressed in world frame
    vertices_outdated: bool, // whether the vertices positions are outdated with regard to body isometry.

    pub faces: Vec<[usize; 3]>,
    pub edges: Vec<([usize; 2], usize, usize)>, // (edge, face A, face B). edge is consistent with A direction, opposite with B direction.

    pub base_isometry: Isometry3<Float>, // The fixed isometry from body frame to mesh frame, initialized at the beginning.
    pub body_isometry: Isometry3<Float>, // Stores the isometry of the body frame to which mesh is attached. So that vertex positions can be updated relatively when body frame moves.
}

impl Mesh {
    /// Read the mesh from a string that is the content of a .mesh file
    pub fn new_from_mesh(content: &str) -> Self {
        let (vertices, tetrahedra) = read_mesh(content);

        // Collect all the faces in the tetrahedra
        // TODO: optimize this
        let mut facets = vec![];
        for tetrahedron in &tetrahedra {
            let v0 = tetrahedron[0];
            let v1 = tetrahedron[1];
            let v2 = tetrahedron[2];
            let v3 = tetrahedron[3];

            facets.push({
                let v = [v1, v2, v3];
                v
            });
            facets.push({
                let v = [v0, v3, v2];
                v
            });
            facets.push({
                let v = [v0, v1, v3];
                v
            });
            facets.push({
                let v = [v0, v2, v1];
                v
            });
        }
        facets.sort_by(|a, b| {
            let mut a_clone = a.clone();
            a_clone.sort();
            let mut b_clone = b.clone();
            b_clone.sort();

            a_clone.cmp(&b_clone)
        });

        // Get the faces that only appear once. They are the boundary faces.
        let mut boundary_facets = vec![];
        for i in 0..facets.len() {
            let cur = facets[i];
            let mut cur_sort = cur.clone();
            cur_sort.sort();

            if i > 0 {
                let mut prev_sort = facets[i - 1].clone();
                prev_sort.sort();
                if cur_sort == prev_sort {
                    continue;
                }
            }

            if i < facets.len() - 1 {
                let mut next_sort = facets[i + 1].clone();
                next_sort.sort();
                if cur_sort == next_sort {
                    continue;
                }
            }
            boundary_facets.push(cur);
        }

        Mesh {
            base_vertices: vertices.clone(),
            vertices,
            vertices_outdated: false,
            faces: boundary_facets,
            edges: vec![],
            base_isometry: Isometry3::identity(),
            body_isometry: Isometry3::identity(),
        }
    }

    pub fn update_base_isometry(&mut self, iso: &Isometry3<Float>) {
        let transform = iso;
        self.base_vertices = self
            .base_vertices
            .iter()
            .map(|v| transform.transform_point(&Point3::from(*v)).coords)
            .collect::<Vec<Vector3<Float>>>();
        self.base_isometry = *iso;

        self.vertices = self.base_vertices.clone();
    }

    pub fn update_vertex_positions(&mut self) {
        if !self.vertices_outdated {
            return;
        }
        self.vertices = self
            .base_vertices
            .iter()
            .map(|v| self.body_isometry.transform_point(&Point3::from(*v)).coords)
            .collect::<Vec<Vector3<Float>>>();
        self.vertices_outdated = false;
    }

    pub fn update_isometry(&mut self, iso: &Isometry3<Float>) {
        self.body_isometry = *iso;
        self.vertices_outdated = true;
    }

    /// Build the mesh from an obj file content
    pub fn new_from_obj(content: &str) -> Self {
        let mut vertices = vec![];
        let mut faces = vec![];
        // Intermediate representation for edges. key is edge, value is (face A, face B).
        let mut edges_map: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
        for line in content.lines() {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 1 {
                continue;
            }
            match parts[0] {
                "v" => {
                    let x: Float = parts[1].parse().unwrap();
                    let y: Float = parts[2].parse().unwrap();
                    let z: Float = parts[3].parse().unwrap();
                    vertices.push(vector![x, y, z]);
                }
                "f" => {
                    // TODO: Each element in face line in obj file could be of
                    // format x/y/z where x is vertex index, y is material
                    // index, and z is vertex normal index. Handle it.
                    let i: usize = parts[1]
                        .split("/")
                        .next()
                        .unwrap()
                        .parse::<usize>()
                        .unwrap()
                        - 1;
                    let j: usize = parts[2]
                        .split("/")
                        .next()
                        .unwrap()
                        .parse::<usize>()
                        .unwrap()
                        - 1;
                    let k: usize = parts[3]
                        .split("/")
                        .next()
                        .unwrap()
                        .parse::<usize>()
                        .unwrap()
                        - 1;
                    faces.push([i, j, k]);

                    // Add edges as well
                    let face_index = faces.len() - 1;
                    let e1 = [i, j];
                    let e2 = [j, k];
                    let e3 = [k, i];
                    for e_candidate in [e1, e2, e3].iter() {
                        if let Some(e) = edges_map.get_mut(&(e_candidate[0], e_candidate[1])) {
                            panic!(
                                "edge cannot be shared by two faces in its same direction: {:?}",
                                e
                            );
                        }
                        if let Some(e) = edges_map.get_mut(&(e_candidate[1], e_candidate[0])) {
                            e.1 = face_index;
                            continue;
                        }
                        // Set face A and face B both to face_index. This face B is placeholder.
                        edges_map
                            .insert((e_candidate[0], e_candidate[1]), (face_index, face_index));
                    }
                }
                _ => {}
            }
        }

        // Check all edges have shared by two different faces
        let edges: Vec<([usize; 2], usize, usize)> = edges_map
            .iter()
            .sorted()
            .map(|(k, v)| {
                let f_A = v.0;
                let f_B = v.1;
                assert_ne!(
                    f_A, f_B,
                    "edge should be shared by two different faces: {}, {}",
                    f_A, f_B
                );
                ([k.0, k.1], f_A, f_B)
            })
            .collect();

        Mesh {
            base_vertices: vertices.clone(),
            vertices: vertices,
            vertices_outdated: false,
            faces: faces,
            edges: edges,
            base_isometry: Isometry3::identity(),
            body_isometry: Isometry3::identity(),
        }
    }
}

/// Computes the (un-normalized) normal of a face. assuming counter-clockwise direction when
/// viewed from outside.
fn compute_normal(face: &[Vector3<Float>; 3]) -> Vector3<Float> {
    let v1 = face[0];
    let v2 = face[1];
    let v3 = face[2];
    let x12 = v2 - v1;
    let x13 = v3 - v1;
    x12.cross(&x13)
}

/// Reads a .mesh file content into a list of vertices & a list of tetrahedra
pub fn read_mesh(content: &str) -> (Vec<Vector3<Float>>, Vec<Vec<usize>>) {
    let mut lines = content.lines();
    let mut num_vertices_to_read = 0;

    // Read number of vertices
    while let Some(line) = lines.next() {
        if line.trim() == "Vertices" {
            let line = lines
                .next()
                .expect("There must be another line after `Vertices`");
            num_vertices_to_read = line.parse().unwrap();
            break;
        }
    }

    // Read vertices
    let vertices: Vec<Vector3<Float>> = (0..num_vertices_to_read)
        .map(|_| {
            let line = lines
                .next()
                .expect("There should still be vertex to be read");
            let parts: Vec<&str> = line.split_whitespace().collect();

            let x: Float = parts[0].parse().unwrap();
            let y: Float = parts[1].parse().unwrap();
            let z: Float = parts[2].parse().unwrap();
            vector![x, -z, y]
        })
        .collect();

    let mut num_tetrahedra_to_read = 0;

    // Read number of tetrahedra
    while let Some(line) = lines.next() {
        if line.trim() == "Tetrahedra" {
            let line = lines
                .next()
                .expect("There must be another line after `Tetrahedra`");
            num_tetrahedra_to_read = line.parse().unwrap();
            break;
        }
    }

    // Read tetrahedra
    let tetrahedra = (0..num_tetrahedra_to_read)
        .map(|_| {
            let line = lines
                .next()
                .expect("There should still be tetrahedron to be read");
            let parts: Vec<&str> = line.split_whitespace().collect();
            let tetrahedron: Vec<usize> = parts
                .iter()
                .take(4)
                .map(|p| p.parse::<usize>().unwrap() - 1)
                .collect::<Vec<_>>()
                .try_into()
                .unwrap();
            tetrahedron
        })
        .collect();

    (vertices, tetrahedra)
}

/// Computes the barycentric coordinates of p projected on the triangle formed
/// by (p, p + u, p + v).
/// Ref: Computing the Barycentric Coordinates of a Projected Point, by Wolfgang
/// Heidrich, 2005
fn projected_barycentric_coord(
    p: &Vector3<Float>,
    q: &Vector3<Float>,
    u: &Vector3<Float>,
    v: &Vector3<Float>,
) -> (Float, Float, Float) {
    let n = u.cross(&v);
    let one_over_4A_squared = 1.0 / n.norm_squared();
    let w = p - q;
    let w3 = u.cross(&w).dot(&n) * one_over_4A_squared;
    let w2 = w.cross(&v).dot(&n) * one_over_4A_squared;
    let w1 = 1. - w2 - w3;
    (w1, w2, w3)
}

/// Returns (contact point, contact normal) if vertex collides with face.
pub fn vertex_face_collision(
    vertex: &Vector3<Float>,
    face: &[Vector3<Float>; 3],
    tol: Float,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let v4 = vertex;
    let v1 = face[0];
    let v2 = face[1];
    let v3 = face[2];
    let x12 = v2 - v1;
    let x13 = v3 - v1;
    let normal = UnitVector3::new_normalize(x12.cross(&x13));

    let distance = (v4 - v1).dot(&normal);
    if distance.abs() > tol {
        return None;
    }

    let (w1, w2, w3) = projected_barycentric_coord(v4, &v1, &x12, &x13);
    if w1.min(w2).min(w3) >= 0.0 && w1.max(w2).max(w3) <= 1.0 {
        return Some((v4.clone(), normal));
    }

    None
}

pub fn edge_edge_collision(
    edgeA: &[Vector3<Float>; 2],
    edgeB: &[Vector3<Float>; 2],
    tol: Float,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let v1 = edgeA[0];
    let v2 = edgeA[1];
    let v3 = edgeB[0];
    let v4 = edgeB[1];

    let x12 = v2 - v1;
    let x34 = v4 - v3;
    let n = x12.cross(&x34);

    // Skip if two edges are parallel
    if n.norm() / (x12.norm() * x34.norm()) < 1e-3 {
        return None;
    }
    let contact_normal = UnitVector3::new_normalize(n);
    let x13 = v3 - v1;

    // No collision if two edges are far away
    let distance = x13.dot(&contact_normal);
    if distance.abs() > tol {
        return None;
    }

    // Formulate and solve the normal equation for two edges, to find the
    // closest point between their line extensions.
    // Ref: Robust Treatment of Collisions, Contact and Friction for Cloth
    // Animation, Bridson and et al., 2002
    let x12_dot_x34 = x12.dot(&x34);
    #[rustfmt::skip]
    let A = Matrix2::new(
        x12.norm_squared(), -x12_dot_x34, 
        -x12_dot_x34, x34.norm_squared()
    );
    let b = Matrix2x1::new(x12.dot(&x13), -x34.dot(&x13));

    let lambda = A
        .lu()
        .solve(&b)
        .expect(&format!("Can not solve A x = b, with A = {}, b = {}", A, b));

    let w1 = lambda[0];
    let w2 = lambda[1];

    // No collision if either point is not on its line segment
    if w1.min(w2) < 0.0 || w1.max(w2) > 1.0 {
        return None;
    }

    // Pick the point on one segment as the contact point
    let contact_point = v1 + w1 * x12;

    // Note: this contact normal is not final. Need to be validated and
    // corrected with face information by correct_contact_normal_direction.
    Some((contact_point, contact_normal))
}

/// Given candidate contact normal, flip the direction if opposite, or report
/// None if invalid. Contact normal should point away from edge.
/// Note: face A has edge direction consistent (i.e. follows right-hand rule),
/// and face B has edge direction opposite.
pub fn correct_contact_normal_direction(
    candidate_contact_normal: &UnitVector3<Float>,
    edge_vec: &Vector3<Float>,
    face_A_normal: &Vector3<Float>,
    face_B_normal: &Vector3<Float>,
) -> Option<UnitVector3<Float>> {
    let mut contact_normal = candidate_contact_normal.clone();
    let side_dir_A = edge_vec.cross(&face_A_normal);
    if contact_normal.dot(&face_A_normal) < 0.0 && contact_normal.dot(&side_dir_A) == 0.0 {
        // Flip if opposite to the face, and on the boundary.
        contact_normal = -contact_normal;
    } else if contact_normal.dot(&side_dir_A) < 0.0 {
        // Flip if towards the inside
        contact_normal = -contact_normal;
    }
    let side_dir_B = -edge_vec.cross(&face_B_normal);
    if contact_normal.dot(&side_dir_B) < 0.0 {
        // Invalid if the corrected normal is towards the inside
        return None;
    }
    return Some(contact_normal);
}

#[cfg(test)]
mod edge_edge_collision_tests {
    use na::{vector, UnitVector3, Vector3};

    use crate::types::Float;

    use super::{correct_contact_normal_direction, edge_edge_collision};

    /// Checks that two unit vectors are either same or opposite
    fn assert_align(a: &UnitVector3<Float>, b: &UnitVector3<Float>) {
        if a != b && *a != -*b {
            panic!("{:?} not aligned with {:?}", a, b);
        }
    }

    #[test]
    fn ee_collision_test1() {
        // Arrange
        let A = [vector![0., 0., 0.], vector![1., 0., 0.]]; // x-axis
        let B = [vector![0., 0., 0.], vector![0., 1., 0.]]; // y-axis

        // Act
        let collision = edge_edge_collision(&A, &B, 1e-3);

        // Assert
        let (cp, n) = collision.expect("should detect collision");
        assert_eq!(cp, vector![0., 0., 0.]); // collide at origin
        assert_align(&n, &Vector3::z_axis());
    }

    #[test]
    fn ee_collision_test2() {
        // Arrange
        let A = [vector![0., 0., 1.], vector![1., 0., 0.]]; // diagonal segment from top to right
        let B = [vector![0.5, -0.5, 0.5], vector![0.5, 0.5, 0.5]]; // towards y-axis

        // Act
        let collision = edge_edge_collision(&A, &B, 1e-3);

        // Assert
        let (cp, n) = collision.expect("should detect collision");
        assert_eq!(cp, vector![0.5, 0., 0.5]);
        let n_expect = UnitVector3::new_normalize(vector![1., 0., 1.]);
        assert_align(&n, &n_expect);
    }

    #[test]
    fn correct_contact_normal_test1() {
        // Arrange
        let contact_normal = -Vector3::z_axis();
        let edge_vec = vector![0., -1., 0.];
        let face_A_normal = vector![0., 0., 1.];
        let face_B_normal = vector![0., 0., 1.];

        // Act
        let n = correct_contact_normal_direction(
            &contact_normal,
            &edge_vec,
            &face_A_normal,
            &face_B_normal,
        );

        // Assert
        let n = n.expect("candidate normal should be valid");
        assert_eq!(n, Vector3::z_axis());
    }
}

/// Returns a list of (contact point, contact normal).
/// Contact normal pointing towards mesh from other_mesh.
pub fn mesh_mesh_collision(
    mesh: &Mesh,
    other_mesh: &Mesh,
    tol: Float,
) -> Vec<(Vector3<Float>, UnitVector3<Float>)> {
    let mut contacts: Vec<(Vector3<Float>, UnitVector3<Float>)> = vec![];

    let vertices = &mesh.vertices;
    let other_vertices = &other_mesh.vertices;

    // Vertices from mesh w/ faces from other mesh
    for vertex in mesh.vertices.iter() {
        for face in other_mesh.faces.iter() {
            let face = [
                other_vertices[face[0]],
                other_vertices[face[1]],
                other_vertices[face[2]],
            ];
            if let Some((contact_point, contact_normal)) = vertex_face_collision(vertex, &face, tol)
            {
                contacts.push((contact_point, contact_normal));
                // vertex not allowed to collide with more than one face
                break;
            }
        }
    }

    // Vertices from other mesh w/ face from mesh
    for vertex in other_vertices.iter() {
        for face in mesh.faces.iter() {
            let face = [vertices[face[0]], vertices[face[1]], vertices[face[2]]];
            if let Some((contact_point, contact_normal)) = vertex_face_collision(vertex, &face, tol)
            {
                // flip the normal direction, since the face is on mesh not other_mesh
                contacts.push((contact_point, -contact_normal));
                // vertex not allowed to collide with more than one face
                break;
            }
        }
    }

    // Edges between two meshes
    for edge in mesh.edges.iter() {
        let e = [vertices[edge.0[0]], vertices[edge.0[1]]];
        for other_edge in other_mesh.edges.iter() {
            let other_e = [
                other_vertices[other_edge.0[0]],
                other_vertices[other_edge.0[1]],
            ];
            if let Some((contact_point, contact_normal)) = edge_edge_collision(&e, &other_e, tol) {
                let other_faces = &other_mesh.faces;
                let f_A = other_faces[other_edge.1];
                let f_B = other_faces[other_edge.2];
                let face_A = [
                    other_vertices[f_A[0]],
                    other_vertices[f_A[1]],
                    other_vertices[f_A[2]],
                ];
                let face_B = [
                    other_vertices[f_B[0]],
                    other_vertices[f_B[1]],
                    other_vertices[f_B[2]],
                ];
                let face_A_normal = compute_normal(&face_A);
                let face_B_normal = compute_normal(&face_B);
                if let Some(contact_normal) = correct_contact_normal_direction(
                    &contact_normal,
                    &(other_e[1] - other_e[0]),
                    &face_A_normal,
                    &face_B_normal,
                ) {
                    let mut contact_exists = false;
                    for (cp, _n) in contacts.iter() {
                        // ignore this edge-edge detection if contact point too
                        // close to existing contact point.
                        // Necessary for stability, as it avoids different contact
                        // normals in a small region.
                        if (contact_point - cp).norm() < tol {
                            contact_exists = true;
                            break;
                        }
                    }

                    if !contact_exists {
                        contacts.push((contact_point, contact_normal));
                        break;
                    }
                }
            }
        }
    }

    contacts
}

#[cfg(test)]
mod mesh_tests {

    use na::{vector, Isometry3, UnitQuaternion, UnitVector3, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        contact::HalfSpace,
        helpers::{
            build_rigid_mesh_box, build_tetrahedron, build_two_cubes, build_two_rigid_mesh_boxes,
            build_two_tetrahedron,
        },
        integrators::Integrator,
        joint::{JointPosition, JointVelocity},
        simulate::step,
        spatial::{pose::Pose, spatial_vector::SpatialVector},
        types::Float,
        util::read_file,
        PI,
    };

    use super::{mesh_mesh_collision, Mesh};

    #[test]
    fn box_mesh_hit_ground() {
        // Arrange
        let file_path = "data/box.mesh";
        let buf = read_file(file_path);
        let mesh = Mesh::new_from_mesh(&buf);

        let l = 2.0; // mesh box width
        let mut state = build_rigid_mesh_box(mesh, l);

        let rot_init = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 2.0);
        let q_init = vec![JointPosition::Pose(Pose {
            rotation: rot_init,
            translation: vector![0.0, 0.0, 2.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0., 0., 0.],
            linear: rot_init.inverse() * vector![1.0, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        let angle = Float::to_radians(-10.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let pose = state.poses()[0];
        assert_close!(pose.translation.dot(&normal), l / 2.0, 1e-2);
        assert_close!(pose.translation.y, 0.0, 1e-2);

        let velocity = state.v[0].spatial(); // final velocity in body frame
        assert_vec_close!(velocity.angular, Vector3::<Float>::zeros(), 1e-5);
        assert_vec_close!(velocity.linear, Vector3::<Float>::zeros(), 1e-5);
    }

    #[test]
    #[ignore] // TODO: mesh to large. make collision checking efficient.
    fn box_mesh_hit_box_mesh() {
        // Arrange
        let file_path = "data/box.mesh";
        let buf = read_file(file_path);
        let mesh = Mesh::new_from_mesh(&buf);

        let l = 2.0; // mesh box width
        let mut state = build_two_rigid_mesh_boxes(mesh, l);

        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, 0.0, l / 2.0],
            }),
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, 0.0, l + 2.0],
            }),
        ];
        state.update_q(&q_init);

        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = vec![];
            let (q, v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
    }

    #[test]
    fn tetra_mesh_hit_ground() {
        // Arrange
        let file_path = "data/tetrahedron.obj";
        let buf = read_file(file_path);
        let mesh = Mesh::new_from_obj(&buf);

        let l = 1.0; // mesh edge length
        let mut state = build_tetrahedron(mesh, l);

        let rot_init = UnitQuaternion::identity();
        let q_init = vec![JointPosition::Pose(Pose {
            rotation: rot_init,
            translation: vector![0.0, 0.0, 2.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0., 0., 0.],
            linear: rot_init.inverse() * vector![1.0, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        let angle = Float::to_radians(10.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let pose = state.poses()[0];
        assert_close!(pose.translation.dot(&normal), 0.0, 1e-2);
        assert_close!(pose.translation.y, 0.0, 1e-2);

        let velocity = state.v[0].spatial(); // final velocity in body frame
        assert_vec_close!(velocity.angular, Vector3::<Float>::zeros(), 1e-5);
        assert_vec_close!(velocity.linear, Vector3::<Float>::zeros(), 1e-5);
    }

    #[test]
    fn tetra_mesh_hit_tetra_mesh() {
        // Arrange
        let file_path = "data/tetrahedron.obj";
        let buf = read_file(file_path);
        let mesh = Mesh::new_from_obj(&buf);

        let l = 1.0; // mesh edge length
        let mut state = build_two_tetrahedron(mesh, l);

        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, 0.0, 0.0],
            }),
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![-l / 2.0, -l / 2.0, 2.0 * l],
            }),
        ];
        state.update_q(&q_init);

        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }

        // Assert
        let tetra1_pose = state.poses()[0];
        assert_close!(tetra1_pose.translation.dot(&normal), 0.0, 1e-2);
        assert_close!(tetra1_pose.translation.y, 0.0, 1e-2);

        let tetra1_velocity = state.v[0].spatial(); // final velocity in body frame
        assert_vec_close!(tetra1_velocity.angular, Vector3::<Float>::zeros(), 1e-5);
        assert_vec_close!(tetra1_velocity.linear, Vector3::<Float>::zeros(), 1e-5);

        state.update_collidable_mesh_vertex_positions();

        let contacts = mesh_mesh_collision(
            &state.bodies[0]
                .collider
                .as_ref()
                .expect("body 0")
                .geometry
                .mesh(),
            &state.bodies[1]
                .collider
                .as_ref()
                .expect("body 1")
                .geometry
                .mesh(),
            1e-2,
        );
        assert_eq!(contacts.len(), 0);
    }

    #[test]
    fn tetra_tetra_collision1() {
        // Arrange
        let l = 1.0;
        let file_path = "data/tetrahedron.obj";
        let buf = read_file(file_path);

        let meshA = Mesh::new_from_obj(&buf);
        let mut meshB = meshA.clone();
        meshB.update_isometry(&Isometry3::translation(0., 0., l));

        // Act
        let contacts = mesh_mesh_collision(&meshA, &meshB, 1e-2);

        // Assert
        assert!(contacts.len() > 0);
    }

    #[test]
    fn tetra_tetra_collision2() {
        // Arrange
        let l = 1.0;
        let file_path = "data/tetrahedron.obj";
        let buf = read_file(file_path);

        let meshA = Mesh::new_from_obj(&buf);
        let mut meshB = meshA.clone();
        meshB.update_isometry(&Isometry3::translation(l / 2.0, 0., l / 2.0));

        // Act
        let contacts = mesh_mesh_collision(&meshA, &meshB, 1e-2);

        // Assert
        assert!(contacts.len() > 0);
    }

    #[test]
    fn tetra_tetra_collision3() {
        // Arrange
        let l = 1.0;
        let file_path = "data/tetrahedron.obj";
        let buf = read_file(file_path);

        let meshA = Mesh::new_from_obj(&buf);
        let mut meshB = meshA.clone();
        meshB.update_isometry(&Isometry3::translation(l / 2.0, -l / 2.0, l / 2.0));
        meshB.update_vertex_positions();

        // Act
        let contacts = mesh_mesh_collision(&meshA, &meshB, 1e-2);

        // Assert
        assert_eq!(contacts.len(), 1);
    }

    #[test]
    fn tetra_tetra_collision4() {
        // Arrange
        let l = 1.0;
        let buf = read_file("data/tetrahedron.obj");
        let meshA = Mesh::new_from_obj(&buf);
        let mut meshB = meshA.clone();
        meshB.update_isometry(&Isometry3::new(
            vector![l / 2.0, -l / 2.0, l],
            Vector3::y_axis().scale(PI / 2.0),
        ));
        meshB.update_vertex_positions();

        // Act
        let contacts = mesh_mesh_collision(&meshA, &meshB, 1e-3);

        // Arrange
        assert_eq!(contacts.len(), 1);
    }

    #[test]
    #[ignore] // Not an official test. For sanity checking only now.
    fn cube_cube_collision1() {
        // Arrange
        let file_path = "data/cube.obj";
        let buf = read_file(file_path);
        let mesh = Mesh::new_from_obj(&buf);

        let l = 1.0; // mesh edge length
        let mut state = build_two_cubes(mesh, l);

        let rot_init = UnitQuaternion::from_euler_angles(PI / 4.0, PI / 4.0, 0.0);
        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, 0.0, 0.0],
            }),
            JointPosition::Pose(Pose {
                rotation: rot_init,
                translation: vector![0.0, 0.0, 2.0 * l],
            }),
        ];
        state.update_q(&q_init);

        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let ground = HalfSpace::new(normal, 0.0);
        state.add_halfspace(ground);

        // Act
        let final_time = 2.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let torque = vec![];
            let (_q, _v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);
        }
    }
}
