use na::{vector, Isometry3, Point3, Vector3};

use crate::types::Float;

/// Mesh collider
#[derive(Clone, PartialEq, Debug)]
pub struct Mesh {
    pub vertices: Vec<Vector3<Float>>,
    pub faces: Vec<[usize; 3]>, // TODO: consistent boundary faces outward orientation

    pub body_isometry: Isometry3<Float>, // Stores the isometry of the body frame to which mesh is attached. So that vertex positions can be updated relatively when body frame moves.
}

impl Mesh {
    /// Read the mesh from a string that is the content of a .mesh file
    pub fn new_from_str(content: &str) -> Self {
        let (vertices, tetrahedra) = read_mesh(content);

        // Collect all the faces in the tetrahedra
        let mut facets = vec![];
        for tetrahedron in &tetrahedra {
            let v0 = tetrahedron[0];
            let v1 = tetrahedron[1];
            let v2 = tetrahedron[2];
            let v3 = tetrahedron[3];

            facets.push({
                let mut v = [v0, v1, v2];
                v.sort();
                v
            });
            facets.push({
                let mut v = [v0, v1, v3];
                v.sort();
                v
            });
            facets.push({
                let mut v = [v0, v2, v3];
                v.sort();
                v
            });
            facets.push({
                let mut v = [v1, v2, v3];
                v.sort();
                v
            });
        }
        facets.sort();

        // Get the faces that only appear once. They are the boundary faces.
        let mut boundary_facets = vec![];
        for i in 0..facets.len() {
            let cur = facets[i];
            if i > 0 && cur == facets[i - 1] {
                continue;
            }
            if i < facets.len() - 1 && cur == facets[i + 1] {
                continue;
            }
            boundary_facets.push(cur);
        }

        Mesh {
            vertices,
            faces: boundary_facets,
            body_isometry: Isometry3::identity(),
        }
    }

    pub fn update_isometry(&mut self, iso: &Isometry3<Float>) {
        let transform = iso * self.body_isometry.inverse();
        self.vertices = self
            .vertices
            .iter()
            .map(|v| transform.transform_point(&Point3::from(*v)).coords)
            .collect::<Vec<Vector3<Float>>>();
        self.body_isometry = *iso;
    }
}

/// Read a .mesh file content into list of vertices & list of tetrahedra
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

#[cfg(test)]
mod mesh_tests {

    use na::{vector, UnitQuaternion, UnitVector3, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        contact::HalfSpace,
        helpers::build_rigid_mesh_box,
        integrators::Integrator,
        joint::{JointPosition, JointVelocity},
        simulate::step,
        spatial::{pose::Pose, spatial_vector::SpatialVector},
        types::Float,
        util::read_file,
        PI,
    };

    use super::Mesh;

    #[test]
    fn rigid_box_hit_ground() {
        // Arrange
        let file_path = "data/box.mesh";
        let buf = read_file(file_path);
        let mesh = Mesh::new_from_str(&buf);

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
}
