use na::{Isometry3, Point3, Vector3};

use crate::{
    hybrid::visual::rigid_mesh::RigidMesh,
    types::Float,
};

pub struct StaticBody {
    pub mesh: RigidMesh,
    pub show_visual: bool,
}

impl StaticBody {
    pub fn new(mut mesh: RigidMesh, iso: Isometry3<Float>) -> Self {
        for vertex in mesh.vertices.iter_mut() {
            *vertex = iso.transform_point(&Point3::from(*vertex)).coords;
        }
        Self {
            mesh,
            show_visual: true,
        }
    }

    pub fn scale(&mut self, scale: Vector3<Float>) {
        for vertex in self.mesh.vertices.iter_mut() {
            *vertex = vertex.component_mul(&scale);
        }
    }

    pub fn update_pose(&mut self, iso: Isometry3<Float>) {
        for vertex in self.mesh.vertices.iter_mut() {
            *vertex = iso.transform_point(&Point3::from(*vertex)).coords;
        }
    }
}

#[cfg(test)]
mod static_body_tests {
    use na::vector;
    use na::Vector3;
    use rand::rng;
    use rand::Rng;

    use crate::hybrid::Hybrid;
    use crate::util::test_utils::random_vector3;
    use crate::{
        assert_vec_close,
        hybrid::{articulated::Articulated, builders::import_static_body},
        types::Float,
    };

    #[tokio::test]
    async fn table_sphere_collision() {
        // Arrange
        let mut state = Hybrid::empty();
        state.add_static_body(import_static_body("table/table.obj").await);
        let mut rng = rng();

        for _ in 0..5 {
            let x = rng.random_range(-0.5..0.5);
            let y = rng.random_range(-0.2..0.2);
            let sphere = Articulated::new_sphere_at("sphere", 1.0, 0.1, &vector![x, y, 1.2]);
            state.add_articulated(sphere);

            // Act
            let final_time = 0.5;
            let dt = 1e-3;
            let num_steps = (final_time / dt) as usize;
            for _s in 0..num_steps {
                state.step(dt, &vec![]);
            }

            // Assert
            let body_v = state.articulated[0].body_twists()[0];
            assert_vec_close!(body_v.linear, Vector3::<Float>::zeros(), 1e-3);

            // Clean up
            state.pop_articulated();
        }
    }

    #[tokio::test]
    async fn table_point_collision() {
        // Arrange
        let mut state = Hybrid::empty();
        state.add_static_body(import_static_body("table/table.obj").await);
        let mut rng = rng();

        for _ in 0..5 {
            let x = rng.random_range(-0.5..0.5);
            let y = rng.random_range(-0.2..0.2);
            let point = Articulated::new_point_at("point", 1.0, &vector![x, y, 1.2]);
            state.add_articulated(point);

            // Act
            let final_time = 0.5;
            let dt = 1e-3;
            let num_steps = (final_time / dt) as usize;
            for _s in 0..num_steps {
                state.step(dt, &vec![]);
            }

            // Assert
            let body_v = state.articulated[0].body_twists()[0];
            assert_vec_close!(body_v.linear, Vector3::<Float>::zeros(), 1e-3);

            // Clean up
            state.pop_articulated();
        }
    }

    #[tokio::test]
    async fn table_cuboid_collision() {
        // Arrange
        let mut state = Hybrid::empty();
        state.add_static_body(import_static_body("table/table.obj").await);
        let mut rng = rng();

        for _ in 0..5 {
            let x = rng.random_range(-0.5..0.5);
            let y = rng.random_range(-0.2..0.2);
            let cube = Articulated::new_cube_at("cube", 1.0, 0.1, &vector![x, y, 1.2]);
            state.add_articulated(cube);

            // Act
            let final_time = 0.5;
            let dt = 1e-3;
            let num_steps = (final_time / dt) as usize;
            for _s in 0..num_steps {
                state.step(dt, &vec![]);
            }

            // Assert
            let body_v = state.articulated[0].body_twists()[0];
            assert_vec_close!(body_v.linear, Vector3::<Float>::zeros(), 1e-3);

            // Clean up
            state.pop_articulated();
        }
    }
}
