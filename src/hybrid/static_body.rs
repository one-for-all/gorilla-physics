use na::{Isometry3, Point3, Vector3};

use crate::{
    hybrid::visual::{rigid_mesh::RigidMesh, CuboidGeometry},
    types::Float,
};

pub struct StaticCuboid {
    pub geom: CuboidGeometry,
    pub iso: Isometry3<Float>,
}

impl StaticCuboid {
    pub fn new(w: Float, d: Float, h: Float, iso: Isometry3<Float>) -> Self {
        let geom = CuboidGeometry { w, d, h };
        Self { geom, iso }
    }
}

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
    use na::Isometry3;
    use na::Vector3;
    use rand::rng;
    use rand::Rng;

    use crate::hybrid::static_body::StaticCuboid;
    use crate::hybrid::Hybrid;
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

    #[test]
    fn cuboid_sphere_collision() {
        // Arrange
        let mut state = Hybrid::empty();
        let cuboid_h = 0.1;

        let cuboid_x = 1.0;
        let cuboid_z = 0.5;
        state.add_static_cuboid(StaticCuboid::new(
            1.,
            1.,
            cuboid_h,
            Isometry3::translation(cuboid_x, 0., cuboid_z),
        ));
        let mut rng = rng();

        for _ in 0..5 {
            let r = 0.1;
            let x = cuboid_x + rng.random_range(-0.2..0.2);
            let y = rng.random_range(-0.2..0.2);
            let z = cuboid_z + cuboid_h / 2. + r + rng.random_range((0.)..0.2);
            let sphere = Articulated::new_sphere_at("sphere", 1.0, r, &vector![x, y, z]);
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
            let body_pos = state.articulated[0].bodies[0].pose.translation;
            assert_vec_close!(body_pos, vector![x, y, cuboid_z + cuboid_h / 2. + r], 1e-2);

            // Clean up
            state.pop_articulated();
        }
    }

    #[test]
    fn cuboid_point_collision() {
        // Arrange
        let mut state = Hybrid::empty();
        let cuboid_h = 0.1;

        let cuboid_x = 1.0;
        let cuboid_z = 0.5;
        state.add_static_cuboid(StaticCuboid::new(
            1.,
            1.,
            cuboid_h,
            Isometry3::translation(cuboid_x, 0., cuboid_z),
        ));
        let mut rng = rng();

        for _ in 0..5 {
            let x = cuboid_x + rng.random_range(-0.2..0.2);
            let y = rng.random_range(-0.2..0.2);
            let z = cuboid_z + cuboid_h / 2. + rng.random_range((0.)..0.2);
            let point = Articulated::new_point_at("sphere", 1.0, &vector![x, y, z]);
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
            let body_pos = state.articulated[0].bodies[0].pose.translation;
            assert_vec_close!(body_pos, vector![x, y, cuboid_z + cuboid_h / 2.], 1e-2);

            // Clean up
            state.pop_articulated();
        }
    }
}
