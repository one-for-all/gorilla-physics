use na::{Isometry, Isometry3, Point3};

use crate::{
    hybrid::{visual::rigid_mesh::RigidMesh, Hybrid, Rigid},
    types::Float,
};

pub struct StaticBody {
    pub mesh: RigidMesh,
}

impl StaticBody {
    pub fn new(mut mesh: RigidMesh, iso: Isometry3<Float>) -> Self {
        for vertex in mesh.vertices.iter_mut() {
            *vertex = iso.transform_point(&Point3::from(*vertex)).coords;
        }
        Self { mesh }
    }
}

#[cfg(test)]
mod static_body_tests {
    use na::vector;
    use na::Vector3;

    use crate::{
        assert_vec_close,
        hybrid::{articulated::Articulated, builders::build_table},
        types::Float,
    };

    #[tokio::test]
    async fn table() {
        // Arrange
        let mut state = build_table().await;

        // Add sphere
        let sphere = Articulated::new_sphere_at("sphere", 1.0, 0.1, &vector![0., 0., 1.2]);
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
    }
}
