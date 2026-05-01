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
    use crate::hybrid::builders::build_table;

    #[tokio::test]
    async fn table() {
        // Arrange
        let table = build_table().await;

        // Act

        // Assert
        println!("{}", table.static_bodies[0].mesh.vertices.len());
    }
}
