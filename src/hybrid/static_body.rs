use na::Isometry3;

use crate::{
    hybrid::{visual::rigid_mesh::RigidMesh, Hybrid, Rigid},
    types::Float,
};

pub struct StaticBody {
    pub mesh: RigidMesh,
    pub iso: Isometry3<Float>,
}

impl StaticBody {
    pub fn new(mesh: RigidMesh, iso: Isometry3<Float>) -> Self {
        Self { mesh, iso }
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
