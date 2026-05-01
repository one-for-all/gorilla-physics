use crate::hybrid::{visual::rigid_mesh::RigidMesh, Hybrid, Rigid};

pub struct StaticBody {
    pub mesh: RigidMesh,
}

impl StaticBody {
    pub fn new(mesh: RigidMesh) -> Self {
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
