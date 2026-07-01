use std::collections::HashMap;

use na::{Isometry3, Vector3};

use crate::{hybrid::visual::rigid_mesh::RigidMesh, types::Float};

pub struct URDFMeshes {
    pub meshes: HashMap<String, Vec<(RigidMesh, Isometry3<Float>, Vector3<Float>)>>,
}

#[cfg(any(target_arch = "wasm32", rust_analyzer))]
use {
    crate::interface::util::{maybe_read_web_file, maybe_read_web_file_gz},
    na::vector,
    na::Translation3,
    na::UnitQuaternion,
    urdf_rs::Robot,
};

impl URDFMeshes {
    pub fn empty() -> Self {
        Self {
            meshes: HashMap::new(),
        }
    }

    #[cfg(any(target_arch = "wasm32", rust_analyzer))]
    pub async fn new(urdf: &Robot) -> Self {
        let mut meshes: HashMap<String, Vec<(RigidMesh, Isometry3<Float>, Vector3<Float>)>> =
            HashMap::new();

        for link in urdf.links.iter() {
            for visual in link.visual.iter() {
                if let urdf_rs::Geometry::Mesh { filename, scale } = &visual.geometry {
                    let path = filename.strip_prefix("package://assets/").unwrap();
                    let base = path.strip_suffix(".stl").unwrap();
                    // Prefer the pre-gzipped mesh (much smaller over the wire,
                    // since hosts don't gzip `.obj`), falling back to the raw
                    // `.obj` for consumers that don't ship `.gz` assets.
                    let gz_fname = format!("mesh/{}.obj.gz", base);
                    let local_fname = format!("mesh/{}.obj", base);
                    let buffer = match maybe_read_web_file_gz(&gz_fname).await {
                        Some(buffer) => Some(buffer),
                        None => maybe_read_web_file(&local_fname).await,
                    };
                    if let Some(buffer) = buffer {
                        let [r, p, y] = visual.origin.rpy.0;
                        let iso = Isometry3::from_parts(
                            Translation3::from(visual.origin.xyz.0),
                            UnitQuaternion::from_euler_angles(r, p, y),
                        );

                        let [r, g, b, _] = visual
                            .material
                            .as_ref()
                            .unwrap()
                            .color
                            .as_ref()
                            .unwrap()
                            .rgba
                            .0;
                        let color = vector![r, g, b];

                        meshes.entry(link.name.clone()).or_default().push((
                            RigidMesh::new_from_obj(&buffer),
                            iso,
                            color,
                        ));
                    }
                }
            }
        }

        Self { meshes }
    }
}
