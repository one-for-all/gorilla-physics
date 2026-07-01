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
        use futures::future::join_all;

        // Collect the unique mesh files referenced by the URDF. Many visuals
        // point at the same mesh (e.g. the four identical leg servos), so we
        // fetch each file only once.
        let mut bases: Vec<String> = Vec::new();
        for link in urdf.links.iter() {
            for visual in link.visual.iter() {
                if let urdf_rs::Geometry::Mesh { filename, .. } = &visual.geometry {
                    let path = filename.strip_prefix("package://assets/").unwrap();
                    let base = path.strip_suffix(".stl").unwrap().to_string();
                    if !bases.contains(&base) {
                        bases.push(base);
                    }
                }
            }
        }

        // Fetch all unique meshes concurrently rather than one at a time. Each
        // fetch prefers the pre-gzipped `.obj.gz` (much smaller over the wire,
        // since hosts don't gzip `.obj`), falling back to the raw `.obj` for
        // consumers that don't ship `.gz` assets.
        let buffers: Vec<Option<String>> = join_all(bases.iter().map(|base| async move {
            let gz_fname = format!("mesh/{}.obj.gz", base);
            match maybe_read_web_file_gz(&gz_fname).await {
                Some(buffer) => Some(buffer),
                None => {
                    let local_fname = format!("mesh/{}.obj", base);
                    maybe_read_web_file(&local_fname).await
                }
            }
        }))
        .await;

        let buffer_by_base: HashMap<&str, &str> = bases
            .iter()
            .zip(buffers.iter())
            .filter_map(|(base, buffer)| buffer.as_deref().map(|b| (base.as_str(), b)))
            .collect();

        let mut meshes: HashMap<String, Vec<(RigidMesh, Isometry3<Float>, Vector3<Float>)>> =
            HashMap::new();

        for link in urdf.links.iter() {
            for visual in link.visual.iter() {
                if let urdf_rs::Geometry::Mesh { filename, .. } = &visual.geometry {
                    let path = filename.strip_prefix("package://assets/").unwrap();
                    let base = path.strip_suffix(".stl").unwrap();
                    if let Some(&buffer) = buffer_by_base.get(base) {
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
                            RigidMesh::new_from_obj(buffer),
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
