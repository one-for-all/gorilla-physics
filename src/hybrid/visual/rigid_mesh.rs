use na::{vector, Vector3};

use crate::types::Float;

#[derive(Clone)]
pub struct RigidMesh {
    pub vertices: Vec<Vector3<Float>>,
    pub faces: Vec<[usize; 3]>,
}

impl RigidMesh {
    pub fn new_from_obj(content: &str) -> Self {
        let mut vertices = vec![];
        let mut faces = vec![];
        for line in content.lines() {
            let mut parts = line.split_whitespace();
            if let Some(heading) = parts.next() {
                match heading {
                    "v" => {
                        let x: Float = parts.next().unwrap().parse().unwrap();
                        let y: Float = parts.next().unwrap().parse().unwrap();
                        let z: Float = parts.next().unwrap().parse().unwrap();
                        vertices.push(vector![x, y, z]);
                    }
                    "f" => {
                        // TODO: Each element in face line in obj file could be of
                        // format x/y/z where x is vertex index, y is material
                        // index, and z is vertex normal index. Handle it.
                        let i: usize = parts
                            .next()
                            .unwrap()
                            .split("/")
                            .next()
                            .unwrap()
                            .parse::<usize>()
                            .unwrap()
                            - 1;
                        let j: usize = parts
                            .next()
                            .unwrap()
                            .split("/")
                            .next()
                            .unwrap()
                            .parse::<usize>()
                            .unwrap()
                            - 1;
                        let k: usize = parts
                            .next()
                            .unwrap()
                            .split("/")
                            .next()
                            .unwrap()
                            .parse::<usize>()
                            .unwrap()
                            - 1;
                        faces.push([i, j, k]);
                    }
                    _ => {}
                }
            }
        }

        Self { vertices, faces }
    }
}
