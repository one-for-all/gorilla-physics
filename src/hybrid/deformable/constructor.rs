use na::Vector3;

use crate::{hybrid::Deformable, types::Float};

/// Constructors for Deformable
impl Deformable {
    pub fn new_from_vtk(buf: &str, k: Float, mass: Float) -> Self {
        let mut lines = buf.lines();
        let mut nodes = vec![];
        let mut n_nodes: usize = 0;

        while let Some(line) = lines.next() {
            if line.starts_with("POINTS") {
                n_nodes = line.split_whitespace().collect::<Vec<&str>>()[1]
                    .parse()
                    .unwrap();
                break;
            }
        }

        for _ in 0..n_nodes {
            let parts: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
            let x: Float = parts[0].parse().unwrap();
            let y: Float = parts[1].parse().unwrap();
            let z: Float = parts[2].parse().unwrap();
            nodes.push(Vector3::new(x, y, z));
        }

        let mut tetrahedra: Vec<Vec<usize>> = vec![];
        let mut n_tetra: usize = 0;

        while let Some(line) = lines.next() {
            if line.starts_with("CELLS") {
                n_tetra = line.split_whitespace().collect::<Vec<&str>>()[1]
                    .parse()
                    .unwrap();
                break;
            }
        }

        for _ in 0..n_tetra {
            let parts: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
            let n0: usize = parts[1].parse().unwrap();
            let n1: usize = parts[2].parse().unwrap();
            let n2: usize = parts[3].parse().unwrap();
            let n3: usize = parts[4].parse().unwrap();
            tetrahedra.push(vec![n0, n1, n2, n3]);
        }

        Self::new(nodes, tetrahedra, k, mass)
    }
}
