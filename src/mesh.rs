use na::{vector, Vector3};

use crate::types::Float;

/// Read a .mesh file content into list of vertices & list of tetrahedra
pub fn read_mesh(file: &str) -> (Vec<Vector3<Float>>, Vec<Vec<usize>>) {
    let mut lines = file.lines();
    let mut num_vertices_to_read = 0;

    // Read vertices number
    while let Some(line) = lines.next() {
        if line.trim() == "Vertices" {
            let line = lines
                .next()
                .expect("There must be another line after `Vertices`");
            num_vertices_to_read = line.parse().unwrap();
            break;
        }
    }

    // Read vertices
    let vertices: Vec<Vector3<Float>> = (0..num_vertices_to_read)
        .map(|_| {
            let line = lines
                .next()
                .expect("There should still be vertex to be read");
            let parts: Vec<&str> = line.split_whitespace().collect();

            let x: Float = parts[0].parse().unwrap();
            let y: Float = parts[1].parse().unwrap();
            let z: Float = parts[2].parse().unwrap();
            vector![x, -z, y]
        })
        .collect();

    let mut num_tetrahedra_to_read = 0;

    // Read tetrahedra number
    while let Some(line) = lines.next() {
        if line.trim() == "Tetrahedra" {
            let line = lines
                .next()
                .expect("There must be another line after `Tetrahedra`");
            num_tetrahedra_to_read = line.parse().unwrap();
            break;
        }
    }

    // Read tetrahedra
    let tetrahedra = (0..num_tetrahedra_to_read)
        .map(|_| {
            let line = lines
                .next()
                .expect("There should still be tetrahedron to be read");
            let parts: Vec<&str> = line.split_whitespace().collect();
            let tetrahedron: Vec<usize> = parts
                .iter()
                .take(4)
                .map(|p| p.parse::<usize>().unwrap() - 1)
                .collect::<Vec<_>>()
                .try_into()
                .unwrap();
            tetrahedron
        })
        .collect();

    (vertices, tetrahedra)
}
