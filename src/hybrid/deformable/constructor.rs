use std::collections::HashSet;

use itertools::Itertools;
use na::{vector, DVector, Vector3};

use crate::{hybrid::Deformable, types::Float};

/// Constructors for Deformable
impl Deformable {
    pub fn new(
        nodes: Vec<Vector3<Float>>,
        tetrahedra: Vec<Vec<usize>>,
        k: Float,
        mass: Float,
    ) -> Self {
        let dof = nodes.len() * 3;
        let q = DVector::from_iterator(dof, nodes.iter().flat_map(|x| x.iter().copied()));
        let qdot = DVector::zeros(q.len());

        let (faces, edges) = Self::compute_boundary_faces_and_edges(&tetrahedra);

        Deformable {
            nodes,
            tetrahedra,
            faces,
            edges,
            q,
            qdot,
            k: k,
            mass,
        }
    }

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

    pub fn new_tetrahedron() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
        ];
        let tetrahedra = vec![vec![0, 1, 2, 3]];

        Deformable::new(nodes, tetrahedra, 1e5, 1.0)
    }

    pub fn new_pyramid() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
            vector![-1., 0., 0.], // right point
            vector![0., -1., 0.], // front point
        ];
        let tetrahedra = vec![
            vec![0, 1, 2, 3],
            vec![0, 2, 4, 3],
            vec![0, 1, 3, 5],
            vec![0, 3, 4, 5],
        ];
        Self::new(nodes, tetrahedra, 1e5, 1.0)
    }

    pub fn new_octahedron() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
            vector![-1., 0., 0.], // left point
            vector![0., -1., 0.], // front point
            vector![0., 0., -1.], // bottom point
        ];
        let tetrahedra = vec![
            // top half
            vec![0, 1, 2, 3],
            vec![0, 2, 4, 3],
            vec![0, 1, 3, 5],
            vec![0, 3, 4, 5],
            // bottom half
            vec![0, 2, 1, 6],
            vec![0, 4, 2, 6],
            vec![0, 1, 5, 6],
            vec![0, 5, 4, 6],
        ];

        let mass = nodes.len() as Float;
        Self::new(nodes, tetrahedra, 1e5, mass)
    }

    pub fn new_cube(k: Float) -> Self {
        let nodes = vec![
            vector![0., 0., 0.], // base front-left
            vector![1., 0., 0.], // base front-right
            vector![0., 1., 0.], // base back-left
            vector![0., 0., 1.], // top front-left
            vector![1., 1., 0.], // base back-right
            vector![1., 1., 1.], // top back-right
            vector![1., 0., 1.], // top front-right
            vector![0., 1., 1.], // top back-left
        ];
        let tetrahedra = vec![
            vec![0, 1, 2, 3], // bottom front
            vec![4, 2, 1, 5], // bottom back
            vec![3, 6, 1, 5], // front right
            vec![7, 3, 2, 5], // back left
            vec![3, 5, 1, 2], // center tetra
        ];
        // let tetrahedra = vec![
        //     vec![1, 4, 0, 6], // bottom front (right)
        //     vec![0, 4, 2, 7], // bottom back
        //     vec![3, 7, 6, 0], // front left
        //     vec![6, 7, 5, 4], // back right
        //     vec![6, 0, 7, 4], // center
        // ];

        let mass = nodes.len() as Float;
        Self::new(nodes, tetrahedra, k, mass)
    }

    /// size is side length, n is number of little cubes in each dimension
    /// k: Spring constant
    /// m: Mass
    pub fn new_dense_cube(size: Float, n: usize, k: Float, m: Float) -> Self {
        let mut nodes = vec![];
        let w = size / n as Float;
        // iterate over nodes
        for i in 0..n + 1 {
            let i = i as Float;
            for j in 0..n + 1 {
                let j = j as Float;
                for k in 0..n + 1 {
                    let k = k as Float;
                    nodes.push(vector![i * w, j * w, k * w]);
                }
            }
        }

        let mut tetrahedra = vec![];
        // iterate over little cubes
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    let base_node = vector![i, j, k];
                    let offsets = vec![
                        vector![0, 0, 0], // base front-left
                        vector![1, 0, 0], // base front-right
                        vector![0, 1, 0], // base back-left
                        vector![0, 0, 1], // top front-left
                        vector![1, 1, 0], // base back-right
                        vector![1, 1, 1], // top back-right
                        vector![1, 0, 1], // top front-right
                        vector![0, 1, 1], // top back-left
                    ];
                    let nodes: Vec<Vector3<usize>> =
                        offsets.iter().map(|o| o + base_node).collect();
                    let base_tet = if (i + j + k) % 2 == 0 {
                        vec![
                            vec![0, 1, 2, 3], // bottom front
                            vec![4, 2, 1, 5], // bottom back
                            vec![3, 6, 1, 5], // front right
                            vec![7, 3, 2, 5], // back left
                            vec![3, 5, 1, 2], // center tetra
                        ]
                    } else {
                        vec![
                            vec![1, 4, 0, 6], // bottom front (right)
                            vec![0, 4, 2, 7], // bottom back
                            vec![3, 7, 6, 0], // front left
                            vec![6, 7, 5, 4], // back right
                            vec![6, 0, 7, 4], // center
                        ]
                    };
                    let tet = base_tet
                        .iter()
                        .map(|t| {
                            t.iter()
                                .map(|ti| {
                                    let node = nodes[*ti];
                                    let ii = node[0];
                                    let ij = node[1];
                                    let ik = node[2];
                                    let index = ii * (n + 1) * (n + 1) + ij * (n + 1) + ik;
                                    index
                                })
                                .collect::<Vec<usize>>()
                        })
                        .collect::<Vec<Vec<usize>>>();
                    tetrahedra.extend(tet);
                }
            }
        }

        Self::new(nodes, tetrahedra, k, m)
    }

    /// Extract the boundary facets, for better visualization
    /// TODO: extract into a global function
    pub fn compute_boundary_faces_and_edges(
        tetrahedra: &Vec<Vec<usize>>,
    ) -> (Vec<[usize; 3]>, Vec<[usize; 2]>) {
        let mut facets = vec![];
        for tetrahedron in tetrahedra.iter() {
            let v0 = tetrahedron[0];
            let v1 = tetrahedron[1];
            let v2 = tetrahedron[2];
            let v3 = tetrahedron[3];

            facets.push({
                let v = [v1, v2, v3];
                v
            });
            facets.push({
                let v = [v0, v3, v2];
                v
            });
            facets.push({
                let v = [v0, v1, v3];
                v
            });
            facets.push({
                let v = [v0, v2, v1];
                v
            });
        }
        facets.sort_by(|a, b| {
            let mut a_clone = a.clone();
            a_clone.sort();
            let mut b_clone = b.clone();
            b_clone.sort();

            a_clone.cmp(&b_clone)
        });

        // Get the faces that only appear once. They are the boundary faces.
        let mut boundary_facets = vec![];
        for i in 0..facets.len() {
            let cur = facets[i];
            let mut cur_sort = cur.clone();
            cur_sort.sort();

            if i > 0 {
                let mut prev_sort = facets[i - 1].clone();
                prev_sort.sort();
                if cur_sort == prev_sort {
                    continue;
                }
            }

            if i < facets.len() - 1 {
                let mut next_sort = facets[i + 1].clone();
                next_sort.sort();
                if cur_sort == next_sort {
                    continue;
                }
            }
            boundary_facets.push(cur);
        }

        // Compute edges
        let mut edges: HashSet<(usize, usize)> = HashSet::new();
        for face in boundary_facets.iter() {
            let i = face[0];
            let j = face[1];
            let k = face[2];
            let e1 = [i, j];
            let e2 = [j, k];
            let e3 = [k, i];
            for e_candidate in [e1, e2, e3].iter() {
                if edges.contains(&(e_candidate[0], e_candidate[1])) {
                    panic!(
                        "edge cannot be shared by two faces in its same direction: {:?}",
                        e_candidate
                    );
                }

                edges.insert((e_candidate[0], e_candidate[1]));
            }
        }

        let edges: Vec<[usize; 2]> = edges.iter().sorted().map(|e| [e.0, e.1]).collect();

        (boundary_facets, edges)
    }
}
