use std::collections::HashSet;

use crate::{
    collision::{
        ccd::{
            accd::{
                edge_edge::{edge_edge_accd, edge_edge_contact},
                point_triangle::{point_triangle_accd, point_triangle_contact},
            },
            edge_edge::{self, edge_edge_ccd},
            point_face_ccd,
        },
        mesh::{edge_edge_collision, vertex_face_collision},
    },
    flog,
    types::Float,
    GRAVITY,
};
use itertools::{izip, Itertools};
use na::{vector, DMatrix, DVector, UnitVector3, Vector3};
use nalgebra_sparse::{CooMatrix, CsrMatrix};

/// Mass-spring modeled deformable
pub struct Deformable {
    pub nodes: Vec<Vector3<Float>>,
    pub tetrahedra: Vec<Vec<usize>>,

    pub faces: Vec<[usize; 3]>, // face normal should be outwards
    pub edges: Vec<[usize; 2]>, // boundary edges

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,

    pub k: Float,
}

impl Deformable {
    pub fn new(nodes: Vec<Vector3<Float>>, tetrahedra: Vec<Vec<usize>>, k: Float) -> Self {
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
        }
    }

    pub fn new_tetrahedron() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
        ];
        let tetrahedra = vec![vec![0, 1, 2, 3]];

        Deformable::new(nodes, tetrahedra, 1e5)
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
        Self::new(nodes, tetrahedra, 1e5)
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
        Self::new(nodes, tetrahedra, 1e5)
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
        Self::new(nodes, tetrahedra, k)
    }

    /// size is side length, n is number of little cubes in each dimension
    pub fn new_dense_cube(size: Float, n: usize, k: Float) -> Self {
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

        Self::new(nodes, tetrahedra, k)
    }

    /// Linear momentum assuming mass = 1
    pub fn linear_momentum(&self) -> Vector3<Float> {
        self.get_velocities()
            .iter()
            .fold(Vector3::<Float>::zeros(), |acc, v| acc + v)
    }

    pub fn translate(&mut self, pos: &Vector3<Float>) {
        let mut i = 0;
        while i < self.dof() {
            let mut q = self.q.fixed_rows_mut::<3>(i);
            for j in 0..3 {
                q[j] += pos[j];
            }
            i += 3;
        }
    }

    pub fn get_positions(&self) -> Vec<Vector3<Float>> {
        let mut p = vec![];
        let mut i = 0;
        while i < self.q.len() {
            p.push(vector![self.q[i], self.q[i + 1], self.q[i + 2]]);
            i += 3;
        }
        p
    }

    pub fn get_velocities(&self) -> Vec<Vector3<Float>> {
        let mut v = vec![];
        let mut i = 0;
        while i < self.q.len() {
            v.push(vector![self.qdot[i], self.qdot[i + 1], self.qdot[i + 2]]);
            i += 3;
        }
        v
    }

    pub fn set_velocity(&mut self, v: Vec<Vector3<Float>>) {
        self.qdot = DVector::from_iterator(self.q.len(), v.iter().flat_map(|x| x.iter().copied()));
    }

    /// free-motion velocity after timestep
    pub fn free_velocity(&self, dt: Float, gravity_enabled: bool) -> DVector<Float> {
        let n_nodes = self.nodes.len();
        let mut adj: CooMatrix<u8> = CooMatrix::new(n_nodes, n_nodes);
        for tetrahedron in self.tetrahedra.iter() {
            for i in 0..tetrahedron.len() - 1 {
                for j in (i + 1)..tetrahedron.len() {
                    let v0 = tetrahedron[i];
                    let v1 = tetrahedron[j];
                    adj.push(v0, v1, 1);
                    adj.push(v1, v0, 1);
                }
            }
        }
        let adj = CsrMatrix::from(&adj);

        let mut edges: Vec<(usize, usize)> = vec![];
        let mut rs: Vec<Float> = vec![]; // rest lengths
        for irow in 0..n_nodes {
            let row = adj.row(irow);
            for icol in row.col_indices() {
                let icol = *icol;
                if irow < icol {
                    assert!(!edges.contains(&(irow, icol)));
                    edges.push((irow, icol));
                    rs.push((self.nodes[irow] - self.nodes[icol]).norm());
                }
            }
        }

        let dof = self.q.len();
        let mut f_elastic: DVector<Float> = DVector::zeros(dof);
        for (edge, r) in izip!(edges.iter(), rs.iter()) {
            let (n0, n1) = edge;
            let i_n0 = n0 * 3;
            let i_n1 = n1 * 3;
            let q0 = vector![self.q[i_n0], self.q[i_n0 + 1], self.q[i_n0 + 2]];
            let q1 = vector![self.q[i_n1], self.q[i_n1 + 1], self.q[i_n1 + 2]];

            let q0q1 = q1 - q0;
            let l = q0q1.norm();

            let k = self.k;
            let f_n0 = k * (l - r) * q0q1 / l;
            let f_n1 = -f_n0;

            for i in 0..3 {
                f_elastic[i_n0 + i] += f_n0[i];
                f_elastic[i_n1 + i] += f_n1[i];
            }
        }

        let mut f_damping: DVector<Float> = DVector::zeros(dof);
        for edge in edges.iter() {
            let (n0, n1) = edge;
            let i_n0 = n0 * 3;
            let i_n1 = n1 * 3;
            let q0 = vector![self.q[i_n0], self.q[i_n0 + 1], self.q[i_n0 + 2]];
            let q1 = vector![self.q[i_n1], self.q[i_n1 + 1], self.q[i_n1 + 2]];

            let q0q1 = q1 - q0;
            let l = q0q1.norm();
            let unit_q0q1 = q0q1 / l;

            let qdot0 = vector![self.qdot[i_n0], self.qdot[i_n0 + 1], self.qdot[i_n0 + 2]];
            let qdot1 = vector![self.qdot[i_n1], self.qdot[i_n1 + 1], self.qdot[i_n1 + 2]];
            let qdot0qdot1 = qdot1 - qdot0;

            let b = 20.;
            let f_n0 = b * qdot0qdot1.dot(&unit_q0q1) * unit_q0q1;
            let f_n1 = -f_n0;

            for i in 0..3 {
                f_damping[i_n0 + i] += f_n0[i];
                f_damping[i_n1 + i] += f_n1[i];
            }
        }

        let mut f_gravity: DVector<Float> = DVector::zeros(dof);
        if gravity_enabled {
            let mut i = 0;
            while i < dof {
                f_gravity[i + 2] = -GRAVITY;
                i += 3;
            }
        }

        let f_total = f_elastic + f_damping + f_gravity;
        let m_v0 = -dt * f_total; // momentum residual with velocity at t0

        let A = self.mass_matrix();
        let v0 = &self.qdot;
        let v_star = v0 - A.clone().try_inverse().unwrap() * m_v0;

        v_star
    }

    pub fn mass_matrix(&self) -> DMatrix<Float> {
        let dof = self.dof();
        let mass = 1.0;
        let M: DMatrix<Float> = mass * DMatrix::identity(dof, dof); // mass matrix
        M
    }

    pub fn dof(&self) -> usize {
        self.q.len()
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

/// Returns a vec of (contact point, contact normal, deformable1 node weights, deformable2 node weights)
/// Note: contact normal point outwards of deformable1
pub fn deformable_deformable_ccd(
    d1: &Deformable,
    d2: &Deformable,
    v_d1: &DVector<Float>,
    v_d2: &DVector<Float>,
    dt: Float,
) -> Vec<(
    Vector3<Float>,
    UnitVector3<Float>,
    Vec<(usize, Float)>,
    Vec<(usize, Float)>,
)> {
    let mut result = vec![];
    let d1_es = &d1.edges;
    let d1_ns = d1.get_positions();
    let d1_e_coords: Vec<[Vector3<Float>; 2]> =
        d1_es.iter().map(|e| e.map(|ei| d1_ns[ei])).collect();
    let d1_fs = &d1.faces;
    let d1_f_coords: Vec<[Vector3<Float>; 3]> =
        d1_fs.iter().map(|f| f.map(|fi| d1_ns[fi])).collect();

    let d2_es = &d2.edges;
    let d2_ns = d2.get_positions();
    let d2_e_coords: Vec<[Vector3<Float>; 2]> =
        d2_es.iter().map(|e| e.map(|ei| d2_ns[ei])).collect();
    let d2_fs = &d2.faces;
    let d2_f_coords: Vec<[Vector3<Float>; 3]> =
        d2_fs.iter().map(|f| f.map(|fi| d2_ns[fi])).collect();

    // d1 face - d2 point
    for (d1_f, d1_f_coord) in izip!(d1_fs.iter(), d1_f_coords.iter()) {
        for (i_d2, d2_n) in d2_ns.iter().enumerate() {
            if let Some((cp, n, ws)) = vertex_face_collision(d2_n, d1_f_coord, 1e-2) {
                let n2_ws = vec![(i_d2, 1.)];
                let n1_ws = vec![(d1_f[0], ws[0]), (d1_f[1], ws[1]), (d1_f[2], ws[2])];
                result.push((cp, n, n1_ws, n2_ws));
            }
        }
    }

    // d1 point - d2 face
    for (i_d1, d1_n) in d1_ns.iter().enumerate() {
        let p_t0 = d1_n;
        let v1: Vector3<Float> = v_d1.fixed_rows::<3>(i_d1 * 3).into();
        let p_t1 = p_t0 + v1 * dt;

        for (d2_f, d2_f_coord) in izip!(d2_fs.iter(), d2_f_coords.iter()) {
            let t0_t0 = &d2_f_coord[0];
            let t1_t0 = &d2_f_coord[1];
            let t2_t0 = &d2_f_coord[2];

            let v2: Vector3<Float> = v_d2.fixed_rows::<3>(d2_f[0] * 3).into();
            let v3: Vector3<Float> = v_d2.fixed_rows::<3>(d2_f[1] * 3).into();
            let v4: Vector3<Float> = v_d2.fixed_rows::<3>(d2_f[2] * 3).into();

            let t0_t1 = t0_t0 + v2 * dt;
            let t1_t1 = t1_t0 + v3 * dt;
            let t2_t1 = t2_t0 + v4 * dt;

            let toi = point_triangle_accd(
                p_t0, t0_t0, t1_t0, t2_t0, &p_t1, &t0_t1, &t1_t1, &t2_t1, 1e-3, 1.0,
            );
            if let Some(toi) = toi {
                let (cp, n, ws) = point_triangle_contact(
                    p_t0, t0_t0, t1_t0, t2_t0, &p_t1, &t0_t1, &t1_t1, &t2_t1, toi,
                );

                let n1_ws = vec![(i_d1, 1.)];
                let n2_ws = vec![(d2_f[0], ws[0]), (d2_f[1], ws[1]), (d2_f[2], ws[2])];
                result.push((cp, -n, n1_ws, n2_ws));
            }
        }
    }

    // edge-edge
    for (i_d1, e1) in d1_e_coords.iter().enumerate() {
        let d1_e = d1_es[i_d1];
        let v1: Vector3<Float> = v_d1.fixed_rows::<3>(d1_e[0] * 3).into();
        let v2: Vector3<Float> = v_d1.fixed_rows::<3>(d1_e[1] * 3).into();

        let ea0_t0 = &e1[0];
        let ea1_t0 = &e1[1];
        let ea0_t1 = ea0_t0 + v1 * dt;
        let ea1_t1 = ea1_t0 + v2 * dt;

        for (i_d2, e2) in d2_e_coords.iter().enumerate() {
            let d2_e = d2_es[i_d2];
            let v3: Vector3<Float> = v_d2.fixed_rows::<3>(d2_e[0] * 3).into();
            let v4: Vector3<Float> = v_d2.fixed_rows::<3>(d2_e[1] * 3).into();

            let eb0_t0 = &e2[0];
            let eb1_t0 = &e2[1];
            let eb0_t1 = eb0_t0 + v3 * dt;
            let eb1_t1 = eb1_t0 + v4 * dt;
            let toi = edge_edge_accd(
                ea0_t0, ea1_t0, eb0_t0, eb1_t0, &ea0_t1, &ea1_t1, &eb0_t1, &eb1_t1, 1e-3, 1.0,
            );
            if let Some(toi) = toi {
                let (cp, n, w1s, w2s) = edge_edge_contact(
                    ea0_t0, ea1_t0, eb0_t0, eb1_t0, &ea0_t1, &ea1_t1, &eb0_t1, &eb1_t1, toi,
                );

                let n1_ws = vec![(d1_e[0], w1s[0]), (d1_e[1], w1s[1])];
                let n2_ws = vec![(d2_e[0], w2s[0]), (d2_e[1], w2s[1])];

                result.push((cp, n, n1_ws, n2_ws));
            }
        }
    }

    result
}
