use na::Vector3;

use crate::types::Float;

/// Max number of faces in a leaf node
const LEAF_SIZE: usize = 4;

#[derive(Clone, Debug)]
struct BvhNode {
    aabb_min: Vector3<Float>,
    aabb_max: Vector3<Float>,
    /// Index of the right child; the left child is always the next node in
    /// `nodes` (preorder layout). Unused for leaves.
    right: usize,
    /// Range into `face_indices`; `count == 0` marks an internal node
    start: usize,
    count: usize,
}

/// Bounding volume hierarchy over the faces of a triangle mesh, for pruning
/// per-face collision tests down to the faces near a query volume
#[derive(Clone, Debug, Default)]
pub struct Bvh {
    nodes: Vec<BvhNode>,
    face_indices: Vec<usize>,
    /// Per-face AABBs, indexed by face, for exact filtering within leaves
    face_aabbs: Vec<(Vector3<Float>, Vector3<Float>)>,
}

impl Bvh {
    pub fn build(vertices: &[Vector3<Float>], faces: &[[usize; 3]]) -> Self {
        let n = faces.len();

        // Per-face AABBs and centroids, indexed by face
        let mut face_aabbs = Vec::with_capacity(n);
        let mut centroids = Vec::with_capacity(n);
        for face in faces.iter() {
            let v1 = vertices[face[0]];
            let v2 = vertices[face[1]];
            let v3 = vertices[face[2]];
            let min = v1.inf(&v2).inf(&v3);
            let max = v1.sup(&v2).sup(&v3);
            centroids.push((min + max) / 2.);
            face_aabbs.push((min, max));
        }

        let mut bvh = Self {
            nodes: vec![],
            face_indices: (0..n).collect(),
            face_aabbs,
        };
        if n > 0 {
            bvh.build_node(0, n, &centroids);
        }
        bvh
    }

    /// Build the node covering `face_indices[start..start + count]`, splitting
    /// recursively at the median centroid along the longest axis
    fn build_node(&mut self, start: usize, count: usize, centroids: &[Vector3<Float>]) -> usize {
        let mut aabb_min = Vector3::repeat(Float::MAX);
        let mut aabb_max = Vector3::repeat(Float::MIN);
        for i_face in self.face_indices[start..start + count].iter() {
            let (min, max) = &self.face_aabbs[*i_face];
            aabb_min = aabb_min.inf(min);
            aabb_max = aabb_max.sup(max);
        }

        let node_index = self.nodes.len();
        self.nodes.push(BvhNode {
            aabb_min,
            aabb_max,
            right: 0,
            start,
            count,
        });

        if count <= LEAF_SIZE {
            return node_index;
        }

        // Longest axis of the centroid bounds
        let mut cmin = Vector3::repeat(Float::MAX);
        let mut cmax = Vector3::repeat(Float::MIN);
        for i_face in self.face_indices[start..start + count].iter() {
            cmin = cmin.inf(&centroids[*i_face]);
            cmax = cmax.sup(&centroids[*i_face]);
        }
        let extent = cmax - cmin;
        let axis = extent.imax();

        // Median split: both halves are non-empty since count > LEAF_SIZE,
        // so recursion always terminates
        let mid = count / 2;
        self.face_indices[start..start + count]
            .select_nth_unstable_by(mid, |a, b| centroids[*a][axis].total_cmp(&centroids[*b][axis]));

        self.nodes[node_index].count = 0; // mark internal
        self.build_node(start, mid, centroids);
        let right = self.build_node(start + mid, count - mid, centroids);
        self.nodes[node_index].right = right;
        node_index
    }

    /// Returns the indices of all faces whose AABB overlaps the query AABB
    pub fn collect_overlapping(
        &self,
        qmin: &Vector3<Float>,
        qmax: &Vector3<Float>,
    ) -> Vec<usize> {
        let mut result = vec![];
        if self.nodes.is_empty() {
            return result;
        }

        let mut stack = vec![0];
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i];
            if node.aabb_min.x > qmax.x
                || node.aabb_max.x < qmin.x
                || node.aabb_min.y > qmax.y
                || node.aabb_max.y < qmin.y
                || node.aabb_min.z > qmax.z
                || node.aabb_max.z < qmin.z
            {
                continue;
            }
            if node.count > 0 {
                for i_face in self.face_indices[node.start..node.start + node.count].iter() {
                    let (min, max) = &self.face_aabbs[*i_face];
                    if min.x <= qmax.x
                        && max.x >= qmin.x
                        && min.y <= qmax.y
                        && max.y >= qmin.y
                        && min.z <= qmax.z
                        && max.z >= qmin.z
                    {
                        result.push(*i_face);
                    }
                }
            } else {
                stack.push(i + 1);
                stack.push(node.right);
            }
        }
        result
    }
}

#[cfg(test)]
mod bvh_tests {
    use na::{vector, Vector3};
    use rand::rng;
    use rand::Rng;

    use super::Bvh;
    use crate::types::Float;

    /// BVH queries return exactly the faces a brute-force AABB overlap test
    /// returns, on a random triangle soup with random query boxes
    #[test]
    fn matches_brute_force() {
        let mut rng = rng();

        let num_faces = 200;
        let mut vertices: Vec<Vector3<Float>> = vec![];
        let mut faces = vec![];
        for i in 0..num_faces {
            let base = vector![
                rng.random_range(-5.0..5.0),
                rng.random_range(-5.0..5.0),
                rng.random_range(-5.0..5.0)
            ];
            for _ in 0..3 {
                vertices.push(
                    base + vector![
                        rng.random_range(-0.5..0.5),
                        rng.random_range(-0.5..0.5),
                        rng.random_range(-0.5..0.5)
                    ],
                );
            }
            faces.push([3 * i, 3 * i + 1, 3 * i + 2]);
        }

        let bvh = Bvh::build(&vertices, &faces);

        for _ in 0..100 {
            let center = vector![
                rng.random_range(-6.0..6.0),
                rng.random_range(-6.0..6.0),
                rng.random_range(-6.0..6.0)
            ];
            let half = vector![
                rng.random_range(0.1..2.0),
                rng.random_range(0.1..2.0),
                rng.random_range(0.1..2.0)
            ];
            let qmin = center - half;
            let qmax = center + half;

            let mut expected: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter(|(_, face)| {
                    let min = vertices[face[0]].inf(&vertices[face[1]]).inf(&vertices[face[2]]);
                    let max = vertices[face[0]].sup(&vertices[face[1]]).sup(&vertices[face[2]]);
                    !(min.x > qmax.x
                        || max.x < qmin.x
                        || min.y > qmax.y
                        || max.y < qmin.y
                        || min.z > qmax.z
                        || max.z < qmin.z)
                })
                .map(|(i, _)| i)
                .collect();

            let mut actual = bvh.collect_overlapping(&qmin, &qmax);
            expected.sort();
            actual.sort();
            assert_eq!(actual, expected);
        }
    }

    /// Building and querying an empty mesh does not panic
    #[test]
    fn empty_mesh() {
        let bvh = Bvh::build(&[], &[]);
        assert!(bvh
            .collect_overlapping(&Vector3::zeros(), &Vector3::zeros())
            .is_empty());
    }
}
