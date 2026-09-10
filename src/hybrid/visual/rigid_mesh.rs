use std::collections::HashMap;

use na::{vector, Vector3};

use crate::{collision::bvh::Bvh, types::Float};

/// A run of faces an OBJ assigned one material to with `usemtl`.
///
/// The range is what the renderer draws as a single geometry group, so a mesh
/// with several materials stays *one* mesh -- one set of vertices, one BVH, one
/// collider -- and only the drawing is split. Nothing in the contact path reads
/// this.
#[derive(Clone, Debug)]
pub struct MeshGroup {
    /// Index into `faces` of the first face the group covers.
    pub start: usize,
    /// How many faces it covers.
    pub count: usize,
    /// The name `usemtl` gave it, which is what a `.mtl` keys its materials by.
    pub material: String,
    /// Diffuse colour, once whoever can find the `.mtl` has filled it in. The
    /// OBJ alone only ever names the material.
    pub color: Option<Vector3<Float>>,
}

#[derive(Clone, Debug)]
pub struct RigidMesh {
    pub vertices: Vec<Vector3<Float>>,
    pub faces: Vec<[usize; 3]>,

    /// The `usemtl` runs of `faces`, in face order. Empty for a mesh drawn in
    /// one colour, which is every mesh that did not come from an OBJ naming
    /// materials.
    pub groups: Vec<MeshGroup>,

    /// Axis-aligned bounding box of the vertices, used for collision
    /// early-outs. Must be kept in sync via `recompute_aabb` whenever the
    /// vertices are mutated.
    pub aabb_min: Vector3<Float>,
    pub aabb_max: Vector3<Float>,

    /// AABB tree over the faces, used to prune per-face collision tests.
    /// Kept in sync with the vertices by `recompute_aabb` as well.
    pub bvh: Bvh,

    /// For each vertex, the index of the first vertex sitting at the same
    /// position -- itself, for a mesh whose faces share their corners.
    ///
    /// The contact path names an *edge* by the pair of vertex indices at its
    /// ends, which is what lets it recognise that two faces carry the same edge
    /// and settle which way it faces out of the mesh from both of their normals.
    /// A mesh whose faces do not share their corners -- an STL-derived one, or
    /// an OBJ that splits a vertex per material or per shading group, which is
    /// most CAD exports -- names each visit to one edge differently, so every
    /// edge looks like a boundary edge carried by one face alone. See
    /// [`crate::hybrid::collision`]'s `distinct_edges` for what that costs.
    ///
    /// Positions are matched *exactly*, on the bits, save that a negative zero
    /// is read as a zero -- which is a coordinate a file can honestly write
    /// either way round. A split vertex is otherwise the same number written
    /// twice by whatever wrote the file -- the same decimal in an OBJ, the same
    /// f32 in a binary STL -- so it parses to the same bits, and a tolerance
    /// here would need a spatial index and would risk merging the genuinely
    /// distinct corners of a fine mesh.
    ///
    /// Kept in sync with the vertices by `recompute_aabb`.
    pub canonical: Vec<usize>,
}

/// Close the group being read: it runs from where it started to the face the
/// parser has now reached.
fn close_group(groups: &mut [MeshGroup], faces: usize) {
    if let Some(open) = groups.last_mut() {
        open.count = faces - open.start;
    }
}

/// The next whitespace-delimited token at `p`, or `None` once the line has no
/// more of them.
///
/// A `#` starts a comment, and it ends the line as far as the parser is
/// concerned -- our own `.obj` assets annotate faces with them, as in
/// `f 1 2 4 # front face`, so the comment words are emphatically not vertices.
/// Hitting one leaves `p` where it was, so asking again keeps saying `None`.
fn next_token(bytes: &[u8], p: &mut usize) -> Option<(usize, usize)> {
    let n = bytes.len();
    while *p < n && bytes[*p] == b' ' {
        *p += 1;
    }
    let start = *p;
    while *p < n && !matches!(bytes[*p], b' ' | b'\n' | b'\r') {
        *p += 1;
    }
    if start == *p || bytes[start] == b'#' {
        *p = start;
        return None;
    }
    Some((start, *p))
}

impl RigidMesh {
    pub fn new_from_obj(content: &str) -> Self {
        let bytes = content.as_bytes();
        let n = bytes.len();

        // Heuristic preallocation to avoid repeated Vec reallocation.
        // Over-estimates slightly; see note below if you want it exact.
        let cap = n / 32;
        let mut vertices = Vec::with_capacity(cap);
        let mut faces = Vec::with_capacity(cap);
        let mut groups: Vec<MeshGroup> = Vec::new();

        let mut i = 0;
        while i < n {
            let kind = bytes[i];
            let next = if i + 1 < n { bytes[i + 1] } else { b'\n' };

            // Only "v " and "f " lines matter; this skips vn, vt, comments, o, etc.
            if (kind == b'v' || kind == b'f') && next == b' ' {
                // Collect up to 3 whitespace-delimited token spans.
                let mut p = i + 2;
                let mut toks = [(0usize, 0usize); 3];
                let mut t = 0;
                while t < 3 {
                    match next_token(bytes, &mut p) {
                        Some(tok) => {
                            toks[t] = tok;
                            t += 1;
                        }
                        None => break,
                    }
                }

                if kind == b'v' {
                    let f = |(s, e): (usize, usize)| -> Float { content[s..e].parse().unwrap() };
                    vertices.push(vector![f(toks[0]), f(toks[1]), f(toks[2])]);
                } else {
                    // `faces` is triangles, and everything downstream of it --
                    // the BVH, the contact path, the renderer -- reads it as
                    // triangles. A quad or an n-gon has nowhere to go: taking
                    // the first three tokens would quietly drop the rest of the
                    // face and leave a hole in the mesh, which is far worse to
                    // debug later than a refusal here. Triangulate first.
                    let mut vertices_on_face = t;
                    while next_token(bytes, &mut p).is_some() {
                        vertices_on_face += 1;
                    }
                    if vertices_on_face != 3 {
                        let mut end = i;
                        while end < n && !matches!(bytes[end], b'\n' | b'\r') {
                            end += 1;
                        }
                        panic!(
                            "RigidMesh stores triangles, but this obj face has {} vertices: `{}`. Triangulate the mesh before loading it.",
                            vertices_on_face,
                            content[i..end].trim_end()
                        );
                    }

                    // TODO: Each element in face line in obj file could be of
                    // format x/y/z where x is vertex index, y is material
                    // index, and z is vertex normal index. Parse these as well.
                    let g = |(s, e): (usize, usize)| -> usize {
                        let mut acc = 0usize;
                        let mut q = s;
                        while q < e {
                            let d = bytes[q].wrapping_sub(b'0');
                            if d < 10 {
                                acc = acc * 10 + d as usize;
                                q += 1;
                            } else {
                                break;
                            }
                        }
                        acc - 1
                    };
                    faces.push([g(toks[0]), g(toks[1]), g(toks[2])]);
                }
            } else if kind == b'u' && bytes[i..].starts_with(b"usemtl") {
                // Every face from here on belongs to the named material, until
                // the next `usemtl`. The name is all that is in the OBJ; the
                // colour behind it lives in the `.mtl` the loader resolves.
                let mut p = i + "usemtl".len();
                while p < n && bytes[p] == b' ' {
                    p += 1;
                }
                let start = p;
                while p < n && !matches!(bytes[p], b'\n' | b'\r') {
                    p += 1;
                }
                close_group(&mut groups, faces.len());
                groups.push(MeshGroup {
                    start: faces.len(),
                    count: 0,
                    material: content[start..p].trim().to_string(),
                    color: None,
                });
            }

            // Advance to the next line.
            while i < n && bytes[i] != b'\n' {
                i += 1;
            }
            i += 1;
        }

        close_group(&mut groups, faces.len());
        // A `usemtl` that no face followed says nothing about how to draw
        // anything, and an empty group would be an empty draw call.
        groups.retain(|group| group.count > 0);

        let mut mesh = Self {
            vertices,
            faces,
            groups,
            aabb_min: Vector3::zeros(),
            aabb_max: Vector3::zeros(),
            bvh: Bvh::default(),
            canonical: vec![],
        };
        mesh.recompute_aabb();
        mesh
    }

    /// The material groups as the renderer needs them: a gap-free cover of
    /// every face, each run paired with the colour to draw it in.
    ///
    /// Empty when the mesh named no materials, which means "draw the whole
    /// thing in one colour" and is the case for every mesh that did not come
    /// from an OBJ with `usemtl` lines. Otherwise the runs are contiguous and
    /// reach the last face: a renderer that is given groups draws *only* what
    /// they cover, so faces before the first `usemtl` -- or belonging to a
    /// material the `.mtl` never defined -- would silently vanish. They are
    /// filled with `fallback`, which is the visual's own colour.
    pub fn draw_groups(&self, fallback: Vector3<Float>) -> Vec<(usize, usize, Vector3<Float>)> {
        if self.groups.is_empty() {
            return vec![];
        }

        let mut drawn: Vec<(usize, usize, Vector3<Float>)> = vec![];
        let mut covered = 0;
        for group in self.groups.iter() {
            if group.start > covered {
                drawn.push((covered, group.start - covered, fallback));
            }
            drawn.push((group.start, group.count, group.color.unwrap_or(fallback)));
            covered = group.start + group.count;
        }
        if covered < self.faces.len() {
            drawn.push((covered, self.faces.len() - covered, fallback));
        }
        drawn
    }

    /// Recompute the cached axis-aligned bounding box, rebuild the face BVH,
    /// and rebuild the map of coincident vertices. Call after any mutation of
    /// `vertices`.
    pub fn recompute_aabb(&mut self) {
        let mut min = Vector3::repeat(Float::MAX);
        let mut max = Vector3::repeat(Float::MIN);
        for v in self.vertices.iter() {
            min = min.inf(v);
            max = max.sup(v);
        }
        self.aabb_min = min;
        self.aabb_max = max;
        self.bvh = Bvh::build(&self.vertices, &self.faces);

        // See `canonical`: the first vertex at each position stands for all of
        // them, so that an edge two faces carry is named the same way by both
        // even on a mesh that repeats its corners rather than sharing them.
        let mut first = HashMap::with_capacity(self.vertices.len());
        self.canonical = self
            .vertices
            .iter()
            .enumerate()
            .map(|(i, v)| {
                // `+ 0.` only to fold -0. onto 0., which have different bits
                let bits = [
                    (v.x + 0.).to_bits(),
                    (v.y + 0.).to_bits(),
                    (v.z + 0.).to_bits(),
                ];
                *first.entry(bits).or_insert(i)
            })
            .collect();
    }

    // Old more readable implemenation
    // pub fn new_from_obj(content: &str) -> Self {
    //     let mut vertices = vec![];
    //     let mut faces = vec![];
    //     for line in content.lines() {
    //         let mut parts = line.split_whitespace();
    //         if let Some(heading) = parts.next() {
    //             match heading {
    //                 "v" => {
    //                     let x: Float = parts.next().unwrap().parse().unwrap();
    //                     let y: Float = parts.next().unwrap().parse().unwrap();
    //                     let z: Float = parts.next().unwrap().parse().unwrap();
    //                     vertices.push(vector![x, y, z]);
    //                 }
    //                 "f" => {
    //                      // TODO: Each element in face line in obj file could be of
    //                      // format x/y/z where x is vertex index, y is material
    //                      // index, and z is vertex normal index. Parse these as well.
    //                     let i: usize = parts
    //                         .next()
    //                         .unwrap()
    //                         .split("/")
    //                         .next()
    //                         .unwrap()
    //                         .parse::<usize>()
    //                         .unwrap()
    //                         - 1;
    //                     let j: usize = parts
    //                         .next()
    //                         .unwrap()
    //                         .split("/")
    //                         .next()
    //                         .unwrap()
    //                         .parse::<usize>()
    //                         .unwrap()
    //                         - 1;
    //                     let k: usize = parts
    //                         .next()
    //                         .unwrap()
    //                         .split("/")
    //                         .next()
    //                         .unwrap()
    //                         .parse::<usize>()
    //                         .unwrap()
    //                         - 1;
    //                     faces.push([i, j, k]);
    //                 }
    //                 _ => {}
    //             }
    //         }
    //     }

    //     Self { vertices, faces }
    // }
}

#[cfg(test)]
mod rigid_mesh_tests {
    use na::vector;

    use crate::hybrid::visual::rigid_mesh::RigidMesh;

    const TWO_TONE: &str = "\
mtllib part.mtl
v 0 0 0
v 1 0 0
v 0 1 0
v 1 1 0
usemtl shell
f 1 2 3
usemtl trim
f 2 4 3
f 1 2 4
";

    /// `usemtl` splits the faces that follow it into groups, and a group runs
    /// until the next one rather than to the end of the file.
    #[test]
    fn new_from_obj_reads_usemtl_into_face_groups() {
        let mesh = RigidMesh::new_from_obj(TWO_TONE);

        assert_eq!(mesh.faces.len(), 3);
        assert_eq!(mesh.groups.len(), 2);
        assert_eq!(mesh.groups[0].material, "shell");
        assert_eq!((mesh.groups[0].start, mesh.groups[0].count), (0, 1));
        assert_eq!(mesh.groups[1].material, "trim");
        assert_eq!((mesh.groups[1].start, mesh.groups[1].count), (1, 2));
        // The OBJ names materials; it never says what colour they are.
        assert!(mesh.groups.iter().all(|group| group.color.is_none()));
    }

    /// A face with more than three vertices cannot be stored, only truncated,
    /// so it is refused rather than silently losing part of the mesh.
    #[test]
    #[should_panic(expected = "this obj face has 4 vertices")]
    fn new_from_obj_rejects_a_quad_face() {
        RigidMesh::new_from_obj("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n");
    }

    /// The same for a degenerate face, which would otherwise index vertex
    /// `0 - 1` and wrap into nonsense.
    #[test]
    #[should_panic(expected = "this obj face has 2 vertices")]
    fn new_from_obj_rejects_a_face_with_too_few_vertices() {
        RigidMesh::new_from_obj("v 0 0 0\nv 1 0 0\nv 1 1 0\nf 1 2\n");
    }

    /// Our own `.obj` assets annotate faces with trailing comments, so a
    /// comment must end the line rather than read as extra vertices -- every
    /// face in `data/cube.obj` and `data/tetrahedron.obj` looks like this.
    #[test]
    fn new_from_obj_reads_a_triangle_annotated_with_a_comment() {
        let mesh = RigidMesh::new_from_obj(
            "v 0 0 0\nv 1 0 0\nv 1 1 0 # a corner\nf 1 2 3 # front face\n",
        );

        assert_eq!(mesh.vertices.len(), 3);
        assert_eq!(mesh.faces, vec![[0, 1, 2]]);
    }

    /// An OBJ that names no materials is drawn in one colour, which is what an
    /// empty group list means to the renderer.
    #[test]
    fn new_from_obj_leaves_a_mesh_without_materials_ungrouped() {
        let mesh = RigidMesh::new_from_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");

        assert_eq!(mesh.faces.len(), 1);
        assert!(mesh.groups.is_empty());
        assert!(mesh.draw_groups(vector![1., 0., 0.]).is_empty());
    }

    /// The heart of it: a renderer given groups draws *only* what they cover, so
    /// faces the materials say nothing about -- here the one before the first
    /// `usemtl` -- have to come back covered by the fallback colour, or they
    /// disappear from the scene entirely.
    #[test]
    fn draw_groups_covers_every_face_including_the_ones_no_material_claimed() {
        let mut mesh = RigidMesh::new_from_obj(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nf 1 2 3\nusemtl trim\nf 2 4 3\n",
        );
        mesh.groups[0].color = Some(vector![0., 0., 1.]);

        let fallback = vector![1., 0., 0.];
        let drawn = mesh.draw_groups(fallback);

        assert_eq!(drawn.len(), 2);
        // The unclaimed face first, in the fallback colour...
        assert_eq!(drawn[0], (0, 1, fallback));
        // ...then the group, in its own.
        assert_eq!(drawn[1], (1, 1, vector![0., 0., 1.]));
        // Contiguous, and reaching the last face.
        assert_eq!(drawn[0].1 + drawn[1].1, mesh.faces.len());
    }

    /// A group whose material the `.mtl` never defined keeps the visual's own
    /// colour rather than dropping out of the draw.
    #[test]
    fn draw_groups_falls_back_for_a_group_with_no_colour() {
        let mesh = RigidMesh::new_from_obj(TWO_TONE);
        let fallback = vector![0.5, 0.5, 0.5];

        let drawn = mesh.draw_groups(fallback);

        assert_eq!(drawn.len(), 2);
        assert!(drawn
            .iter()
            .all(|(_start, _count, color)| *color == fallback));
    }
}
