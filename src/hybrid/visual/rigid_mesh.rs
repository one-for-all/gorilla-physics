use na::{vector, Vector3};

use crate::types::Float;

#[derive(Clone, Debug)]
pub struct RigidMesh {
    pub vertices: Vec<Vector3<Float>>,
    pub faces: Vec<[usize; 3]>,
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
                    while p < n && bytes[p] == b' ' {
                        p += 1;
                    }
                    let start = p;
                    while p < n && !matches!(bytes[p], b' ' | b'\n' | b'\r') {
                        p += 1;
                    }
                    if p == start {
                        break;
                    }
                    toks[t] = (start, p);
                    t += 1;
                }

                if kind == b'v' {
                    let f = |(s, e): (usize, usize)| -> Float { content[s..e].parse().unwrap() };
                    vertices.push(vector![f(toks[0]), f(toks[1]), f(toks[2])]);
                } else {
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
            }

            // Advance to the next line.
            while i < n && bytes[i] != b'\n' {
                i += 1;
            }
            i += 1;
        }

        Self { vertices, faces }
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
