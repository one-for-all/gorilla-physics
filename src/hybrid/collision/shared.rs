//! Geometry and bookkeeping shared by the collision routines.

use std::collections::{hash_map::Entry, HashMap, HashSet};

use itertools::izip;
use na::{Isometry3, Point3, UnitVector3, Vector3};

use crate::{
    collision::mesh::projected_barycentric_coord, hybrid::visual::rigid_mesh::RigidMesh,
    types::Float, util::tangentials, PI,
};

/// How far apart two colliders can be and still count as touching.
///
/// One tolerance for every pair of shapes, so that a body makes contact at the
/// same distance whichever kind of collider it carries and whichever kind it
/// meets -- a body carrying a mesh and one carrying the equivalent cuboid have
/// to behave the same way.
pub const CONTACT_TOLERANCE: Float = 1e-3;

/// How close to a halfspace a point counts as touching it.
///
/// Matches `HalfSpace::has_inside`, so a mesh contacts the plane at the same
/// point a collision sphere or cuboid on the same body would. Widening it into a
/// speculative band is tempting but buys nothing: the body just comes to rest
/// hovering at the edge of the band instead of on the plane.
pub const HALFSPACE_CONTACT_TOLERANCE: Float = 1e-8;

/// Distance below which two contact points count as the same one.
///
/// A vertex shared by several faces is reported once per face, and a mesh whose
/// faces do not share their vertices -- an STL-derived one -- repeats the vertex
/// itself; duplicated contact points are duplicated rows in the solve.
pub const WELD_DISTANCE: Float = 1e-6;

/// How far from either end of two edges their closest points have to sit to
/// count as an edge-edge contact.
///
/// A closest point sitting on an endpoint is a corner of one body against the
/// other, which the vertex-face passes already report -- and report with the
/// normal of the face the point rests on, rather than with the common
/// perpendicular of the two edges, which is meaningless there.
pub const EDGE_END_MARGIN: Float = 1e-6;

/// How far from parallel, as the sine of the angle between them, two edges have
/// to be for their common perpendicular to be a usable contact normal.
pub const EDGE_PARALLEL_SINE: Float = 1e-6;

/// Below this, a face has no area, hence no outward direction to contact along.
pub const DEGENERATE_FACE_AREA: Float = 1e-12;

/// Tangential directions a contact patch is sampled along, on top of its
/// deepest point. See [`support_polygon`].
pub const SUPPORT_DIRECTIONS: usize = 8;

/// A point of contact between two bodies, in whichever frame the routine
/// reporting it works in
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    /// Where the two touch
    pub point: Vector3<Float>,
    /// Which way the contact pushes, running from the first body to the second
    pub normal: UnitVector3<Float>,
}

impl Contact {
    pub fn new(point: Vector3<Float>, normal: UnitVector3<Float>) -> Self {
        Self { point, normal }
    }

    /// The same contact seen from the other body, i.e. with the normal running
    /// from the second body to the first
    pub fn flipped(self) -> Self {
        Self {
            point: self.point,
            normal: -self.normal,
        }
    }

    /// The same contact carried into another frame by `iso`
    pub fn transformed(self, iso: &Isometry3<Float>) -> Self {
        // Rotating a unit normal leaves it a unit normal
        Self {
            point: (iso * Point3::from(self.point)).coords,
            normal: iso.rotation * self.normal,
        }
    }
}

/// A contact, and how far the two bodies have been driven into each other along
/// its normal -- negative while they are merely within tolerance of touching
///
/// The depth is what [`support_polygon`] ranks a patch's contacts by, and is
/// dropped once that has run.
#[derive(Clone, Copy, Debug)]
pub struct DeepContact {
    pub contact: Contact,
    pub depth: Float,
}

/// Something a contact list holds at a point, and so holds only one of
pub trait Welded {
    fn weld_point(&self) -> Vector3<Float>;
}

impl Welded for Contact {
    fn weld_point(&self) -> Vector3<Float> {
        self.point
    }
}

impl Welded for DeepContact {
    fn weld_point(&self) -> Vector3<Float> {
        self.contact.point
    }
}

impl Welded for Vector3<Float> {
    fn weld_point(&self) -> Vector3<Float> {
        *self
    }
}

/// Add `item`, unless one is already held at the same point; whether it was
/// added
pub fn push_welded<T: Welded>(items: &mut Vec<T>, item: T) -> bool {
    let point = item.weld_point();
    if items
        .iter()
        .any(|held| (held.weld_point() - point).norm_squared() < WELD_DISTANCE * WELD_DISTANCE)
    {
        return false;
    }
    items.push(item);

    true
}

/// An axis-aligned box
#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: Vector3<Float>,
    pub max: Vector3<Float>,
}

impl Aabb {
    pub fn new(min: Vector3<Float>, max: Vector3<Float>) -> Self {
        Self { min, max }
    }

    /// The mesh's own bounds, in the mesh's own frame
    pub fn of_mesh(mesh: &RigidMesh) -> Self {
        Self::new(mesh.aabb_min, mesh.aabb_max)
    }

    /// The bounds of the given points, or `None` if there are none
    pub fn of_points(points: impl IntoIterator<Item = Vector3<Float>>) -> Option<Self> {
        let mut bounds: Option<Self> = None;
        for point in points {
            bounds = Some(match bounds {
                Some(aabb) => Self::new(aabb.min.inf(&point), aabb.max.sup(&point)),
                None => Self::new(point, point),
            });
        }

        bounds
    }

    /// The box centred on `center` reaching `extent` along each axis
    pub fn centered(center: &Vector3<Float>, extent: &Vector3<Float>) -> Self {
        Self::new(center - extent, center + extent)
    }

    /// The box grown by `tol` in every direction
    pub fn inflated(&self, tol: Float) -> Self {
        let tol_vec = Vector3::repeat(tol);
        Self::new(self.min - tol_vec, self.max + tol_vec)
    }

    /// Whether the two boxes share any point
    pub fn overlaps(&self, other: &Self) -> bool {
        (0..3).all(|axis| self.min[axis] <= other.max[axis] && other.min[axis] <= self.max[axis])
    }

    /// The part of the box the two share, or `None` if they are apart
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        let shared = Self::new(self.min.sup(&other.min), self.max.inf(&other.max));
        (0..3)
            .all(|axis| shared.min[axis] <= shared.max[axis])
            .then_some(shared)
    }

    /// The closest point of the box to `point`, which is `point` itself when it
    /// is inside
    pub fn closest_point(&self, point: &Vector3<Float>) -> Vector3<Float> {
        point.sup(&self.min).inf(&self.max)
    }

    /// How far `point` is from the box, squared; zero when it is inside
    pub fn distance_sq_to(&self, point: &Vector3<Float>) -> Float {
        (self.closest_point(point) - point).norm_squared()
    }

    /// The axis-aligned box containing this one after `iso` has moved it
    pub fn transformed(&self, iso: &Isometry3<Float>) -> Self {
        let center = iso
            .transform_point(&Point3::from((self.min + self.max) / 2.))
            .coords;
        // A box of half-extent `half` reaches |R| * half along each axis of the
        // frame it is rotated into
        let extent =
            iso.rotation.to_rotation_matrix().matrix().abs() * ((self.max - self.min) / 2.);

        Self::centered(&center, &extent)
    }
}

/// Where `point` lands on the triangle, if it lies over it and within `tol`:
/// the closest point on the face, and the offset from there to `point`
///
/// The offset is left as it is rather than normalized -- which way round it
/// becomes a contact normal is the caller's to decide, and how long it is says
/// how much gap is left.
pub fn point_triangle_contact(
    point: &Vector3<Float>,
    v: &[Vector3<Float>; 3],
    tol_sq: Float,
) -> Option<(Vector3<Float>, Vector3<Float>)> {
    let edge1 = v[1] - v[0];
    let edge2 = v[2] - v[0];
    let (w1, w2, w3) = projected_barycentric_coord(point, &v[0], &edge1, &edge2);

    // The closest point on the triangle is outside the face, hence on one of its
    // edges, which the edge passes are what report
    if w1 < 0. || w2 < 0. || w3 < 0. {
        return None;
    }

    // The projection of the point onto the face
    // TODO: do ccd to avoid the passing through case
    let closest_point = w1 * v[0] + w2 * v[1] + w3 * v[2];
    let offset = point - closest_point;

    (offset.norm_squared() < tol_sq).then_some((closest_point, offset))
}

/// The closest pair of points on two segments, one on each
pub fn segment_closest_points(
    p1: &Vector3<Float>,
    q1: &Vector3<Float>,
    p2: &Vector3<Float>,
    q2: &Vector3<Float>,
) -> (Vector3<Float>, Vector3<Float>) {
    let (s, t) = segment_closest_params(p1, q1, p2, q2);
    (p1 + (q1 - p1) * s, p2 + (q2 - p2) * t)
}

/// How far along each segment, as a fraction of its length, its end of the
/// closest pair of points sits
fn segment_closest_params(
    p1: &Vector3<Float>,
    q1: &Vector3<Float>,
    p2: &Vector3<Float>,
    q2: &Vector3<Float>,
) -> (Float, Float) {
    let eps = 1e-12;

    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Where along each segment, as a fraction of its length, the closest point sits
    let mut s = 0.;
    let mut t = 0.;
    if a <= eps && e <= eps {
        return (0., 0.);
    } else if a <= eps {
        t = (f / e).clamp(0., 1.);
    } else {
        let c = d1.dot(&r);
        if e <= eps {
            s = (-c / a).clamp(0., 1.);
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            // Parallel segments leave s free; any point does, so take an end
            s = if denom > eps {
                ((b * f - c * e) / denom).clamp(0., 1.)
            } else {
                0.
            };
            t = (b * s + f) / e;
            // Clamping t moves the closest point on the other segment too
            if t < 0. {
                t = 0.;
                s = (-c / a).clamp(0., 1.);
            } else if t > 1. {
                t = 1.;
                s = ((b - c) / a).clamp(0., 1.);
            }
        }
    }

    (s, t)
}

/// Where the two edges cross, if they cross within `tol` of each other
///
/// The contact sits halfway across the gap -- or the overlap -- between them,
/// and the normal runs along their common perpendicular, out of the body
/// carrying the first edge. `outward1` is which way that edge faces out of its
/// own body, and settles which of the two ways along the perpendicular that is.
pub fn edge_edge_contact(
    edge1: (&Vector3<Float>, &Vector3<Float>),
    outward1: &Vector3<Float>,
    edge2: (&Vector3<Float>, &Vector3<Float>),
    tol_sq: Float,
) -> Option<DeepContact> {
    let (a, b) = edge1;
    let (c, d) = edge2;
    let direction1 = b - a;
    let direction2 = d - c;

    // Parallel edges have no common perpendicular to contact along, and touch
    // along the whole of their overlap rather than at a point; the corners at
    // the ends of that overlap are what holds them apart, and the vertex-face
    // passes report those
    let perpendicular = direction1.cross(&direction2);
    if perpendicular.norm_squared()
        < EDGE_PARALLEL_SINE
            * EDGE_PARALLEL_SINE
            * direction1.norm_squared()
            * direction2.norm_squared()
    {
        return None;
    }

    let (s, t) = segment_closest_params(a, b, c, d);
    if s < EDGE_END_MARGIN
        || s > 1. - EDGE_END_MARGIN
        || t < EDGE_END_MARGIN
        || t > 1. - EDGE_END_MARGIN
    {
        return None;
    }

    let on1 = a + direction1 * s;
    let on2 = c + direction2 * t;
    let gap = on2 - on1;
    if gap.norm_squared() > tol_sq {
        return None;
    }

    // The common perpendicular runs either way; take the way out of the first
    // body, so that the normal stays put once the two are through each other
    // rather than merely touching. Only a body folded flat along the edge leaves
    // that undecided, and then the way the second edge lies is all there is to
    // go on.
    let mut orientation = perpendicular.dot(outward1);
    if orientation.abs() < 1e-12 {
        orientation = perpendicular.dot(&gap);
    }
    let normal = UnitVector3::new_normalize(if orientation < 0. {
        -perpendicular
    } else {
        perpendicular
    });

    Some(DeepContact {
        contact: Contact::new((on1 + on2) / 2., normal),
        // The normal runs out of the first body, so the second edge sitting back
        // along it is the two crossed rather than merely close
        depth: -gap.dot(&normal),
    })
}

/// The part of the polygon inside the halfspace `{p : (p - point) . normal <= 0}`,
/// with new corners where its edges cross the boundary
pub fn clip_polygon(
    polygon: &[Vector3<Float>],
    normal: &Vector3<Float>,
    point: &Vector3<Float>,
) -> Vec<Vector3<Float>> {
    let mut clipped = vec![];
    for (i, a) in polygon.iter().enumerate() {
        let b = &polygon[(i + 1) % polygon.len()];
        let da = (a - point).dot(normal);
        let db = (b - point).dot(normal);
        if da <= 0. {
            clipped.push(*a);
        }
        // The edge crosses the boundary; cut it there
        if (da > 0.) != (db > 0.) {
            clipped.push(a + (b - a) * (da / (da - db)));
        }
    }

    clipped
}

/// The corners of the patch these items form: the deepest of them, and the
/// outermost along each of a ring of directions across the patch
///
/// Two bodies meeting face to face touch at every vertex and every edge crossing
/// in the overlap, which for anything but the coarsest mesh is hundreds of
/// contacts, each of them three rows of the contact QP. The patch is held just
/// as well by its corners, so only those are kept.
///
/// Cutting the set by depth instead does not work: tilt a flat contact by a
/// hundredth of a degree and its deepest points all lie along one edge of the
/// patch, leaving the body free to pivot about that edge and screw itself into
/// the other body.
///
/// Each item is given as its own index, where it sits, and how deep it is; what
/// comes back is the indices worth keeping, each of them once. `normal` is the
/// direction the patch faces, whose tangent plane the ring is spread in.
pub fn support_polygon(
    normal: &UnitVector3<Float>,
    items: impl IntoIterator<Item = (usize, Vector3<Float>, Float)>,
) -> Vec<usize> {
    let (t, b) = tangentials(normal);
    let directions = support_directions::<SUPPORT_DIRECTIONS>();

    // Best (index, score) for depth, then for each direction
    let mut support: [Option<(usize, Float)>; SUPPORT_DIRECTIONS + 1] =
        [None; SUPPORT_DIRECTIONS + 1];
    for (index, point, depth) in items {
        keep_furthest(&mut support[0], index, depth);

        let (along_t, along_b) = (point.dot(&t), point.dot(&b));
        for (slot, (cos, sin)) in izip!(support[1..].iter_mut(), directions.iter()) {
            keep_furthest(slot, index, along_t * cos + along_b * sin);
        }
    }

    // Neighbouring directions routinely pick the same corner, and the deepest
    // item is usually a corner too
    let mut kept: Vec<usize> = vec![];
    for &(index, _score) in support.iter().flatten() {
        if !kept.contains(&index) {
            kept.push(index);
        }
    }

    kept
}

/// Keep whichever of the two items scores higher.
fn keep_furthest(best: &mut Option<(usize, Float)>, index: usize, score: Float) {
    match best {
        Some((_, furthest)) if *furthest >= score => {}
        _ => *best = Some((index, score)),
    }
}

/// `N` directions spread evenly around the unit circle, as (cos, sin) pairs
///
/// These are the tangential directions a contact patch is sampled along to find
/// the corners of its support polygon.
fn support_directions<const N: usize>() -> [(Float, Float); N] {
    core::array::from_fn(|i| {
        let angle = 2. * PI * (i as Float) / (N as Float);
        (angle.cos(), angle.sin())
    })
}

/// A face of a mesh that came near another collider, in the working frame
pub struct CandidateFace {
    /// Its three corners
    pub points: [Vector3<Float>; 3],
    /// Which of the mesh's vertex positions those corners are, which is what
    /// names its edges: `RigidMesh::canonical`'s index rather than the face's
    /// own, so that two faces meeting along an edge name it the same way even
    /// when the mesh gives each of them its own copy of both ends. See
    /// [`distinct_edges`], whose outward direction depends on gathering both
    /// visits to an edge.
    pub indices: [usize; 3],
    /// Its outward normal, of the length the cross product gives it
    pub normal: Vector3<Float>,
}

impl CandidateFace {
    /// Its three edges, each as its two ends and the key naming it
    fn edges(&self) -> [(Vector3<Float>, Vector3<Float>, (usize, usize)); 3] {
        core::array::from_fn(|i| {
            let j = (i + 1) % 3;
            (
                self.points[i],
                self.points[j],
                edge_key(self.indices[i], self.indices[j]),
            )
        })
    }
}

/// Names an edge whichever way round its two ends are given
fn edge_key(i: usize, j: usize) -> (usize, usize) {
    if i <= j {
        (i, j)
    } else {
        (j, i)
    }
}

/// The given faces of the mesh, brought into the working frame by `transform` if
/// the mesh is not already in it, with the degenerate ones dropped
pub fn candidate_faces(
    mesh: &RigidMesh,
    i_faces: &[usize],
    transform: Option<&Isometry3<Float>>,
) -> Vec<CandidateFace> {
    i_faces
        .iter()
        .filter_map(|&i_face| {
            let corners = mesh.faces[i_face];
            let indices = corners.map(|index| mesh.canonical[index]);
            let points = corners.map(|index| match transform {
                Some(iso) => (iso * Point3::from(mesh.vertices[index])).coords,
                None => mesh.vertices[index],
            });
            let normal = (points[1] - points[0]).cross(&(points[2] - points[0]));

            // A face with no area has no outward direction to report, and no
            // room on it for a vertex to land either
            (normal.norm_squared() > DEGENERATE_FACE_AREA * DEGENERATE_FACE_AREA).then_some(
                CandidateFace {
                    points,
                    indices,
                    normal,
                },
            )
        })
        .collect()
}

/// The distinct corners of the given faces
///
/// Every face around a vertex carries it, so the indices are deduplicated here.
/// They are canonical indices, so a vertex several faces keep their own copy of
/// is one vertex here too, rather than being reported once per copy and left to
/// the weld in [`push_welded`].
pub fn candidate_vertices(faces: &[CandidateFace]) -> Vec<Vector3<Float>> {
    let mut seen = HashSet::new();
    let mut vertices = vec![];
    for face in faces.iter() {
        for (point, index) in izip!(face.points.iter(), face.indices.iter()) {
            if seen.insert(*index) {
                vertices.push(*point);
            }
        }
    }

    vertices
}

/// An edge of a mesh, and which way it faces out of it
pub struct MeshEdge {
    /// Its two ends, in the working frame
    pub ends: [Vector3<Float>; 2],
    /// The sum of the unit normals of the candidate faces carrying it
    pub outward: Vector3<Float>,
}

/// The distinct edges of the given faces, each with the way it faces out of the
/// mesh
///
/// Every interior edge belongs to two faces, so walking the faces' edges reaches
/// it twice, and an edge pass would report each crossing twice only to weld one
/// of them away again.
///
/// Gathering the two visits is also what settles the outward direction. A single
/// face's normal will not do on its own: an edge of a box belongs to two faces
/// at right angles, and whichever of them the loop happened to reach first would
/// leave the contact normal below either square on the edge or flat along it --
/// and where that normal ends up pointing *into* the mesh instead of out of it,
/// the pair of contacts is a weld rather than a rough one. So the key is
/// `CandidateFace`'s canonical index rather than the face's own, which is what
/// makes this hold for a mesh that repeats its corners instead of sharing them.
/// An edge that only one candidate face carries -- because its neighbour is out
/// of the region being looked at -- keeps that face's normal alone.
pub fn distinct_edges(faces: &[CandidateFace]) -> Vec<MeshEdge> {
    // Kept in the order the faces raise them, so that the contacts a pass
    // reports -- and hence which of two coincident ones the weld keeps -- do not
    // depend on the hashing
    let mut order: Vec<(usize, usize)> = vec![];
    let mut edges: HashMap<(usize, usize), MeshEdge> = HashMap::new();
    for face in faces.iter() {
        let unit = face.normal.normalize();
        for (a, b, key) in face.edges() {
            match edges.entry(key) {
                Entry::Occupied(mut held) => held.get_mut().outward += unit,
                Entry::Vacant(slot) => {
                    order.push(key);
                    slot.insert(MeshEdge {
                        ends: [a, b],
                        outward: unit,
                    });
                }
            }
        }
    }

    order
        .into_iter()
        .filter_map(|key| edges.remove(&key))
        .collect()
}
