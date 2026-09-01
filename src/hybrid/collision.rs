use itertools::izip;
use na::{Isometry3, Point3, UnitVector3, Vector3};

use crate::{
    collision::halfspace::HalfSpace,
    collision::mesh::{closest_point_on_triangle, projected_barycentric_coord, TriangleRegion},
    hybrid::visual::{rigid_mesh::RigidMesh, CuboidGeometry, SphereGeometry},
    types::Float,
    util::tangentials,
    PI,
};

/// Return the contact point, and contact normal from sphere to cuboid, if in contact
pub fn sphere_cuboid_collide(
    sphere_center: &Vector3<Float>,
    sphere_geometry: &SphereGeometry,
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let half = cuboid_geometry.half_extents();
    let r = sphere_geometry.r;

    // Transform sphere center into cuboid-local space
    let local_center = cuboid_iso
        .inverse_transform_point(&Point3::from(*sphere_center))
        .coords;

    // Early-out AABB check
    if local_center.x.abs() > half.x + r
        || local_center.y.abs() > half.y + r
        || local_center.z.abs() > half.z + r
    {
        return None;
    }

    // Closest point on AABB to sphere center (handles vertex/edge/face uniformly)
    let closest_local = Vector3::new(
        local_center.x.clamp(-half.x, half.x),
        local_center.y.clamp(-half.y, half.y),
        local_center.z.clamp(-half.z, half.z),
    );

    let diff = closest_local - local_center;
    let dist_sq = diff.norm_squared();

    if dist_sq > r * r {
        return None;
    }

    // Transform contact point back to world space
    let contact_world = cuboid_iso.transform_point(&Point3::from(closest_local));

    // Normal: from sphere center toward contact point, in world space
    let normal_world = cuboid_iso.transform_vector(&diff);
    Some((
        contact_world.coords,
        UnitVector3::new_normalize(normal_world),
    ))
}

/// Return the contact point, and contact normal from sphere to cuboid, if in contact
///
/// World-frame variant of `sphere_cuboid_collide`: the closest point is built
/// directly in world coordinates by projecting onto the cuboid's world-frame
/// axes, instead of round-tripping the sphere center through the cuboid-local
/// frame. This keeps the contact point numerically stable from frame to frame.
pub fn sphere_cuboid_collide_world(
    sphere_center: &Vector3<Float>,
    sphere_geometry: &SphereGeometry,
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let half = cuboid_geometry.half_extents();
    let r = sphere_geometry.r;

    let cuboid_center = cuboid_iso.translation.vector;
    let rotation = cuboid_iso.rotation.to_rotation_matrix();

    // Vector from cuboid center to sphere center, in world frame
    let d = sphere_center - cuboid_center;

    // Closest point on the cuboid to the sphere center, accumulated in world
    // frame: project d onto each cuboid axis, clamp to the half extent, and
    // add the clamped extent along that world-frame axis.
    let mut closest_world = cuboid_center;
    for axis in 0..3 {
        let u = rotation.matrix().column(axis);
        let dist = d.dot(&u).clamp(-half[axis], half[axis]);
        closest_world += dist * u;
    }

    let diff = closest_world - sphere_center;
    let dist_sq = diff.norm_squared();

    if dist_sq > r * r {
        return None;
    }

    // Normal: from sphere center toward contact point
    Some((closest_world, UnitVector3::new_normalize(diff)))
}

/// Return the contact point, and contact normal outward from the cuboid, if in contact
pub fn cuboid_point_collide(
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
    point: &Vector3<Float>,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let half = cuboid_geometry.half_extents();

    // Transform point into cuboid-local space
    let local_point = cuboid_iso
        .inverse_transform_point(&Point3::from(*point))
        .coords;

    // Early-out AABB check
    if local_point.x.abs() > half.x || local_point.y.abs() > half.y || local_point.z.abs() > half.z
    {
        return None;
    }

    // Point is inside the cuboid; find the face of least penetration
    let mut min_depth = Float::MAX;
    let mut local_normal = Vector3::zeros();
    for axis in 0..3 {
        let depth = half[axis] - local_point[axis].abs();
        if depth < min_depth {
            min_depth = depth;
            let mut n = Vector3::zeros();
            n[axis] = if local_point[axis] >= 0. { 1. } else { -1. };
            local_normal = n;
        }
    }

    let normal_world = cuboid_iso.transform_vector(&local_normal);
    Some((*point, UnitVector3::new_normalize(normal_world)))
}

/// How far apart two cuboids can be and still count as touching.
///
/// Matches the tolerance `mesh_cuboid_collide` uses, so a body carrying a mesh
/// and one carrying the equivalent cuboid make contact at the same distance.
const CUBOID_CUBOID_CONTACT_TOLERANCE: Float = 1e-3;

/// How much a face axis is favoured over an edge-edge axis of the same depth.
///
/// A cuboid resting squarely on another has a face axis and several edge-edge
/// axes overlapping by exactly the same amount, and rounding alone decides
/// between them. Only the face axis yields a polygon of contact points rather
/// than a single one, so it has to win those ties.
const CUBOID_CUBOID_EDGE_AXIS_BIAS: Float = 1e-6;

/// Which pair of features the separating-axis test settled on.
#[derive(Clone, Copy)]
enum SatFeature {
    /// A face of the first cuboid
    Face1,
    /// A face of the second cuboid
    Face2,
    /// An edge of the first cuboid along its axis `i`, against an edge of the
    /// second along its axis `j`
    Edge(usize, usize),
}

/// Collision detection between two cuboids
/// Returns a list of (contact point, normal) where normal points from cuboid1 to cuboid2
///
/// The normal is the axis of least overlap found by the separating-axis test
/// over the 15 candidate axes: the three face normals of each cuboid, and the
/// nine cross products of their edge directions. Pairs nowhere near each other
/// are dropped before that, by their bounding spheres and then their AABBs.
///
/// A face contact is reported as the polygon the two facing faces share, up to
/// four points, so that a cuboid landing flat is held by its whole face instead
/// of pivoting on one corner; an edge-edge contact is the single point where the
/// two edges come closest.
pub fn cuboid_cuboid_collide(
    iso1: &Isometry3<Float>,
    geometry1: &CuboidGeometry,
    iso2: &Isometry3<Float>,
    geometry2: &CuboidGeometry,
) -> Vec<(Vector3<Float>, UnitVector3<Float>)> {
    let tol = CUBOID_CUBOID_CONTACT_TOLERANCE;

    let half1 = geometry1.half_extents();
    let half2 = geometry2.half_extents();
    let axes1 = cuboid_axes(iso1);
    let axes2 = cuboid_axes(iso2);

    // Between the centers, i.e. the direction from cuboid 1 towards cuboid 2
    let d = iso2.translation.vector - iso1.translation.vector;

    // Early-out: two cuboids too far apart to touch are the common case -- most
    // pairs in a scene never come near each other -- so cut them with a single
    // length comparison between their bounding spheres, before the axes below.
    let reach = half1.norm() + half2.norm() + tol;
    if d.norm_squared() > reach * reach {
        return vec![];
    }

    // Early-out: and then with the world axes, which the 15 candidates below do
    // not cover. A cuboid reaches |R| * half from its center along a world axis,
    // so this is the overlap test of the two world-frame AABBs.
    for axis in 0..3 {
        let reach: Float = (0..3)
            .map(|i| half1[i] * axes1[i][axis].abs() + half2[i] * axes2[i][axis].abs())
            .sum();
        if d[axis].abs() > reach + tol {
            return vec![];
        }
    }

    // The three face normals of each cuboid, then the nine cross products of
    // their edge directions. Face axes come first: they separate far more pairs
    // than the cross products do, and the test stops at the first axis that does.
    let candidates: [(Vector3<Float>, SatFeature); 15] = core::array::from_fn(|c| match c {
        0..=2 => (axes1[c], SatFeature::Face1),
        3..=5 => (axes2[c - 3], SatFeature::Face2),
        _ => {
            let (i, j) = ((c - 6) / 3, (c - 6) % 3);
            (axes1[i].cross(&axes2[j]), SatFeature::Edge(i, j))
        }
    });

    let mut best_overlap = Float::MAX;
    let mut best_axis = Vector3::zeros();
    let mut best_feature = SatFeature::Face1;
    for (axis, feature) in candidates {
        // Parallel edges leave no cross product to test along, and the face
        // axes already cover every direction that pair could separate along
        let norm = axis.norm();
        if norm < 1e-9 {
            continue;
        }
        let axis = axis / norm;

        // How far the two cuboids reach towards each other along the axis, less
        // how far apart their centers are along it
        let overlap = cuboid_radius(&half1, &axes1, &axis) + cuboid_radius(&half2, &axes2, &axis)
            - d.dot(&axis).abs();
        if overlap < -tol {
            return vec![];
        }

        let biased = match feature {
            SatFeature::Edge(_, _) => overlap + CUBOID_CUBOID_EDGE_AXIS_BIAS,
            _ => overlap,
        };
        if biased < best_overlap {
            best_overlap = biased;
            // Orient every candidate axis from cuboid 1 towards cuboid 2
            best_axis = if d.dot(&axis) >= 0. { axis } else { -axis };
            best_feature = feature;
        }
    }

    let normal = UnitVector3::new_normalize(best_axis);
    let contacts = match best_feature {
        SatFeature::Face1 => {
            cuboid_face_contacts(iso1, &half1, &axes1, &normal, iso2, &half2, &axes2, tol)
        }
        // The reference face is on cuboid 2, so it faces back towards cuboid 1
        SatFeature::Face2 => {
            cuboid_face_contacts(iso2, &half2, &axes2, &(-normal), iso1, &half1, &axes1, tol)
        }
        SatFeature::Edge(i, j) => {
            cuboid_edge_contacts(iso1, &half1, &axes1, i, iso2, &half2, &axes2, j, &normal)
        }
    };

    contacts.into_iter().map(|cp| (cp, normal)).collect()
}

/// The cuboid's own axes, in world frame
fn cuboid_axes(iso: &Isometry3<Float>) -> [Vector3<Float>; 3] {
    let rotation = iso.rotation.to_rotation_matrix();
    core::array::from_fn(|i| rotation.matrix().column(i).into_owned())
}

/// How far the cuboid reaches from its center along `direction`, a unit vector
fn cuboid_radius(
    half: &Vector3<Float>,
    axes: &[Vector3<Float>; 3],
    direction: &Vector3<Float>,
) -> Float {
    (0..3).map(|i| half[i] * axes[i].dot(direction).abs()).sum()
}

/// The face of the cuboid pointing the furthest along `direction`, as its
/// center, its outward normal, and its two side axes each scaled to the half
/// extent along it
fn cuboid_support_face(
    iso: &Isometry3<Float>,
    half: &Vector3<Float>,
    axes: &[Vector3<Float>; 3],
    direction: &Vector3<Float>,
) -> (Vector3<Float>, Vector3<Float>, [Vector3<Float>; 2]) {
    let mut i_axis = 0;
    for i in 1..3 {
        if axes[i].dot(direction).abs() > axes[i_axis].dot(direction).abs() {
            i_axis = i;
        }
    }

    let sign = if axes[i_axis].dot(direction) >= 0. {
        1.
    } else {
        -1.
    };
    let normal = sign * axes[i_axis];
    let center = iso.translation.vector + half[i_axis] * normal;
    let (j, k) = ((i_axis + 1) % 3, (i_axis + 2) % 3);

    (center, normal, [half[j] * axes[j], half[k] * axes[k]])
}

/// The four corners of a face, in winding order
fn face_corners(center: &Vector3<Float>, sides: &[Vector3<Float>; 2]) -> [Vector3<Float>; 4] {
    [
        center + sides[0] + sides[1],
        center + sides[0] - sides[1],
        center - sides[0] - sides[1],
        center - sides[0] + sides[1],
    ]
}

/// The part of the polygon inside the halfspace `{p : (p - point) . normal <= 0}`,
/// with new corners where its edges cross the boundary
fn clip_polygon(
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

/// The contact points of the reference cuboid's face pointing along `direction`
/// against the incident cuboid, i.e. the corners of the incident cuboid's facing
/// face that lie over the reference face and reach it
fn cuboid_face_contacts(
    ref_iso: &Isometry3<Float>,
    ref_half: &Vector3<Float>,
    ref_axes: &[Vector3<Float>; 3],
    direction: &UnitVector3<Float>,
    inc_iso: &Isometry3<Float>,
    inc_half: &Vector3<Float>,
    inc_axes: &[Vector3<Float>; 3],
    tol: Float,
) -> Vec<Vector3<Float>> {
    let (ref_center, ref_normal, ref_sides) =
        cuboid_support_face(ref_iso, ref_half, ref_axes, direction);
    // The incident face is the one turned back towards the reference face
    let (inc_center, _inc_normal, inc_sides) =
        cuboid_support_face(inc_iso, inc_half, inc_axes, &(-direction.into_inner()));

    // Cut the incident face down to the part of it lying over the reference
    // face, against the reference face's four side planes
    let mut polygon = face_corners(&inc_center, &inc_sides).to_vec();
    for side in ref_sides.iter() {
        for sign in [1., -1.] {
            let normal = sign * side;
            polygon = clip_polygon(&polygon, &normal, &(ref_center + normal));
            if polygon.is_empty() {
                return vec![];
            }
        }
    }

    // Keep the corners that reach the reference face, and put the contact
    // halfway across the gap -- or the overlap -- so that it favours neither
    // cuboid's surface
    polygon
        .into_iter()
        .filter_map(|p| {
            let depth = (p - ref_center).dot(&ref_normal);
            (depth <= tol).then(|| p - 0.5 * depth * ref_normal)
        })
        .collect()
}

/// The cuboid's edge along its axis `i_axis` that reaches furthest along `direction`
fn cuboid_support_edge(
    iso: &Isometry3<Float>,
    half: &Vector3<Float>,
    axes: &[Vector3<Float>; 3],
    i_axis: usize,
    direction: &Vector3<Float>,
) -> [Vector3<Float>; 2] {
    let mut center = iso.translation.vector;
    for i in 0..3 {
        if i == i_axis {
            continue;
        }
        let sign = if axes[i].dot(direction) >= 0. {
            1.
        } else {
            -1.
        };
        center += sign * half[i] * axes[i];
    }

    let offset = half[i_axis] * axes[i_axis];
    [center - offset, center + offset]
}

/// The closest pair of points on two segments, one on each
fn segment_closest_points(
    p1: &Vector3<Float>,
    q1: &Vector3<Float>,
    p2: &Vector3<Float>,
    q2: &Vector3<Float>,
) -> (Vector3<Float>, Vector3<Float>) {
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
        return (*p1, *p2);
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

    (p1 + d1 * s, p2 + d2 * t)
}

/// The single contact point where the two cuboids' closest edges, along the
/// given axis of each, come nearest to each other
fn cuboid_edge_contacts(
    iso1: &Isometry3<Float>,
    half1: &Vector3<Float>,
    axes1: &[Vector3<Float>; 3],
    i1: usize,
    iso2: &Isometry3<Float>,
    half2: &Vector3<Float>,
    axes2: &[Vector3<Float>; 3],
    i2: usize,
    normal: &UnitVector3<Float>,
) -> Vec<Vector3<Float>> {
    let edge1 = cuboid_support_edge(iso1, half1, axes1, i1, &normal.into_inner());
    let edge2 = cuboid_support_edge(iso2, half2, axes2, i2, &(-normal.into_inner()));
    let (p1, p2) = segment_closest_points(&edge1[0], &edge1[1], &edge2[0], &edge2[1]);

    // Halfway across the gap -- or the overlap -- between the two edges
    vec![(p1 + p2) / 2.]
}

/// Collision detection between mesh and sphere
/// Returns a list of (contact point, normal) where normal points from mesh to sphere
pub fn mesh_sphere_collide(
    mesh: &RigidMesh,
    sphere_center: &Vector3<Float>,
    sphere_radius: Float,
) -> Vec<(Vector3<Float>, UnitVector3<Float>)> {
    let mut cp_normal_list = vec![];

    // Early-out: every contact point lies on the mesh, hence inside its AABB;
    // no contact if the sphere center is farther than the radius from the AABB
    let closest = sphere_center.sup(&mesh.aabb_min).inf(&mesh.aabb_max);
    if (closest - sphere_center).norm_squared() > sphere_radius * sphere_radius {
        return cp_normal_list;
    }

    let vertices = &mesh.vertices;
    let radius_sq = sphere_radius * sphere_radius;

    // The mesh feature (in global vertex indices) that a closest point lies on
    enum Feature {
        Face([usize; 3]),
        Edge(usize, usize),
        Vertex(usize),
    }

    // Only faces whose AABB overlaps the sphere's AABB can be in contact
    let r_vec = Vector3::repeat(sphere_radius);
    let mut candidates: Vec<(Vector3<Float>, Vector3<Float>, Feature)> = vec![];
    for i_face in mesh
        .bvh
        .collect_overlapping(&(sphere_center - r_vec), &(sphere_center + r_vec))
    {
        let face = &mesh.faces[i_face];
        let v1 = vertices[face[0]];
        let v2 = vertices[face[1]];
        let v3 = vertices[face[2]];

        // Closest point on the triangle, handling vertex/edge/face regions
        let (closest_point, region) = closest_point_on_triangle(sphere_center, &v1, &v2, &v3);
        let closest_point_to_sphere_center = sphere_center - closest_point;
        if closest_point_to_sphere_center.norm_squared() > radius_sq {
            continue;
        }

        let feature = match region {
            TriangleRegion::Face => Feature::Face(*face),
            TriangleRegion::Edge(a, b) => Feature::Edge(face[a], face[b]),
            TriangleRegion::Vertex(a) => Feature::Vertex(face[a]),
        };
        candidates.push((closest_point, closest_point_to_sphere_center, feature));
    }

    // Contact welding: a contact on an edge or vertex is redundant when an
    // adjacent triangle already contacts the sphere on its interior, since
    // that triangle owns the region around the shared feature; keeping the
    // edge/vertex contact would add a duplicate with a tilted normal that
    // pushes the sphere sideways on flat, triangulated surfaces. Contacts on
    // boundary features (e.g. the rim of a table) have no such adjacent
    // face contact and survive.
    for (closest_point, diff, feature) in candidates.iter() {
        let covered = match feature {
            Feature::Face(_) => false,
            Feature::Edge(i, j) => candidates.iter().any(
                |(_, _, f)| matches!(f, Feature::Face(vs) if vs.contains(i) && vs.contains(j)),
            ),
            Feature::Vertex(i) => candidates.iter().any(|(_, _, f)| match f {
                Feature::Face(vs) => vs.contains(i),
                Feature::Edge(a, b) => a == i || b == i,
                Feature::Vertex(_) => false,
            }),
        };
        if covered {
            continue;
        }

        // Skip duplicate contacts from adjacent faces sharing the vertex or
        // edge that the closest point lies on
        if cp_normal_list
            .iter()
            .any(|(cp, _): &(Vector3<Float>, _)| (cp - closest_point).norm_squared() < 1e-12)
        {
            continue;
        }

        let n = UnitVector3::new_normalize(*diff);
        cp_normal_list.push((*closest_point, n));
    }

    cp_normal_list
}

/// Collision detection between a mesh and a point
/// Returns a list of (contact point, normal) where normal points from mesh to point
pub fn mesh_point_collide(
    mesh: &RigidMesh,
    point: &Vector3<Float>,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let tol = 1e-3;

    // Early-out: contact requires the point to be within tol of a face, and
    // every face lies inside the mesh AABB
    let closest = point.sup(&mesh.aabb_min).inf(&mesh.aabb_max);
    if (closest - point).norm_squared() > tol * tol {
        return None;
    }

    let vertices = &mesh.vertices;

    // Only faces whose AABB overlaps the tol-inflated point can be in contact
    let tol_vec = Vector3::repeat(tol);
    let candidates = mesh
        .bvh
        .collect_overlapping(&(point - tol_vec), &(point + tol_vec));
    for i_face in candidates {
        let face = &mesh.faces[i_face];
        let v1 = vertices[face[0]];
        let v2 = vertices[face[1]];
        let v3 = vertices[face[2]];
        let edge1 = v2 - v1;
        let edge2 = v3 - v1;

        let (w1, w2, w3) = projected_barycentric_coord(point, &v1, &edge1, &edge2);

        // Check if closest point is inside the face
        if w1 < 0. || w2 < 0. || w3 < 0. {
            continue;
        }

        // Projected point
        let closest_point = w1 * v1 + w2 * v2 + w3 * v3;

        // check if point is close to the face
        // TODO: do ccd to avoid the passing through case
        if (point - closest_point).norm_squared() < tol * tol {
            let normal = UnitVector3::new_normalize(edge1.cross(&edge2)); // outward normal of the face
            return Some((*point, normal));
        }
    }
    None
}

/// Collision detection between a mesh and a cuboid
/// Returns a list of (contact point, normal) where normal points from mesh to cuboid
pub fn mesh_cuboid_collide(
    mesh: &RigidMesh,
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
) -> Vec<(Vector3<Float>, UnitVector3<Float>)> {
    let mut cp_normal_list = vec![];
    let tol = 1e-3;

    // Early-out: contact requires a cuboid corner within tol of a mesh face,
    // so the cuboid's world-frame AABB must overlap the mesh AABB inflated by
    // tol. The cuboid's AABB half-extent along each world axis is |R| * half.
    let half = cuboid_geometry.half_extents();
    let extent = cuboid_iso.rotation.to_rotation_matrix().matrix().abs() * half;
    let center = cuboid_iso.translation.vector;
    for axis in 0..3 {
        if center[axis] - extent[axis] > mesh.aabb_max[axis] + tol
            || center[axis] + extent[axis] < mesh.aabb_min[axis] - tol
        {
            return cp_normal_list;
        }
    }

    let vertices = &mesh.vertices;

    // cuboid point - mesh face
    // Only corners within tol of the mesh AABB can contact a face; filter the
    // rest out once instead of testing them against every face
    let tol_sq = tol * tol;
    let cuboid_points: Vec<Vector3<Float>> = cuboid_geometry
        .points(cuboid_iso)
        .into_iter()
        .filter(|p| {
            let closest = p.sup(&mesh.aabb_min).inf(&mesh.aabb_max);
            (closest - p).norm_squared() <= tol_sq
        })
        .collect();
    if cuboid_points.is_empty() {
        return cp_normal_list;
    }

    // Only faces whose AABB overlaps the tol-inflated AABB of the surviving
    // corners can be in contact
    let mut qmin = Vector3::repeat(Float::MAX);
    let mut qmax = Vector3::repeat(Float::MIN);
    for point in cuboid_points.iter() {
        qmin = qmin.inf(point);
        qmax = qmax.sup(point);
    }
    let tol_vec = Vector3::repeat(tol);
    let candidates = mesh
        .bvh
        .collect_overlapping(&(qmin - tol_vec), &(qmax + tol_vec));

    for i_face in candidates {
        let face = &mesh.faces[i_face];
        let v1 = vertices[face[0]];
        let v2 = vertices[face[1]];
        let v3 = vertices[face[2]];
        let edge1 = v2 - v1;
        let edge2 = v3 - v1;
        for point in cuboid_points.iter() {
            let (w1, w2, w3) = projected_barycentric_coord(point, &v1, &edge1, &edge2);

            // Check if closest point is inside the face
            if w1 < 0. || w2 < 0. || w3 < 0. {
                continue;
            }

            // Projected point
            let closest_point = w1 * v1 + w2 * v2 + w3 * v3;

            // check if point is close to the face
            // TODO: do ccd to avoid the passing through case
            if (point - closest_point).norm_squared() < tol_sq {
                let normal = UnitVector3::new_normalize(edge1.cross(&edge2)); // outward normal of the face
                cp_normal_list.push((*point, normal));
            }
        }
    }

    // TODO: cuboid face - mesh point
    // TODO: edge - edge

    cp_normal_list
}

/// How close to the halfspace a vertex counts as touching it.
///
/// Matches `HalfSpace::has_inside`, so a mesh contacts the plane at the same
/// point a collision sphere or cuboid on the same body would. Widening it into a
/// speculative band is tempting but buys nothing: the body just comes to rest
/// hovering at the edge of the band instead of on the plane.
const MESH_HALFSPACE_CONTACT_TOLERANCE: Float = 1e-8;

/// Tangential directions the contact patch is sampled along, on top of the
/// deepest point. See [`mesh_halfspace_collide`].
const MESH_HALFSPACE_SUPPORT_DIRECTIONS: usize = 8;

/// Distance below which two contact points count as the same one.
///
/// STL meshes repeat a vertex once per triangle sharing it, and neighbouring
/// sample directions routinely pick the same corner; duplicated contact points
/// are duplicated rows in the solve.
const MESH_HALFSPACE_WELD_DISTANCE: Float = 1e-6;

/// Collision detection between a mesh and a halfspace.
/// Returns a list of (contact point, normal) where normal points out of the
/// halfspace, i.e. from the halfspace to the mesh.
///
/// `mesh_iso` takes the mesh from its own frame to world. Unlike the static-body
/// meshes above, a mesh hanging off an articulated body moves every step, so the
/// halfspace is brought into the mesh's frame rather than transforming (and
/// re-BVH-ing) the mesh into the world's.
///
/// A vertex counts as touching once it is within `MESH_HALFSPACE_CONTACT_TOLERANCE`
/// of the plane, and the contacts are reduced to the corners of the patch the
/// mesh rests on, at most `MESH_HALFSPACE_SUPPORT_DIRECTIONS + 1` of them. Both
/// exist so that a body resting on a face is held by that whole face, rather
/// than by whichever vertices happen to be deepest.
pub fn mesh_halfspace_collide(
    mesh: &RigidMesh,
    mesh_iso: &Isometry3<Float>,
    halfspace: &HalfSpace,
) -> Vec<(Vector3<Float>, UnitVector3<Float>)> {
    let tol = MESH_HALFSPACE_CONTACT_TOLERANCE;

    // The halfspace expressed in mesh frame. Rotation preserves the norm, so the
    // transformed normal is still a unit vector.
    let normal = mesh_iso.inverse_transform_vector(&halfspace.normal);
    let point = mesh_iso
        .inverse_transform_point(&Point3::from(halfspace.point))
        .coords;

    // Early-out: the deepest the mesh can reach is the AABB corner furthest
    // along -normal, so if that corner clears the plane nothing else is inside
    let mut deepest_corner = Vector3::zeros();
    for axis in 0..3 {
        deepest_corner[axis] = if normal[axis] >= 0. {
            mesh.aabb_min[axis]
        } else {
            mesh.aabb_max[axis]
        };
    }
    if (deepest_corner - point).dot(&normal) > tol {
        return vec![];
    }

    // A mesh landing flat puts hundreds of vertices under the plane at once and
    // each one is three rows in the contact QP, so the set has to be cut down.
    // Cutting it by depth does not work: tilt a flat face by a hundredth of a
    // degree and its deepest vertices all lie along the single lowest edge. A
    // body supported on a line is free to rotate about that line, nothing pushes
    // the penetration back out again, and the face pivots its way into the
    // ground. So keep the support polygon instead -- the deepest vertex plus the
    // outermost one along each of a ring of tangential directions.
    let (t, b) = tangentials(&UnitVector3::new_normalize(normal));
    let directions: [(Float, Float); MESH_HALFSPACE_SUPPORT_DIRECTIONS] =
        core::array::from_fn(|i| {
            let angle = 2. * PI * (i as Float) / (MESH_HALFSPACE_SUPPORT_DIRECTIONS as Float);
            (angle.cos(), angle.sin())
        });

    // Best (vertex index, score) for depth, then for each direction
    let mut support: [Option<(usize, Float)>; MESH_HALFSPACE_SUPPORT_DIRECTIONS + 1] =
        [None; MESH_HALFSPACE_SUPPORT_DIRECTIONS + 1];

    // A polyhedron's deepest point below a plane is always a vertex, so testing
    // vertices alone finds every contact -- no face or edge pass needed.
    //
    // Only the vertices inside the halfspace can score, and the BVH cuts the
    // scan down to the faces reaching in: the slab a resting mesh has below the
    // plane, rather than all of its vertices, which for a robot's visual mesh is
    // tens of thousands. The faces it returns are a superset of the ones
    // reaching in, and a vertex shared by several of them is simply scored more
    // than once, so the support set is the same one the full scan finds.
    for i_face in mesh.bvh.collect_below_plane(&normal, &point, tol) {
        for index in mesh.faces[i_face] {
            let offset = mesh.vertices[index] - point;
            let depth = offset.dot(&normal);
            if depth > tol {
                continue;
            }

            keep_furthest(&mut support[0], index, -depth);

            let (along_t, along_b) = (offset.dot(&t), offset.dot(&b));
            for (slot, (cos, sin)) in izip!(support[1..].iter_mut(), directions.iter()) {
                keep_furthest(slot, index, along_t * cos + along_b * sin);
            }
        }
    }

    let mut contacts: Vec<(Vector3<Float>, UnitVector3<Float>)> = vec![];
    let mut kept: Vec<Vector3<Float>> = vec![];
    for &(index, _score) in support.iter().flatten() {
        let vertex = mesh.vertices[index];
        if kept.iter().any(|point| {
            (point - vertex).norm_squared()
                < MESH_HALFSPACE_WELD_DISTANCE * MESH_HALFSPACE_WELD_DISTANCE
        }) {
            continue;
        }
        kept.push(vertex);
        contacts.push(((mesh_iso * Point3::from(vertex)).coords, halfspace.normal));
    }

    contacts
}

/// Keep whichever of the two vertices scores higher.
fn keep_furthest(best: &mut Option<(usize, Float)>, index: usize, score: Float) {
    match best {
        Some((_, furthest)) if *furthest >= score => {}
        _ => *best = Some((index, score)),
    }
}

#[cfg(test)]
mod collision_tests {
    use na::{vector, UnitQuaternion, UnitVector3, Vector3};

    use crate::{
        assert_vec_close,
        collision::halfspace::HalfSpace,
        hybrid::{articulated::Articulated, Hybrid, Rigid},
        inertia::SpatialInertia,
        joint::{Joint, JointVelocity},
        spatial::{pose::Pose, spatial_vector::SpatialVector, transform::Transform3D},
        types::Float,
        WORLD_FRAME,
    };

    /// A mesh contacts a halfspace on the vertices below it, reported in world
    /// frame, and only when it actually reaches the plane
    #[test]
    fn mesh_halfspace_collide_finds_vertices_below_the_plane() {
        use na::{Isometry3, Translation3};

        use crate::{
            hybrid::{
                collision::{mesh_halfspace_collide, MESH_HALFSPACE_CONTACT_TOLERANCE},
                visual::rigid_mesh::RigidMesh,
            },
            PI,
        };

        // Unit cube centred on its own origin
        let cube = RigidMesh::new_from_obj(
            "v -0.5 -0.5 -0.5\nv 0.5 -0.5 -0.5\nv 0.5 0.5 -0.5\nv -0.5 0.5 -0.5\n\
             v -0.5 -0.5 0.5\nv 0.5 -0.5 0.5\nv 0.5 0.5 0.5\nv -0.5 0.5 0.5\n\
             f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n\
             f 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n",
        );
        let ground = HalfSpace::new(Vector3::z_axis(), 0.);
        let tol = 1e-9;

        // Straddling the plane: the four bottom vertices are inside it
        let contacts = mesh_halfspace_collide(&cube, &Isometry3::identity(), &ground);
        assert_eq!(contacts.len(), 4);
        for (cp, normal) in contacts.iter() {
            assert!((cp.z + 0.5).abs() < tol);
            // The contact normal is the halfspace's own, pointing out of it
            assert!((normal.z - 1.).abs() < tol);
        }

        // Clear of the plane: no contact
        let above = Isometry3::translation(0., 0., 1.);
        assert!(mesh_halfspace_collide(&cube, &above, &ground).is_empty());

        // Within the contact band the face still carries the body, so that a
        // face resting flat is held by all of itself and not by the sliver of it
        // that happens to have gone through the plane
        let hovering = Isometry3::translation(0., 0., 0.5 + MESH_HALFSPACE_CONTACT_TOLERANCE / 2.);
        assert_eq!(mesh_halfspace_collide(&cube, &hovering, &ground).len(), 4);

        // Past the band, nothing
        let clear = Isometry3::translation(0., 0., 0.5 + 5. * MESH_HALFSPACE_CONTACT_TOLERANCE);
        assert!(mesh_halfspace_collide(&cube, &clear, &ground).is_empty());

        // A quarter turn about x maps the cube's -y face downwards; contacts
        // come back in world frame, so they follow the isometry
        let tipped = Isometry3::from_parts(
            Translation3::new(0., 0., -0.25),
            UnitQuaternion::from_euler_angles(PI / 2., 0., 0.),
        );
        let contacts = mesh_halfspace_collide(&cube, &tipped, &ground);
        assert_eq!(contacts.len(), 4);
        for (cp, _normal) in contacts.iter() {
            assert!((cp.z + 0.75).abs() < 1e-6);
        }
    }

    /// A face resting on the plane is held by a polygon, not by the row of
    /// vertices that happen to be deepest
    #[test]
    fn mesh_halfspace_collide_keeps_the_support_polygon_of_a_tilted_face() {
        use na::{Isometry3, Translation3};

        use crate::{
            hybrid::{collision::mesh_halfspace_collide, visual::rigid_mesh::RigidMesh},
            PI,
        };

        // A 1 m square plate, finely subdivided. Ranking contacts by depth only
        // goes wrong once a face carries far more vertices than the reduction
        // keeps, which is every real robot mesh.
        let n = 10;
        let mut obj = String::new();
        for i in 0..=n {
            for j in 0..=n {
                let (x, y) = (
                    -0.5 + i as Float / n as Float,
                    -0.5 + j as Float / n as Float,
                );
                obj.push_str(&format!("v {x} {y} 0\n"));
            }
        }
        let index = |i: usize, j: usize| i * (n + 1) + j + 1; // obj indices are 1-based
        for i in 0..n {
            for j in 0..n {
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    index(i, j),
                    index(i + 1, j),
                    index(i + 1, j + 1)
                ));
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    index(i, j),
                    index(i + 1, j + 1),
                    index(i, j + 1)
                ));
            }
        }
        let plate = RigidMesh::new_from_obj(&obj);
        let ground = HalfSpace::new(Vector3::z_axis(), 0.);

        // Tilted by a hundredth of a degree and set just below the plane, the
        // way a body sits while it rocks itself level
        let resting = Isometry3::from_parts(
            Translation3::new(0., 0., -1e-4),
            UnitQuaternion::from_euler_angles(0.01 * PI / 180., 0., 0.),
        );
        let contacts = mesh_halfspace_collide(&plate, &resting, &ground);
        assert!(contacts.len() >= 3);

        // The contacts have to span the plate both ways. Taken by depth they
        // would all sit along its lowest edge, leaving the body free to pivot
        // about that edge and sink.
        let span = |values: Vec<Float>| {
            values.iter().cloned().fold(Float::MIN, Float::max)
                - values.iter().cloned().fold(Float::MAX, Float::min)
        };
        assert!(span(contacts.iter().map(|(cp, _)| cp.x).collect()) > 0.5);
        assert!(span(contacts.iter().map(|(cp, _)| cp.y).collect()) > 0.5);
    }

    /// A body resting on its mesh stays where it was put
    #[test]
    fn mesh_resting_on_halfspace_does_not_drift() {
        use na::{Isometry3, Matrix3};

        use crate::{
            hybrid::visual::{rigid_mesh::RigidMesh, Visual},
            joint::JointPosition,
        };

        // A plate the size of the test robot's footprint, 48 x 68 mm
        let (half_x, half_y) = (0.024, 0.034);
        let n = 10;
        let mut obj = String::new();
        for i in 0..=n {
            for j in 0..=n {
                let x = -half_x + 2. * half_x * i as Float / n as Float;
                let y = -half_y + 2. * half_y * j as Float / n as Float;
                obj.push_str(&format!("v {x} {y} 0\n"));
            }
        }
        let at = |i: usize, j: usize| i * (n + 1) + j + 1;
        for i in 0..n {
            for j in 0..n {
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    at(i, j),
                    at(i + 1, j),
                    at(i + 1, j + 1)
                ));
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    at(i, j),
                    at(i + 1, j + 1),
                    at(i, j + 1)
                ));
            }
        }

        // Mass and inertia of the test robot's base link
        let m = 0.0157812;
        let com = vector![0.0025, -0.0256, 0.0102];
        let moment_com = Matrix3::new(
            3.62346e-06,
            7.4487e-08,
            -4.8746e-08,
            7.4487e-08,
            1.46159e-06,
            4.94826e-07,
            -4.8746e-08,
            4.94826e-07,
            4.45457e-06,
        );

        let drift_of = |com: Vector3<Float>| {
            let frame = "plate";
            let moment =
                moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
            let mut body = Rigid::new(SpatialInertia::new(moment, m * com, m, frame));
            body.visual.push((
                Visual::RigidMesh(RigidMesh::new_from_obj(&obj)),
                Isometry3::identity(),
                None,
                None,
                true,
            ));

            let mut articulated = Articulated::new(
                vec![body],
                vec![Joint::new_floating(Transform3D::identity(
                    frame,
                    WORLD_FRAME,
                ))],
            );
            // Set down a hair above the plane, the way a dropped robot is
            articulated.set_joint_q(
                0,
                JointPosition::Pose(Pose::translation(vector![0., 0., 1e-3])),
            );

            let mut hybrid = Hybrid::empty();
            hybrid.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));
            hybrid.add_articulated(articulated);
            for _ in 0..300 {
                hybrid.step(1. / 60., &vec![]);
            }
            hybrid.articulated[0].bodies[0].pose.translation
        };

        let centered = drift_of(Vector3::zeros());
        let offset = drift_of(com);
        println!("centred CoM: {centered:?}");
        println!("offset CoM:  {offset:?}");

        // 5 s of resting must not walk the body anywhere
        for drift in [centered, offset] {
            assert!(drift.x.abs() < 1e-4, "drifted {} m in x", drift.x);
            assert!(drift.y.abs() < 1e-4, "drifted {} m in y", drift.y);
        }
    }

    /// The world-frame variant agrees with the local-frame one on vertex,
    /// edge, and face contacts of a rotated cuboid
    #[test]
    fn sphere_cuboid_collide_world_matches_local() {
        use na::Isometry3;

        use crate::hybrid::{
            collision::{sphere_cuboid_collide, sphere_cuboid_collide_world},
            visual::{CuboidGeometry, SphereGeometry},
        };

        let sphere = SphereGeometry { r: 1.0 };
        let cuboid = CuboidGeometry {
            w: 2.0,
            d: 1.0,
            h: 0.5,
        };
        let iso = Isometry3::new(vector![0.3, -0.2, 0.1], vector![0.4, 0.5, 0.6]);

        // Sphere centers producing face, edge, vertex contacts, and a miss
        let centers = [
            vector![0.3, -0.2, 1.2], // near top face
            vector![1.5, 0.5, 0.5],  // near an edge
            vector![1.5, 0.9, 0.6],  // near a vertex
            vector![5.0, 5.0, 5.0],  // no contact
        ];

        for center in centers.iter() {
            let local = sphere_cuboid_collide(center, &sphere, &iso, &cuboid);
            let world = sphere_cuboid_collide_world(center, &sphere, &iso, &cuboid);
            match (local, world) {
                (Some((cp_l, n_l)), Some((cp_w, n_w))) => {
                    assert_vec_close!(cp_l, cp_w, 1e-9);
                    assert_vec_close!(n_l, n_w, 1e-9);
                }
                (None, None) => {}
                (l, w) => panic!("variants disagree: local={:?} world={:?}", l, w),
            }
        }
    }

    /// collision between a halfspace and an articulated with point contacts
    #[test]
    fn halfspace_articulated_points() {
        // Arrange
        let mut state = Hybrid::empty();

        let halfspace = HalfSpace::new(Vector3::z_axis(), 0.0);
        state.add_halfspace(halfspace);

        let frame = "body";
        let m = 1.;
        let w = 1.;
        let inertia = SpatialInertia::cuboid_at(&Vector3::zeros(), m, w, w, w, frame);
        let mut cube = Rigid::new(inertia);
        // add four point contacts at the bottom
        cube.add_point_at(&vector![w / 2., w / 2., -w / 2.]);
        cube.add_point_at(&vector![w / 2., -w / 2., -w / 2.]);
        cube.add_point_at(&vector![-w / 2., w / 2., -w / 2.]);
        cube.add_point_at(&vector![-w / 2., -w / 2., -w / 2.]);
        let joint = Joint::new_floating(Transform3D::move_z(frame, WORLD_FRAME, w));
        let articulated = Articulated::new(vec![cube], vec![joint]);

        state.add_articulated(articulated);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let pose = state.articulated[0].bodies[0].pose;
        let v = state.articulated[0].v();
        assert_vec_close!(
            pose.as_dvec(),
            Pose::translation(vector![0., 0., w / 2.]).as_dvec(),
            1e-3
        );
        assert_vec_close!(v, SpatialVector::zero().as_dvector(), 1e-3);
    }

    /// collision between a halfspace and an articulated sphere
    #[test]
    fn halfspace_articulated_sphere() {
        // Arrange
        let mut state = Hybrid::empty();

        let halfspace = HalfSpace::new(Vector3::z_axis(), 0.0);
        state.add_halfspace(halfspace);

        let frame = "body";
        let m = 1.;
        let r = 1.;
        let sphere = Rigid::new_sphere(m, r, frame);
        let sphere_joint = Joint::new_floating(Transform3D::move_z(frame, WORLD_FRAME, 2. * r));
        let articulated = Articulated::new(vec![sphere], vec![sphere_joint]);

        state.add_articulated(articulated);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let pose = state.articulated[0].bodies[0].pose;
        let v = state.articulated[0].v();
        assert_vec_close!(
            pose.as_dvec(),
            Pose::translation(vector![0., 0., r]).as_dvec(),
            1e-2
        );
        assert_vec_close!(v, SpatialVector::zero().as_dvector(), 1e-3);
    }

    /// Collision between spheres of two articulated bodies
    #[test]
    fn articulated_spheres() {
        // Arrange
        let mut state = Hybrid::empty();
        state.disable_gravity();

        let m = 1.0;
        let r = 1.0;
        let sphere = Articulated::new_sphere("sphere", m, r);

        let v = -5.0;
        let mut sphere2 = Articulated::new_sphere_at("sphere2", m, r, &vector![3. * r, 0., 0.]);
        sphere2.set_joint_v(
            0,
            JointVelocity::Spatial(SpatialVector::linear(vector![v, 0., 0.])),
        );

        state.add_articulated(sphere);
        state.add_articulated(sphere2);

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        // Perfectly inelastic collision
        let sphere_pose = state.articulated[0].bodies[0].pose;
        assert!(sphere_pose.translation.x < 0.);
        assert_eq!(sphere_pose.translation.y, 0.);
        assert_eq!(sphere_pose.translation.z, 0.);
        assert_eq!(sphere_pose.rotation, UnitQuaternion::identity());
        let sphere_v = state.articulated[0].v();
        assert!(sphere_v[3] < 0.);

        let sphere2_pose = state.articulated[1].bodies[0].pose;
        assert!(sphere2_pose.translation.x > sphere_pose.translation.x);
        assert_eq!(sphere2_pose.translation.y, 0.);
        assert_eq!(sphere2_pose.translation.z, 0.);
        let sphere2_v = state.articulated[1].v();
        assert_vec_close!(sphere_v, sphere2_v, 1e-3);
    }

    /// Contact handling between a sphere and a cuboid's vertex
    #[test]
    fn articulated_sphere_cuboid_vertex() {
        // Arrange
        let mut state = Hybrid::empty();
        state.disable_gravity();

        let m = 1.0;
        let r = 1.0;
        let sphere = Articulated::new_sphere("sphere", m, r);

        let v = -5.0;
        let w = 1.0;
        let x_init = r + w;
        let mut cuboid = Articulated::new_cube_at("cuboid", m, w, &vector![x_init, x_init, x_init]);
        cuboid.set_joint_v(
            0,
            JointVelocity::Spatial(SpatialVector::linear(vector![v, v, v])),
        );

        state.add_articulated(sphere);
        state.add_articulated(cuboid);

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        // Perfectly inelastic collision
        let sphere_pose = state.articulated[0].bodies[0].pose;
        assert_vec_close!(
            UnitVector3::new_normalize(sphere_pose.translation),
            UnitVector3::new_normalize(vector![-1., -1., -1.]),
            1e-3
        );
        assert!(sphere_pose.rotation.angle() < 1e-6);
        let sphere_v = state.articulated[0].body_twists()[0];
        assert_vec_close!(
            UnitVector3::new_normalize(sphere_v.linear),
            UnitVector3::new_normalize(vector![-1., -1., -1.]),
            1e-3
        );
        assert_vec_close!(sphere_v.angular, Vector3::<Float>::zeros(), 1e-6);

        let cuboid_pose = state.articulated[1].bodies[0].pose;
        assert!(cuboid_pose.translation.x > sphere_pose.translation.x);
        assert!(cuboid_pose.translation.y > sphere_pose.translation.y);
        assert!(cuboid_pose.translation.z > sphere_pose.translation.z);
        let cuboid_v = state.articulated[1].body_twists()[0];
        assert_vec_close!(cuboid_v.linear, sphere_v.linear, 1e-2);
        assert_vec_close!(cuboid_v.angular, Vector3::<Float>::zeros(), 1e-6);
    }

    /// Contact handling between a sphere and a cuboid's edge
    #[test]
    fn articulated_sphere_cuboid_edge() {
        // Arrange
        let mut state = Hybrid::empty();
        state.disable_gravity();

        let m = 1.0;
        let r = 1.0;
        let sphere = Articulated::new_sphere("sphere", m, r);

        let v = -2.0;
        let w = 2. * r + 0.1;
        let x_init = r + w;
        let mut cuboid = Articulated::new_cube_at("cuboid", m, w, &vector![x_init, 0., x_init]);
        cuboid.set_joint_v(
            0,
            JointVelocity::Spatial(SpatialVector::linear(vector![v, 0., v])),
        );

        state.add_articulated(sphere);
        state.add_articulated(cuboid);

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        // Perfectly inelastic collision
        let sphere_pose = state.articulated[0].bodies[0].pose;
        assert_vec_close!(
            UnitVector3::new_normalize(sphere_pose.translation),
            UnitVector3::new_normalize(vector![-1., 0., -1.]),
            1e-3
        );
        assert!(sphere_pose.rotation.angle() < 1e-6);
        let sphere_v = state.articulated[0].body_twists()[0];
        assert_vec_close!(
            UnitVector3::new_normalize(sphere_v.linear),
            UnitVector3::new_normalize(vector![-1., 0., -1.]),
            1e-3
        );
        assert_vec_close!(sphere_v.angular, Vector3::<Float>::zeros(), 1e-6);

        let cuboid_pose = state.articulated[1].bodies[0].pose;
        assert!(cuboid_pose.translation.x > sphere_pose.translation.x);
        assert_eq!(cuboid_pose.translation.y, sphere_pose.translation.y);
        assert!(cuboid_pose.translation.z > sphere_pose.translation.z);
        let cuboid_v = state.articulated[1].body_twists()[0];
        assert_vec_close!(cuboid_v.linear, sphere_v.linear, 1e-2);
        assert_vec_close!(cuboid_v.angular, Vector3::<Float>::zeros(), 1e-6);
    }

    /// Contact handling between a sphere and a cuboid's face
    #[test]
    fn articulated_sphere_cuboid_face() {
        // Arrange
        let mut state = Hybrid::empty();
        state.disable_gravity();

        let m = 1.0;
        let r = 1.0;
        let sphere = Articulated::new_sphere("sphere", m, r);

        let v = -2.0;
        let w = 2. * r + 0.1;
        let x_init = r + w;
        let mut cuboid = Articulated::new_cube_at("cuboid", m, w, &vector![x_init, 0., 0.]);
        cuboid.set_joint_v(
            0,
            JointVelocity::Spatial(SpatialVector::linear(vector![v, 0., 0.])),
        );

        state.add_articulated(sphere);
        state.add_articulated(cuboid);

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        // Perfectly inelastic collision
        let sphere_pose = state.articulated[0].bodies[0].pose;
        assert_vec_close!(
            UnitVector3::new_normalize(sphere_pose.translation),
            UnitVector3::new_normalize(vector![-1., 0., 0.]),
            1e-3
        );
        assert!(sphere_pose.rotation.angle() < 1e-6);
        let sphere_v = state.articulated[0].body_twists()[0];
        assert_vec_close!(
            UnitVector3::new_normalize(sphere_v.linear),
            UnitVector3::new_normalize(vector![-1., 0., 0.]),
            1e-3
        );
        assert_vec_close!(sphere_v.angular, Vector3::<Float>::zeros(), 1e-6);

        let cuboid_pose = state.articulated[1].bodies[0].pose;
        assert!(cuboid_pose.translation.x > sphere_pose.translation.x);
        assert_eq!(cuboid_pose.translation.y, sphere_pose.translation.y);
        assert_eq!(cuboid_pose.translation.z, sphere_pose.translation.z);
        let cuboid_v = state.articulated[1].body_twists()[0];
        assert_vec_close!(cuboid_v.linear, sphere_v.linear, 1e-2);
        assert_vec_close!(cuboid_v.angular, Vector3::<Float>::zeros(), 1e-6);
    }

    /// A cuboid resting flat on another is held by the four corners of the face
    /// it lands on, and the normal is the face normal
    #[test]
    fn cuboid_cuboid_collide_rests_flat_on_four_corners() {
        use na::Isometry3;

        use crate::hybrid::{collision::cuboid_cuboid_collide, visual::CuboidGeometry};

        let ground = CuboidGeometry {
            w: 4.,
            d: 4.,
            h: 1.,
        };
        let ground_iso = Isometry3::translation(0., 0., -0.5);
        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };
        let cube_iso = Isometry3::translation(0., 0., 0.5);

        let contacts = cuboid_cuboid_collide(&ground_iso, &ground, &cube_iso, &cube);
        assert_eq!(contacts.len(), 4);
        for (cp, normal) in contacts.iter() {
            // On the shared face, at the corners of the smaller cuboid
            assert!(cp.z.abs() < 1e-9);
            assert!((cp.x.abs() - 0.5).abs() < 1e-9);
            assert!((cp.y.abs() - 0.5).abs() < 1e-9);
            // Pointing from the ground up to the cuboid resting on it
            assert_vec_close!(**normal, vector![0., 0., 1.], 1e-9);
        }

        // Held the same way once the two overlap
        let sunk = Isometry3::translation(0., 0., 0.5 - 1e-2);
        assert_eq!(
            cuboid_cuboid_collide(&ground_iso, &ground, &sunk, &cube).len(),
            4
        );

        // And not at all once it clears the ground
        let clear = Isometry3::translation(0., 0., 1.);
        assert!(cuboid_cuboid_collide(&ground_iso, &ground, &clear, &cube).is_empty());

        // Nor when it is alongside rather than above
        let beside = Isometry3::translation(3., 0., 0.5);
        assert!(cuboid_cuboid_collide(&ground_iso, &ground, &beside, &cube).is_empty());
    }

    /// Two cuboids meeting edge to edge contact at the single point where the
    /// edges cross, with the normal along the cross product of the two edges
    #[test]
    fn cuboid_cuboid_collide_finds_the_edge_edge_point() {
        use na::{Isometry3, Translation3};

        use crate::{
            hybrid::{collision::cuboid_cuboid_collide, visual::CuboidGeometry},
            PI,
        };

        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };

        // A cube tipped 45 degrees about x, so that its lowest feature is the
        // edge running along x, hanging sqrt(2)/2 below its center
        let tipped = Isometry3::from_parts(
            Translation3::new(0., 0., (2. as Float).sqrt() / 2.),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 4.),
        );
        // And another tipped the same way about y, so its lowest edge runs along
        // y: the two cross at the origin
        let crossing = Isometry3::from_parts(
            Translation3::new(0., 0., -(2. as Float).sqrt() / 2.),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.),
        );

        let contacts = cuboid_cuboid_collide(&crossing, &cube, &tipped, &cube);
        assert_eq!(contacts.len(), 1);
        let (cp, normal) = contacts[0];
        assert_vec_close!(cp, Vector3::<Float>::zeros(), 1e-9);
        assert_vec_close!(*normal, vector![0., 0., 1.], 1e-9);

        // Sliding the upper cube along its own edge leaves the two edges
        // crossing where they did
        let along = tipped * Translation3::new(0.25, 0., 0.);
        let contacts = cuboid_cuboid_collide(&crossing, &cube, &along, &cube);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].0, Vector3::<Float>::zeros(), 1e-9);

        // Lifting it off separates them
        let lifted = Translation3::new(0., 0., 0.1) * tipped;
        assert!(cuboid_cuboid_collide(&crossing, &cube, &lifted, &cube).is_empty());
    }

    /// The contact points of two cuboids overlapping only partly are the corners
    /// of the overlap, whichever cuboid's face they come from
    #[test]
    fn cuboid_cuboid_collide_clips_to_the_shared_face() {
        use na::Isometry3;

        use crate::hybrid::{collision::cuboid_cuboid_collide, visual::CuboidGeometry};

        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };
        let lower = Isometry3::translation(0., 0., 0.);
        // Overlapping the lower cube's top face over a quarter of it
        let upper = Isometry3::translation(0.5, 0.5, 1.);

        let contacts = cuboid_cuboid_collide(&lower, &cube, &upper, &cube);
        assert_eq!(contacts.len(), 4);
        for (cp, normal) in contacts.iter() {
            assert!((cp.z - 0.5).abs() < 1e-9);
            // The overlap runs from 0 to 0.5 along both x and y
            assert!(cp.x > -1e-9 && cp.x < 0.5 + 1e-9);
            assert!(cp.y > -1e-9 && cp.y < 0.5 + 1e-9);
            assert_vec_close!(**normal, vector![0., 0., 1.], 1e-9);
        }
    }
}
