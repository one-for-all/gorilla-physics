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
            Feature::Edge(i, j) => candidates.iter().any(|(_, _, f)| {
                matches!(f, Feature::Face(vs) if vs.contains(i) && vs.contains(j))
            }),
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
    for (index, vertex) in mesh.vertices.iter().enumerate() {
        let offset = vertex - point;
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
        contacts.push((
            (mesh_iso * Point3::from(vertex)).coords,
            halfspace.normal,
        ));
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
                let (x, y) = (-0.5 + i as Float / n as Float, -0.5 + j as Float / n as Float);
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
                obj.push_str(&format!("f {} {} {}\n", at(i, j), at(i + 1, j), at(i + 1, j + 1)));
                obj.push_str(&format!("f {} {} {}\n", at(i, j), at(i + 1, j + 1), at(i, j + 1)));
            }
        }

        // Mass and inertia of the test robot's base link
        let m = 0.0157812;
        let com = vector![0.0025, -0.0256, 0.0102];
        let moment_com = Matrix3::new(
            3.62346e-06, 7.4487e-08, -4.8746e-08,
            7.4487e-08, 1.46159e-06, 4.94826e-07,
            -4.8746e-08, 4.94826e-07, 4.45457e-06,
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
            ));

            let mut articulated = Articulated::new(
                vec![body],
                vec![Joint::new_floating(Transform3D::identity(frame, WORLD_FRAME))],
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
}



