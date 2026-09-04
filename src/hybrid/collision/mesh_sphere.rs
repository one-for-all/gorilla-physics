//! Contacts between a mesh and the simpler colliders: a sphere, and a point.

use na::{UnitVector3, Vector3};

use crate::{
    collision::mesh::{closest_point_on_triangle, TriangleRegion},
    hybrid::{
        collision::shared::{
            candidate_faces, point_triangle_contact, push_welded, Aabb, Contact, CONTACT_TOLERANCE,
        },
        visual::rigid_mesh::RigidMesh,
    },
    types::Float,
};

/// Collision detection between a mesh and a sphere
/// Returns a list of contacts whose normals point from the mesh to the sphere
pub fn mesh_sphere_collide(
    mesh: &RigidMesh,
    sphere_center: &Vector3<Float>,
    sphere_radius: Float,
) -> Vec<Contact> {
    let mut contacts = vec![];

    // Early-out: every contact point lies on the mesh, hence inside its AABB;
    // no contact if the sphere center is farther than the radius from the AABB
    let mesh_aabb = Aabb::of_mesh(mesh);
    let radius_sq = sphere_radius * sphere_radius;
    if mesh_aabb.distance_sq_to(sphere_center) > radius_sq {
        return contacts;
    }

    // The mesh feature (in global vertex indices) that a closest point lies on
    enum Feature {
        Face([usize; 3]),
        Edge(usize, usize),
        Vertex(usize),
    }

    // Only faces whose AABB overlaps the sphere's AABB can be in contact
    let sphere_aabb = Aabb::centered(sphere_center, &Vector3::repeat(sphere_radius));
    let mut candidates: Vec<(Vector3<Float>, Vector3<Float>, Feature)> = vec![];
    for i_face in mesh
        .bvh
        .collect_overlapping(&sphere_aabb.min, &sphere_aabb.max)
    {
        let face = &mesh.faces[i_face];
        let v1 = mesh.vertices[face[0]];
        let v2 = mesh.vertices[face[1]];
        let v3 = mesh.vertices[face[2]];

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
        push_welded(
            &mut contacts,
            Contact::new(*closest_point, UnitVector3::new_normalize(*diff)),
        );
    }

    contacts
}

/// Collision detection between a mesh and a point
/// Returns the contact point, and the contact normal from the mesh to the point,
/// if the point is within `CONTACT_TOLERANCE` of one of the mesh's faces
pub fn mesh_point_collide(mesh: &RigidMesh, point: &Vector3<Float>) -> Vec<Contact> {
    let tol = CONTACT_TOLERANCE;

    // Early-out: contact requires the point to be within tol of a face, and
    // every face lies inside the mesh AABB
    if Aabb::of_mesh(mesh).distance_sq_to(point) > tol * tol {
        return vec![];
    }

    // Only faces whose AABB overlaps the tol-inflated point can be in contact
    let query = Aabb::new(*point, *point).inflated(tol);
    let i_faces = mesh.bvh.collect_overlapping(&query.min, &query.max);
    for face in candidate_faces(mesh, &i_faces, None) {
        if point_triangle_contact(point, &face.points, tol * tol).is_some() {
            // The contact sits where the point is, pushed out along the face's
            // own outward normal
            return vec![Contact::new(
                *point,
                UnitVector3::new_normalize(face.normal),
            )];
        }
    }

    vec![]
}
