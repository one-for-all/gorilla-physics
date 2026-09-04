//! Collision detection between the colliders a hybrid scene carries.
//!
//! Every routine here reports a list of [`Contact`]s, whose normals run from the
//! first of the two shapes named towards the second. The geometry they share --
//! tolerances, welding, support-polygon reduction, and the segment, triangle and
//! mesh-face primitives -- lives in [`shared`].

mod cuboid_cuboid;
mod mesh_cuboid;
mod mesh_halfspace;
mod mesh_mesh;
mod mesh_sphere;
mod oriented_box;
mod shared;
mod sphere_cuboid;

pub use cuboid_cuboid::cuboid_cuboid_collide;
pub use mesh_cuboid::mesh_cuboid_collide;
pub use mesh_halfspace::mesh_halfspace_collide;
pub use mesh_mesh::mesh_mesh_collide;
pub use mesh_sphere::{mesh_point_collide, mesh_sphere_collide};
pub use shared::Contact;
pub use sphere_cuboid::{cuboid_point_collide, sphere_cuboid_collide, sphere_cuboid_collide_local};

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
