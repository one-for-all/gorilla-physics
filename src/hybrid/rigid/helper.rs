use crate::{
    hybrid::{mesh::URDFMeshes, visual::Visual, Rigid},
    inertia::SpatialInertia,
    joint::{constraint::constraint_revolute::RevoluteConstraintJoint, Joint},
    spatial::transform::Transform3D,
    types::Float,
    PI,
};
use na::{Isometry, Isometry3, Translation3, UnitQuaternion, Vector3};
use nalgebra::{Matrix3, UnitVector3, Vector};
use urdf_rs::Robot;

pub fn build_rigid(frame: &str, link_name: &str, urdf: &Robot, meshes: &mut URDFMeshes) -> Rigid {
    let link_urdf = urdf.links.iter().find(|&l| l.name == link_name).unwrap();

    let inertial = &link_urdf.inertial;
    let m = inertial.mass.value;
    let com = Vector::from(inertial.origin.xyz.0);
    let ixx = inertial.inertia.ixx;
    let ixy = inertial.inertia.ixy;
    let ixz = inertial.inertia.ixz;
    let iyy = inertial.inertia.iyy;
    let iyz = inertial.inertia.iyz;
    let izz = inertial.inertia.izz;

    #[rustfmt::skip]
    let moment_com = Matrix3::new(
        ixx, ixy, ixz,
        ixy, iyy, iyz,
        ixz, iyz, izz
    );

    let moment =
        moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
    let cross_part = m * com;

    let mut body = Rigid::new(SpatialInertia::new(moment, cross_part, m, frame));

    if let Some(link_meshes) = meshes.meshes.remove(link_name) {
        for (mesh, iso, color) in link_meshes.into_iter() {
            body.visual
                .push((Visual::RigidMesh(mesh), iso, Some(color)));
        }
    }
    body
}

pub fn build_joint(
    from: &str,
    to: &str,
    joint_name: &str,
    urdf: &Robot,
    axis: UnitVector3<Float>,
    q: Float,
) -> Joint {
    let urdf_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();

    let rpy;
    // Note: this is hack to make sure urdf number gets rounded to its true value
    // TODO: remove the hack. make general
    if joint_name == "left_front_spring" {
        rpy = Vec::from([PI, 0., 0.]);
    } else {
        rpy = Vec::from(urdf_joint.origin.rpy.0);
    }

    let joint = Joint::new_revolute_with_q(
        q,
        Transform3D::new_xyz_rpy(from, to, &Vec::from(urdf_joint.origin.xyz.0), &rpy),
        axis,
    );
    joint
}

pub fn build_fixed_joint(from: &str, to: &str, joint_name: &str, urdf: &Robot) -> Joint {
    let urdf_joint = urdf.joints.iter().find(|&j| j.name == joint_name).unwrap();
    let joint = Joint::new_fixed(Transform3D::new_xyz_rpy(
        from,
        to,
        &Vec::from(urdf_joint.origin.xyz.0),
        &Vec::from(urdf_joint.origin.rpy.0),
    ));
    joint
}

pub fn build_revolute_constraint(
    body1_frame: &str,
    body2_frame: &str,
    closing_joint_name: &str,
    urdf: &Robot,
) -> RevoluteConstraintJoint {
    let closing_joint_1_name = format!("closing_{}_1_frame", closing_joint_name);
    let closing_joint_1 = urdf
        .joints
        .iter()
        .find(|&j| j.name == closing_joint_1_name)
        .unwrap();

    let xyz = closing_joint_1.origin.xyz;
    let rpy = closing_joint_1.origin.rpy;
    let iso_to_body1 = Isometry3::from_parts(
        Translation3::new(xyz[0], xyz[1], xyz[2]),
        UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
    );

    let closing_joint_2_name = format!("closing_{}_2_frame", closing_joint_name);
    let closing_joint_2 = urdf
        .joints
        .iter()
        .find(|&j| j.name == closing_joint_2_name)
        .unwrap();

    let xyz = closing_joint_2.origin.xyz;
    let rpy = closing_joint_2.origin.rpy;
    let iso_to_body2 = Isometry3::from_parts(
        Translation3::new(xyz[0], xyz[1], xyz[2]),
        UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
    );

    RevoluteConstraintJoint::new(
        body1_frame,
        iso_to_body1,
        body2_frame,
        iso_to_body2,
        Vector3::z_axis(),
    )
}
