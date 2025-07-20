use na::{Isometry3, MatrixXx6, UnitVector3, Vector3};

use crate::{joint::constraint_revolute::RevoluteConstraintJoint, types::Float};

pub enum Constraint {
    Revolute(RevoluteConstraintJoint),
    Cylindrical(CylindricalConstraintJoint),
}

impl Constraint {
    pub fn frame1(&self) -> &str {
        match self {
            Self::Revolute(constraint) => constraint.frame1.as_str(),
            Self::Cylindrical(constraint) => constraint.frame1.as_str(),
        }
    }

    pub fn frame2(&self) -> &str {
        match self {
            Self::Revolute(constraint) => constraint.frame2.as_str(),
            Self::Cylindrical(constraint) => constraint.frame2.as_str(),
        }
    }

    pub fn to_frame1(&self) -> &Isometry3<Float> {
        match self {
            Self::Revolute(constraint) => &constraint.to_frame1,
            Self::Cylindrical(constraint) => &constraint.to_frame1,
        }
    }

    pub fn constraint_matrix(&self) -> MatrixXx6<Float> {
        match self {
            Self::Revolute(constraint) => constraint.constraint_matrix(),
            Self::Cylindrical(constraint) => constraint.constraint_matrix(),
        }
    }
}

/// Constrains two frames to only have relative cylindrical motion
pub struct CylindricalConstraintJoint {
    pub axis: UnitVector3<Float>, // axis expressed in the frame after frame 1

    pub frame1: String,
    pub to_frame1: Isometry3<Float>,

    pub frame2: String,
    pub to_frame2: Isometry3<Float>,
}

impl CylindricalConstraintJoint {
    pub fn new(
        frame1: &str,
        to_frame1: Isometry3<Float>,
        frame2: &str,
        to_frame2: Isometry3<Float>,
        axis: UnitVector3<Float>,
    ) -> Self {
        Self {
            axis,
            frame1: frame1.to_string(),
            to_frame1,
            frame2: frame2.to_string(),
            to_frame2,
        }
    }

    /// Matrix that transforms spatial velocity to constraint space
    pub fn constraint_matrix(&self) -> MatrixXx6<Float> {
        let mut matrix = MatrixXx6::zeros(4);

        // Compute two vectors perpendicular to the rotation axis
        let t = {
            let candidate = self.axis.cross(&Vector3::x_axis());
            if candidate.norm() > 1e-3 {
                UnitVector3::new_normalize(candidate)
            } else {
                UnitVector3::new_normalize(self.axis.cross(&Vector3::y_axis()))
            }
        };
        let b = UnitVector3::new_normalize(self.axis.cross(&t));

        // Sets two remaining rotational axes
        matrix
            .fixed_view_mut::<1, 3>(0, 0)
            .copy_from(&t.transpose());
        matrix
            .fixed_view_mut::<1, 3>(1, 0)
            .copy_from(&b.transpose());

        // Sets two remaining linear axes
        matrix
            .fixed_view_mut::<1, 3>(2, 3)
            .copy_from(&t.transpose());
        matrix
            .fixed_view_mut::<1, 3>(3, 3)
            .copy_from(&b.transpose());

        matrix
    }
}

#[cfg(test)]
mod cylindrical_constraint_tests {

    use crate::{
        builders::navbot_builder::{build_navbot, NavbotMeshes},
        collision::mesh::Mesh,
        simulate::step,
        util::read_file,
    };

    #[test]
    fn navbot() {
        // Arrange
        let mut navbot_meshes = NavbotMeshes::new();
        let file_paths = vec![
            "navbot/esp32pcb.obj",
            "navbot/top_plate.obj",
            "navbot/side_plate_left.obj",
            "navbot/side_plate_right.obj",
            "navbot/leg_inner_left.obj",
            "navbot/leg_outer_left.obj",
            "navbot/pin.obj",
            "navbot/foot_motor_left.obj",
            "navbot/encoder.obj",
            "navbot/foot_plate_left.obj",
            "navbot/link_plate_left.obj",
        ];
        let buffers: Vec<String> = file_paths
            .iter()
            .map(|&path| read_file(&("data/".to_string() + path)))
            .collect();
        for (i, buf) in buffers.iter().enumerate() {
            let mesh = Some(Mesh::new_from_obj(buf, false));
            match i {
                0 => navbot_meshes.esp32pcb = mesh,
                1 => navbot_meshes.top_plate = mesh,
                2 => navbot_meshes.side_plate_left = mesh,
                3 => navbot_meshes.side_plate_right = mesh,
                4 => navbot_meshes.leg_inner_left = mesh,
                5 => navbot_meshes.leg_outer_left = mesh,
                6 => navbot_meshes.foot_pin_left = mesh,
                7 => navbot_meshes.foot_motor_left = mesh,
                8 => navbot_meshes.foot_encoder_left = mesh,
                9 => navbot_meshes.foot_plate_left = mesh,
                10 => navbot_meshes.link_plate_left = mesh,
                _ => {}
            }
        }

        let mut state = build_navbot(navbot_meshes);

        // Act
        let mut data = vec![];
        let final_time = 1.0;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );

            data.push(*_q[1].float());
        }

        // Assert
        println!("final: {}", data[data.len() - 1]);
        // plot(&data, final_time, dt, num_steps, "navbot");
    }
}
