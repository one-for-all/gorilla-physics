use std::process::exit;

use gorilla_physics::{
    collision::mesh::{read_mesh, Mesh},
    fem_deformable::read_fem_box,
    util::read_file,
};
use nalgebra::DVector;

/// Read a bunny as a mass-spring model, and simulate it falling under gravity
#[tokio::main]
async fn main() {
    let mut deformable = read_fem_box().await;

    deformable.extract_boundary_facets();
    println!("num faces: {}", deformable.boundary_facets.len());

    let buf = read_file("data/box.mesh");
    let rigid = Mesh::new_from_mesh(&buf);

    println!("rigid num faces: {}", rigid.faces.len());

    exit(0);

    let tau = DVector::zeros(deformable.n_vertices * 3);
    let final_time = 1.0;
    let dt = 1.0 / 60.0; // 1.0 / 600.0;
    let num_steps = (final_time / dt) as usize;

    // let mut data = vec![];
    for _ in 0..30 {
        // for i in 0..deformable.n_vertices {
        //     tau[3 * i + 2] = -9.81;
        // }
        deformable.step(dt, &tau).await;
        // data.push(deformable.q[2]);
    }

    // plot(&data, final_time, dt, num_steps, "bunny");
}
