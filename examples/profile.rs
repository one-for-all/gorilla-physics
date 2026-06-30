use std::time::Instant;

use gorilla_physics::{hybrid::visual::rigid_mesh::RigidMesh, util::read_file};

pub fn main() {
    let content = read_file("data/gamer_setup_pack.obj");

    let start = Instant::now();
    let rigid_mesh = RigidMesh::new_from_obj(&content);
    let duration = start.elapsed();
    println!("old impl time elapsed: {:?}", duration);

    // let start = Instant::now();
    // let rigid_mesh = RigidMesh::opt_new_from_obj(&content);
    // let duration = start.elapsed();
    // println!("new impl time elapsed: {:?}", duration);
}
