use gorilla_physics::{mass_spring_deformable::read_mass_spring_bunny, plot::plot};
use nalgebra::DVector;

/// Read a bunny as a mass-spring model, and simulate it falling under gravity
pub fn main() {
    let mut bunny = read_mass_spring_bunny();

    let mut tau = DVector::zeros(bunny.n_vertices * 3);
    let final_time = 3.0;
    let dt = 1.0 / 600.0;
    let num_steps = (final_time / dt) as usize;

    let mut data = vec![];
    for _ in 0..num_steps {
        for i in 0..bunny.n_vertices {
            tau[3 * i + 2] = -9.81;
        }
        bunny.step(dt, &tau);
        data.push(bunny.q[2]);
    }

    plot(&data, final_time, dt, num_steps, "bunny");
}
