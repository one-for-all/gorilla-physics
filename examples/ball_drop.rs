use gorilla_physics::inertia::SpatialInertia;
use gorilla_physics::mechanism::MechanismState;
use gorilla_physics::rigid_body::RigidBody;
use gorilla_physics::simulate::simulate;

pub fn main() {
    let _world = RigidBody::default(); // world has no inertia
    let body = RigidBody::new(SpatialInertia::default());

    let com = body.inertia.center_of_mass();

    let state = MechanismState::new();

    simulate(&state, 0.02, 1e-2);

    println!("{:#?}", com);
}
