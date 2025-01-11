# Gorilla Physics Engine (for robotics in Rust)

I used [Rapier](https://github.com/dimforge/rapier) before, but it does not seem to be quite made for robotics, and misses some features I needed, so I embarked on this journey to build a physics engine myself.

I am learning a lot as I build it, so it is also a learning project for me.

## Getting Started

Run all tests:

`cargo test -- --nocapture`

Run the pendulum example:

`cargo run --bin horizontal_rod_pendulum`

It should generate a plot that plots angle of the rod against the time axis.

## TODO

- Deploy the visualization to github page
- Add section on visualization in README

## References

It mainly implements the theory in Roy Featherstone's book [Rigid Body Dynamics Algorithm](https://royfeatherstone.org/).

Also built with a lot of references to Julia package [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl).
