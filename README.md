# Gorilla Physics Engine (for robotics in Rust)

I used [Rapier](https://github.com/dimforge/rapier) before, but it did not seem to be quite made for robotics, and was missing some features I needed, so I embarked on this journey to build a physics engine myself.

I am learning a lot as I build it, so it is also a learning project for me.

## Getting Started

Run all tests in lib:

`cargo test --lib -- --nocapture`

Run the acrobot example:

`cargo run --bin acrobot`

It should generate a plot ("acrobot swingup energy.png") that shows the total energy of the system against time.

Profile the run-time of an example:

`samply record cargo run --bin acrobot`

This would run the example, and show profiling result & flame graph in browser, assuming [samply](https://github.com/mstange/samply) is installed, and Firefox profiler plugin (in Chrome) is installed.

## References

It mainly implements the theory in Roy Featherstone's book [Rigid Body Dynamics Algorithm](https://royfeatherstone.org/).

Also built with a lot of references to Julia package [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl).

And with the help of a lot of papers. They are referenced in code when used.
