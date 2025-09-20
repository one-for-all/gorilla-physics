use na::{vector, zero, Matrix2, Vector2};

use crate::{types::Float, GRAVITY};

pub struct Particle {
    pub pos: Vector2<Float>,
    pub vel: Vector2<Float>,
    pub C: Matrix2<Float>, // affine momentum matrix

    pub mass: Float,
}

pub struct Node {
    pub vel: Vector2<Float>,
    pub mass: Float,
}

pub struct MPMDeformable {
    pub n_grid: usize,  // number of grids on one axis
    pub spacing: Float, // spacing between grid points

    pub particles: Vec<Particle>,
    pub grid: Vec<Vec<Node>>,
}

impl MPMDeformable {
    pub fn new() -> Self {
        let n_grid = 64;
        let spacing = 1.0;

        // create a square of particles
        let mut particles = vec![];
        let x_mid = n_grid / 2;
        let x_halfwdith = 10;
        for i in (x_mid - x_halfwdith)..(x_mid + x_halfwdith) {
            for j in (x_mid - x_halfwdith)..(x_mid + x_halfwdith) {
                let x = i as Float * spacing;
                let y = j as Float * spacing;
                particles.push(Particle {
                    pos: vector![x, y],
                    vel: vector![0., 0.], // initial speed
                    C: zero(),
                    mass: 1.,
                });
            }
        }

        let grid = (0..n_grid)
            .map(|_| {
                (0..n_grid)
                    .map(|_| Node {
                        vel: zero(),
                        mass: 0.,
                    })
                    .collect()
            })
            .collect();

        MPMDeformable {
            n_grid,
            spacing,
            particles,
            grid,
        }
    }

    pub fn step(&mut self, dt: Float) {
        // reset grid
        for i in 0..self.n_grid {
            for j in 0..self.n_grid {
                let node = &mut self.grid[i][j];
                node.mass = 0.;
                node.vel = zero();
            }
        }

        // P2G
        let vector2_0d5 = Vector2::repeat(0.5); // re-used Vector2
        for particle in self.particles.iter() {
            // quadratic interpolation weights
            // ref:
            //  1.  https://github.com/yuanming-hu/taichi_mpm/blob/master/mls-mpm88-explained.cpp
            //  2. The Material Point Method for Simulating Continuum Materials, 2016, Chenfanfu Jiang and et al., Eqn 123 modified
            let particle_coord = particle.pos / self.spacing;
            let base_coord: Vector2<Float> = (particle_coord - vector2_0d5).map(|x| x.floor());
            let fx = particle_coord - base_coord;
            let weights = [
                vector2_0d5.component_mul(&(Vector2::repeat(1.5) - fx).map(|x| x.powi(2))),
                Vector2::repeat(0.75) - (fx - Vector2::repeat(1.0)).map(|x| x.powi(2)),
                vector2_0d5.component_mul(&(fx - vector2_0d5).map(|x| x.powi(2))),
            ];

            let base_int_coord: Vector2<usize> = base_coord.map(|x| x as usize);

            for i in 0..3 {
                for j in 0..3 {
                    let weight = weights[i].x * weights[j].y;
                    let dpos = (vector![i as Float, j as Float] - fx).scale(self.spacing);
                    let Q = particle.C * dpos; // TODO: figure out this APIC step

                    let mass_contrib = weight * particle.mass;
                    let grid_int_coord = base_int_coord + vector![i, j];
                    let node = &mut self.grid[grid_int_coord.x][grid_int_coord.y];
                    node.mass += mass_contrib;
                    node.vel += mass_contrib * (particle.vel + Q);
                    // Note: at this point, vel is momentum, not velocity. It will be corrected in the grid update step.
                }
            }
        }

        // grid velocity update
        for i in 0..self.n_grid {
            for j in 0..self.n_grid {
                let node = &mut self.grid[i][j];
                if node.mass > 0. {
                    // convert momentum to velocity
                    node.vel /= node.mass;

                    // apply gravity
                    node.vel += dt * vector![0., -GRAVITY];

                    // boundary condition
                    if i <= 1 || i >= self.n_grid - 2 {
                        node.vel.x = 0.;
                    }
                    if j <= 1 || j >= self.n_grid - 2 {
                        node.vel.y = 0.;
                    }
                }
            }
        }

        // G2P
        for particle in self.particles.iter_mut() {
            let particle_coord = particle.pos / self.spacing;
            let base_coord: Vector2<Float> = (particle_coord - vector2_0d5).map(|x| x.floor());
            let fx = particle_coord - base_coord;
            let weights = [
                vector2_0d5.component_mul(&(Vector2::repeat(1.5) - fx).map(|x| x.powi(2))),
                Vector2::repeat(0.75) - (fx - Vector2::repeat(1.0)).map(|x| x.powi(2)),
                vector2_0d5.component_mul(&(fx - vector2_0d5).map(|x| x.powi(2))),
            ];

            particle.C = zero();
            particle.vel = zero();

            let base_int_coord: Vector2<usize> = base_coord.map(|x| x as usize);
            for i in 0..3 {
                for j in 0..3 {
                    let weight = weights[i].x * weights[j].y;
                    let dpos = vector![i as Float, j as Float] - fx;

                    let grid_int_coord = base_int_coord + vector![i, j];
                    let node_v = self.grid[grid_int_coord.x][grid_int_coord.y].vel;

                    let vel_contrib = node_v * weight;
                    particle.vel += vel_contrib;
                    particle.C += 4. / self.spacing * vel_contrib * dpos.transpose();
                }
            }

            // advect particles
            particle.pos += particle.vel * dt;

            // safety clamp
            particle.pos = particle
                .pos
                .map(|x| x.clamp(self.spacing, (self.n_grid - 2) as Float * self.spacing));
        }
    }
}
