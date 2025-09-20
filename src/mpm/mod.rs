use na::{vector, zero, Matrix2, Vector2, SVD};
use rand::{rng, Rng};

use crate::{types::Float, GRAVITY};

// Material properties
const VOL: Float = 1.; // particle volume
const HARDENING: Float = 10.0; // Snow hardening factor
const E: Float = 1e4; // Young's modulus
const NU: Float = 0.2; // Poisson ratio

// Initial Lame parameters
const MU_0: Float = E / (2. * (1. + NU));
const LAMBDA_0: Float = E * NU / ((1. + NU) * (1. - 2. * NU));

fn polar_decomposition(F: &Matrix2<f64>) -> (Matrix2<f64>, Matrix2<f64>) {
    let svd = SVD::new(F.clone(), true, true);
    let (u, sigma, v_t) = (svd.u.unwrap(), svd.singular_values, svd.v_t.unwrap());

    let r = &u * &v_t;
    let s = &v_t.transpose() * Matrix2::from_diagonal(&sigma) * &v_t;

    (r, s)
}

pub struct Particle {
    pub pos: Vector2<Float>,
    pub vel: Vector2<Float>,

    pub F: Matrix2<Float>, // deformation gradient
    pub C: Matrix2<Float>, // affine momentum matrix

    pub Jp: Float, // determinant of F, i.e. volume ratio

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
        let spacing = 2.0 / n_grid as Float;

        // create a square of particles
        let mut rng = rng();
        let mut particles = vec![];
        let x_mid = n_grid / 2;
        let x_halfwdith = 20;
        let y_halfwidth = 5;
        for i in (x_mid - x_halfwdith)..(x_mid + x_halfwdith) {
            for j in (x_mid - y_halfwidth)..(x_mid + y_halfwidth) {
                let x = i as Float * spacing;
                let y = j as Float * spacing;

                let half_s = spacing / 2.;
                for _ in 0..10 {
                    let disturb = vector![
                        rng.random_range(-half_s..half_s),
                        rng.random_range(-half_s..half_s)
                    ];
                    particles.push(Particle {
                        pos: vector![x, y] + disturb,
                        vel: vector![0., 0.], // initial speed
                        F: Matrix2::identity(),
                        C: zero(),
                        Jp: 1.,
                        mass: 1.,
                    });
                }
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

            // Compute current Lame parameters
            // ref: MPM course - Eqn. 86
            let e = (HARDENING * (1. - particle.Jp)).exp();
            let mu = MU_0 * e;
            let lambda = LAMBDA_0 * e;

            // Current volume
            let J = particle.F.determinant();

            // Polar decomposition for fixed corotated model
            let (r, s) = polar_decomposition(&particle.F);

            // Compute inverse of D
            // ref: MPM course, paragraph after Eqn. 176
            let D_inv = (4. / self.spacing) / self.spacing;
            // ref: MPM course, Eqn. 52
            let PF = 2. * mu * (particle.F - r) * particle.F.transpose()
                + Matrix2::from_diagonal_element(lambda * (J - 1.) * J); // TODO: verify this step

            // Cauchy stress times dt and divided by spacing
            let stress = -(dt * VOL) * (D_inv * PF);

            // Fused APIC momentum + MLS-MPM stress contribution
            // ref: MLS-MPM, Eqn. 29
            let affine = stress + particle.mass * particle.C;

            let base_int_coord: Vector2<usize> = base_coord.map(|x| x as usize);
            for i in 0..3 {
                for j in 0..3 {
                    let weight = weights[i].x * weights[j].y;
                    let dpos = (vector![i as Float, j as Float] - fx).scale(self.spacing);

                    let mass_contrib = weight * particle.mass;
                    let node = &mut self.grid[base_int_coord.x + i][base_int_coord.y + j];
                    node.mass += mass_contrib;
                    node.vel += mass_contrib * (particle.vel + affine * dpos);
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
                    node.vel += dt * vector![0., -GRAVITY]; // TODO: change back to GRAVITY

                    // boundary condition
                    if i <= 1 || i >= self.n_grid - 2 {
                        node.vel.x = 0.;
                    }
                    if j <= 1 || j >= self.n_grid - 2 {
                        node.vel.y = 0.;
                    }

                    // A needle in the middle
                    if i == self.n_grid / 2 {
                        if j <= 5 {
                            node.vel.y = 0.;
                        }
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

                    let node_v = self.grid[base_int_coord.x + i][base_int_coord.y + j].vel;

                    let vel_contrib = node_v * weight;
                    particle.vel += vel_contrib;
                    particle.C += 4. / self.spacing * vel_contrib * dpos.transpose();
                }
            }

            // advect particles
            particle.pos += particle.vel * dt;

            // MLS-MPM F-update
            let mut F = (Matrix2::identity() + dt * particle.C) * particle.F;

            let svd = SVD::new(F, true, true);
            let (u, mut sigma, v_t) = (svd.u.unwrap(), svd.singular_values, svd.v_t.unwrap());

            // Snow plasticity
            for i in 0..2 {
                sigma[i] = sigma[i].clamp(1. - 2.5e-2, 1. + 7.5e-3);
            }

            let J_old = F.determinant();
            F = u * Matrix2::from_diagonal(&sigma) * &v_t;

            let Jp_new = (particle.Jp * J_old / F.determinant()).clamp(0.6, 20.);

            particle.Jp = Jp_new;
            particle.F = F;

            // safety clamp
            particle.pos = particle
                .pos
                .map(|x| x.clamp(self.spacing, (self.n_grid - 2) as Float * self.spacing));
        }
    }
}
