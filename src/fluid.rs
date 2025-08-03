use na::{vector, zero, Vector2};
use rand::prelude::*;
use rand::Rng;
use rand_chacha::ChaCha8Rng;

use crate::types::Float;
use crate::{flog, GRAVITY};

const FLUID_DENSITY: Float = 1000.0; // density of water

/// the smaller the distance, the larger the kernel
/// parametrized by dx - grid size
fn bilinear_hat_kernel(dist: Vector2<Float>, dx: Float) -> Float {
    hat_function(dist.x / dx) * hat_function(dist.y / dx)
}

/// the function that has the shape of a triangle hat
/// hat(0) -> 1
/// hat(1) -> 0
/// hat(>1) -> 0
fn hat_function(r: Float) -> Float {
    let r_abs = r.abs();
    if r_abs <= 1. {
        return 1. - r_abs;
    }
    return 0.;
}

/// given the position, return the index of the grid that this position lies on
fn get_grid_cell_index(pos: &Vector2<Float>, dx: Float) -> (i32, i32) {
    ((pos.x / dx) as i32, (pos.y / dx) as i32)
}

/// given grid index, returns the position of the center of the grid
fn get_grid_cell_position(i: Float, j: Float, dx: Float) -> Vector2<Float> {
    vector![i * dx + 0.5 * dx, j * dx + 0.5 * dx]
}

/// Interpolate the velocity at the given position
fn interp_vel(
    u_grid: &Vec<Vec<Float>>,
    v_grid: &Vec<Vec<Float>>,
    dx: Float,
    m: usize,
    n: usize,
    pos: &Vector2<Float>,
) -> Vector2<Float> {
    let (i, j) = get_grid_cell_index(&pos, dx);
    if i >= 0 && i < m as i32 && j >= 0 && j < n as i32 {
        let i = i as usize;
        let j = j as usize;
        let cell_pos = get_grid_cell_position(i as Float, j as Float, dx);
        let offset = dx / 2.0;
        let x1 = cell_pos.x - offset;
        let x2 = cell_pos.x + offset;
        let y1 = cell_pos.y - offset;
        let y2 = cell_pos.y + offset;

        let u1 = u_grid[i][j];
        let u2 = u_grid[i + 1][j];
        let v1 = v_grid[i][j];
        let v2 = v_grid[i][j + 1];

        // the interpolated values
        let u = ((x2 - pos.x) / (x2 - x1)) * u1 + ((pos.x - x1) / (x2 - x1)) * u2;
        let v = ((y2 - pos.y) / (y2 - y1)) * v1 + ((pos.y - y1) / (y2 - y1)) * v2;
        return vector![u, v];
    }

    // vector![Float::NAN, Float::NAN] // Unknown
    vector![0., 0.]
}

/// Runge-Kutta 3rd order integration
fn RK3(
    particle: &mut Particle2D,
    init_vel: &Vector2<Float>,
    dt: Float,
    u_grid: &Vec<Vec<Float>>,
    v_grid: &Vec<Vec<Float>>,
    dx: Float,
    m: usize,
    n: usize,
) {
    let mut init_vel = init_vel.clone();
    if init_vel.x.is_nan() || init_vel.y.is_nan() {
        init_vel = interp_vel(&u_grid, &v_grid, dx, m, n, &particle.pos);
    }

    let mut k1 = init_vel;
    let mut k2 = interp_vel(u_grid, v_grid, dx, m, n, &(particle.pos + k1 * 0.5 * dt));
    let mut k3 = interp_vel(u_grid, v_grid, dx, m, n, &(particle.pos + k2 * 0.75 * dt));
    k1 = k1 * (2. / 9. * dt);
    k2 = k2 * (3. / 9. * dt);
    k3 = k3 * (4. / 9. * dt);

    particle.pos += k1 + k2 + k3;
}

#[derive(Clone, Debug)]
enum Label {
    Fluid,
    Solid,
    Empty,
}

/// Model a particle in 2D fluid
pub struct Particle2D {
    pub pos: Vector2<Float>,
    pub vel: Vector2<Float>,
}

/// 2D Fluid simulation
/// Ref: Fluid Simulation for Computer Graphics, Robert Bridson, 2015
pub struct Fluid2D {
    m: usize,  // number of cells in x-axis
    n: usize,  // number of cells in y-axis
    dx: Float, // cell size in both x and y-axis

    labels: Vec<Vec<Label>>, // cell labels - Fluid/Solid/Empty
    p: Vec<Vec<Float>>,      // pressures grid
    u: Vec<Vec<Float>>,      // x-axis velocity grid
    v: Vec<Vec<Float>>,      // y-axis velocity grid

    pub particles: Vec<Particle2D>, // fluid particles for advection
}

impl Fluid2D {
    pub fn new() -> Self {
        let m = 50;
        let n = 25;
        let dx = 5e-3;
        let mut labels = vec![vec![Label::Empty; n]; m];

        // Set up solid boundary on 4 sides
        for i in 0..m {
            labels[i][0] = Label::Solid;
            labels[i][n - 1] = Label::Solid;
        }
        for j in 0..n {
            labels[0][j] = Label::Solid;
            labels[m - 1][j] = Label::Solid;
        }

        // Add fluid to fill the left half of the box
        for i in 1..m / 2 {
            for j in 1..n - 1 {
                labels[i][j] = Label::Fluid;
            }
        }

        let p = vec![vec![0.; n]; m];
        let u = vec![vec![0.; n]; m + 1];
        let v = vec![vec![0.; n + 1]; m];

        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut particles = vec![];
        for i in 0..m {
            for j in 0..n {
                if !matches!(labels[i][j], Label::Fluid) {
                    continue;
                }
                let x = i as Float * dx;
                let y = j as Float * dx;
                for _ in 0..4 {
                    let x_add = rng.random_range(0.0..dx);
                    let y_add = rng.random_range(0.0..dx);
                    particles.push(Particle2D {
                        pos: vector![x + x_add, y + y_add],
                        vel: zero(),
                    });
                }
            }
        }

        Fluid2D {
            m,
            n,
            dx,
            labels,
            p,
            u,
            v,
            particles,
        }
    }

    pub fn step(&mut self, dt: Float) {
        // update the grid labels
        for i in 0..self.m {
            for j in 0..self.n {
                if !matches!(self.labels[i][j], Label::Solid) {
                    self.labels[i][j] = Label::Empty;
                }
            }
        }
        for particle in self.particles.iter() {
            let (i, j) = get_grid_cell_index(&particle.pos, self.dx);
            self.labels[i as usize][j as usize] = Label::Fluid;
        }

        // transfer particle vel to grid
        // For each component of velocity in each fluid grid cell
        // we calculate weighted average of particles around it defined
        // by a kernel function and set this as the vel in the cell.

        // structures to accumulate grid numerator and denominator of weighted average before divide
        let mut u_num = vec![vec![0.; self.n]; self.m + 1];
        let mut u_den = vec![vec![0.; self.n]; self.m + 1];
        let mut v_num = vec![vec![0.; self.n + 1]; self.m];
        let mut v_den = vec![vec![0.; self.n + 1]; self.m];

        // loop over particles and accumulate num and den at each grid point
        for particle in self.particles.iter() {
            let (a, b) = get_grid_cell_index(&particle.pos, self.dx);
            for i in (a - 3).max(0)..(a + 3).min((self.m + 1) as i32) {
                let i = i as usize;
                for j in (b - 3).max(0)..(b + 3).min((self.n + 1) as i32) {
                    let j = j as usize;
                    if j < self.n {
                        let kernel = bilinear_hat_kernel(
                            particle.pos
                                - get_grid_cell_position(i as Float - 0.5, j as Float, self.dx),
                            self.dx,
                        );
                        u_num[i][j] += particle.vel.x * kernel;
                        u_den[i][j] += kernel;
                    }
                    if i < self.m {
                        let kernel = bilinear_hat_kernel(
                            particle.pos
                                - get_grid_cell_position(i as Float, j as Float - 0.5, self.dx),
                            self.dx,
                        );
                        v_num[i][j] += particle.vel.y * kernel;
                        v_den[i][j] += kernel;
                    }
                }
            }
        }

        // additional pass over grid to divide and update actual velocities
        for i in 0..self.m + 1 {
            for j in 0..self.n + 1 {
                if j < self.n && u_den[i][j] != 0. {
                    self.u[i][j] = u_num[i][j] / u_den[i][j];
                }
                if i < self.m && v_den[i][j] != 0. {
                    self.v[i][j] = v_num[i][j] / v_den[i][j];
                }
            }
        }

        // Apply body force
        for row in self.v.iter_mut() {
            for data in row.iter_mut() {
                *data -= dt * GRAVITY;
            }
        }

        // Pressure solve
        // Construct b vector
        let mut b = vec![vec![0.; self.n]; self.m]; // negative of the divergence at each cell
        let scale = 1.0 / self.dx;
        for i in 0..self.m {
            for j in 0..self.n {
                if !matches!(self.labels[i][j], Label::Fluid) {
                    continue;
                }
                let u_in = {
                    if i > 0 && matches!(self.labels[i - 1][j], Label::Solid) {
                        0.
                    } else {
                        self.u[i][j]
                    }
                };
                let u_out = {
                    if i < self.m - 1 && matches!(self.labels[i + 1][j], Label::Solid) {
                        0.
                    } else {
                        self.u[i + 1][j]
                    }
                };
                let v_in = {
                    if j > 0 && matches!(self.labels[i][j - 1], Label::Solid) {
                        0.
                    } else {
                        self.v[i][j]
                    }
                };
                let v_out = {
                    if j < self.n - 1 && matches!(self.labels[i][j + 1], Label::Solid) {
                        0.
                    } else {
                        self.v[i][j + 1]
                    }
                };
                b[i][j] = -scale * (u_out - u_in + v_out - v_in);
            }
        }

        // Construct A matrix
        let mut A_diag = vec![vec![0.; self.n]; self.m];
        let mut Ax = vec![vec![0.; self.n]; self.m];
        let mut Ay = vec![vec![0.; self.n]; self.m];
        let scale = dt / (FLUID_DENSITY * self.dx * self.dx);
        for i in 0..self.m {
            for j in 0..self.n {
                if !matches!(self.labels[i][j], Label::Fluid) {
                    continue;
                }

                if i > 0
                    && (matches!(self.labels[i - 1][j], Label::Fluid)
                        || matches!(self.labels[i - 1][j], Label::Empty))
                {
                    A_diag[i][j] += scale;
                }
                if i < self.m - 1 {
                    if matches!(self.labels[i + 1][j], Label::Fluid) {
                        A_diag[i][j] += scale;
                        Ax[i][j] = -scale;
                    } else if matches!(self.labels[i + 1][j], Label::Empty) {
                        A_diag[i][j] += scale;
                    }
                }

                if j > 0
                    && (matches!(self.labels[i][j - 1], Label::Fluid)
                        || matches!(self.labels[i][j - 1], Label::Empty))
                {
                    A_diag[i][j] += scale;
                }
                if j < self.n - 1 {
                    if matches!(self.labels[i][j + 1], Label::Fluid) {
                        A_diag[i][j] += scale;
                        Ay[i][j] = -scale;
                    } else if matches!(self.labels[i][j + 1], Label::Empty) {
                        A_diag[i][j] += scale;
                    }
                }
            }
        }

        // Construct preconditioner
        let mut precon = vec![vec![0.; self.n]; self.m];
        let tau = 0.97;
        let sigma = 0.25;
        for i in 0..self.m {
            for j in 0..self.n {
                if !matches!(self.labels[i][j], Label::Fluid) {
                    continue;
                }

                let mut Ax_im1j = 0.;
                let mut Ax_ijm1 = 0.;
                let mut Ay_ijm1 = 0.;
                let mut Ay_im1j = 0.;
                let mut precon_im1j = 0.;
                let mut precon_ijm1 = 0.;
                if i > 0 && matches!(self.labels[i - 1][j], Label::Fluid) {
                    Ax_im1j = Ax[i - 1][j];
                    Ay_im1j = Ay[i - 1][j];
                    precon_im1j = precon[i - 1][j];
                }
                if j > 0 && matches!(self.labels[i][j - 1], Label::Fluid) {
                    Ax_ijm1 = Ax[i][j - 1];
                    Ay_ijm1 = Ay[i][j - 1];
                    precon_ijm1 = precon[i][j - 1];
                }

                let Adiag_ij = A_diag[i][j];
                let mut e = Adiag_ij
                    - (Ax_im1j * precon_im1j).powi(2)
                    - (Ay_ijm1 * precon_ijm1).powi(2)
                    - tau
                        * (Ax_im1j * Ay_im1j * precon_im1j.powi(2)
                            + Ay_ijm1 * Ax_ijm1 * precon_ijm1.powi(2));

                if e < sigma * Adiag_ij {
                    e = Adiag_ij;
                }
                precon[i][j] = 1.0 / e.sqrt();
            }
        }

        // Solve for pressure using preconditioned conjugate gradient
        self.pcg(&A_diag, &Ax, &Ay, &b, &precon);

        // Apply pressure force
        let scale = dt / (FLUID_DENSITY * self.dx);
        for i in 0..self.m {
            for j in 0..self.n {
                // update u
                if i > 0 {
                    if matches!(self.labels[i - 1][j], Label::Fluid)
                        || matches!(self.labels[i][j], Label::Fluid)
                    {
                        if matches!(self.labels[i - 1][j], Label::Solid)
                            || matches!(self.labels[i][j], Label::Solid)
                        {
                            self.u[i][j] = 0.;
                        } else {
                            self.u[i][j] -= scale * (self.p[i][j] - self.p[i - 1][j]);
                        }
                    } else {
                        self.u[i][j] = 0.; // unknown velocity
                    }
                }

                // update v
                if j > 0 {
                    if matches!(self.labels[i][j - 1], Label::Fluid)
                        || matches!(self.labels[i][j], Label::Fluid)
                    {
                        if matches!(self.labels[i][j - 1], Label::Solid)
                            || matches!(self.labels[i][j], Label::Solid)
                        {
                            self.v[i][j] = 0.;
                        } else {
                            self.v[i][j] -= scale * (self.p[i][j] - self.p[i][j - 1]);
                        }
                    } else {
                        self.v[i][j] = 0.; // unknown velocity
                    }
                }
            }
        }

        // transfer the velocities from the grid to the particles
        // TODO: this step is where PIC/FLIP/APIC differ
        for particle in self.particles.iter_mut() {
            let pic_interp = interp_vel(&self.u, &self.v, self.dx, self.m, self.n, &particle.pos);
            particle.vel = pic_interp;
        }

        // advect particles
        self.advect_particles(dt);

        // TODO: perform velocity grid extrapolation after grid-to-particle and after particle-to-grid
    }

    /// Move the particles according to the velocity field
    fn advect_particles(&mut self, dt: Float) {
        for particle in self.particles.iter_mut() {
            let mut sub_time = 0.;
            let mut finished = false;
            while !finished {
                let cur_vel = interp_vel(&self.u, &self.v, self.dx, self.m, self.n, &particle.pos);
                let mut dT = self.dx / (cur_vel.norm() + Float::MIN_POSITIVE);
                if sub_time + dT >= dt {
                    dT = dt - sub_time;
                    finished = true;
                } else if sub_time + 2.0 * dT >= dt {
                    dT = 0.5 * (dt - sub_time);
                }

                RK3(
                    particle, &cur_vel, dt, &self.u, &self.v, self.dx, self.m, self.n,
                );
                sub_time += dT;

                if particle.pos.x < 0. || {
                    particle.pos.y < 0. || particle.pos.x.is_nan() || particle.pos.x.is_nan()
                } {
                    flog!("RK3 error. Skipping particle");
                    break;
                }

                let (j, k) = get_grid_cell_index(&particle.pos, self.dx);
                if matches!(self.labels[j as usize][k as usize], Label::Solid) {
                    // TODO: project the particle back from solid
                }
            }
        }
    }

    /// Solve Ap = b by preconditioned conjugate gradient method,
    /// and update the velocity field accordingly
    fn pcg(
        &mut self,
        A_diag: &Vec<Vec<Float>>,
        Ax: &Vec<Vec<Float>>,
        Ay: &Vec<Vec<Float>>,
        b: &Vec<Vec<Float>>,
        precon: &Vec<Vec<Float>>,
    ) {
        if b.iter().flatten().sum::<Float>() == 0. {
            return;
        }

        // reset pressure to 0
        self.p = vec![vec![0.; self.n]; self.m];
        let mut r = b.clone();
        let z = self.apply_precon(&r, &precon, &Ax, &Ay);
        let mut s = z.clone();

        let mut sigma = self.dot(&z, &r);
        let mut converged = false;
        let PCG_MAX_ITERS = 200;
        let PCG_TOL = 1e-6;
        for _ in 0..PCG_MAX_ITERS {
            let z = self.apply_A(&s, &A_diag, &Ax, &Ay);
            let alpha = sigma / self.dot(&z, &s);
            // update pressure and residual
            for i in 0..self.m {
                for j in 0..self.n {
                    self.p[i][j] += alpha * s[i][j];
                    r[i][j] -= alpha * z[i][j];
                }
            }
            // check if within tolerance
            let r_max = r
                .iter()
                .flatten()
                .max_by(|&a, &b| a.partial_cmp(b).unwrap())
                .unwrap();
            if *r_max < PCG_TOL {
                converged = true;
                break;
            }

            // otherwise, new auxiliary vector
            let z = self.apply_precon(&r, &precon, &Ax, &Ay);
            let new_sigma = self.dot(&z, &r);
            let beta = new_sigma / sigma;
            for i in 0..self.m {
                for j in 0..self.n {
                    s[i][j] = z[i][j] + (beta * s[i][j]);
                }
            }

            sigma = new_sigma;
        }

        assert!(
            converged,
            "PCG did not converge after {} iterations",
            PCG_MAX_ITERS
        );
    }

    /// Returns A * s
    fn apply_A(
        &self,
        s: &Vec<Vec<Float>>,
        A_diag: &Vec<Vec<Float>>,
        Ax: &Vec<Vec<Float>>,
        Ay: &Vec<Vec<Float>>,
    ) -> Vec<Vec<Float>> {
        let mut z = vec![vec![0.; self.n]; self.m];
        for i in 0..self.m {
            for j in 0..self.n {
                if !self.is_fluid(i, j) {
                    continue;
                }
                z[i][j] = A_diag[i][j] * s[i][j] + Ax[i][j] * s[i + 1][j] + Ay[i][j] * s[i][j + 1];
                if i > 0 && i < self.m - 1 {
                    z[i][j] += Ax[i - 1][j] * s[i - 1][j];
                }
                if j > 0 && j < self.n - 1 {
                    z[i][j] += Ay[i][j - 1] * s[i][j - 1];
                }
            }
        }

        z
    }

    /// Returns the dot product between z and r
    fn dot(&self, z: &Vec<Vec<Float>>, r: &Vec<Vec<Float>>) -> Float {
        let mut sum = 0.;
        for i in 0..self.m {
            for j in 0..self.n {
                sum += z[i][j] * r[i][j]
            }
        }
        sum
    }

    /// Solve for z in z = M * r
    fn apply_precon(
        &self,
        r: &Vec<Vec<Float>>,
        precon: &Vec<Vec<Float>>,
        Ax: &Vec<Vec<Float>>,
        Ay: &Vec<Vec<Float>>,
    ) -> Vec<Vec<Float>> {
        // first solve Lq = r
        let mut q = vec![vec![0.; self.n]; self.m];
        for i in 0..self.m {
            for j in 0..self.n {
                if !self.is_fluid(i, j) {
                    continue;
                }

                let mut Ax_im1j = 0.;
                let mut Ay_ijm1 = 0.;
                let mut precon_im1j = 0.;
                let mut precon_ijm1 = 0.;
                let mut q_im1j = 0.;
                let mut q_ijm1 = 0.;
                if i > 0 && i < self.m - 1 {
                    if self.is_fluid(i - 1, j) {
                        Ax_im1j = Ax[i - 1][j];
                        precon_im1j = precon[i - 1][j];
                        q_im1j = q[i - 1][j];
                    }
                }
                if j > 0 && j < self.n - 1 {
                    if self.is_fluid(i, j - 1) {
                        Ay_ijm1 = Ay[i][j - 1];
                        precon_ijm1 = precon[i][j - 1];
                        q_ijm1 = q[i][j - 1];
                    }
                }

                let t =
                    r[i][j] - (Ax_im1j * precon_im1j * q_im1j) - (Ay_ijm1 * precon_ijm1 * q_ijm1);
                q[i][j] = t * precon[i][j];
            }
        }

        // then solve L^T z = q;
        let mut z = vec![vec![0.; self.n]; self.m];
        for i in (0..self.m).rev() {
            for j in (0..self.n).rev() {
                if !self.is_fluid(i, j) {
                    continue;
                }
                let Ax_ij = Ax[i][j];
                let Ay_ij = Ay[i][j];
                let precon_ij = precon[i][j];
                let mut z_ip1j = 0.;
                let mut z_ijp1 = 0.;
                if i > 0 && i < self.m - 1 && self.is_fluid(i + 1, j) {
                    z_ip1j = z[i + 1][j];
                }
                if j > 0 && j < self.n - 1 && self.is_fluid(i, j + 1) {
                    z_ijp1 = z[i][j + 1];
                }
                let t = q[i][j] - (Ax_ij * precon_ij * z_ip1j) - (Ay_ij * precon_ij * z_ijp1);

                z[i][j] = t * precon_ij;
            }
        }

        z
    }

    fn is_fluid(&self, i: usize, j: usize) -> bool {
        if i == self.m && j == self.n {
            panic!("shouldn't have both i and edge at boundary");
        }
        if i == self.m && matches!(self.labels[i - 1][j], Label::Fluid) {
            return true;
        }
        if j == self.n && matches!(self.labels[i][j - 1], Label::Fluid) {
            return true;
        }

        matches!(self.labels[i][j], Label::Fluid)
    }
}

#[cfg(test)]
mod fluid2D_tests {
    use crate::fluid::Fluid2D;

    #[test]
    fn fluid() {
        // Arrange
        let mut state = Fluid2D::new();

        // Act
        let final_time = 1.;
        let dt = 1. / 240.;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
        }
    }

    // TODO: add meaningful tests for Fluid2D
}
