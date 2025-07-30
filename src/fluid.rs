use na::{vector, zero, DMatrix, Vector2};
use rand::prelude::*;
use rand::{rng, Rng};

use crate::types::Float;

#[derive(Clone)]
enum Label {
    Fluid,
    Solid,
    Air,
}

pub struct Particle2D {
    pub pos: Vector2<Float>,
    pub vel: Vector2<Float>,
}

pub struct Fluid2D {
    m: usize,  // number of cells in x-axis
    n: usize,  // number of cells in y-axis
    dx: Float, // cell size

    labels: Vec<Vec<Label>>, // cell labels - Fluid/Solid/Empty
    p: Vec<Vec<Float>>,      // pressures
    u: Vec<Vec<Float>>,      // x-axis velocity
    v: Vec<Vec<Float>>,      // y-axis velocity

    pub particles: Vec<Particle2D>, // fluid particles
}

impl Fluid2D {
    pub fn new() -> Self {
        let m = 100;
        let n = 50;
        let dx = 5e-3;
        let mut labels = vec![vec![Label::Air; n]; m];

        // Set up solid boundary
        for i in 0..m {
            labels[i][0] = Label::Solid;
        }
        for j in 0..n {
            labels[0][j] = Label::Solid;
            labels[m - 1][j] = Label::Solid;
        }

        // Add fluid
        for i in 1..m / 2 {
            for j in 1..n {
                labels[i][j] = Label::Fluid;
            }
        }

        let p = vec![vec![0.; n]; m];
        let u = vec![vec![0.; n]; m + 1];
        let v = vec![vec![0.; n + 1]; m];

        let mut rng = rng();
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

    pub fn step() {}
}
