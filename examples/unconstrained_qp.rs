use clarabel::solver::{DefaultSettingsBuilder, DefaultSolver, IPSolver};
use clarabel::{algebra::*, solver::SupportedConeT};
use nalgebra::Matrix2;

fn main() {
    #[rustfmt::skip]
    let P = Matrix2::new(
        0.67795455, 1.1745795,
        1.1745795, 2.7052722
    );

    // Define symmetric PSD matrix P
    let P = CscMatrix::from(P.row_iter());

    let q = vec![0.10088674, -0.15378475];

    // No constraints
    let A = CscMatrix::zeros((0, P.m));
    let b = vec![];

    // No cones
    let cones: Vec<SupportedConeT<f32>> = vec![];

    let settings = DefaultSettingsBuilder::default()
        .verbose(true)
        .build()
        .unwrap();

    let mut solver = DefaultSolver::new(&P, &q, &A, &b, &cones, settings);

    solver.solve();

    println!("Status: {:?}", solver.solution.status);
    println!("Solution: {:?}", solver.solution.x);
}
