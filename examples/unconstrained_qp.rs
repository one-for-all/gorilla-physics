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

    // Changing parameters to sqrt of default values can make it work.
    // The default values were tuned for f64 case.
    // Default settings: https://clarabel.org/stable/api_settings/
    // Issue comment: https://github.com/oxfordcontrol/Clarabel.jl/issues/105#issuecomment-1344322700
    let tol: f32 = 1e-8;
    let settings = DefaultSettingsBuilder::default()
        .verbose(true)
        // .tol_gap_abs(tol.sqrt())
        // .tol_gap_rel(tol.sqrt())
        // .tol_feas(tol.sqrt())
        .build()
        .unwrap();

    let mut solver = DefaultSolver::new(&P, &q, &A, &b, &cones, settings);

    solver.solve();

    println!("Status: {:?}", solver.solution.status);
    println!("Solution: {:?}", solver.solution.x);
}
