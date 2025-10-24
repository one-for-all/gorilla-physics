use crate::types::Float;

/// Cardano's method to solve a cubic equation
/// Solve for x in ax^3 + bx^2 + cx + d = 0
/// ref: https://en.wikipedia.org/wiki/Cubic_equation#Cardano's_formula
pub fn solve_cubic(a: Float, b: Float, c: Float, d: Float) -> Option<Float> {
    if a == 0. {
        return solve_quadratic(b, c, d);
    }

    // Convert to depressed form
    // ref: https://en.wikipedia.org/wiki/Cubic_equation#Depressed_cubic
    let p = (3. * a * c - b * b) / (3. * a * a);
    let q = (2. * b * b * b - 9. * a * b * c + 27. * a * a * d) / (27. * a * a * a);
    let discriminant = (q * q / 4.0) + (p * p * p / 27.0);
    let offset = -b / (3. * a);

    if discriminant > 0.0 {
        // One real root
        let sqrt_disc = discriminant.sqrt();
        let u = (-q / 2. + sqrt_disc).cbrt();
        let v = (-q / 2. - sqrt_disc).cbrt();
        return Some(u + v + offset);
    } else {
        panic!(
            "should not have an equation with non-singular root in edge-edge ccd. discriminant: {}, abcd: {}, {}, {}, {}",
            discriminant, a, b, c, d
        );
    }
}

pub fn solve_quadratic(a: Float, b: Float, c: Float) -> Option<Float> {
    if a == 0. {
        return solve_linear(b, c);
    }

    let det = b * b - 4. * a * c;
    if det < 0. {
        return None;
    }
    if det == 0. {
        return Some(-b / (2. * a));
    } else {
        let sqrt = det.sqrt();
        return Some((-b - sqrt) / (2. * a)); // TODO: there should be two solutions
    }
}

pub fn solve_linear(a: Float, b: Float) -> Option<Float> {
    if a == 0. {
        if b == 0. {
            return Some(0.);
        }
        return None;
    }
    return Some(-b / a);
}

#[cfg(test)]
mod cubic_tests {
    use crate::{assert_close, collision::ccd::solver::solve_cubic};

    #[test]
    fn single_root1() {
        let sol = solve_cubic(1., 0., 1., 1.).unwrap();
        assert_close!(sol, -0.6823278, 1e-5);
    }

    #[test]
    fn single_root2() {
        let sol = solve_cubic(1., 0., -3., 5.).unwrap();
        assert_close!(sol, -2.27902, 1e-5);
    }

    #[test]
    fn single_root3() {
        let sol = solve_cubic(2., 4., 5., 10.).unwrap();
        assert_close!(sol, -2.0, 1e-5);
    }
}
