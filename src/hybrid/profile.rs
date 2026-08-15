//! Where the time inside [`Hybrid::step`](super::Hybrid::step) goes.
//!
//! A scene that will not keep up with the wall clock says so through
//! `Simulator.realtimeRatio`, and that is the whole of what it says: the number
//! conflates the contact solve, the collision scan, and whatever hardware a
//! controller is emulating, all of which live behind the same `step` call and
//! none of which a JS-side timer can tell apart. This splits that one call into
//! the phases it actually runs, so the answer to "what is slow" is read off
//! rather than guessed at.
//!
//! **The numbers are only meaningful in aggregate, and that is by design.**
//! `performance.now()` in a browser is deliberately coarse -- Chrome quantises
//! it to 100 us unless the page is cross-origin isolated, which is far larger
//! than a single phase of a single step. Truncation to that grid is unbiased
//! over many samples, though, because the phase boundaries drift freely against
//! it: charging a phase 0 us on one step and 100 us on the next averages out to
//! what it really costs. So a phase total is trustworthy after a thousand steps
//! and worthless after one, which is why [`StepProfile::take`] hands back an
//! accumulation and a step count rather than a per-step figure, and why nothing
//! here ever reports the cost of an individual step.
//!
//! The instrument itself is nine clock reads per step. At `run(60, 0)` -- 3600
//! steps a simulated second -- that is ~32k reads a second, well under a
//! millisecond of the second they are measuring, so it stays on rather than
//! being something to switch in when a scene misbehaves.

use crate::types::Float;

/// A phase of one [`Hybrid::step`](super::Hybrid::step), in the order the step
/// runs them.
///
/// The discriminants index [`StepProfile::totals`], so the order here and the
/// order of [`PHASE_NAMES`] are the same order, and both are `step`'s.
#[derive(Clone, Copy)]
pub enum Phase {
    /// Per-body jacobians and the articulated mass matrix.
    Kinematics = 0,
    /// The controllers' torques: `ArticulatedController::control`.
    Control,
    /// The controllers' own time: `ArticulatedController::step`. This is where
    /// an emulated microcontroller runs, and it is charged nothing else.
    Board,
    /// Unconstrained velocity of everything, and the stacked-vector offsets.
    FreeVelocity,
    /// Every collision pass, and the contact jacobian each contact produces.
    Collision,
    /// Bilateral constraints, joint range constraints, dry friction joints.
    Constraints,
    /// Building the QP: the block mass matrix, P, q, A, b, the cones, and the
    /// `DefaultSolver` that is constructed fresh around them each step.
    Assemble,
    /// `DefaultSolver::solve` alone.
    Solve,
    /// Writing the solved velocity back and integrating poses.
    Integrate,
}

/// How many phases a step is split into.
pub const PHASES: usize = 9;

/// Display names, in [`Phase`] order. Short enough to sit in the corner of the
/// view without wrapping.
pub const PHASE_NAMES: [&str; PHASES] = [
    "kinematics",
    "control",
    "board",
    "free vel",
    "collision",
    "constraints",
    "assemble",
    "solve",
    "integrate",
];

/// Accumulated time per phase, since whoever is reading last reset it.
pub struct StepProfile {
    /// Milliseconds charged to each [`Phase`], indexed by its discriminant.
    totals: [Float; PHASES],
    /// Steps counted into `totals`, so a reader can turn it into a per-step
    /// figure -- and can tell an idle scene from a slow one.
    steps: usize,
    /// The clock reading the current phase started at.
    mark: Float,
}

impl StepProfile {
    pub fn new() -> Self {
        StepProfile {
            totals: [0.; PHASES],
            steps: 0,
            mark: 0.,
        }
    }

    /// Open a step. The first phase is timed from here.
    pub fn begin(&mut self) {
        self.mark = now_ms();
    }

    /// Charge everything since the last mark to `phase`, and start the next one.
    ///
    /// Phases are contiguous by construction -- each call both closes one and
    /// opens the next -- so nothing in a step can go uncharged, and the totals
    /// sum to the wall time the steps took.
    pub fn mark(&mut self, phase: Phase) {
        let now = now_ms();
        self.totals[phase as usize] += now - self.mark;
        self.mark = now;
    }

    /// Close a step. Only the count; the last phase's time is charged by the
    /// [`mark`](Self::mark) that ended it.
    pub fn finish(&mut self) {
        self.steps += 1;
    }

    /// The accumulation so far, and reset.
    ///
    /// Reading is destructive because the point is a rolling window: a caller
    /// polling twice a second gets what the last half second cost, not a total
    /// since the page loaded that any recent change is invisible in.
    pub fn take(&mut self) -> ([Float; PHASES], usize) {
        let taken = (self.totals, self.steps);
        self.totals = [0.; PHASES];
        self.steps = 0;
        taken
    }
}

/// A monotonic clock in milliseconds.
///
/// `Instant` is not implemented on `wasm32-unknown-unknown` -- it panics rather
/// than failing to compile -- so the browser's own clock stands in there. The
/// two share no epoch, which nothing here depends on: every reading is used as
/// a difference against another reading from the same clock.
#[cfg(target_arch = "wasm32")]
fn now_ms() -> Float {
    thread_local! {
        // Looked up once: `window()` allocates a JsValue per call, and this is
        // on a path that runs thousands of times a second.
        static CLOCK: Option<web_sys::Performance> =
            web_sys::window().and_then(|w| w.performance());
    }
    CLOCK.with(|clock| clock.as_ref().map(|p| p.now()).unwrap_or(0.) as Float)
}

#[cfg(not(target_arch = "wasm32"))]
fn now_ms() -> Float {
    thread_local! {
        static CLOCK: std::time::Instant = std::time::Instant::now();
    }
    CLOCK.with(|start| start.elapsed().as_secs_f64() as Float * 1e3)
}
