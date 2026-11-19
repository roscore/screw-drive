//! Ball / roller / lead screw linear axis model (pure Rust, host-testable).
//!
//! Textbook relations only; all preset values are generic "typical" estimates.
//! Internally SI (m, N, s, rad); inputs/outputs carry their unit in the field name.

use std::f64::consts::PI;

pub const G: f64 = 9.81;
/// Young's modulus of steel (Pa).
pub const E_STEEL: f64 = 206e9;
/// Density of steel (kg/m³).
pub const RHO_STEEL: f64 = 7850.0;
/// Trapezoidal thread flank half-angle (15°): effective friction μ' = μ / cos 15°.
const TRAP_HALF_ANGLE: f64 = 15.0 * PI / 180.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Ball,
    Roller,
    TrapBronze,
    TrapPom,
}

impl Kind {
    pub const ALL: [Kind; 4] = [Kind::Ball, Kind::Roller, Kind::TrapBronze, Kind::TrapPom];
    pub fn preset(self) -> Preset {
        match self {
            // eta: forward efficiency for rolling screws; mu: sliding thread friction.
            Kind::Ball => Preset { eta: Some(0.90), mu: 0.0, ca_per_mm2: 45.0, dn_max: 70_000.0 },
            Kind::Roller => Preset { eta: Some(0.85), mu: 0.0, ca_per_mm2: 130.0, dn_max: 140_000.0 },
            Kind::TrapBronze => Preset { eta: None, mu: 0.10, ca_per_mm2: 8.0, dn_max: 12_000.0 },
            Kind::TrapPom => Preset { eta: None, mu: 0.15, ca_per_mm2: 3.0, dn_max: 8_000.0 },
        }
    }
}

/// Generic typical values (own rough estimates, not catalogue data).
#[derive(Clone, Copy, Debug)]
pub struct Preset {
    /// Forward efficiency (rolling screws).
    pub eta: Option<f64>,
    /// Thread friction coefficient (sliding screws).
    pub mu: f64,
    /// Default dynamic load rating Ca ≈ ca_per_mm2 · d² (N, d in mm).
    pub ca_per_mm2: f64,
    /// Max DN = d (mm) · n (rpm).
    pub dn_max: f64,
}

impl Preset {
    pub fn default_ca_kn(&self, d_mm: f64) -> f64 {
        self.ca_per_mm2 * d_mm * d_mm / 1000.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Support {
    FixedFree,
    FixedSimple,
    SimpleSimple,
    FixedFixed,
}

impl Support {
    pub const ALL: [Support; 4] = [Support::FixedFree, Support::FixedSimple, Support::SimpleSimple, Support::FixedFixed];
    /// Euler end-fixity factor k in P = k·π²·E·I / L².
    pub fn buckling_k(self) -> f64 {
        match self {
            Support::FixedFree => 0.25,
            Support::SimpleSimple => 1.0,
            Support::FixedSimple => 2.046,
            Support::FixedFixed => 4.0,
        }
    }
    /// First-mode eigenvalue β·L of a uniform beam (ω = β² · sqrt(EI/ρA) / L²).
    pub fn whirl_beta(self) -> f64 {
        match self {
            Support::FixedFree => 1.875,
            Support::SimpleSimple => PI,
            Support::FixedSimple => 3.927,
            Support::FixedFixed => 4.730,
        }
    }
    /// Whether the end at x = 0 (motor side) / x = L is fixed (two bearings) or not.
    pub fn ends(self) -> (bool, Option<bool>) {
        match self {
            Support::FixedFree => (true, None),
            Support::FixedSimple => (true, Some(false)),
            Support::SimpleSimple => (false, Some(false)),
            Support::FixedFixed => (true, Some(true)),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub kind: Kind,
    pub d_mm: f64,
    pub lead_mm: f64,
    pub length_mm: f64,
    pub support: Support,
    pub ca_kn: f64,
    pub motor_torque: f64,
    pub motor_rpm: f64,
    /// Reduction motor → screw (motor revs per screw rev).
    pub ratio: f64,
    pub mass_kg: f64,
    pub angle_deg: f64,
    pub f_ext: f64,
    pub stroke_mm: f64,
    pub v_mm_s: f64,
    pub acc: f64,
    pub cycles_per_day: f64,
    pub years: f64,
    pub guide_mu: f64,
}

impl Default for Inputs {
    fn default() -> Self {
        Inputs {
            kind: Kind::Ball,
            d_mm: 16.0,
            lead_mm: 5.0,
            length_mm: 400.0,
            support: Support::FixedSimple,
            ca_kn: Kind::Ball.preset().default_ca_kn(16.0),
            motor_torque: 0.8,
            motor_rpm: 3000.0,
            ratio: 1.0,
            mass_kg: 20.0,
            angle_deg: 60.0,
            f_ext: 300.0,
            stroke_mm: 200.0,
            v_mm_s: 150.0,
            acc: 2.0,
            cycles_per_day: 2000.0,
            years: 3.0,
            guide_mu: 0.01,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Status {
    Ok,
    Warn,
    Fail,
}

fn grade(margin: f64, warn_below: f64) -> Status {
    if margin.is_nan() || margin < 1.0 {
        Status::Fail
    } else if margin < warn_below {
        Status::Warn
    } else {
        Status::Ok
    }
}

/// One phase of the move: duration (s), start / end speed (m/s, signed), axial force (N, signed).
#[derive(Clone, Copy, Debug)]
pub struct Phase {
    pub t: f64,
    pub v0: f64,
    pub v1: f64,
    pub force: f64,
}

impl Phase {
    pub fn dist(&self) -> f64 {
        0.5 * (self.v0 + self.v1).abs() * self.t
    }
}

/// Trapezoidal (or triangular) move over `s` m with speed limit `v` and acceleration `a`.
/// Returns (accel time, cruise time, peak speed).
pub fn trapezoid(s: f64, v: f64, a: f64) -> (f64, f64, f64) {
    if s <= 0.0 || v <= 0.0 || a <= 0.0 {
        return (0.0, 0.0, 0.0);
    }
    let vp = v.min((s * a).sqrt());
    let ta = vp / a;
    (ta, (s - vp * ta) / vp, vp)
}

/// Lead angle λ = atan(l / (π·d)) (rad).
pub fn lead_angle(d_mm: f64, lead_mm: f64) -> f64 {
    (lead_mm / (PI * d_mm)).atan()
}

/// Equivalent friction angle φ: from μ (with the trapezoidal flank correction), or from a preset
/// forward efficiency η by solving η = tanλ / tan(λ+φ).
pub fn friction_angle(p: &Preset, lambda: f64) -> f64 {
    match p.eta {
        Some(eta) => (lambda.tan() / eta).atan() - lambda,
        None => (p.mu / TRAP_HALF_ANGLE.cos()).atan(),
    }
}

/// Forward and back-drive efficiency (back-drive ≤ 0 means self-locking).
pub fn efficiencies(lambda: f64, phi: f64) -> (f64, f64) {
    (lambda.tan() / (lambda + phi).tan(), (lambda - phi).tan() / lambda.tan())
}

/// Torque (N·m) to push `force` (N) with lead `lead_m` (m) at efficiency `eta`.
pub fn drive_torque(force: f64, lead_m: f64, eta: f64) -> f64 {
    force * lead_m / (2.0 * PI * eta)
}

/// Root diameter estimate (mm): d − 0.6·l, but never less than 0.75·d.
pub fn root_diameter(d_mm: f64, lead_mm: f64) -> f64 {
    d_mm - (0.6 * lead_mm).min(0.25 * d_mm)
}

/// Euler buckling load (N) for root diameter `dr` (m) and free length `l` (m).
pub fn buckling_load(dr: f64, l: f64, s: Support) -> f64 {
    let i = PI * dr.powi(4) / 64.0;
    s.buckling_k() * PI * PI * E_STEEL * i / (l * l)
}

/// First whirling (critical) speed (rpm) of a solid steel shaft of diameter `dr` (m), length `l` (m):
/// ω = β² · sqrt(E·I / (ρ·A)) / L².
pub fn critical_rpm(dr: f64, l: f64, s: Support) -> f64 {
    let (i, a) = (PI * dr.powi(4) / 64.0, PI * dr * dr / 4.0);
    let w = s.whirl_beta().powi(2) * (E_STEEL * i / (RHO_STEEL * a)).sqrt() / (l * l);
    w / (2.0 * PI) * 60.0
}

/// Cubic-mean load over (|force|, distance) pairs, weighted by distance (= revolutions).
pub fn cubic_mean(loads: &[(f64, f64)]) -> f64 {
    let (num, den) = loads.iter().fold((0.0, 0.0), |(n, d), (f, s)| (n + f.abs().powi(3) * s, d + s));
    if den <= 0.0 {
        0.0
    } else {
        (num / den).cbrt()
    }
}

/// Basic rating life L10 in revolutions: (Ca / Fm)³ · 10⁶.
pub fn l10_rev(ca: f64, fm: f64) -> f64 {
    if fm <= 0.0 {
        f64::INFINITY
    } else {
        (ca / fm).powi(3) * 1e6
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Check {
    /// ≥ 1 passes; larger is safer.
    pub margin: f64,
    pub status: Status,
}

#[derive(Clone, Debug)]
pub struct Results {
    pub lambda_deg: f64,
    pub eta: f64,
    pub eta_back: f64,
    pub self_locking: bool,
    /// Forward move then return move (signed speed / force along +axis).
    pub phases: Vec<Phase>,
    pub t_move: f64,
    pub v_peak: f64,
    pub f_peak: f64,
    pub fm: f64,
    pub j_screw: f64,
    pub j_load: f64,
    /// Peak torque at the screw and at the motor (N·m).
    pub t_screw: f64,
    pub t_motor: f64,
    pub screw_rpm: f64,
    pub motor_rpm_need: f64,
    pub f_avail: f64,
    pub v_motor_max: f64,
    pub dr_mm: f64,
    pub p_buckle: f64,
    pub n_crit: f64,
    /// Linear speed limits (mm/s) from 0.8·critical speed and from DN.
    pub v_crit: f64,
    pub v_dn: f64,
    pub dn: f64,
    pub l10_rev: f64,
    pub l10_km: f64,
    pub l10_years: f64,
    pub torque: Check,
    pub speed: Check,
    pub buckling: Check,
    pub whirl: Check,
    pub dn_check: Check,
    pub life: Check,
    pub overall: Status,
}

impl Results {
    pub fn checks(&self) -> [Check; 6] {
        [self.torque, self.speed, self.buckling, self.whirl, self.dn_check, self.life]
    }
    pub fn min_margin(&self) -> f64 {
        self.checks().iter().map(|c| c.margin).fold(f64::INFINITY, f64::min)
    }
}

/// Phases of one full cycle: extend (accel, cruise, decel) then retract.
fn build_phases(inp: &Inputs) -> Vec<Phase> {
    let (ta, tc, vp) = trapezoid(inp.stroke_mm / 1000.0, inp.v_mm_s / 1000.0, inp.acc);
    let th = inp.angle_deg.to_radians();
    let m = inp.mass_kg;
    let static_f = m * G * th.sin() + inp.f_ext;
    let fric = inp.guide_mu * m * G * th.cos();
    let a = if ta > 0.0 { vp / ta } else { 0.0 };
    let mut out = vec![];
    for dir in [1.0, -1.0] {
        for (t, v0, v1, acc) in [(ta, 0.0, vp, a), (tc, vp, vp, 0.0), (ta, vp, 0.0, -a)] {
            out.push(Phase { t, v0: dir * v0, v1: dir * v1, force: m * acc * dir + static_f + fric * dir });
        }
    }
    out
}

pub fn compute(inp: &Inputs) -> Results {
    let p = inp.kind.preset();
    let lead = inp.lead_mm / 1000.0;
    let lam = lead_angle(inp.d_mm, inp.lead_mm);
    let (eta, eta_back) = efficiencies(lam, friction_angle(&p, lam));
    let phases = build_phases(inp);
    let t_move = phases[..3].iter().map(|p| p.t).sum::<f64>();
    let v_peak = phases[1].v0.abs();
    let f_peak = phases.iter().map(|p| p.force.abs()).fold(0.0, f64::max);
    let fm = cubic_mean(&phases.iter().map(|p| (p.force, p.dist())).collect::<Vec<_>>());

    let (d, l) = (inp.d_mm / 1000.0, inp.length_mm / 1000.0);
    let j_screw = RHO_STEEL * PI * d.powi(4) * l / 32.0;
    let k = lead / (2.0 * PI);
    let j_load = inp.mass_kg * k * k;
    // Thrust without the load's own inertia drives through η; inertia added as reflected J.
    let t_screw = phases
        .iter()
        .map(|ph| {
            let acc = if ph.t > 0.0 { (ph.v1 - ph.v0) / ph.t } else { 0.0 };
            let f_static = ph.force - inp.mass_kg * acc;
            (drive_torque(f_static, lead, eta) + (j_screw + j_load / eta) * acc / k).abs()
        })
        .fold(0.0, f64::max);
    let ratio = inp.ratio.max(1e-6);
    let t_motor = t_screw / ratio;
    let screw_rpm = v_peak / lead * 60.0;
    let motor_rpm_need = screw_rpm * ratio;
    let f_avail = inp.motor_torque * ratio * 2.0 * PI * eta / lead;
    let v_motor_max = inp.motor_rpm / ratio / 60.0 * inp.lead_mm;

    let dr_mm = root_diameter(inp.d_mm, inp.lead_mm);
    let dr = dr_mm / 1000.0;
    let p_buckle = buckling_load(dr, l, inp.support);
    let n_crit = critical_rpm(dr, l, inp.support);
    let v_crit = 0.8 * n_crit / 60.0 * inp.lead_mm;
    let v_dn = p.dn_max / inp.d_mm / 60.0 * inp.lead_mm;
    let dn = inp.d_mm * screw_rpm;

    let l10 = l10_rev(inp.ca_kn * 1000.0, fm);
    let revs_per_day = 2.0 * inp.stroke_mm / inp.lead_mm * inp.cycles_per_day;
    let l10_years = if revs_per_day > 0.0 { l10 / revs_per_day / 365.0 } else { f64::INFINITY };

    let ratio_or_inf = |a: f64, b: f64| if b > 0.0 { a / b } else { f64::INFINITY };
    let torque = { let m = ratio_or_inf(inp.motor_torque, t_motor); Check { margin: m, status: grade(m, 1.25) } };
    let speed = { let m = ratio_or_inf(inp.motor_rpm, motor_rpm_need); Check { margin: m, status: grade(m, 1.1) } };
    let buckling = { let m = ratio_or_inf(p_buckle, f_peak); Check { margin: m, status: grade(m, 2.0) } };
    let whirl = { let m = ratio_or_inf(0.8 * n_crit, screw_rpm); Check { margin: m, status: grade(m, 1.25) } };
    let dn_check = { let m = ratio_or_inf(p.dn_max, dn); Check { margin: m, status: grade(m, 1.25) } };
    let life = { let m = ratio_or_inf(l10_years, inp.years); Check { margin: m, status: grade(m, 1.5) } };
    let overall = [torque, speed, buckling, whirl, dn_check, life].iter().map(|c| c.status).max().unwrap_or(Status::Ok);

    Results {
        lambda_deg: lam.to_degrees(),
        eta,
        eta_back,
        self_locking: eta_back <= 0.0,
        phases,
        t_move,
        v_peak,
        f_peak,
        fm,
        j_screw,
        j_load,
        t_screw,
        t_motor,
        screw_rpm,
        motor_rpm_need,
        f_avail,
        v_motor_max,
        dr_mm,
        p_buckle,
        n_crit,
        v_crit,
        v_dn,
        dn,
        l10_rev: l10,
        l10_km: l10 * inp.lead_mm / 1e6,
        l10_years,
        torque,
        speed,
        buckling,
        whirl,
        dn_check,
        life,
        overall,
    }
}

/// Position (m), speed (m/s) and axial force (N) at time `t` (wrapped to one cycle).
pub fn sample(r: &Results, t: f64) -> (f64, f64, f64) {
    let total: f64 = r.phases.iter().map(|p| p.t).sum();
    if total <= 0.0 {
        return (0.0, 0.0, r.phases.first().map_or(0.0, |p| p.force));
    }
    let mut t = t.rem_euclid(total);
    let mut x = 0.0;
    for ph in &r.phases {
        if t <= ph.t || std::ptr::eq(ph, r.phases.last().unwrap()) {
            let t = t.min(ph.t);
            let a = if ph.t > 0.0 { (ph.v1 - ph.v0) / ph.t } else { 0.0 };
            return (x + ph.v0 * t + 0.5 * a * t * t, ph.v0 + a * t, ph.force);
        }
        x += 0.5 * (ph.v0 + ph.v1) * ph.t;
        t -= ph.t;
    }
    (x, 0.0, 0.0)
}

pub fn cycle_time(r: &Results) -> f64 {
    r.phases.iter().map(|p| p.t).sum()
}

pub const SEARCH_D: [f64; 8] = [8., 10., 12., 14., 16., 20., 25., 32.];
pub const SEARCH_LEAD: [f64; 11] = [1., 2., 4., 5., 8., 10., 12., 16., 20., 25., 32.];

/// One search hit: diameter, lead and its results.
pub struct Hit {
    pub d_mm: f64,
    pub lead_mm: f64,
    pub res: Results,
}

/// Try every (d, lead) of the chosen type with the default Ca for that size; keep those with no
/// failed check (lead ≤ 2·d for a plausible thread), sort by (min margin ≥ 1.5) then torque margin.
pub fn search(base: &Inputs, top: usize) -> Vec<Hit> {
    let p = base.kind.preset();
    let mut hits = vec![];
    for d in SEARCH_D {
        for l in SEARCH_LEAD {
            if l > 2.0 * d {
                continue;
            }
            let inp = Inputs { d_mm: d, lead_mm: l, ca_kn: p.default_ca_kn(d), ..*base };
            let res = compute(&inp);
            if res.overall != Status::Fail {
                hits.push(Hit { d_mm: d, lead_mm: l, res });
            }
        }
    }
    hits.sort_by(|a, b| {
        let ka = (a.res.min_margin() >= 1.5, a.res.torque.margin);
        let kb = (b.res.min_margin() >= 1.5, b.res.torque.margin);
        kb.partial_cmp(&ka).unwrap_or(std::cmp::Ordering::Equal)
    });
    hits.truncate(top);
    hits
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol * b.abs().max(1e-12), "{a} vs {b}");
    }

    #[test]
    fn lead_angle_16_5() {
        close(lead_angle(16.0, 5.0).to_degrees(), 5.68, 0.002);
    }

    #[test]
    fn self_locking_and_back_drive() {
        let trap = Preset { eta: None, mu: 0.1, ca_per_mm2: 1.0, dn_max: 1.0 };
        let lam = lead_angle(10.0, 2.0);
        let (_, back) = efficiencies(lam, friction_angle(&trap, lam));
        assert!(back <= 0.0, "trapezoidal d10 l2 μ0.1 self-locks ({back})");
        let ball = Preset { eta: Some(0.9), ..trap };
        let lam = lead_angle(16.0, 5.0);
        let (fwd, back) = efficiencies(lam, friction_angle(&ball, lam));
        close(fwd, 0.9, 1e-9);
        assert!(back > 0.8, "ball screw back-drives ({back})");
    }

    #[test]
    fn torque_for_1000n_10mm() {
        close(drive_torque(1000.0, 0.010, 1.0), 1.5915, 1e-4);
    }

    #[test]
    fn l10_cubic_law() {
        close(l10_rev(5000.0, 5000.0), 1e6, 1e-12);
        close(l10_rev(5000.0, 2000.0) / l10_rev(5000.0, 4000.0), 8.0, 1e-12);
    }

    #[test]
    fn buckling_scaling() {
        let p1 = buckling_load(0.012, 0.5, Support::SimpleSimple);
        close(buckling_load(0.012, 1.0, Support::SimpleSimple), p1 / 4.0, 1e-12);
        close(buckling_load(0.012, 0.5, Support::FixedFixed), 4.0 * p1, 1e-12);
    }

    #[test]
    fn critical_speed_scaling() {
        let n1 = critical_rpm(0.012, 0.5, Support::FixedSimple);
        close(critical_rpm(0.012, 1.0, Support::FixedSimple), n1 / 4.0, 1e-12);
        // Hand check, simple-simple: ω = π²/L² · sqrt(E·dr²/(16ρ)).
        let w = PI * PI / 0.25 * (E_STEEL * 0.012f64.powi(2) / 16.0 / RHO_STEEL).sqrt();
        close(critical_rpm(0.012, 0.5, Support::SimpleSimple), w * 60.0 / (2.0 * PI), 1e-12);
    }

    #[test]
    fn cubic_mean_of_constant() {
        close(cubic_mean(&[(700.0, 0.1), (-700.0, 0.3), (700.0, 2.0)]), 700.0, 1e-12);
    }

    #[test]
    fn move_profile_and_triangular_case() {
        let (ta, tc, vp) = trapezoid(0.2, 0.15, 2.0);
        close(vp, 0.15, 1e-12);
        close(vp * ta + vp * tc, 0.2, 1e-12);
        let (ta, tc, vp) = trapezoid(0.01, 1.0, 1.0);
        close(vp, 0.1, 1e-12);
        assert!(tc.abs() < 1e-12 && (ta - 0.1).abs() < 1e-12);
        let r = compute(&Inputs::default());
        let (x, _, _) = sample(&r, r.t_move);
        close(x, 0.2, 1e-9);
        let (x, _, _) = sample(&r, cycle_time(&r) - 1e-9);
        assert!(x.abs() < 1e-6);
    }

    #[test]
    fn search_hits_pass_and_are_sorted() {
        let base = Inputs::default();
        assert_eq!(compute(&base).overall, Status::Ok, "defaults are a sane design");
        let hits = search(&base, 12);
        assert!(!hits.is_empty());
        assert!(hits.len() <= 12);
        for h in &hits {
            assert!(h.res.checks().iter().all(|c| c.status != Status::Fail));
        }
        for w in hits.windows(2) {
            let k = |h: &Hit| (h.res.min_margin() >= 1.5, h.res.torque.margin);
            assert!(k(&w[0]) >= k(&w[1]));
        }
    }
}
