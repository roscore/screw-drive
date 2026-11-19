//! Planar joint mechanisms driven by linear actuators, and their coupling with the screw model.
//!
//! Textbook kinematics/statics only. SI internally (m, rad, N, N·m). Every mechanism maps the joint
//! angle q to the actuator length s(q) and the transmission ratio J(q) = ds/dq (m/rad); by virtual
//! work the joint torque of an actuator force F is τ = F·J, so the moment arm is h = |J|.
//!
//! Knee frame (sagittal plane): origin at the knee axis, x forward, y up along the thigh. Flexion q
//! rotates the shank backwards (clockwise seen from the robot's right side), so a point on the shank
//! at body angle a sits at angle a − q.
//!
//! Ankle frame: origin at the ankle axis, x forward, y lateral (left), z up along the shank. The foot
//! rotates by R = Ry(pitch)·Rx(roll) (pitch axis on the shank, roll axis on the foot; positive pitch =
//! toes down / plantarflexion, positive roll = left edge up).

use crate::{buckling_load, compute, cubic_mean, l10_rev, Inputs, Kind, Status, G};
use std::f64::consts::PI;

pub type P2 = [f64; 2];
pub type P3 = [f64; 3];
pub type M3 = [[f64; 3]; 3];

fn sub2(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot2(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn len2(a: P2) -> f64 {
    dot2(a, a).sqrt()
}
/// a rotated by +90°.
fn perp(a: P2) -> P2 {
    [-a[1], a[0]]
}
/// Acute angle (deg, 0…90) between the lines along `a` and `b`; 90° is ideal force transmission.
fn line_angle_deg(a: P2, b: P2) -> f64 {
    let n = len2(a) * len2(b);
    if n <= 0.0 {
        return 0.0;
    }
    (dot2(a, b).abs() / n).min(1.0).acos().to_degrees()
}

/// Transmission angle below this is a warning (deg).
pub const MU_WARN: f64 = 20.0;
/// Transmission angle below this fails (deg).
pub const MU_FAIL: f64 = 10.0;

/// Knee driven by an actuator pushing a lever on the shank (slider-crank).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SliderCrank {
    /// Actuator base pivot on the thigh (m, knee frame).
    pub ax: f64,
    pub ay: f64,
    /// Shank lever radius (m).
    pub r: f64,
    /// Lever angle at q = 0 (rad, from +x, counter-clockwise).
    pub q0: f64,
}

impl SliderCrank {
    pub fn base(&self) -> P2 {
        [self.ax, self.ay]
    }
    pub fn rod_end(&self, q: f64) -> P2 {
        let a = self.q0 - q;
        [self.r * a.cos(), self.r * a.sin()]
    }
    pub fn length(&self, q: f64) -> f64 {
        len2(sub2(self.rod_end(q), self.base()))
    }
    /// ds/dq (m/rad), analytic.
    pub fn ds_dq(&self, q: f64) -> f64 {
        let a = self.q0 - q;
        let d = sub2(self.rod_end(q), self.base());
        let dp = [self.r * a.sin(), -self.r * a.cos()];
        dot2(d, dp) / len2(d)
    }
    /// Angle between the rod and the lever (deg, 90 = rod perpendicular to the lever, h = r).
    pub fn transmission_deg(&self, q: f64) -> f64 {
        let p = self.rod_end(q);
        line_angle_deg(sub2(p, self.base()), p)
    }
}

/// Knee driven through a four-bar: the actuator (base B on the thigh) pushes the tip K of a crank
/// pivoting at C on the thigh; a coupler K–P drives the shank lever O–P.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FourBar {
    pub bx: f64,
    pub by: f64,
    /// Crank pivot on the thigh (ground offset from the knee axis).
    pub cx: f64,
    pub cy: f64,
    /// Crank radius |C–K|, coupler length |K–P|, shank lever radius |O–P| (m).
    pub rc: f64,
    pub lc: f64,
    pub rs: f64,
    /// Shank lever angle at q = 0 (rad).
    pub q0: f64,
    /// Assembly mode: +1 or −1 (side of the line C–P the crank tip lies on).
    pub branch: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FourBarPose {
    pub k: P2,
    pub p: P2,
    pub s: f64,
    pub ds_dq: f64,
    /// Coupler–shank-lever transmission angle (deg, 0…90).
    pub mu_deg: f64,
    /// Sign of (K−C)×(P−K): flips when the linkage passes a toggle / changes assembly mode.
    pub side: f64,
}

impl FourBar {
    pub fn base(&self) -> P2 {
        [self.bx, self.by]
    }
    pub fn pivot(&self) -> P2 {
        [self.cx, self.cy]
    }
    pub fn lever(&self, q: f64) -> P2 {
        let a = self.q0 - q;
        [self.rs * a.cos(), self.rs * a.sin()]
    }
    /// Crank tip by closed-form circle intersection, or None if the loop cannot close.
    pub fn crank_tip(&self, q: f64) -> Option<P2> {
        let (c, p) = (self.pivot(), self.lever(q));
        let dv = sub2(p, c);
        let d = len2(dv);
        if d < 1e-12 || d > self.rc + self.lc || d < (self.rc - self.lc).abs() {
            return None;
        }
        let e = [dv[0] / d, dv[1] / d];
        let a = (self.rc * self.rc - self.lc * self.lc + d * d) / (2.0 * d);
        let h = (self.rc * self.rc - a * a).max(0.0).sqrt();
        let n = perp(e);
        let b = if self.branch < 0.0 { -1.0 } else { 1.0 };
        Some([c[0] + a * e[0] + b * h * n[0], c[1] + a * e[1] + b * h * n[1]])
    }
    pub fn pose(&self, q: f64) -> Option<FourBarPose> {
        let k = self.crank_tip(q)?;
        let p = self.lever(q);
        let a = self.q0 - q;
        let dp = [self.rs * a.sin(), -self.rs * a.cos()];
        let kc = sub2(k, self.pivot());
        let kp = sub2(k, p);
        // Velocity loop: K moves on the crank circle, dK = θ'·perp(K−C), and |K−P| is constant:
        // (K−P)·(dK − dP) = 0.
        let den = dot2(kp, perp(kc));
        if den.abs() < 1e-12 {
            return None;
        }
        let th = dot2(kp, dp) / den;
        let dk = [perp(kc)[0] * th, perp(kc)[1] * th];
        let kb = sub2(k, self.base());
        let s = len2(kb);
        if s < 1e-9 {
            return None;
        }
        let side = kc[0] * (p[1] - k[1]) - kc[1] * (p[0] - k[0]);
        Some(FourBarPose { k, p, s, ds_dq: dot2(kb, dk) / s, mu_deg: line_angle_deg(kp, p), side: side.signum() })
    }
}

/// Two parallel linear actuators from the shank to a foot lever (pitch + roll ankle).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ankle {
    /// Lateral spacing of the two actuators (m): pivots at y = ±w/2.
    pub w: f64,
    /// Upper pivot height above the ankle axis (m).
    pub h: f64,
    /// Foot lever: lower pivots at x = −c (behind the ankle axis), z = 0 in the foot frame (m).
    pub c: f64,
    /// Upper pivots at x = −e on the shank (m); e = c gives vertical rods at neutral.
    pub e: f64,
}

fn mat_vec(m: &M3, v: P3) -> P3 {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}
fn mat_mul(a: &M3, b: &M3) -> M3 {
    let mut o = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    o
}
fn dot3(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn sub3(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn ry(p: f64) -> (M3, M3) {
    let (s, c) = p.sin_cos();
    ([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], [[-s, 0.0, c], [0.0, 0.0, 0.0], [-c, 0.0, -s]])
}
fn rx(r: f64) -> (M3, M3) {
    let (s, c) = r.sin_cos();
    ([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], [[0.0, 0.0, 0.0], [0.0, -s, -c], [0.0, c, -s]])
}

impl Ankle {
    /// Foot rotation R = Ry(pitch)·Rx(roll).
    pub fn rot(pitch: f64, roll: f64) -> M3 {
        mat_mul(&ry(pitch).0, &rx(roll).0)
    }
    /// Upper (shank) pivot of actuator i (0 = left, +y; 1 = right, −y).
    pub fn upper(&self, i: usize) -> P3 {
        [-self.e, if i == 0 { 0.5 } else { -0.5 } * self.w, self.h]
    }
    /// Lower pivot of actuator i in the foot frame.
    pub fn foot_point(&self, i: usize) -> P3 {
        [-self.c, if i == 0 { 0.5 } else { -0.5 } * self.w, 0.0]
    }
    pub fn lower(&self, i: usize, pitch: f64, roll: f64) -> P3 {
        mat_vec(&Self::rot(pitch, roll), self.foot_point(i))
    }
    pub fn lengths(&self, pitch: f64, roll: f64) -> [f64; 2] {
        [0, 1].map(|i| dot3(sub3(self.lower(i, pitch, roll), self.upper(i)), sub3(self.lower(i, pitch, roll), self.upper(i))).sqrt())
    }
    /// J[i][k] = ∂s_i/∂(pitch, roll)_k (m/rad), analytic from the vector loop.
    pub fn jacobian(&self, pitch: f64, roll: f64) -> [[f64; 2]; 2] {
        let ((ryv, ryd), (rxv, rxd)) = (ry(pitch), rx(roll));
        let (dp, dr) = (mat_mul(&ryd, &rxv), mat_mul(&ryv, &rxd));
        let r = mat_mul(&ryv, &rxv);
        [0, 1].map(|i| {
            let f = self.foot_point(i);
            let d = sub3(mat_vec(&r, f), self.upper(i));
            let s = dot3(d, d).sqrt();
            [dot3(d, mat_vec(&dp, f)) / s, dot3(d, mat_vec(&dr, f)) / s]
        })
    }
    /// Joint torques (pitch, roll) from the two actuator forces: τ = Jᵀ·F.
    pub fn torques(j: &[[f64; 2]; 2], f: [f64; 2]) -> [f64; 2] {
        [j[0][0] * f[0] + j[1][0] * f[1], j[0][1] * f[0] + j[1][1] * f[1]]
    }
    /// Largest pure pitch torque and pure roll torque reachable with |F_i| ≤ f_max (N·m), or None
    /// at a singular pose.
    pub fn capacity(j: &[[f64; 2]; 2], f_max: f64) -> Option<[f64; 2]> {
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if det.abs() < 1e-12 {
            return None;
        }
        // F = J⁻ᵀ·τ; for τ = (1, 0): F ∝ (J11, −J01); for τ = (0, 1): F ∝ (−J10, J00).
        let p = f_max * det.abs() / j[1][1].abs().max(j[0][1].abs()).max(1e-15);
        let r = f_max * det.abs() / j[1][0].abs().max(j[0][0].abs()).max(1e-15);
        Some([p, r])
    }
    /// Smallest acute angle between a rod and its foot lever (ankle axis → lower pivot), deg.
    pub fn transmission_deg(&self, pitch: f64, roll: f64) -> f64 {
        (0..2)
            .map(|i| {
                let lo = self.lower(i, pitch, roll);
                let rod = sub3(lo, self.upper(i));
                // Lever projected on the plane normal to the pitch axis (y), the one that carries pitch torque.
                line_angle_deg([rod[0], rod[2]], [lo[0], lo[2]])
            })
            .fold(90.0, f64::min)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mech {
    Knee(SliderCrank),
    FourBar(FourBar),
    Ankle(Ankle),
}

/// Why a sample is flagged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flag {
    None,
    /// Transmission angle below MU_WARN / MU_FAIL.
    LowAngle,
    /// The four-bar loop cannot close at this q.
    Unassembled,
    /// The four-bar passed a toggle (assembly-mode flip) since the previous sample.
    Flip,
    /// Moment arm (nearly) zero: dead point.
    DeadPoint,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub q: f64,
    pub valid: bool,
    /// Actuator length (m); for the ankle both rods are equal along pure pitch.
    pub s: f64,
    pub ds_dq: f64,
    /// Joint torque per newton of each actuator (m): τ_av = F·h_torque.
    pub h_torque: f64,
    /// Actuator speed per joint speed (m/rad): ω_max = v_max / h_speed.
    pub h_speed: f64,
    pub mu_deg: f64,
    pub status: Status,
    pub flag: Flag,
}

impl Sample {
    fn invalid(q: f64, flag: Flag) -> Sample {
        Sample { q, valid: false, s: 0.0, ds_dq: 0.0, h_torque: 0.0, h_speed: 0.0, mu_deg: 0.0, status: Status::Fail, flag }
    }
}

fn grade_mu(mu: f64) -> (Status, Flag) {
    if mu < MU_FAIL {
        (Status::Fail, Flag::LowAngle)
    } else if mu < MU_WARN {
        (Status::Warn, Flag::LowAngle)
    } else {
        (Status::Ok, Flag::None)
    }
}

/// Moment arm below which the pose counts as a dead point (m).
pub const H_DEAD: f64 = 1e-4;

impl Mech {
    /// Actuator length at q (m); NaN-free: None when the pose does not exist.
    pub fn length(&self, q: f64) -> Option<f64> {
        match self {
            Mech::Knee(k) => Some(k.length(q)),
            Mech::FourBar(f) => f.pose(q).map(|p| p.s),
            Mech::Ankle(a) => Some(a.lengths(q, 0.0)[0]),
        }
    }
    /// Kinematics at joint angle q (ankle: pitch with zero roll).
    pub fn sample(&self, q: f64) -> Sample {
        let (s, j, ht, hs, mu) = match self {
            Mech::Knee(k) => {
                let j = k.ds_dq(q);
                (k.length(q), j, j.abs(), j.abs(), k.transmission_deg(q))
            }
            Mech::FourBar(f) => match f.pose(q) {
                Some(p) => (p.s, p.ds_dq, p.ds_dq.abs(), p.ds_dq.abs(), p.mu_deg),
                None => return Sample::invalid(q, Flag::Unassembled),
            },
            Mech::Ankle(a) => {
                let jm = a.jacobian(q, 0.0);
                let (ht, hs) = match Ankle::capacity(&jm, 1.0) {
                    Some(c) => (c[0], jm[0][0].abs().max(jm[1][0].abs())),
                    None => (0.0, 0.0),
                };
                (a.lengths(q, 0.0)[0], jm[0][0], ht, hs, a.transmission_deg(q, 0.0))
            }
        };
        let (mut status, mut flag) = grade_mu(mu);
        if ht < H_DEAD {
            (status, flag) = (Status::Fail, Flag::DeadPoint);
        }
        Sample { q, valid: true, s, ds_dq: j, h_torque: ht, h_speed: hs, mu_deg: mu, status, flag }
    }
}

/// Central-difference ds/dq, used by the tests to check the analytic Jacobians.
pub fn ds_dq_numeric(m: &Mech, q: f64, dq: f64) -> Option<f64> {
    Some((m.length(q + dq)? - m.length(q - dq)?) / (2.0 * dq))
}

/// Required joint torque: τ_req(q) = τ_c + m_seg·g·L_com·cos(q + offset).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Load {
    pub tau_c: f64,
    pub m_seg: f64,
    pub l_com: f64,
    pub offset: f64,
    /// Target joint speed (rad/s).
    pub omega: f64,
}

impl Load {
    pub fn tau_req(&self, q: f64) -> f64 {
        self.tau_c + self.m_seg * G * self.l_com * (q + self.offset).cos()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Setup {
    pub mech: Mech,
    pub q_min: f64,
    pub q_max: f64,
    pub load: Load,
}

/// Number of samples over the joint range.
pub const N_SAMPLES: usize = 181;

/// Kinematics over the joint range; flags assembly-mode flips of the four-bar.
pub fn sweep(st: &Setup) -> Vec<Sample> {
    let mut out: Vec<Sample> = Vec::with_capacity(N_SAMPLES);
    let mut side = 0.0;
    for i in 0..N_SAMPLES {
        let q = st.q_min + (st.q_max - st.q_min) * i as f64 / (N_SAMPLES - 1) as f64;
        let mut s = st.mech.sample(q);
        if let Mech::FourBar(f) = &st.mech {
            if let Some(p) = f.pose(q) {
                if side != 0.0 && p.side != side {
                    s.status = Status::Fail;
                    s.flag = Flag::Flip;
                }
                side = p.side;
            }
        }
        out.push(s);
    }
    out
}

/// Screw actuator configuration for a mechanism (geometry, support and duty come from the screw panel).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Actuator {
    pub kind: Kind,
    pub d_mm: f64,
    pub lead_mm: f64,
    pub ca_kn: f64,
    pub motor_torque: f64,
    pub motor_rpm: f64,
    pub ratio: f64,
}

impl Actuator {
    pub fn new(kind: Kind, d_mm: f64, lead_mm: f64, motor_torque: f64, motor_rpm: f64, ratio: f64) -> Self {
        Actuator { kind, d_mm, lead_mm, ca_kn: kind.preset().default_ca_kn(d_mm), motor_torque, motor_rpm, ratio }
    }
    pub fn from_inputs(i: &Inputs) -> Self {
        Actuator { kind: i.kind, d_mm: i.d_mm, lead_mm: i.lead_mm, ca_kn: i.ca_kn, motor_torque: i.motor_torque, motor_rpm: i.motor_rpm, ratio: i.ratio }
    }
    /// Screw panel inputs with this actuator's screw and motor.
    pub fn apply(&self, base: &Inputs) -> Inputs {
        Inputs {
            kind: self.kind,
            d_mm: self.d_mm,
            lead_mm: self.lead_mm,
            ca_kn: self.ca_kn,
            motor_torque: self.motor_torque,
            motor_rpm: self.motor_rpm,
            ratio: self.ratio,
            ..*base
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CurvePt {
    pub q: f64,
    pub tau_req: f64,
    pub tau_av: f64,
    /// Joint speed limit (rad/s) from min(motor, 0.8·critical, DN) linear speed.
    pub omega_max: f64,
    pub f_req: f64,
    /// Torque margin τ_av / |τ_req| (∞ when nothing is required).
    pub margin: f64,
}

#[derive(Clone, Debug)]
pub struct Coupled {
    pub pts: Vec<CurvePt>,
    pub stroke_mm: f64,
    /// Screw length used for buckling / whirl: stroke + nut allowance (mm).
    pub screw_len_mm: f64,
    pub f_avail: f64,
    /// Linear speed limit (m/s).
    pub v_lim: f64,
    pub min_margin: f64,
    pub q_at_min: f64,
    pub f_peak: f64,
    pub fm: f64,
    pub buckling_sf: f64,
    pub l10_years: f64,
    pub back_drivable: bool,
    /// Time for 0 → 90° (clipped to the range) at the speed limit, ignoring acceleration;
    /// None if the actuator cannot hold the load somewhere on the way.
    pub squat_time: Option<f64>,
    pub torque: Status,
    pub speed: Status,
    pub speed_margin: f64,
    pub buckling: Status,
    pub dn: Status,
    pub whirl: Status,
    pub life: Status,
    pub mech: Status,
    pub overall: Status,
}

fn grade(m: f64, warn: f64) -> Status {
    if m.is_nan() || m < 1.0 {
        Status::Fail
    } else if m < warn {
        Status::Warn
    } else {
        Status::Ok
    }
}

fn ratio_or_inf(a: f64, b: f64) -> f64 {
    if b > 0.0 { a / b } else { f64::INFINITY }
}

/// Available joint torque / speed of `act` on the mechanism, and the screw checks under the load.
/// `base` supplies end support, duty cycle (cycles per day, years) and guide friction.
pub fn couple(st: &Setup, samples: &[Sample], act: &Actuator, base: &Inputs) -> Coupled {
    let valid: Vec<&Sample> = samples.iter().filter(|s| s.valid).collect();
    let (s_lo, s_hi) = valid.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), s| (a.min(s.s), b.max(s.s)));
    let stroke = if valid.is_empty() { 0.0 } else { s_hi - s_lo };
    let stroke_mm = stroke * 1000.0;
    let screw_len_mm = stroke_mm + 3.0 * act.d_mm + 20.0;
    let mut inp = act.apply(base);
    inp.length_mm = screw_len_mm;
    let r = compute(&inp);
    let v_lim = r.v_motor_max.min(r.v_crit).min(r.v_dn).max(0.0) / 1000.0;
    let v_motor = r.v_motor_max / 1000.0;
    let lead = act.lead_mm / 1000.0;

    let pts: Vec<CurvePt> = samples
        .iter()
        .map(|s| {
            let tau_req = st.load.tau_req(s.q);
            let tau_av = r.f_avail * s.h_torque;
            let omega_max = if s.h_speed > 0.0 { v_lim / s.h_speed } else { f64::INFINITY };
            let f_req = if s.h_torque > 0.0 { tau_req.abs() / s.h_torque } else { f64::INFINITY };
            let margin = if tau_req.abs() > 0.0 { tau_av / tau_req.abs() } else { f64::INFINITY };
            CurvePt { q: s.q, tau_req, tau_av, omega_max, f_req, margin }
        })
        .collect();
    let (min_margin, q_at_min) = pts.iter().fold((f64::INFINITY, st.q_min), |acc, p| if p.margin < acc.0 { (p.margin, p.q) } else { acc });
    let f_peak = pts.iter().map(|p| p.f_req).fold(0.0, f64::max);
    // Cubic-mean screw force over the stroke (weights = actuator travel between samples).
    let spectrum: Vec<(f64, f64)> = samples
        .windows(2)
        .zip(pts.windows(2))
        .filter(|(s, _)| s[0].valid && s[1].valid)
        .map(|(s, p)| (0.5 * (p[0].f_req + p[1].f_req), (s[1].s - s[0].s).abs()))
        .collect();
    let fm = cubic_mean(&spectrum);
    let buckling_sf = ratio_or_inf(buckling_load(r.dr_mm / 1000.0, screw_len_mm / 1000.0, base.support), f_peak);
    let revs_per_day = 2.0 * stroke_mm / act.lead_mm * base.cycles_per_day;
    let l10_years = if revs_per_day > 0.0 { l10_rev(act.ca_kn * 1000.0, fm) / revs_per_day / 365.0 } else { f64::INFINITY };

    // Speed: needed linear speed at the target joint speed, against the motor, whirl and DN limits.
    let v_need = valid.iter().map(|s| st.load.omega * s.h_speed).fold(0.0, f64::max);
    let speed_margin = ratio_or_inf(v_motor, v_need);
    let n_need = v_need / lead * 60.0;
    let whirl_m = ratio_or_inf(0.8 * r.n_crit, n_need);
    let dn_m = ratio_or_inf(act.kind.preset().dn_max, act.d_mm * n_need);

    let squat_time = {
        let (a, b) = (st.q_min.max(0.0), st.q_max.min(PI / 2.0));
        let n = 90;
        let mut t = 0.0;
        let mut ok = b > a && v_lim > 0.0;
        let mut prev: Option<f64> = None;
        for i in 0..=n {
            if !ok {
                break;
            }
            let q = a + (b - a) * i as f64 / n as f64;
            let s = st.mech.sample(q);
            let need = st.load.tau_req(q).abs();
            if !s.valid || r.f_avail * s.h_torque < need {
                ok = false;
                break;
            }
            if let Some(p) = prev {
                t += (s.s - p).abs() / v_lim;
            }
            prev = Some(s.s);
        }
        ok.then_some(t)
    };

    let mech = samples.iter().map(|s| s.status).max().unwrap_or(Status::Fail);
    let torque = grade(min_margin, 1.25);
    let speed = grade(speed_margin, 1.1);
    let buckling = grade(buckling_sf, 2.0);
    let whirl = grade(whirl_m, 1.25);
    let dn = grade(dn_m, 1.25);
    let life = grade(ratio_or_inf(l10_years, base.years), 1.5);
    let overall = [torque, speed, buckling, whirl, dn, life, mech].into_iter().max().unwrap_or(Status::Ok);
    Coupled {
        pts,
        stroke_mm,
        screw_len_mm,
        f_avail: r.f_avail,
        v_lim,
        min_margin,
        q_at_min,
        f_peak,
        fm,
        buckling_sf,
        l10_years,
        back_drivable: !r.self_locking,
        squat_time,
        torque,
        speed,
        speed_margin,
        buckling,
        dn,
        whirl,
        life,
        mech,
        overall,
    }
}

/// One cell of the ankle pitch–roll workspace.
#[derive(Clone, Copy, Debug)]
pub struct Cell {
    pub pitch: f64,
    pub roll: f64,
    /// min(pure-pitch, pure-roll) torque capacity with |F_i| ≤ f_max (N·m); 0 when singular.
    pub tau_min: f64,
    pub singular: bool,
}

/// Grid over pitch × roll (rad); singular where |det J| < 10 % of its neutral-pose value.
pub fn ankle_workspace(a: &Ankle, pitch: (f64, f64), roll: (f64, f64), n: (usize, usize), f_max: f64) -> Vec<Cell> {
    let det = |j: [[f64; 2]; 2]| j[0][0] * j[1][1] - j[0][1] * j[1][0];
    let d0 = det(a.jacobian(0.0, 0.0)).abs();
    let mut out = Vec::with_capacity(n.0 * n.1);
    for ir in 0..n.1 {
        for ip in 0..n.0 {
            let p = pitch.0 + (pitch.1 - pitch.0) * (ip as f64 + 0.5) / n.0 as f64;
            let r = roll.0 + (roll.1 - roll.0) * (ir as f64 + 0.5) / n.1 as f64;
            let j = a.jacobian(p, r);
            let singular = det(j).abs() < 0.1 * d0;
            let tau_min = Ankle::capacity(&j, f_max).map_or(0.0, |c| c[0].min(c[1]));
            out.push(Cell { pitch: p, roll: r, tau_min: if singular { 0.0 } else { tau_min }, singular });
        }
    }
    out
}

/// Number of built-in presets (see `preset`).
pub const PRESETS: usize = 4;

/// Built-in presets: approximate, for comparison only — own plausible estimates for a ~1.7 m /
/// ~60 kg humanoid, not data of any real robot.
/// 0: "Optimus-style" knee (slider-crank, roller screw, ~8 kN class),
/// 1: "Optimus-style" dual-rod ankle (roller screws, ~3.9 kN class),
/// 2: generic dual-rod ankle (ball screws), 3: generic four-bar knee (ball screw).
pub fn preset(i: usize) -> (Setup, Actuator) {
    let deg = |d: f64| d.to_radians();
    let knee_load = Load { tau_c: 15.0, m_seg: 30.0, l_com: 0.25, offset: deg(-90.0), omega: deg(150.0) };
    let ankle_load = Load { tau_c: 50.0, m_seg: 60.0, l_com: 0.12, offset: 0.0, omega: deg(180.0) };
    match i {
        1 => (
            Setup { mech: Mech::Ankle(Ankle { w: 0.07, h: 0.30, c: 0.05, e: 0.045 }), q_min: deg(-30.0), q_max: deg(35.0), load: ankle_load },
            Actuator::new(Kind::Roller, 10.0, 2.0, 1.5, 6000.0, 1.0),
        ),
        2 => (
            Setup { mech: Mech::Ankle(Ankle { w: 0.09, h: 0.26, c: 0.06, e: 0.06 }), q_min: deg(-30.0), q_max: deg(35.0), load: Load { tau_c: 30.0, ..ankle_load } },
            Actuator::new(Kind::Ball, 12.0, 4.0, 0.9, 4500.0, 1.0),
        ),
        3 => (
            Setup {
                mech: Mech::FourBar(FourBar { bx: -0.03, by: 0.30, cx: -0.04, cy: 0.06, rc: 0.06, lc: 0.10, rs: 0.035, q0: deg(270.0), branch: -1.0 }),
                q_min: 0.0,
                q_max: deg(120.0),
                load: knee_load,
            },
            Actuator::new(Kind::Ball, 16.0, 4.0, 2.5, 4000.0, 1.0),
        ),
        _ => (
            Setup { mech: Mech::Knee(SliderCrank { ax: -0.03, ay: 0.28, r: 0.05, q0: deg(235.0) }), q_min: 0.0, q_max: deg(120.0), load: knee_load },
            Actuator::new(Kind::Roller, 12.0, 2.0, 3.0, 5000.0, 1.0),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn knee() -> SliderCrank {
        match preset(0).0.mech {
            Mech::Knee(k) => k,
            _ => unreachable!(),
        }
    }
    fn four_bar() -> FourBar {
        match preset(3).0.mech {
            Mech::FourBar(f) => f,
            _ => unreachable!(),
        }
    }
    fn ankle() -> Ankle {
        match preset(1).0.mech {
            Mech::Ankle(a) => a,
            _ => unreachable!(),
        }
    }

    #[test]
    fn slider_crank_jacobian_matches_central_difference() {
        let m = Mech::Knee(knee());
        for i in 0..=120 {
            let q = (i as f64).to_radians();
            let a = knee().ds_dq(q);
            let n = ds_dq_numeric(&m, q, 1e-6).unwrap();
            assert!((a - n).abs() < 1e-7, "q {i}°: {a} vs {n}");
        }
    }

    #[test]
    fn slider_crank_arm_equals_r_when_perpendicular() {
        let k = knee();
        // Rod ⟂ lever: (P − B)·P = 0, i.e. r² = B·P. Bisection on f(q) = (P−B)·P over the range.
        let f = |q: f64| {
            let p = k.rod_end(q);
            dot2(sub2(p, k.base()), p)
        };
        let (mut lo, mut hi) = (0.0f64, 120f64.to_radians());
        // The preset is chosen so the perpendicular pose lies inside the range.
        let (mut flo, fhi) = (f(lo), f(hi));
        assert!(flo * fhi < 0.0, "perpendicular pose in range");
        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            let fm = f(mid);
            if fm * flo > 0.0 {
                lo = mid;
                flo = fm;
            } else {
                hi = mid;
            }
        }
        let q = 0.5 * (lo + hi);
        assert!((k.ds_dq(q).abs() - k.r).abs() < 1e-9);
        assert!((k.transmission_deg(q) - 90.0).abs() < 1e-5);
        // Elsewhere the arm is shorter than r.
        assert!(k.ds_dq(0.0).abs() < k.r && k.ds_dq(2.0).abs() < k.r);
    }

    #[test]
    fn four_bar_loop_closure_and_jacobian() {
        let f = four_bar();
        let m = Mech::FourBar(f);
        for i in 0..=240 {
            let q = (i as f64 * 0.5).to_radians();
            let p = f.pose(q).expect("preset assembles over its range");
            let (kc, kp) = (len2(sub2(p.k, f.pivot())), len2(sub2(p.k, p.p)));
            assert!((kc - f.rc).abs() < 1e-9 && (kp - f.lc).abs() < 1e-9, "closure at {q}");
            assert!((len2(p.p) - f.rs).abs() < 1e-12);
            let n = ds_dq_numeric(&m, q, 1e-6).unwrap();
            assert!((p.ds_dq - n).abs() < 1e-7, "J at {q}: {} vs {n}", p.ds_dq);
        }
        let st = preset(3).0;
        assert!(sweep(&st).iter().all(|s| s.valid && s.status != Status::Fail));
    }

    #[test]
    fn four_bar_transmission_angle_flags() {
        // Shorter coupler over a wider range: the coupler swings close to the shank lever line.
        let mut st = preset(3).0;
        if let Mech::FourBar(f) = &mut st.mech {
            f.lc = 0.08;
        }
        st.q_max = 180f64.to_radians();
        let ss = sweep(&st);
        let mut seen = (false, false);
        for s in &ss {
            if !s.valid {
                assert_eq!(s.status, Status::Fail);
                continue;
            }
            if s.flag == Flag::Flip || s.flag == Flag::DeadPoint {
                continue;
            }
            let want = if s.mu_deg < MU_FAIL { Status::Fail } else if s.mu_deg < MU_WARN { Status::Warn } else { Status::Ok };
            assert_eq!(s.status, want, "μ {} at q {}", s.mu_deg, s.q.to_degrees());
            seen.0 |= want == Status::Warn;
            seen.1 |= want == Status::Fail;
        }
        assert!(seen.0 && seen.1, "extended range reaches both the warning and the fail band");
        // Pulling the coupler too short makes the loop impossible: flagged, never NaN.
        let mut f = four_bar();
        f.lc = 0.01;
        let s = Mech::FourBar(f).sample(0.5);
        assert!(!s.valid && s.flag == Flag::Unassembled && s.status == Status::Fail && s.h_torque == 0.0);
    }

    #[test]
    fn ankle_pure_pitch_symmetric() {
        let a = ankle();
        for i in -30..=35 {
            let s = a.lengths((i as f64).to_radians(), 0.0);
            assert!((s[0] - s[1]).abs() < 1e-12);
        }
        let s = a.lengths(0.0, 0.2);
        assert!((s[0] - s[1]).abs() > 1e-3, "roll makes them differ");
    }

    #[test]
    fn ankle_jacobian_matches_central_difference() {
        let a = ankle();
        let h = 1e-6;
        for &(p, r) in &[(0.0, 0.0), (0.3, -0.1), (-0.4, 0.25), (0.5, 0.3)] {
            let j = a.jacobian(p, r);
            let (sp, sm) = (a.lengths(p + h, r), a.lengths(p - h, r));
            let (rp, rm) = (a.lengths(p, r + h), a.lengths(p, r - h));
            for i in 0..2 {
                assert!((j[i][0] - (sp[i] - sm[i]) / (2.0 * h)).abs() < 1e-7);
                assert!((j[i][1] - (rp[i] - rm[i]) / (2.0 * h)).abs() < 1e-7);
            }
        }
    }

    #[test]
    fn ankle_torque_is_power_balanced() {
        // τ = Jᵀ·F ⇒ τ·q̇ = F·ṡ for any joint velocity (virtual work); ṡ by finite difference.
        let a = ankle();
        let (p, r) = (0.2, -0.15);
        let f = [1200.0, -800.0];
        let j = a.jacobian(p, r);
        let tau = Ankle::torques(&j, f);
        for qd in [[1.0, 0.0], [0.0, 1.0], [0.7, -1.3]] {
            let dt = 1e-7;
            let (s1, s0) = (a.lengths(p + qd[0] * dt, r + qd[1] * dt), a.lengths(p - qd[0] * dt, r - qd[1] * dt));
            let power_act = f[0] * (s1[0] - s0[0]) / (2.0 * dt) + f[1] * (s1[1] - s0[1]) / (2.0 * dt);
            let power_joint = tau[0] * qd[0] + tau[1] * qd[1];
            assert!((power_act - power_joint).abs() < 1e-5 * power_joint.abs().max(1.0), "{power_act} vs {power_joint}");
        }
        // Capacity: pure-pitch torque with equal forces at neutral equals 2·F·|J|.
        let j0 = a.jacobian(0.0, 0.0);
        let c = Ankle::capacity(&j0, 1000.0).unwrap();
        assert!((c[0] - 2000.0 * j0[0][0].abs()).abs() < 1e-9);
    }

    #[test]
    fn ankle_workspace_flags_singular_cells() {
        let ok = ankle_workspace(&ankle(), (-0.5, 0.6), (-0.3, 0.3), (8, 6), 1000.0);
        assert_eq!(ok.len(), 48);
        assert!(ok.iter().all(|c| !c.singular && c.tau_min > 0.0));
        // Large pitch swings the foot lever onto the rod line: ∂s/∂pitch → 0, det J → 0.
        let s = ankle_workspace(&ankle(), (-1.5, 1.5), (-0.3, 0.3), (31, 5), 1000.0);
        assert!(s.iter().any(|c| c.singular && c.tau_min == 0.0));
        assert!(s.iter().any(|c| !c.singular));
    }

    #[test]
    fn coupling_tau_equals_force_times_arm() {
        let (mut st, act) = preset(0);
        st.load.omega = 0.01; // slow enough that speed never limits
        let ss = sweep(&st);
        let base = Inputs::default();
        let c = couple(&st, &ss, &act, &base);
        let f = compute(&Inputs { length_mm: c.screw_len_mm, ..act.apply(&base) }).f_avail;
        assert!((c.f_avail - f).abs() < 1e-9);
        for (s, p) in ss.iter().zip(&c.pts) {
            assert!((p.tau_av - f * s.h_torque).abs() < 1e-9);
            assert!((p.omega_max * s.h_speed - c.v_lim).abs() < 1e-9);
            assert!(p.tau_av.is_finite() && p.omega_max.is_finite() && p.f_req.is_finite());
        }
        assert_eq!(ss.len(), N_SAMPLES);
        assert!(N_SAMPLES >= 121);
    }

    #[test]
    fn presets_are_sane_designs() {
        let base = Inputs::default();
        for i in 0..PRESETS {
            let (st, act) = preset(i);
            let c = couple(&st, &sweep(&st), &act, &base);
            assert_ne!(c.overall, Status::Fail, "preset {i}: {c:?}");
            assert!(c.min_margin >= 1.25 && c.squat_time.is_some(), "preset {i}");
        }
    }
}
