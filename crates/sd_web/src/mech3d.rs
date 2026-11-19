//! 3D view of a joint mechanism: thigh / shank / foot boxes, links and pins, and the screw actuator
//! (motor, spinning screw, sliding nut, push rod) between its pivots. Drag the shank or foot to move
//! the joint.
use crate::gl::Paint;
use crate::math::{self, Orbit, OrbitCam, M4, V};
use crate::view3d::{along, Layout, Scene};
use sd_core::mech::{Ankle, Mech, Setup};
use std::f32::consts::TAU;

/// Knee axis height above the ground (m), so the foot touches the ground with a straight leg.
const KNEE_Y: f32 = 0.47;
/// Ankle axis height (m).
const ANKLE_Y: f32 = 0.11;
/// Lateral offset of the actuator plane from the bone plane (m).
const ACT_Z: f32 = 0.045;

/// What a drag in the mechanism view asks the app to change.
pub enum MAction {
    /// Knee angle (rad).
    Q(f64),
    /// Ankle pitch / roll change (rad).
    PitchRoll(f64, f64),
}

#[derive(Clone, Copy, PartialEq)]
enum Pick {
    None,
    Limb,
}

pub struct MechView {
    pub cam: OrbitCam,
    hover: Pick,
    drag: Pick,
    pointers: Vec<(i32, f32, f32)>,
    pinch: f32,
    inv_vp: M4,
    size: (f32, f32),
    /// Pick spheres (centre, radius) of the draggable limb.
    targets: Vec<(V, f32)>,
    ankle: bool,
}

/// Knee-plane point (x forward, y up, m) to world, at lateral offset z.
fn kw(p: [f64; 2], z: f32) -> V {
    [p[0] as f32, KNEE_Y + p[1] as f32, z]
}
/// Ankle-frame point (x forward, y left, z up) to world (x, up, −left).
fn aw(p: [f64; 3]) -> V {
    [p[0] as f32, ANKLE_Y + p[2] as f32, -p[1] as f32]
}

/// Actuator drawing parameters (all m).
pub struct ActDraw {
    pub d: f32,
    pub lead: f32,
    pub dr: f32,
    /// Actuator length at the shortest pose and the stroke.
    pub s_min: f32,
    pub stroke: f32,
    /// Housing tint: red when the required force exceeds the available force.
    pub over: bool,
}

impl Default for MechView {
    fn default() -> Self {
        MechView {
            cam: OrbitCam::new(Orbit { target: [0.0, 0.35, 0.0], yaw: 1.1, pitch: 0.2, dist: 1.3 }),
            hover: Pick::None,
            drag: Pick::None,
            pointers: vec![],
            pinch: 0.0,
            inv_vp: math::IDENTITY,
            size: (1.0, 1.0),
            targets: vec![],
            ankle: false,
        }
    }
}

impl MechView {
    pub fn reset(&mut self, st: &Setup) {
        let o = match st.mech {
            Mech::Ankle(_) => Orbit { target: [0.0, 0.22, 0.0], yaw: 0.9, pitch: 0.25, dist: 0.95 },
            _ => Orbit { target: [0.0, KNEE_Y - 0.02, 0.0], yaw: 1.15, pitch: 0.18, dist: 1.35 },
        };
        self.cam.set_goal(o);
    }

    /// Draw one frame at joint angle `q` (ankle: pitch) and `roll` (ankle only).
    #[allow(clippy::too_many_arguments)]
    pub fn frame(&mut self, sc: &mut Scene, w: i32, h: i32, dt: f32, st: &Setup, q: f64, roll: f64, act: &ActDraw) {
        self.cam.step(dt);
        let eye = self.cam.cur.eye();
        let view = math::look_at(eye, self.cam.cur.target, [0.0, 1.0, 0.0]);
        let proj = math::perspective(0.8, w as f32 / h.max(1) as f32, 0.01, 60.0);
        self.inv_vp = math::invert(&math::mul(&proj, &view)).unwrap_or(math::IDENTITY);
        self.size = (w as f32, h as f32);
        sc.r.begin(&view, &proj, eye, [0, 0, w, h]);

        let rgba = |c: [f32; 3]| [c[0], c[1], c[2], 1.0];
        let lit = |c: [f32; 3]| Paint::lit(rgba(c));
        let glow = if self.hover == Pick::Limb || self.drag == Pick::Limb { 0.25 } else { 0.0 };
        let bone = [0.80, 0.82, 0.86];
        let limb = {
            let mut p = lit([0.30, 0.52, 0.86]);
            p.glow = glow;
            p
        };
        let link = [0.55, 0.58, 0.64];
        let pin = [0.25, 0.26, 0.30];
        let mut boxes: Vec<(M4, Paint)> = vec![];
        let mut cyls: Vec<(M4, Paint)> = vec![];
        // Box spanning world points a → b with thickness t (in-plane) and depth dz (along `side`).
        let bar = |a: V, b: V, t: f32, dz: f32, side: V| -> M4 {
            let dv = math::sub(b, a);
            let l = math::dot(dv, dv).sqrt().max(1e-6);
            let x = math::scale(dv, 1.0 / l);
            let y = math::norm(math::cross(side, x));
            math::basis(x, y, math::cross(x, y), [l, t, dz], math::scale(math::add(a, b), 0.5))
        };
        let zax = [0.0, 0.0, 1.0];
        let pin_at = |c: V, r: f32, len: f32, axis: V| along(math::sub(c, math::scale(axis, 0.5 * len)), axis, len, r);
        let mut acts: Vec<(V, V)> = vec![];
        self.ankle = matches!(st.mech, Mech::Ankle(_));
        self.targets.clear();

        match st.mech {
            Mech::Knee(_) | Mech::FourBar(_) => {
                let qf = q as f32;
                let knee = [0.0, KNEE_Y, 0.0];
                let sh = [-qf.sin(), -qf.cos(), 0.0];
                let ankle = math::add(knee, math::scale(sh, 0.40));
                let ft = [qf.cos(), -qf.sin(), 0.0];
                boxes.push((bar(knee, math::add(knee, [0.0, 0.44, 0.0]), 0.075, 0.06, zax), lit(bone)));
                boxes.push((bar(knee, ankle, 0.065, 0.055, zax), limb));
                let heel = math::add(ankle, math::scale(ft, -0.05));
                let toe = math::add(ankle, math::scale(ft, 0.16));
                boxes.push((bar(math::add(heel, math::scale(sh, 0.03)), math::add(toe, math::scale(sh, 0.03)), 0.045, 0.09, zax), limb));
                cyls.push((pin_at(knee, 0.03, 0.10, zax), lit(pin)));
                self.targets.push((math::add(knee, math::scale(sh, 0.25)), 0.07));
                self.targets.push((ankle, 0.08));
                let z = ACT_Z;
                let (base, end) = match st.mech {
                    Mech::Knee(k) => {
                        let p = kw(k.rod_end(q), z);
                        boxes.push((bar([0.0, KNEE_Y, z], p, 0.022, 0.014, zax), lit(link)));
                        (kw(k.base(), z), p)
                    }
                    Mech::FourBar(f) => {
                        let lv = kw(f.lever(q), z);
                        boxes.push((bar([0.0, KNEE_Y, z], lv, 0.022, 0.014, zax), lit(link)));
                        let c = kw(f.pivot(), z);
                        cyls.push((pin_at(c, 0.011, 0.05, zax), lit(pin)));
                        // Bracket tying the crank pivot to the thigh.
                        boxes.push((bar([c[0], c[1], 0.0], c, 0.02, 0.02, [0.0, 1.0, 0.0]), lit(link)));
                        match f.crank_tip(q) {
                            Some(k) => {
                                let k = kw(k, z);
                                boxes.push((bar(c, k, 0.02, 0.014, zax), lit([0.85, 0.62, 0.25])));
                                boxes.push((bar(k, lv, 0.016, 0.012, zax), lit(link)));
                                cyls.push((pin_at(lv, 0.009, 0.04, zax), lit(pin)));
                                (kw(f.base(), z), k)
                            }
                            None => (kw(f.base(), z), lv),
                        }
                    }
                    Mech::Ankle(_) => unreachable!(),
                };
                // Bracket from the thigh to the actuator base.
                boxes.push((bar([base[0], base[1], 0.0], base, 0.025, 0.025, [0.0, 1.0, 0.0]), lit(link)));
                acts.push((base, end));
            }
            Mech::Ankle(a) => {
                let ankle = aw([0.0; 3]);
                boxes.push((bar(ankle, aw([0.0, 0.0, a.h + 0.10]), 0.06, 0.06, [0.0, 0.0, 1.0]), lit(bone)));
                let r = Ankle::rot(q, roll);
                let col = |i: usize| aw([r[0][i], r[1][i], r[2][i]]);
                let (fx, fy, fz) = (math::sub(col(0), ankle), math::sub(col(1), ankle), math::sub(col(2), ankle));
                // Foot plate in the foot frame: x −0.08…0.17, |y| ≤ 0.055, sole 0.05…0.08 below the axis.
                let fc = aw(sd_core_rot(&r, [0.045, 0.0, -0.065]));
                boxes.push((math::basis(fx, fz, math::scale(fy, -1.0), [0.25, 0.03, 0.11], fc), limb));
                // Heel lever carrying both lower pivots.
                let l0 = aw(a.lower(0, q, roll));
                let l1 = aw(a.lower(1, q, roll));
                let lm = math::scale(math::add(l0, l1), 0.5);
                boxes.push((bar(l0, l1, 0.02, 0.02, fx), lit(link)));
                boxes.push((bar(ankle, lm, 0.02, 0.025, fy), lit(link)));
                let yax = math::norm(fy);
                cyls.push((pin_at(ankle, 0.022, 0.08, [0.0, 0.0, 1.0]), lit(pin)));
                cyls.push((pin_at(ankle, 0.016, 0.07, yax), lit(pin)));
                for i in 0..2 {
                    let up = aw(a.upper(i));
                    // Upper bracket from the shank.
                    boxes.push((bar([0.0, up[1], up[2]], up, 0.02, 0.02, [0.0, 1.0, 0.0]), lit(link)));
                    acts.push((up, if i == 0 { l0 } else { l1 }));
                }
                self.targets.push((fc, 0.10));
            }
        }

        // Actuators: motor can at the base, translucent housing over a spinning screw, nut, push rod.
        let lead_vis = act.lead.max(0.003);
        let nut_len = 2.0 * act.d + 0.01;
        let screw_len = (act.stroke + nut_len).max(0.02);
        let ml = (0.3 * act.s_min).clamp(0.015, 0.06);
        let mr = 0.5 * act.d + 0.012;
        sc.ensure_thread(act.d, lead_vis, screw_len, act.dr);
        let body = if act.over { [0.90, 0.25, 0.22] } else { [0.17, 0.18, 0.21] };
        let mut thread_parts: Vec<M4> = vec![];
        let mut translucent: Vec<(M4, Paint)> = vec![];
        for (b, e) in &acts {
            let dv = math::sub(*e, *b);
            let s = math::dot(dv, dv).sqrt().max(1e-4);
            let u = math::scale(dv, 1.0 / s);
            let helper = if u[1].abs() < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] };
            let ez = math::norm(math::cross(u, helper));
            let ey = math::cross(ez, u);
            let lay = Layout { o: *b, ax: u, ey, ez, ..Layout::default() };
            let travel = (s - act.s_min).clamp(0.0, act.stroke.max(0.0));
            let spin = TAU * travel / lead_vis;
            let x0 = ml;
            let xn = x0 + 0.5 * nut_len + travel;
            cyls.push((lay.rod(0.0, ml, 0.0, 0.0, mr, 0.0), lit(body)));
            cyls.push((lay.rod(x0, x0 + screw_len, 0.0, 0.0, 0.5 * act.dr, spin), lit([0.72, 0.74, 0.78])));
            let screw_lay = Layout { o: lay.at([x0, 0.0, 0.0]), ..lay };
            thread_parts.push(screw_lay.spun(spin));
            cyls.push((lay.rod(xn - 0.5 * nut_len, xn + 0.5 * nut_len, 0.0, 0.0, 0.5 * act.d + 0.006, 0.0), lit(if act.over { body } else { [0.30, 0.32, 0.36] })));
            cyls.push((lay.rod(xn, s, 0.0, 0.0, 0.25 * act.d + 0.003, 0.0), lit([0.62, 0.64, 0.68])));
            let glass = Paint::lit(if act.over { [0.95, 0.3, 0.25, 0.30] } else { [0.6, 0.7, 0.85, 0.22] });
            translucent.push((lay.rod(ml, ml + screw_len, 0.0, 0.0, 0.5 * act.d + 0.009, 0.0), glass));
            for p in [*b, *e] {
                cyls.push((pin_at(p, 0.008, 0.035, ez), lit([0.2, 0.2, 0.22])));
            }
        }

        for (m, _) in &boxes {
            sc.r.shadow(&sc.cube, m);
        }
        for (m, _) in &cyls {
            sc.r.shadow(&sc.cyl, m);
        }
        for (m, p) in &boxes {
            sc.r.draw(&sc.cube, m, *p);
        }
        for (m, p) in &cyls {
            sc.r.draw(&sc.cyl, m, *p);
        }
        if let Some(t) = sc.thread_mesh() {
            for m in &thread_parts {
                sc.r.draw(t, m, lit([0.72, 0.74, 0.78]));
            }
        }
        for (m, p) in &translucent {
            sc.r.draw(&sc.cyl, m, *p);
        }
    }

    fn ray(&self, x: f32, y: f32) -> (V, V) {
        math::ndc_ray(&self.inv_vp, 2.0 * x / self.size.0 - 1.0, 1.0 - 2.0 * y / self.size.1)
    }

    fn pick(&self, x: f32, y: f32) -> Pick {
        let (o, d) = self.ray(x, y);
        if self.targets.iter().any(|(c, r)| math::ray_sphere(o, d, *c, *r).is_some()) {
            Pick::Limb
        } else {
            Pick::None
        }
    }

    pub fn down(&mut self, id: i32, x: f32, y: f32) {
        self.pointers.retain(|p| p.0 != id);
        self.pointers.push((id, x, y));
        if self.pointers.len() == 1 {
            self.drag = self.pick(x, y);
            self.cam.held = self.drag == Pick::None;
        } else {
            self.drag = Pick::None;
            self.pinch = self.spread();
        }
    }

    fn spread(&self) -> f32 {
        match self.pointers.as_slice() {
            [a, b, ..] => ((a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt(),
            _ => 0.0,
        }
    }

    fn pan(&mut self, dx: f32, dy: f32) {
        let c = self.cam.cur;
        let f = math::norm(math::sub(c.target, c.eye()));
        let right = math::norm(math::cross(f, [0.0, 1.0, 0.0]));
        let up = math::cross(right, f);
        let k = c.dist * 0.0015;
        self.cam.pan(math::add(math::scale(right, -dx * k), math::scale(up, dy * k)));
    }

    /// Pointer move; returns a joint change, if any, and the cursor to show.
    pub fn moved(&mut self, id: i32, x: f32, y: f32, buttons: u16) -> (Option<MAction>, &'static str) {
        let Some(i) = self.pointers.iter().position(|p| p.0 == id) else {
            self.hover = self.pick(x, y);
            return (None, if self.hover == Pick::None { "default" } else { "grab" });
        };
        let (dx, dy) = (x - self.pointers[i].1, y - self.pointers[i].2);
        self.pointers[i] = (id, x, y);
        if self.pointers.len() >= 2 {
            let s = self.spread();
            if self.pinch > 0.0 && s > 0.0 {
                self.cam.zoom(self.pinch / s);
            }
            self.pinch = s;
            self.pan(0.5 * dx, 0.5 * dy);
            return (None, "default");
        }
        let act = match self.drag {
            Pick::Limb if self.ankle => {
                let k = 0.004 * 1000.0 / self.size.1.max(200.0);
                Some(MAction::PitchRoll((dy * k) as f64, (dx * k) as f64))
            }
            Pick::Limb => {
                // Point under the pointer on the knee plane → shank angle.
                let (o, d) = self.ray(x, y);
                math::ray_plane(o, d, [0.0, KNEE_Y, 0.0], [0.0, 0.0, 1.0]).map(|p| {
                    let v = math::sub(p, [0.0, KNEE_Y, 0.0]);
                    MAction::Q((-v[0]).atan2(-v[1]) as f64)
                })
            }
            Pick::None => {
                if buttons & 2 != 0 {
                    self.pan(dx, dy);
                } else {
                    self.cam.rotate(-dx * 0.008, dy * 0.008);
                }
                None
            }
        };
        (act, if self.drag == Pick::None { "default" } else { "grabbing" })
    }

    pub fn up(&mut self, id: i32) {
        self.pointers.retain(|p| p.0 != id);
        if self.pointers.is_empty() {
            self.drag = Pick::None;
            self.cam.held = false;
        }
    }
}

fn sd_core_rot(r: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2])
}
