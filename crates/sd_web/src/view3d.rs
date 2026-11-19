//! 3D actuator scene: helical screw, travelling nut + carriage, end supports, motor (+ belt),
//! force arrows, whirl ghost; orbit camera, presets, carriage / end-support dragging.
use crate::gl::{self, Mesh, Paint, Renderer};
use crate::math::{self, Orbit, OrbitCam, M4, V};
use sd_core::{Inputs, Kind, Results, Status, Support};
use std::f32::consts::{PI, TAU};

/// What a drag in the view asks the app to change.
pub enum Action {
    /// Forward-move position along the stroke (m).
    Scrub(f64),
    /// New length between supports (mm).
    Length(f64),
}

#[derive(Clone, Copy, PartialEq)]
enum Pick {
    None,
    Carriage,
    End,
}

/// Layout of the current frame, kept for picking.
#[derive(Clone, Copy, Default)]
struct Layout {
    o: V,
    ax: V,
    ey: V,
    ez: V,
    len: f32,
    x_start: f32,
    x_end: f32,
    carriage: V,
    end: V,
    pick_r: f32,
}

impl Layout {
    fn fw(&self, p: V) -> V {
        math::add(math::add(math::scale(self.ax, p[0]), math::scale(self.ey, p[1])), math::scale(self.ez, p[2]))
    }
    fn at(&self, p: V) -> V {
        math::add(self.o, self.fw(p))
    }
    /// Box centred at frame point `c` with frame-aligned size `s`.
    fn boxm(&self, c: V, s: V) -> M4 {
        math::basis(self.ax, self.ey, self.ez, s, self.at(c))
    }
    /// Cylinder (mesh axis y) along frame x from `x0` to `x1` at frame (y, z), radius `r`, spun by `spin`.
    fn rod(&self, x0: f32, x1: f32, y: f32, z: f32, r: f32, spin: f32) -> M4 {
        let (s, c) = spin.sin_cos();
        let px = math::add(math::scale(self.ey, c), math::scale(self.ez, s));
        let pz = math::cross(px, self.ax);
        math::basis(px, self.ax, pz, [2.0 * r, x1 - x0, 2.0 * r], self.at([0.5 * (x0 + x1), y, z]))
    }
    /// Unscaled mesh built along y from frame origin (the thread), spun by `spin`.
    fn spun(&self, spin: f32) -> M4 {
        let (s, c) = spin.sin_cos();
        let px = math::add(math::scale(self.ey, c), math::scale(self.ez, s));
        math::basis(px, self.ax, math::cross(px, self.ax), [1.0; 3], self.o)
    }
}

/// Cylinder / cone mesh from `a` along unit `dir` with length `len` and radius `r` (mesh axis y).
fn along(a: V, dir: V, len: f32, r: f32) -> M4 {
    let helper = if dir[1].abs() < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] };
    let x = math::norm(math::cross(helper, dir));
    let z = math::cross(x, dir);
    math::basis(x, dir, z, [2.0 * r, len, 2.0 * r], math::add(a, math::scale(dir, 0.5 * len)))
}

/// Helical thread ridge around a core of radius `r0` up to `r1`, along y from 0 to `len`,
/// triangular profile of half-width `w` (all in metres). Non-indexed (no u16 limit).
pub fn thread(r0: f32, r1: f32, lead: f32, len: f32, w: f32) -> gl::Geo {
    let mut g = gl::Geo::default();
    let seg = 20.0;
    let n = ((len / lead) * seg).ceil().max(1.0) as usize;
    let du = lead / seg;
    let sec = |u: f32| {
        let (s, c) = (TAU * u / lead).sin_cos();
        let p = |r: f32, y: f32| [r * c, y.clamp(0.0, len), r * s];
        [p(r0, u - w), p(r1, u), p(r0, u + w)]
    };
    let mut tri = |a: V, b: V, c: V| {
        let nrm = math::norm(math::cross(math::sub(b, a), math::sub(c, a)));
        for p in [a, b, c] {
            g.pos.extend(p);
            g.nrm.extend(nrm);
        }
    };
    for i in 0..n {
        let (a, b) = (sec(i as f32 * du), sec((i + 1) as f32 * du));
        for k in 0..2 {
            tri(a[k], b[k], b[k + 1]);
            tri(a[k], b[k + 1], a[k + 1]);
        }
    }
    g
}

/// Exaggerated first whirling mode shape (0…1) along ξ = x/L for each end support (approximate).
pub fn mode_shape(s: Support, xi: f32) -> f32 {
    match s {
        Support::SimpleSimple => (PI * xi).sin(),
        Support::FixedFixed => 0.5 * (1.0 - (TAU * xi).cos()),
        Support::FixedFree => 1.0 - (0.5 * PI * xi).cos(),
        Support::FixedSimple => 1.5 * xi.powf(0.7) * (PI * xi).sin(),
    }
}

pub struct Toggles {
    pub color: bool,
    pub arrows: bool,
    pub whirl: bool,
}

pub struct Scene {
    pub r: Renderer,
    cyl: Mesh,
    cube: Mesh,
    cone: Mesh,
    thread: Option<(Mesh, [u64; 3], f32)>,
    pub cam: OrbitCam,
    pub preset: u8,
    lay: Layout,
    hover: Pick,
    drag: Pick,
    pointers: Vec<(i32, f32, f32)>,
    pinch: f32,
    inv_vp: M4,
    size: (f32, f32),
}

fn status_rgb(s: Status, margin: f64) -> [f32; 3] {
    let (g, y, r) = ([0.30, 0.78, 0.40], [0.95, 0.78, 0.20], [0.92, 0.30, 0.25]);
    match s {
        Status::Fail => r,
        _ => {
            let t = ((margin - 1.0) as f32).clamp(0.0, 1.0);
            [0, 1, 2].map(|i| y[i] + (g[i] - y[i]) * t)
        }
    }
}

impl Scene {
    pub fn new(r: Renderer) -> Option<Scene> {
        let cone = gl::frustum([0.0, 0.5, 0.0], &[[-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5]]);
        let mut s = Scene {
            cyl: r.mesh(&gl::cylinder(28))?,
            cube: r.mesh(&gl::cube())?,
            cone: r.mesh(&cone)?,
            r,
            thread: None,
            cam: OrbitCam::new(Orbit { target: [0.0, 0.2, 0.0], yaw: 0.6, pitch: 0.35, dist: 1.0 }),
            preset: 0,
            lay: Layout::default(),
            hover: Pick::None,
            drag: Pick::None,
            pointers: vec![],
            pinch: 0.0,
            inv_vp: math::IDENTITY,
            size: (1.0, 1.0),
        };
        s.r.cell = 0.05;
        s.r.fog = [2.5, 10.0];
        Some(s)
    }

    fn preset_orbit(&self, inp: &Inputs) -> Orbit {
        let l = (inp.length_mm as f32 / 1000.0).max(0.2);
        let mid = self.lay.at([0.5 * self.lay.len, 0.0, 0.0]);
        match self.preset {
            1 => Orbit { target: mid, yaw: 0.0, pitch: 1.5, dist: 1.5 * l + 0.2 },
            2 => Orbit { target: mid, yaw: 0.0, pitch: 0.02, dist: 1.5 * l + 0.2 },
            3 => Orbit { target: self.lay.carriage, yaw: 0.35, pitch: 0.25, dist: 0.35 + 0.1 * l },
            _ => Orbit { target: mid, yaw: 0.6, pitch: 0.35, dist: 1.6 * l + 0.25 },
        }
    }

    pub fn reset_view(&mut self, inp: &Inputs) {
        let o = self.preset_orbit(inp);
        self.cam.set_goal(o);
    }

    /// Draw one frame. `pos` is the nut position along the stroke (m), `force` the axial force now.
    pub fn frame(&mut self, w: i32, h: i32, dt: f32, inp: &Inputs, res: &Results, pos: f64, force: f64, tog: &Toggles, t: f64) {
        let d = inp.d_mm as f32 / 1000.0;
        let lead = inp.lead_mm as f32 / 1000.0;
        let len = inp.length_mm as f32 / 1000.0;
        let dr = res.dr_mm as f32 / 1000.0;
        let lead_vis = lead.max(len / 120.0);
        let key = [inp.d_mm.to_bits(), inp.lead_mm.to_bits(), inp.length_mm.to_bits()];
        if self.thread.as_ref().map(|t| t.1) != Some(key) {
            if let Some((m, _, _)) = self.thread.take() {
                self.r.free(m);
            }
            let g = thread(0.5 * dr, 0.5 * d, lead_vis, len, (0.35 * lead_vis).min(0.25 * d));
            self.thread = self.r.mesh(&g).map(|m| (m, key, lead_vis));
        }

        // Frame layout: x along the axis, y "up" in the tilt plane, z sideways.
        let th = (inp.angle_deg as f32).to_radians();
        let (sn, cs) = th.sin_cos();
        let (ax, ey, ez) = ([cs, sn, 0.0], [-sn, cs, 0.0], [0.0, 0.0, 1.0]);
        let ratio = inp.ratio as f32;
        let belt = (ratio - 1.0).abs() > 0.01;
        let mr = 0.022 + 0.012 * (inp.motor_torque as f32).sqrt().min(3.0);
        let ml = 2.2 * mr + 0.03;
        let hs = 0.5 * d + 0.02;
        let base_y = -hs - 0.006;
        let nut_len = 2.5 * d + 0.015;
        let blk = 0.018 + 0.5 * d;
        let x_start = blk * 2.2 + 0.5 * nut_len;
        let x_end = (len - blk * 2.2 - 0.5 * nut_len).max(x_start);
        let x_lo = if belt { -0.06 } else { -0.04 - ml };
        let (y_lo, y_hi) = (base_y - 0.012, hs + 0.08);
        let lift = [x_lo, len + 0.03].iter().flat_map(|x| [y_lo, y_hi].map(|y| x * sn + y * cs)).fold(f32::MAX, f32::min);
        // Motor end anchored, so dragging the far support moves it 1:1 with the pointer.
        let o = [-0.2 * cs, 0.005 - lift, 0.0];
        let mut lay = Layout { o, ax, ey, ez, len, x_start, x_end, ..Layout::default() };
        let xn = (x_start + pos as f32).clamp(x_start, x_end);
        let carriage_c = [xn, hs + 0.03, 0.0];
        lay.carriage = lay.at(carriage_c);
        lay.end = lay.at([len, 0.0, 0.0]);
        lay.pick_r = 0.05 + d;
        self.lay = lay;

        if self.preset == 3 && self.drag == Pick::None {
            self.cam.goal.target = lay.carriage;
        }
        self.cam.step(dt);
        let eye = self.cam.cur.eye();
        let view = math::look_at(eye, self.cam.cur.target, [0.0, 1.0, 0.0]);
        let proj = math::perspective(0.8, w as f32 / h.max(1) as f32, 0.01, 60.0);
        self.inv_vp = math::invert(&math::mul(&proj, &view)).unwrap_or(math::IDENTITY);
        self.size = (w as f32, h as f32);
        self.r.begin(&view, &proj, eye, [0, 0, w, h]);

        let steel = [0.72, 0.74, 0.78];
        let worst = res.checks().iter().copied().max_by_key(|c| c.status).map(|c| (c.status, res.min_margin()));
        let screw_rgb = match (tog.color, worst) {
            (true, Some((s, m))) => status_rgb(s, m),
            _ => steel,
        };
        let rgba = |c: [f32; 3]| [c[0], c[1], c[2], 1.0];
        let glow = |p: Pick| if self.hover == p || self.drag == p { 0.25 } else { 0.0 };
        let spin = TAU * (xn - x_start) / lead_vis;
        let mut parts: Vec<(&Mesh, M4, Paint)> = vec![];
        let lit = |c: [f32; 3]| Paint::lit(rgba(c));

        // Screw core + thread (spinning).
        parts.push((&self.cyl, lay.rod(0.0, len, 0.0, 0.0, 0.5 * dr, spin), lit(screw_rgb)));
        if let Some((m, _, _)) = &self.thread {
            parts.push((m, lay.spun(spin), lit(screw_rgb)));
        }
        // Base beam.
        let beam_x0 = x_lo;
        parts.push((&self.cube, lay.boxm([0.5 * (beam_x0 + len + 0.03), base_y - 0.006, 0.0], [len + 0.03 - beam_x0, 0.012, 1.6 * d + 0.03]), lit([0.50, 0.52, 0.56])));
        // End supports: fixed = two bearing blocks, simple = one; the far end is draggable.
        let (fixed0, far) = inp.support.ends();
        let block = |x: f32| lay.boxm([x, 0.5 * (base_y + 0.5 * d + 0.012), 0.0], [blk, 0.5 * d + 0.012 - base_y, d + 0.03]);
        let sup = [0.33, 0.36, 0.42];
        let n0 = if fixed0 { 2 } else { 1 };
        for i in 0..n0 {
            parts.push((&self.cube, block(0.5 * blk + 1.1 * blk * i as f32), lit(sup)));
        }
        if let Some(ff) = far {
            let n1 = if ff { 2 } else { 1 };
            for i in 0..n1 {
                let mut p = lit(sup);
                p.glow = glow(Pick::End);
                parts.push((&self.cube, block(len - 0.5 * blk - 1.1 * blk * i as f32), p));
            }
        } else {
            // Free end: just a small end cap marker for dragging.
            let mut p = lit(sup);
            p.glow = glow(Pick::End);
            parts.push((&self.cyl, lay.rod(len, len + 0.006, 0.0, 0.0, 0.5 * d + 0.003, 0.0), p));
        }
        // Nut + flange + carriage + payload.
        let nut_rgb = match inp.kind {
            Kind::TrapBronze => [0.80, 0.55, 0.25],
            Kind::TrapPom => [0.92, 0.92, 0.88],
            _ => [0.30, 0.32, 0.36],
        };
        parts.push((&self.cyl, lay.rod(xn - 0.5 * nut_len, xn + 0.5 * nut_len, 0.0, 0.0, 0.5 * d + 0.008, 0.0), lit(nut_rgb)));
        parts.push((&self.cyl, lay.rod(xn - 0.5 * nut_len, xn - 0.5 * nut_len + 0.006, 0.0, 0.0, 0.5 * d + 0.02, 0.0), lit(nut_rgb)));
        let mut car = lit([0.22, 0.45, 0.82]);
        car.glow = glow(Pick::Carriage);
        parts.push((&self.cube, lay.boxm([xn, 0.5 * d + 0.014, 0.0], [nut_len + 0.01, 0.012, 2.4 * d + 0.05]), car));
        let pay = (inp.mass_kg as f32).cbrt() * 0.02 + 0.02;
        parts.push((&self.cube, lay.boxm([xn, 0.5 * d + 0.02 + 0.5 * pay, 0.0], [pay, pay, pay]), car));
        // Motor: inline with coupling, or folded alongside with belt pulleys.
        let can = [0.16, 0.17, 0.19];
        if belt {
            let rp_m = 0.012f32;
            let rp_s = (rp_m * ratio).clamp(0.006, 0.05);
            let zm = -(rp_s + rp_m + 0.025);
            let xp = -0.03;
            parts.push((&self.cyl, lay.rod(xp - 0.006, xp + 0.006, 0.0, 0.0, rp_s, spin), lit([0.6, 0.6, 0.62])));
            parts.push((&self.cyl, lay.rod(xp - 0.006, xp + 0.006, 0.0, zm, rp_m, spin * ratio), lit([0.6, 0.6, 0.62])));
            parts.push((&self.cyl, lay.rod(xp, 0.0, 0.0, 0.0, 0.3 * d, 0.0), lit(steel)));
            parts.push((&self.cyl, lay.rod(xp + 0.008, xp + 0.008 + ml, 0.0, zm, mr, 0.0), lit(can)));
            for sgn in [1.0, -1.0] {
                let (a, b) = (lay.at([xp, sgn * rp_s, 0.0]), lay.at([xp, sgn * rp_m, zm]));
                let dv = math::sub(b, a);
                let l = math::dot(dv, dv).sqrt();
                parts.push((&self.cyl, along(a, math::norm(dv), l, 0.0025), lit([0.1, 0.1, 0.1])));
            }
        } else {
            parts.push((&self.cyl, lay.rod(-0.03, 0.0, 0.0, 0.0, 0.3 * d + 0.004, spin), lit(steel)));
            parts.push((&self.cyl, lay.rod(-0.04 - ml, -0.03, 0.0, 0.0, mr, 0.0), lit(can)));
        }

        for (m, model, _) in &parts {
            self.r.shadow(m, model);
        }
        for (m, model, p) in &parts {
            self.r.draw(m, model, *p);
        }

        // Force arrows: thrust on the nut along the axis, gravity on the payload.
        if tog.arrows {
            let fs = 0.25 / res.f_peak.max(1.0) as f32;
            let g = (inp.mass_kg * sd_core::G) as f32;
            let thrust_dir = if force >= 0.0 { ax } else { math::scale(ax, -1.0) };
            let arrows = [
                (lay.at([xn, 0.0, 0.5 * d + 0.03]), thrust_dir, (force.abs() as f32 * fs).max(0.01), [0.2, 0.75, 0.95]),
                (lay.at([xn, 0.5 * d + 0.02 + 0.5 * pay, 0.5 * pay + 0.02]), [0.0, -1.0, 0.0], (g * fs).max(0.01), [0.95, 0.45, 0.2]),
            ];
            for (a, dir, l, c) in arrows {
                let head = 0.025f32.min(0.5 * l);
                self.r.draw(&self.cyl, &along(a, dir, l - head, 0.003), Paint::flat(rgba(c)));
                self.r.draw(&self.cone, &along(math::add(a, math::scale(dir, l - head)), dir, head, 0.008), Paint::flat(rgba(c)));
            }
        }
        // Whirl ghost: exaggerated first-mode shape when running above 80 % of critical speed.
        if tog.whirl && res.screw_rpm > 0.8 * res.n_crit {
            let amp = 0.06 * len * (t as f32 * 9.0).sin();
            let n = 24;
            for i in 0..n {
                let (x0, x1) = (len * i as f32 / n as f32, len * (i + 1) as f32 / n as f32);
                let y = amp * mode_shape(inp.support, (i as f32 + 0.5) / n as f32);
                self.r.draw(&self.cyl, &lay.rod(x0, x1, y, 0.0, 0.5 * d + 0.002, 0.0), Paint::lit([0.95, 0.3, 0.25, 0.35]));
            }
        }
    }

    fn ray(&self, x: f32, y: f32) -> (V, V) {
        math::ndc_ray(&self.inv_vp, 2.0 * x / self.size.0 - 1.0, 1.0 - 2.0 * y / self.size.1)
    }

    fn pick(&self, x: f32, y: f32) -> Pick {
        let (o, d) = self.ray(x, y);
        let hc = math::ray_sphere(o, d, self.lay.carriage, self.lay.pick_r);
        let he = math::ray_sphere(o, d, self.lay.end, self.lay.pick_r);
        match (hc, he) {
            (Some(a), Some(b)) => if a <= b { Pick::Carriage } else { Pick::End },
            (Some(_), None) => Pick::Carriage,
            (None, Some(_)) => Pick::End,
            _ => Pick::None,
        }
    }

    /// Axial frame coordinate (m) under the pointer, on the plane through the axis facing the camera.
    fn axial(&self, x: f32, y: f32) -> Option<f32> {
        let (o, d) = self.ray(x, y);
        let ax = self.lay.ax;
        let n = math::norm(math::sub(d, math::scale(ax, math::dot(d, ax))));
        let p = math::ray_plane(o, d, self.lay.o, n)?;
        Some(math::dot(math::sub(p, self.lay.o), ax))
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

    /// Pointer move; `buttons` is the MouseEvent buttons mask. Returns a model change, if any.
    pub fn moved(&mut self, id: i32, x: f32, y: f32, buttons: u16) -> (Option<Action>, &'static str) {
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
            Pick::Carriage => self.axial(x, y).map(|a| Action::Scrub(((a - self.lay.x_start).max(0.0)) as f64)),
            Pick::End => self.axial(x, y).map(|a| Action::Length((a * 1000.0).clamp(50.0, 3000.0).round() as f64)),
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

    /// Visual travel range along the axis (m), for mapping a scrub to the stroke.
    pub fn travel(&self) -> f32 {
        self.lay.x_end - self.lay.x_start
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_mesh_size_and_bounds() {
        let g = thread(0.006, 0.008, 0.005, 0.1, 0.0015);
        // 20 turns × 20 segments × 2 faces × 2 triangles × 3 vertices.
        assert_eq!(g.pos.len() / 3, 20 * 20 * 12);
        assert!(g.pos.chunks(3).all(|p| (0.0..=0.1).contains(&p[1]) && (p[0] * p[0] + p[2] * p[2]).sqrt() <= 0.00801));
    }

    #[test]
    fn mode_shapes_vanish_at_supports() {
        for s in [Support::SimpleSimple, Support::FixedFixed, Support::FixedSimple] {
            assert!(mode_shape(s, 0.0).abs() < 1e-6 && mode_shape(s, 1.0).abs() < 1e-5);
        }
        assert!(mode_shape(Support::FixedFree, 0.0).abs() < 1e-6 && mode_shape(Support::FixedFree, 1.0) > 0.99);
    }
}
