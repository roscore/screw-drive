//! Tiny canvas-2D plotting: force–speed envelope and the move-vs-time chart.
use sd_core::{sample, cycle_time, Results};
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d as Ctx, HtmlCanvasElement};

pub struct Colors {
    pub fg: &'static str,
    pub grid: &'static str,
    pub mut_: &'static str,
}

pub const DARK: Colors = Colors { fg: "#e4e7ec", grid: "#333a45", mut_: "#9aa3b2" };
pub const LIGHT: Colors = Colors { fg: "#1d2128", grid: "#d5d9e0", mut_: "#5b6472" };
const C_MOTOR: &str = "#2f9be0";
const C_CRIT: &str = "#e0533a";
const C_DN: &str = "#d39a10";
const C_PT: &str = "#3fbf6f";

struct Frame {
    ctx: Ctx,
    x0: f64,
    y0: f64,
    w: f64,
    h: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
}

impl Frame {
    fn px(&self, x: f64) -> f64 {
        self.x0 + x / self.xmax * self.w
    }
    fn py(&self, y: f64) -> f64 {
        self.y0 + self.h - (y - self.ymin) / (self.ymax - self.ymin) * self.h
    }
    fn line(&self, pts: &[(f64, f64)], color: &str, width: f64) {
        let c = &self.ctx;
        c.set_stroke_style_str(color);
        c.set_line_width(width);
        c.begin_path();
        for (i, (x, y)) in pts.iter().enumerate() {
            if i == 0 {
                c.move_to(self.px(*x), self.py(*y));
            } else {
                c.line_to(self.px(*x), self.py(*y));
            }
        }
        c.stroke();
    }
    fn text(&self, s: &str, x: f64, y: f64, color: &str) {
        self.ctx.set_fill_style_str(color);
        let _ = self.ctx.fill_text(s, x, y);
    }
}

fn setup(canvas: &HtmlCanvasElement, xmax: f64, ymin: f64, ymax: f64, col: &Colors, xlabel: &str, yticks: bool) -> Option<Frame> {
    let dpr = web_sys::window()?.device_pixel_ratio();
    let (cw, ch) = (canvas.client_width() as f64, canvas.client_height() as f64);
    if cw < 10.0 {
        return None;
    }
    canvas.set_width((cw * dpr) as u32);
    canvas.set_height((ch * dpr) as u32);
    let ctx: Ctx = canvas.get_context("2d").ok()??.dyn_into().ok()?;
    ctx.scale(dpr, dpr).ok()?;
    ctx.set_font("11px system-ui, sans-serif");
    let f = Frame { ctx, x0: 44.0, y0: 8.0, w: cw - 54.0, h: ch - 32.0, xmax: xmax.max(1e-9), ymin, ymax: ymax.max(ymin + 1e-9) };
    // Axes and 4 grid lines with tick labels.
    for i in 0..=4 {
        let y = ymin + (f.ymax - ymin) * i as f64 / 4.0;
        f.line(&[(0.0, y), (f.xmax, y)], col.grid, 1.0);
        if yticks {
            f.text(&fmt(y), 2.0, f.py(y) + 4.0, col.mut_);
        }
        let x = f.xmax * i as f64 / 4.0;
        f.line(&[(x, ymin), (x, f.ymax)], col.grid, 1.0);
        f.text(&fmt(x), f.px(x) - 10.0, ch - 12.0, col.mut_);
    }
    f.text(xlabel, f.x0 + f.w - 70.0, ch - 1.0, col.fg);
    Some(f)
}

pub fn fmt(v: f64) -> String {
    let a = v.abs();
    if a >= 1e5 {
        format!("{:.0}k", v / 1e3)
    } else if a >= 100.0 || a == 0.0 {
        format!("{v:.0}")
    } else if a >= 10.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

/// Motor-limited thrust vs linear speed, 0.8·critical and DN speed limits, operating points.
pub fn envelope(canvas: &HtmlCanvasElement, r: &Results, col: &Colors, labels: [&str; 4]) {
    let vmax = r.v_motor_max.max(r.v_peak * 1000.0).max(r.v_crit.min(r.v_dn).min(r.v_motor_max * 2.0)) * 1.15;
    let fmax = r.f_avail.max(r.f_peak) * 1.15;
    let Some(f) = setup(canvas, vmax, 0.0, fmax, col, "mm/s · N", true) else { return };
    f.line(&[(0.0, r.f_avail), (r.v_motor_max, r.f_avail), (r.v_motor_max, 0.0)], C_MOTOR, 2.0);
    for (i, (v, c)) in [(r.v_crit, C_CRIT), (r.v_dn, C_DN)].into_iter().enumerate() {
        if v < vmax {
            f.line(&[(v, 0.0), (v, fmax)], c, 1.5);
        } else {
            // Off-scale limit: pinned marker at the right edge with its value.
            let y = fmax * (0.55 - 0.12 * i as f64);
            f.line(&[(vmax * 0.97, y), (vmax, y)], c, 3.0);
            let s = format!("{} mm/s →", fmt(v));
            f.text(&s, f.px(vmax) - 8.0 - 6.0 * s.len() as f64, f.py(y) + 4.0, c);
        }
    }
    f.ctx.set_fill_style_str(C_PT);
    for ph in &r.phases {
        let (v, fo) = (ph.v0.abs().max(ph.v1.abs()) * 1000.0, ph.force.abs());
        f.ctx.begin_path();
        let _ = f.ctx.arc(f.px(v), f.py(fo), 4.0, 0.0, std::f64::consts::TAU);
        f.ctx.fill();
    }
    for (i, (s, c)) in labels.iter().zip([C_MOTOR, C_CRIT, C_DN, C_PT]).enumerate() {
        f.text(s, f.x0 + 8.0, f.y0 + 12.0 + 13.0 * i as f64, c);
    }
}

/// Position, speed and thrust over one cycle (each scaled to its own range) and a time cursor.
pub fn timeline(canvas: &HtmlCanvasElement, r: &Results, t: f64, col: &Colors, labels: [String; 3]) {
    let total = cycle_time(r);
    let Some(f) = setup(canvas, total, -1.0, 1.0, col, "s", false) else { return };
    let n = 200;
    let pts: Vec<(f64, (f64, f64, f64))> = (0..=n).map(|i| {
        let ti = total * i as f64 / n as f64;
        (ti, sample(r, ti))
    }).collect();
    let xmax = pts.iter().map(|p| p.1 .0.abs()).fold(1e-9, f64::max);
    let vmax = pts.iter().map(|p| p.1 .1.abs()).fold(1e-9, f64::max);
    let fmax = pts.iter().map(|p| p.1 .2.abs()).fold(1e-9, f64::max);
    let series: [(&dyn Fn(&(f64, f64, f64)) -> f64, &str); 3] = [
        (&|s| s.0 / xmax, C_MOTOR),
        (&|s| s.1 / vmax, C_PT),
        (&|s| s.2 / fmax, C_CRIT),
    ];
    for (i, (g, c)) in series.iter().enumerate() {
        let line: Vec<(f64, f64)> = pts.iter().map(|(ti, s)| (*ti, g(s))).collect();
        f.line(&line, c, 1.8);
        f.text(&labels[i], f.x0 + 8.0, f.y0 + 12.0 + 13.0 * i as f64, c);
    }
    let tc = t.rem_euclid(total.max(1e-9));
    f.line(&[(tc, -1.0), (tc, 1.0)], col.fg, 1.0);
}
