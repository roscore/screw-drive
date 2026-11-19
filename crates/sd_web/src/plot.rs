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
    xmin: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
}

impl Frame {
    fn px(&self, x: f64) -> f64 {
        self.x0 + (x - self.xmin) / (self.xmax - self.xmin) * self.w
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
    setup_x(canvas, 0.0, xmax, ymin, ymax, col, xlabel, yticks)
}

#[allow(clippy::too_many_arguments)]
fn setup_x(canvas: &HtmlCanvasElement, xmin: f64, xmax: f64, ymin: f64, ymax: f64, col: &Colors, xlabel: &str, yticks: bool) -> Option<Frame> {
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
    let f = Frame { ctx, x0: 44.0, y0: 8.0, w: cw - 54.0, h: ch - 32.0, xmin, xmax: xmax.max(xmin + 1e-9), ymin, ymax: ymax.max(ymin + 1e-9) };
    // Axes and 4 grid lines with tick labels.
    for i in 0..=4 {
        let y = ymin + (f.ymax - ymin) * i as f64 / 4.0;
        f.line(&[(0.0, y), (f.xmax, y)], col.grid, 1.0);
        if yticks {
            f.text(&fmt(y), 2.0, f.py(y) + 4.0, col.mut_);
        }
        let x = xmin + (f.xmax - xmin) * i as f64 / 4.0;
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

/// One line of a `series` plot: y values (same length as x; non-finite points are skipped), colour, width.
pub struct Series<'a> {
    pub y: Vec<f64>,
    pub color: &'a str,
    pub width: f64,
    pub label: String,
}

/// Several lines over a shared x axis with auto y range (finite values only), an optional vertical
/// cursor and an optional dashed horizontal reference line.
pub fn series(canvas: &HtmlCanvasElement, x: &[f64], lines: &[Series], cursor: Option<f64>, href: Option<(f64, &str)>, col: &Colors, xlabel: &str) {
    let (xmin, xmax) = (x.first().copied().unwrap_or(0.0), x.last().copied().unwrap_or(1.0));
    let fin = lines.iter().flat_map(|s| s.y.iter().copied()).chain(href.map(|h| h.0)).filter(|v| v.is_finite());
    let (lo, hi) = fin.fold((0.0f64, 0.0f64), |(a, b), v| (a.min(v), b.max(v)));
    let hi = if hi - lo < 1e-9 { lo + 1.0 } else { hi + 0.08 * (hi - lo) };
    let lo = if lo < 0.0 { lo - 0.05 * (hi - lo) } else { 0.0 };
    let Some(f) = setup_x(canvas, xmin, xmax, lo, hi, col, xlabel, true) else { return };
    if let Some((y, c)) = href {
        let _ = f.ctx.set_line_dash(&js_sys::Array::of2(&5.0.into(), &4.0.into()));
        f.line(&[(xmin, y), (xmax, y)], c, 1.2);
        let _ = f.ctx.set_line_dash(&js_sys::Array::new());
    }
    for (i, s) in lines.iter().enumerate() {
        // Break the polyline at non-finite points.
        let mut seg: Vec<(f64, f64)> = vec![];
        for (xi, yi) in x.iter().zip(&s.y) {
            if yi.is_finite() {
                seg.push((*xi, yi.clamp(lo, hi)));
            } else if !seg.is_empty() {
                f.line(&seg, s.color, s.width);
                seg.clear();
            }
        }
        if !seg.is_empty() {
            f.line(&seg, s.color, s.width);
        }
        f.text(&s.label, f.x0 + 8.0, f.y0 + 12.0 + 13.0 * i as f64, s.color);
    }
    if let Some(c) = cursor {
        f.line(&[(c, lo), (c, hi)], col.fg, 1.0);
    }
}

/// Colour ramp red → yellow → green for t in 0…1.
pub fn ramp(t: f64) -> String {
    let t = t.clamp(0.0, 1.0);
    let (r, g) = if t < 0.5 { (230.0, 70.0 + 300.0 * t) } else { (230.0 - 330.0 * (t - 0.5), 220.0) };
    format!("rgb({:.0},{:.0},70)", r.max(0.0), g.min(220.0))
}

/// Pitch–roll workspace heat map (degrees on both axes): cell colour = value / vmax on `ramp`,
/// singular cells hatched grey; `mark` is the current pose.
#[allow(clippy::too_many_arguments)]
pub fn heatmap(canvas: &HtmlCanvasElement, cells: &[(f64, f64, f64, bool)], n: (usize, usize), px: (f64, f64), ry: (f64, f64), vmax: f64, mark: (f64, f64), col: &Colors, labels: [&str; 3]) {
    let Some(f) = setup_x(canvas, px.0, px.1, ry.0, ry.1, col, labels[0], true) else { return };
    let (cw, chh) = (f.w / n.0 as f64, f.h / n.1 as f64);
    for (p, r, v, sing) in cells {
        let (x, y) = (f.px(*p) - 0.5 * cw, f.py(*r) - 0.5 * chh);
        let c = if *sing { "#808080".to_string() } else { ramp(v / vmax.max(1e-9)) };
        f.ctx.set_fill_style_str(&c);
        f.ctx.fill_rect(x, y, cw + 0.5, chh + 0.5);
        if *sing {
            f.line(&[(*p - 0.4 * (px.1 - px.0) / n.0 as f64, *r - 0.4 * (ry.1 - ry.0) / n.1 as f64), (*p + 0.4 * (px.1 - px.0) / n.0 as f64, *r + 0.4 * (ry.1 - ry.0) / n.1 as f64)], "#303030", 1.0);
        }
    }
    f.ctx.set_stroke_style_str(col.fg);
    f.ctx.set_line_width(2.0);
    f.ctx.begin_path();
    let _ = f.ctx.arc(f.px(mark.0), f.py(mark.1), 5.0, 0.0, std::f64::consts::TAU);
    f.ctx.stroke();
    f.text(labels[1], f.x0 + 6.0, f.y0 + 12.0, col.fg);
    f.text(labels[2], f.x0 + 6.0, f.y0 + 25.0, col.fg);
}
