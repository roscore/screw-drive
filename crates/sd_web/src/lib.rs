//! Web UI for screw-drive: inputs ↔ sd_core, results table, HUD, plots, search, 3D view.
pub mod gl;
pub mod math;
pub mod plot;
pub mod view3d;

use sd_core::{compute, cycle_time, sample, search, Hit, Inputs, Kind, Results, Status, Support};
use std::cell::RefCell;
use std::rc::Rc;
use view3d::{Action, Scene, Toggles};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, Event, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement, PointerEvent, WheelEvent};

struct App {
    inp: Inputs,
    res: Results,
    scene: Option<Scene>,
    hits: Vec<Hit>,
    t: f64,
    playing: bool,
    slow: f64,
    ko: bool,
    dark: bool,
    last_ts: f64,
    tog: Toggles,
}

thread_local! {
    static APP: RefCell<Option<App>> = const { RefCell::new(None) };
}

fn with<R>(f: impl FnOnce(&mut App) -> R) -> Option<R> {
    APP.with(|a| a.borrow_mut().as_mut().map(f))
}

fn doc() -> Document {
    web_sys::window().unwrap().document().unwrap()
}
fn el(id: &str) -> Element {
    doc().get_element_by_id(id).unwrap_or_else(|| panic!("#{id}"))
}
fn input(id: &str) -> HtmlInputElement {
    el(id).unchecked_into()
}
fn select(id: &str) -> HtmlSelectElement {
    el(id).unchecked_into()
}
fn num(id: &str, fallback: f64) -> f64 {
    let v = input(id).value_as_number();
    if v.is_finite() { v } else { fallback }
}
fn set_num(id: &str, v: f64) {
    input(id).set_value(&format!("{}", (v * 1000.0).round() / 1000.0));
}
fn on(target: &web_sys::EventTarget, ev: &str, f: impl FnMut(Event) + 'static) {
    let c = Closure::<dyn FnMut(Event)>::new(f);
    let _ = target.add_event_listener_with_callback(ev, c.as_ref().unchecked_ref());
    c.forget();
}
fn on_id(id: &str, ev: &str, f: impl FnMut(Event) + 'static) {
    on(&el(id), ev, f);
}

const NUM_IDS: [&str; 15] = ["d", "lead", "len", "ca", "mt", "mrpm", "ratio", "mass", "angle", "fext", "stroke", "vmax", "acc", "cpd", "years"];

fn write_inputs(i: &Inputs) {
    select("kind").set_value(&(Kind::ALL.iter().position(|k| *k == i.kind).unwrap_or(0)).to_string());
    select("support").set_value(&(Support::ALL.iter().position(|s| *s == i.support).unwrap_or(0)).to_string());
    let vals = [i.d_mm, i.lead_mm, i.length_mm, i.ca_kn, i.motor_torque, i.motor_rpm, i.ratio, i.mass_kg, i.angle_deg, i.f_ext, i.stroke_mm, i.v_mm_s, i.acc, i.cycles_per_day, i.years];
    for (id, v) in NUM_IDS.iter().zip(vals) {
        set_num(id, v);
    }
}

fn read_inputs(old: &Inputs) -> Inputs {
    let kind = Kind::ALL[select("kind").value().parse::<usize>().unwrap_or(0).min(3)];
    let support = Support::ALL[select("support").value().parse::<usize>().unwrap_or(0).min(3)];
    let c = |id, lo: f64, hi: f64, old: f64| num(id, old).clamp(lo, hi);
    Inputs {
        kind,
        support,
        d_mm: c("d", 6.0, 40.0, old.d_mm),
        lead_mm: c("lead", 1.0, 40.0, old.lead_mm),
        length_mm: c("len", 50.0, 3000.0, old.length_mm),
        ca_kn: c("ca", 0.01, 1e4, old.ca_kn),
        motor_torque: c("mt", 0.001, 1e3, old.motor_torque),
        motor_rpm: c("mrpm", 1.0, 1e5, old.motor_rpm),
        ratio: c("ratio", 0.1, 20.0, old.ratio),
        mass_kg: c("mass", 0.0, 1e4, old.mass_kg),
        angle_deg: c("angle", 0.0, 90.0, old.angle_deg),
        f_ext: c("fext", -1e6, 1e6, old.f_ext),
        stroke_mm: c("stroke", 1.0, 1e4, old.stroke_mm),
        v_mm_s: c("vmax", 0.1, 1e5, old.v_mm_s),
        acc: c("acc", 0.001, 1e3, old.acc),
        cycles_per_day: c("cpd", 0.0, 1e7, old.cycles_per_day),
        years: c("years", 0.01, 100.0, old.years),
        guide_mu: old.guide_mu,
    }
}

fn tr(ko: bool, en: &'static str, k: &'static str) -> &'static str {
    if ko { k } else { en }
}

fn cls(s: Status) -> &'static str {
    match s {
        Status::Ok => "ok",
        Status::Warn => "warn",
        Status::Fail => "fail",
    }
}

fn st(ko: bool, s: Status) -> &'static str {
    match s {
        Status::Ok => "OK",
        Status::Warn => tr(ko, "warning", "주의"),
        Status::Fail => tr(ko, "fail", "불가"),
    }
}

fn years(v: f64) -> String {
    if v > 999.0 { ">999".into() } else { format!("{v:.1}") }
}

fn render_numbers(a: &App) {
    let (r, ko) = (&a.res, a.ko);
    let f = plot::fmt;
    let row = |label: &str, val: String, s: Option<Status>| {
        let (c, t) = s.map_or(("", String::new()), |s| (cls(s), st(ko, s).to_string()));
        format!("<tr><td>{label}</td><td>{val}</td><td class=\"{c}\">{t}</td></tr>")
    };
    let back = if r.self_locking { tr(ko, "no (self-locking)", "불가 (셀프 록킹)") } else { tr(ko, "yes", "가능") };
    let rows = [
        row(tr(ko, "Lead angle λ", "리드각 λ"), format!("{:.2}°", r.lambda_deg), None),
        row(tr(ko, "Efficiency fwd / back", "효율 정 / 역"), format!("{:.2} / {:.2}", r.eta, r.eta_back), None),
        row(tr(ko, "Back-drivable", "역구동"), back.into(), None),
        row(tr(ko, "Peak thrust", "최대 추력"), format!("{} N", f(r.f_peak)), None),
        row(tr(ko, "Cubic-mean load Fm", "등가 하중 Fm"), format!("{} N", f(r.fm)), None),
        row(tr(ko, "Peak motor torque", "최대 모터 토크"), format!("{} N·m (×{:.2})", f(r.t_motor), r.torque.margin), Some(r.torque.status)),
        row(tr(ko, "Motor speed needed", "필요 모터 속도"), format!("{} rpm (×{:.2})", f(r.motor_rpm_need), r.speed.margin), Some(r.speed.status)),
        row(tr(ko, "Thrust available", "가용 추력"), format!("{} N", f(r.f_avail)), None),
        row(tr(ko, "Motor-limited speed", "모터 한계 속도"), format!("{} mm/s", f(r.v_motor_max)), None),
        row(tr(ko, "Buckling load (root ⌀)", "좌굴 하중 (골지름)"), format!("{} N, SF {:.1} (⌀{:.1})", f(r.p_buckle), r.buckling.margin, r.dr_mm), Some(r.buckling.status)),
        row(tr(ko, "Critical speed (×0.8)", "위험 속도 (×0.8)"), format!("{} rpm (×{:.2})", f(0.8 * r.n_crit), r.whirl.margin), Some(r.whirl.status)),
        row("DN", format!("{} (×{:.2})", f(r.dn), r.dn_check.margin), Some(r.dn_check.status)),
        row(tr(ko, "L10 life", "L10 수명"), format!("{:.2e} rev, {} km, {} y", r.l10_rev, f(r.l10_km.min(1e9)), years(r.l10_years)), Some(r.life.status)),
        row(tr(ko, "Move time (one way)", "편도 이동 시간"), format!("{:.3} s", r.t_move), None),
        row(tr(ko, "Inertia screw / load", "관성 스크루 / 부하"), format!("{:.2e} / {:.2e} kg·m²", r.j_screw, r.j_load), None),
        row(tr(ko, "Overall", "종합"), String::new(), Some(r.overall)),
    ];
    el("results").set_inner_html(&rows.concat());
    let hud = format!(
        "{}: <b>{} N</b><br>{}: <b>{} N·m</b><br>{}: <b>{} rpm</b><br>{}: <b>{} mm/s</b><br>L10: <b>{} y</b><br>{}: <b>{:.1}</b><br>{}: <b>×{:.2}</b><br>{}: <b>{}</b><br>{}: <b class=\"{}\">{}</b>",
        tr(ko, "Thrust", "추력"), f(r.f_peak),
        tr(ko, "Motor torque", "모터 토크"), f(r.t_motor),
        tr(ko, "Motor", "모터"), f(r.motor_rpm_need),
        tr(ko, "Speed", "속도"), f(r.v_peak * 1000.0),
        years(r.l10_years),
        tr(ko, "Buckling SF", "좌굴 SF"), r.buckling.margin,
        tr(ko, "Crit. speed margin", "위험속도 여유"), r.whirl.margin,
        tr(ko, "Back-drivable", "역구동"), if r.self_locking { tr(ko, "no", "불가") } else { tr(ko, "yes", "가능") },
        tr(ko, "Overall", "종합"), cls(r.overall), st(ko, r.overall),
    );
    el("hud").set_inner_html(&hud);
    let hits: String = a.hits.iter().enumerate().map(|(i, h)| {
        format!(
            "<tr data-i=\"{i}\"><td class=\"{}\">{} × {}</td><td>×{:.2}</td><td>{:.2}</td><td>{}</td><td>{}</td></tr>",
            cls(h.res.overall), h.d_mm, h.lead_mm, h.res.torque.margin, h.res.min_margin(), years(h.res.l10_years), f(h.res.motor_rpm_need)
        )
    }).collect();
    el("hits").set_inner_html(&hits);
    let colors = if a.dark { &plot::DARK } else { &plot::LIGHT };
    plot::envelope(&el("p-env").unchecked_into(), r, colors, [
        tr(ko, "motor-limited thrust", "모터 한계 추력"),
        tr(ko, "0.8 × critical speed", "0.8 × 위험 속도"),
        tr(ko, "DN limit", "DN 한계"),
        tr(ko, "operating points", "운전점"),
    ]);
}

fn recompute(a: &mut App) {
    a.res = compute(&a.inp);
    render_numbers(a);
}

fn apply_lang(ko: bool) {
    let key = if ko { "data-ko" } else { "data-en" };
    let list = doc().query_selector_all("[data-en]").unwrap();
    for i in 0..list.length() {
        if let Some(e) = list.item(i).and_then(|n| n.dyn_into::<Element>().ok()) {
            if let Some(t) = e.get_attribute(key) {
                e.set_text_content(Some(&t));
            }
        }
    }
    doc().document_element().unwrap().set_attribute("lang", if ko { "ko" } else { "en" }).ok();
    el("lang").set_text_content(Some(if ko { "English" } else { "한국어" }));
}

fn set_play_label(a: &App) {
    el("t-play").set_text_content(Some(if a.playing { tr(a.ko, "Pause", "일시정지") } else { tr(a.ko, "Play", "재생") }));
    el("t-play").class_list().toggle_with_force("on", a.playing).ok();
}

/// Time on the forward move where the nut is at `p` (m), by bisection.
fn time_at(r: &Results, p: f64) -> f64 {
    let (mut lo, mut hi) = (0.0, r.t_move);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if sample(r, mid).0 < p { lo = mid } else { hi = mid }
    }
    lo
}

fn frame(ts: f64) {
    with(|a| {
        let dt = ((ts - a.last_ts) / 1000.0).clamp(0.0, 0.1);
        a.last_ts = ts;
        let total = cycle_time(&a.res).max(1e-6);
        if a.playing {
            a.t = (a.t + dt * a.slow).rem_euclid(total);
            input("time").set_value(&format!("{}", (a.t / total * 1000.0).round()));
        }
        let (pos, v, force) = sample(&a.res, a.t);
        el("tlabel").set_text_content(Some(&format!("t {:.2} s · x {:.0} mm · v {:.0} mm/s · F {:.0} N", a.t, pos * 1000.0, v * 1000.0, force)));
        let colors = if a.dark { &plot::DARK } else { &plot::LIGHT };
        let ko = a.ko;
        plot::timeline(&el("p-time").unchecked_into(), &a.res, a.t, colors, [
            format!("{} (0…{:.0} mm)", tr(ko, "position", "위치"), a.inp.stroke_mm),
            format!("{} (±{:.0} mm/s)", tr(ko, "speed", "속도"), a.res.v_peak * 1000.0),
            format!("{} (±{} N)", tr(ko, "thrust", "추력"), plot::fmt(a.res.f_peak)),
        ]);
        if let Some(s) = a.scene.as_mut() {
            let c: HtmlCanvasElement = el("view").unchecked_into();
            let dpr = web_sys::window().unwrap().device_pixel_ratio();
            let (w, h) = ((c.client_width() as f64 * dpr) as u32, (c.client_height() as f64 * dpr) as u32);
            if c.width() != w || c.height() != h {
                c.set_width(w);
                c.set_height(h);
            }
            s.frame(w as i32, h as i32, dt as f32, &a.inp, &a.res, pos, force, &a.tog, a.t);
        }
    });
}

fn start_loop() {
    let cb: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let cb2 = cb.clone();
    *cb.borrow_mut() = Some(Closure::new(move |ts: f64| {
        frame(ts);
        if let Some(c) = cb2.borrow().as_ref() {
            let _ = web_sys::window().unwrap().request_animation_frame(c.as_ref().unchecked_ref());
        }
    }));
    let _ = web_sys::window().unwrap().request_animation_frame(cb.borrow().as_ref().unwrap().as_ref().unchecked_ref());
}

fn canvas_xy(e: &PointerEvent) -> (f32, f32) {
    let c = el("view");
    let r = c.get_bounding_client_rect();
    let dpr = web_sys::window().unwrap().device_pixel_ratio();
    (((e.client_x() as f64 - r.left()) * dpr) as f32, ((e.client_y() as f64 - r.top()) * dpr) as f32)
}

fn wire_view() {
    let c = el("view");
    on(&c, "contextmenu", |e| e.prevent_default());
    on(&c, "pointerdown", |e| {
        let e: PointerEvent = e.unchecked_into();
        let _ = el("view").set_pointer_capture(e.pointer_id());
        let (x, y) = canvas_xy(&e);
        with(|a| if let Some(s) = a.scene.as_mut() { s.down(e.pointer_id(), x, y) });
    });
    on(&c, "pointermove", |e| {
        let e: PointerEvent = e.unchecked_into();
        let (x, y) = canvas_xy(&e);
        with(|a| {
            let Some(s) = a.scene.as_mut() else { return };
            let travel = s.travel() as f64;
            let (act, cursor) = s.moved(e.pointer_id(), x, y, e.buttons());
            let _ = el("view").unchecked_ref::<HtmlElement>().style().set_property("cursor", cursor);
            match act {
                Some(Action::Scrub(p)) => {
                    let stroke = a.inp.stroke_mm / 1000.0;
                    a.t = time_at(&a.res, p.min(travel).min(stroke));
                    a.playing = false;
                    set_play_label(a);
                    let total = cycle_time(&a.res).max(1e-6);
                    input("time").set_value(&format!("{}", (a.t / total * 1000.0).round()));
                }
                Some(Action::Length(l)) if l != a.inp.length_mm => {
                    a.inp.length_mm = l;
                    set_num("len", l);
                    recompute(a);
                }
                _ => {}
            }
        });
    });
    for ev in ["pointerup", "pointercancel"] {
        on(&c, ev, |e| {
            let e: PointerEvent = e.unchecked_into();
            with(|a| if let Some(s) = a.scene.as_mut() { s.up(e.pointer_id()) });
        });
    }
    on(&c, "wheel", |e| {
        e.prevent_default();
        let e: WheelEvent = e.unchecked_into();
        with(|a| if let Some(s) = a.scene.as_mut() { s.cam.zoom((e.delta_y() * 0.0015).exp() as f32) });
    });
    on(&c, "dblclick", |_| {
        with(|a| if let Some(s) = a.scene.as_mut() { s.reset_view(&a.inp) });
    });
    on_id("cam", "change", |_| {
        with(|a| if let Some(s) = a.scene.as_mut() {
            s.preset = select("cam").value().parse().unwrap_or(0);
            s.reset_view(&a.inp);
        });
    });
    for (id, which) in [("t-color", 0), ("t-arrows", 1), ("t-whirl", 2)] {
        on_id(id, "click", move |_| {
            with(|a| {
                let f = match which { 0 => &mut a.tog.color, 1 => &mut a.tog.arrows, _ => &mut a.tog.whirl };
                *f = !*f;
                el(id).class_list().toggle_with_force("on", *f).ok();
            });
        });
    }
    on_id("t-play", "click", |_| {
        with(|a| {
            a.playing = !a.playing;
            set_play_label(a);
        });
    });
}

#[wasm_bindgen(start)]
pub fn start() {
    let win = web_sys::window().unwrap();
    let ko = win.navigator().language().is_some_and(|l| l.to_lowercase().starts_with("ko"));
    let dark = win.match_media("(prefers-color-scheme: dark)").ok().flatten().is_some_and(|m| m.matches());
    let inp = Inputs::default();
    write_inputs(&inp);
    apply_lang(ko);
    let canvas: HtmlCanvasElement = el("view").unchecked_into();
    let scene = gl::Renderer::new(&canvas).and_then(Scene::new).map(|mut s| {
        s.r.theme = if dark { gl::DARK } else { gl::LIGHT };
        s
    });
    if scene.is_none() {
        let _ = el("nogl").unchecked_ref::<HtmlElement>().style().set_property("display", "flex");
        for id in ["cam", "t-color", "t-arrows", "t-whirl"] {
            let _ = el(id).unchecked_ref::<HtmlElement>().style().set_property("display", "none");
        }
    }
    let res = compute(&inp);
    let app = App { inp, res, scene, hits: vec![], t: 0.0, playing: true, slow: 0.25, ko, dark, last_ts: 0.0, tog: Toggles { color: true, arrows: true, whirl: true } };
    APP.with(|a| *a.borrow_mut() = Some(app));
    with(|a| {
        if let Some(s) = a.scene.as_mut() {
            // Run one layout pass so presets know where the screw is.
            s.frame(1, 1, 0.0, &a.inp, &a.res, 0.0, 0.0, &a.tog, 0.0);
            s.reset_view(&a.inp);
            s.cam.cur = s.cam.goal;
        }
        render_numbers(a);
        set_play_label(a);
    });

    for id in NUM_IDS.iter().chain(["kind", "support"].iter()) {
        let id = *id;
        on_id(id, "change", move |_| {
            with(|a| {
                let mut n = read_inputs(&a.inp);
                if id == "kind" || id == "d" {
                    n.ca_kn = (n.kind.preset().default_ca_kn(n.d_mm) * 10.0).round() / 10.0;
                    set_num("ca", n.ca_kn);
                }
                a.inp = n;
                recompute(a);
            });
        });
    }
    on_id("lang", "click", |_| {
        with(|a| {
            a.ko = !a.ko;
            apply_lang(a.ko);
            set_play_label(a);
            render_numbers(a);
        });
    });
    on_id("find", "click", |_| {
        with(|a| {
            a.hits = search(&a.inp, 12);
            render_numbers(a);
        });
    });
    on_id("hits", "click", |e| {
        let Some(tr) = e.target().and_then(|t| t.dyn_into::<Element>().ok()).and_then(|t| t.closest("tr").ok().flatten()) else { return };
        let Some(i) = tr.get_attribute("data-i").and_then(|s| s.parse::<usize>().ok()) else { return };
        with(|a| {
            if let Some(h) = a.hits.get(i) {
                a.inp.d_mm = h.d_mm;
                a.inp.lead_mm = h.lead_mm;
                a.inp.ca_kn = (a.inp.kind.preset().default_ca_kn(h.d_mm) * 10.0).round() / 10.0;
                write_inputs(&a.inp);
                recompute(a);
            }
        });
    });
    on_id("time", "input", |_| {
        with(|a| {
            a.t = num("time", 0.0) / 1000.0 * cycle_time(&a.res);
            a.playing = false;
            set_play_label(a);
        });
    });
    on_id("slow", "change", |_| {
        with(|a| a.slow = select("slow").value().parse().unwrap_or(0.25));
    });
    on(&win, "resize", |_| {
        with(|a| render_numbers(a));
    });
    wire_view();
    start_loop();
}
