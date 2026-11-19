//! "Mechanism" tab: knee / ankle linkage inputs, actuator comparison, plots, HUD and 3D view.
use crate::mech3d::{ActDraw, MAction, MechView};
use crate::{cls, el, input, plot, select, st, tr, with, years, App};
use sd_core::mech::{self, ankle_workspace, couple, preset, sweep, Actuator, Ankle, Cell, Coupled, Flag, Mech, Sample, Setup};
use sd_core::{root_diameter, Kind, Status};
use wasm_bindgen::JsCast;
use web_sys::{Element, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement};

/// Plot / table colours of the four comparison configs.
pub const CFG_COLORS: [&str; 4] = ["#2f9be0", "#3fbf6f", "#d39a10", "#b065d8"];
const C_REQ: &str = "#e0533a";
/// Workspace grid (pitch × roll).
const WS: (usize, usize) = (26, 13);

pub struct MechState {
    pub preset: usize,
    pub setup: Setup,
    /// Roll half-range of the ankle workspace (rad).
    pub roll_lim: f64,
    pub cfgs: [Actuator; 4],
    pub on: [bool; 4],
    pub samples: Vec<Sample>,
    pub panel: Option<Coupled>,
    pub cmp: [Option<Coupled>; 4],
    pub cells: Vec<Cell>,
    /// Actuator length range over the whole workspace (m), for drawing.
    pub s_range: (f64, f64),
    pub q: f64,
    pub roll: f64,
    pub playing: bool,
    pub phase: f64,
    pub view: MechView,
    /// Joint pose of the last plot redraw (plots only follow the cursor when it moved).
    drawn: Option<(f64, f64)>,
    /// The first opening of the tab applies the selected preset (incl. its actuator) once.
    pub opened: bool,
}

impl MechState {
    pub fn new() -> Self {
        let (setup, a0) = preset(0);
        MechState {
            preset: 0,
            setup,
            roll_lim: 20f64.to_radians(),
            cfgs: [
                a0,
                Actuator::new(Kind::Ball, 16.0, 5.0, 1.2, 4000.0, 1.0),
                Actuator::new(Kind::TrapBronze, 12.0, 3.0, 2.0, 1500.0, 1.0),
                Actuator::new(Kind::Ball, 8.0, 2.0, 0.18, 6000.0, 1.0),
            ],
            on: [true; 4],
            samples: vec![],
            panel: None,
            cmp: [None, None, None, None],
            cells: vec![],
            s_range: (0.0, 0.0),
            q: 0.0,
            roll: 0.0,
            playing: true,
            phase: 0.0,
            view: MechView::default(),
            drawn: None,
            opened: false,
        }
    }
}

fn is_ankle(st: &Setup) -> bool {
    matches!(st.mech, Mech::Ankle(_))
}

#[derive(Clone, Copy)]
enum U {
    Mm,
    Deg,
    Raw,
}

impl U {
    fn to_ui(self, v: f64) -> f64 {
        match self {
            U::Mm => v * 1000.0,
            U::Deg => v.to_degrees(),
            U::Raw => v,
        }
    }
    fn to_si(self, v: f64) -> f64 {
        match self {
            U::Mm => v / 1000.0,
            U::Deg => v.to_radians(),
            U::Raw => v,
        }
    }
}

type Field = (&'static str, &'static str, U);

const KNEE_F: [Field; 4] = [
    ("Base pivot a_x (mm)", "베이스 피벗 a_x (mm)", U::Mm),
    ("Base pivot a_y (mm)", "베이스 피벗 a_y (mm)", U::Mm),
    ("Shank lever r (mm)", "정강이 레버 r (mm)", U::Mm),
    ("Lever offset q0 (°)", "레버 오프셋 q0 (°)", U::Deg),
];
const FOUR_F: [Field; 9] = [
    ("Actuator base x (mm)", "액추에이터 베이스 x (mm)", U::Mm),
    ("Actuator base y (mm)", "액추에이터 베이스 y (mm)", U::Mm),
    ("Crank pivot x, ground offset (mm)", "크랭크 피벗 x, 그라운드 오프셋 (mm)", U::Mm),
    ("Crank pivot y, ground offset (mm)", "크랭크 피벗 y, 그라운드 오프셋 (mm)", U::Mm),
    ("Crank radius (mm)", "크랭크 반경 (mm)", U::Mm),
    ("Coupler length (mm)", "커플러 길이 (mm)", U::Mm),
    ("Shank lever radius (mm)", "정강이 레버 반경 (mm)", U::Mm),
    ("Lever offset q0 (°)", "레버 오프셋 q0 (°)", U::Deg),
    ("Assembly mode (+1 / −1)", "조립 모드 (+1 / −1)", U::Raw),
];
const ANKLE_F: [Field; 4] = [
    ("Lateral spacing w (mm)", "좌우 간격 w (mm)", U::Mm),
    ("Upper pivot height H (mm)", "상부 피벗 높이 H (mm)", U::Mm),
    ("Foot lever c (mm)", "발 레버 c (mm)", U::Mm),
    ("Upper pivot behind axis e (mm)", "상부 피벗 후방 거리 e (mm)", U::Mm),
];
const LOAD_F: [Field; 8] = [
    ("Range min (°)", "범위 최소 (°)", U::Deg),
    ("Range max (°)", "범위 최대 (°)", U::Deg),
    ("Constant torque (N·m)", "상수 토크 (N·m)", U::Raw),
    ("Gravity: segment mass (kg)", "중력: 분절 질량 (kg)", U::Raw),
    ("Gravity: COM distance (mm)", "중력: 무게중심 거리 (mm)", U::Mm),
    ("Gravity: angle offset (°)", "중력: 각도 오프셋 (°)", U::Deg),
    ("Target joint speed (°/s)", "목표 관절 속도 (°/s)", U::Deg),
    ("Workspace roll ± (°)", "작업영역 롤 ± (°)", U::Deg),
];

fn geo(m: &Mech) -> (&'static [Field], Vec<f64>) {
    match m {
        Mech::Knee(k) => (&KNEE_F, vec![k.ax, k.ay, k.r, k.q0]),
        Mech::FourBar(f) => (&FOUR_F, vec![f.bx, f.by, f.cx, f.cy, f.rc, f.lc, f.rs, f.q0, f.branch]),
        Mech::Ankle(a) => (&ANKLE_F, vec![a.w, a.h, a.c, a.e]),
    }
}

fn set_geo(m: &mut Mech, v: &[f64]) {
    let pos = |x: f64| x.abs().max(1e-3);
    match m {
        Mech::Knee(k) => {
            if let [ax, ay, r, q0] = *v {
                *k = mech::SliderCrank { ax, ay, r: pos(r), q0 };
            }
        }
        Mech::FourBar(f) => {
            if let [bx, by, cx, cy, rc, lc, rs, q0, br] = *v {
                *f = mech::FourBar { bx, by, cx, cy, rc: pos(rc), lc: pos(lc), rs: pos(rs), q0, branch: if br < 0.0 { -1.0 } else { 1.0 } };
            }
        }
        Mech::Ankle(a) => {
            if let [w, h, c, e] = *v {
                *a = Ankle { w: pos(w), h: pos(h), c, e };
            }
        }
    }
}

fn load_vals(m: &MechState) -> Vec<f64> {
    let (s, l) = (&m.setup, &m.setup.load);
    vec![s.q_min, s.q_max, l.tau_c, l.m_seg, l.l_com, l.offset, l.omega, m.roll_lim]
}

fn set_load(m: &mut MechState, v: &[f64]) {
    if let [q0, q1, tc, ms, lc, off, om, rl] = *v {
        let (lo, hi) = (q0.min(q1), q0.max(q1));
        m.setup.q_min = lo;
        m.setup.q_max = if hi - lo < 1e-3 { lo + 1e-3 } else { hi };
        m.setup.load = mech::Load { tau_c: tc, m_seg: ms.max(0.0), l_com: lc, offset: off, omega: om.abs().max(1e-3) };
        m.roll_lim = rl.abs().clamp(1f64.to_radians(), 60f64.to_radians());
    }
}

fn fields_html(ko: bool, fields: &[Field], vals: &[f64], attr: &str, skip_last: bool) -> String {
    let n = if skip_last { fields.len() - 1 } else { fields.len() };
    (0..n)
        .map(|i| {
            let (en, k, u) = fields[i];
            let v = (u.to_ui(vals[i]) * 1000.0).round() / 1000.0;
            format!("<label><span>{}</span><input type=\"number\" step=\"any\" {attr}=\"{i}\" value=\"{v}\"></label>", tr(ko, en, k))
        })
        .collect()
}

fn read_fields(container: &str, attr: &str, fields: &[Field], old: &[f64]) -> Vec<f64> {
    let mut v = old.to_vec();
    let Ok(list) = el(container).query_selector_all(&format!("[{attr}]")) else { return v };
    for i in 0..list.length() {
        let Some(e) = list.item(i).and_then(|n| n.dyn_into::<HtmlInputElement>().ok()) else { continue };
        let Some(k) = e.get_attribute(attr).and_then(|s| s.parse::<usize>().ok()) else { continue };
        let x = e.value_as_number();
        if k < v.len() && x.is_finite() {
            v[k] = fields[k].2.to_si(x);
        }
    }
    v
}

/// Rebuild the dimension and load inputs (on mechanism change and language toggle).
fn render_inputs(a: &App) {
    let (m, ko) = (&a.m, a.ko);
    select("m-preset").set_value(&m.preset.to_string());
    let kind = match m.setup.mech {
        Mech::Knee(_) => "0",
        Mech::FourBar(_) => "1",
        Mech::Ankle(_) => "2",
    };
    select("m-kind").set_value(kind);
    let (f, v) = geo(&m.setup.mech);
    el("m-dims").set_inner_html(&fields_html(ko, f, &v, "data-g", false));
    let ankle = is_ankle(&m.setup);
    let mut html = fields_html(ko, &LOAD_F, &load_vals(m), "data-l", !ankle);
    if ankle {
        html = html.replace(tr(ko, "Range min (°)", "범위 최소 (°)"), tr(ko, "Pitch min (°, + = toes down)", "피치 최소 (°, + = 발끝 내림)"));
        html = html.replace(tr(ko, "Range max (°)", "범위 최대 (°)"), tr(ko, "Pitch max (°)", "피치 최대 (°)"));
    }
    el("m-load").set_inner_html(&html);
    let _ = el("m-roll-wrap").unchecked_ref::<HtmlElement>().style().set_property("display", if ankle { "flex" } else { "none" });
    let _ = el("m-ws-card").unchecked_ref::<HtmlElement>().style().set_property("display", if ankle { "block" } else { "none" });
}

/// Recompute kinematics, couplings and the workspace, then redraw everything.
pub fn recompute(a: &mut App) {
    let m = &mut a.m;
    m.samples = sweep(&m.setup);
    let panel = Actuator::from_inputs(&a.inp);
    m.panel = Some(couple(&m.setup, &m.samples, &panel, &a.inp));
    for i in 0..4 {
        m.cmp[i] = m.on[i].then(|| couple(&m.setup, &m.samples, &m.cfgs[i], &a.inp));
    }
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for s in m.samples.iter().filter(|s| s.valid) {
        lo = lo.min(s.s);
        hi = hi.max(s.s);
    }
    m.cells.clear();
    if let Mech::Ankle(an) = &m.setup.mech {
        let f = m.panel.as_ref().map_or(0.0, |c| c.f_avail);
        m.cells = ankle_workspace(an, (m.setup.q_min, m.setup.q_max), (-m.roll_lim, m.roll_lim), WS, f);
        for c in &m.cells {
            for s in an.lengths(c.pitch, c.roll) {
                lo = lo.min(s);
                hi = hi.max(s);
            }
        }
    }
    m.s_range = if lo.is_finite() { (lo, hi) } else { (0.1, 0.1) };
    m.q = m.q.clamp(m.setup.q_min, m.setup.q_max);
    m.roll = m.roll.clamp(-m.roll_lim, m.roll_lim);
    render(a);
}

/// Static parts: summary table, comparison table, plots (and inputs when `inputs`).
pub fn render(a: &mut App) {
    render_results(a);
    render_compare(a);
    render_plots(a);
}

pub fn render_all(a: &mut App) {
    render_inputs(a);
    render(a);
}

fn deg(v: f64) -> String {
    format!("{:.0}°", v.to_degrees())
}

fn flag_text(ko: bool, ss: &[Sample]) -> String {
    let mut out = vec![];
    for (fl, en, k) in [
        (Flag::Unassembled, "loop cannot close (out of reach / toggle)", "링크 폐합 불가 (도달 불가 / 토글)"),
        (Flag::DeadPoint, "dead point (h ≈ 0)", "사점 (h ≈ 0)"),
        (Flag::LowAngle, "transmission angle < 20°", "전달각 < 20°"),
    ] {
        let qs: Vec<f64> = ss.iter().filter(|s| s.flag == fl).map(|s| s.q).collect();
        if let (Some(a), Some(b)) = (qs.first(), qs.last()) {
            out.push(format!("{} ({}…{})", tr(ko, en, k), deg(*a), deg(*b)));
        }
    }
    out.join("; ")
}

fn squat(ko: bool, c: &Coupled) -> String {
    c.squat_time.map_or(tr(ko, "cannot", "불가").to_string(), |t| format!("{t:.2} s"))
}

fn render_results(a: &App) {
    let (m, ko) = (&a.m, a.ko);
    let Some(c) = &m.panel else { return };
    let f = plot::fmt;
    let row = |label: &str, val: String, s: Option<Status>| {
        let (cl, t) = s.map_or(("", String::new()), |s| (cls(s), st(ko, s).to_string()));
        format!("<tr><td>{label}</td><td>{val}</td><td class=\"{cl}\">{t}</td></tr>")
    };
    let mu_min = m.samples.iter().filter(|s| s.valid).map(|s| s.mu_deg).fold(90.0, f64::min);
    let h = m.samples.iter().filter(|s| s.valid).map(|s| s.h_torque).fold((f64::INFINITY, 0.0f64), |(a, b), v| (a.min(v), b.max(v)));
    let flags = flag_text(ko, &m.samples);
    let rows = [
        row(tr(ko, "Actuator stroke", "액추에이터 스트로크"), format!("{} mm", f(c.stroke_mm)), None),
        row(tr(ko, "Screw length (stroke + nut)", "스크루 길이 (스트로크 + 너트)"), format!("{} mm", f(c.screw_len_mm)), None),
        row(tr(ko, "Moment arm min / max", "모멘트 암 최소 / 최대"), format!("{} / {} mm", f(h.0.min(1e9) * 1000.0), f(h.1 * 1000.0)), None),
        row(tr(ko, "Min transmission angle", "최소 전달각"), format!("{mu_min:.1}°"), Some(c.mech)),
        row(tr(ko, "Actuator force available", "가용 액추에이터 힘"), format!("{} N", f(c.f_avail)), None),
        row(tr(ko, "Linear speed limit", "선형 속도 한계"), format!("{} mm/s", f(c.v_lim * 1000.0)), None),
        row(tr(ko, "Min torque margin (at)", "최소 토크 여유 (위치)"), format!("×{:.2} @ {}", c.min_margin.min(999.0), deg(c.q_at_min)), Some(c.torque)),
        row(tr(ko, "Speed margin at target", "목표 속도 여유"), format!("×{:.2}", c.speed_margin.min(999.0)), Some(c.speed)),
        row(tr(ko, "Peak screw force", "최대 스크루 힘"), format!("{} N", f(c.f_peak.min(1e9))), None),
        row(tr(ko, "Cubic-mean force Fm", "등가 하중 Fm"), format!("{} N", f(c.fm.min(1e9))), None),
        row(tr(ko, "Buckling SF", "좌굴 SF"), format!("{:.1}", c.buckling_sf.min(999.0)), Some(c.buckling)),
        row(tr(ko, "Critical speed / DN", "위험 속도 / DN"), String::new(), Some(c.whirl.max(c.dn))),
        row(tr(ko, "L10 life", "L10 수명"), format!("{} y", years(c.l10_years)), Some(c.life)),
        row(tr(ko, "Back-drivable", "역구동"), (if c.back_drivable { tr(ko, "yes", "가능") } else { tr(ko, "no", "불가") }).into(), None),
        row(tr(ko, "Squat time 0→90° (range-clipped)", "스쿼트 시간 0→90° (범위 내)"), squat(ko, c), None),
        row(tr(ko, "Overall", "종합"), String::new(), Some(c.overall)),
    ];
    let mut html = rows.concat();
    if !flags.is_empty() {
        html.push_str(&format!("<tr><td colspan=\"3\" class=\"{}\">{flags}</td></tr>", cls(c.mech)));
    }
    el("m-results").set_inner_html(&html);
    let p = &a.inp;
    let kind = kind_name(ko, p.kind);
    el("m-panel").set_inner_html(&format!(
        "{}: <b>{kind} {}×{}</b>, {} N·m @ {} rpm, ×{} — {}",
        tr(ko, "Screw panel actuator", "스크루 패널 액추에이터"),
        p.d_mm, p.lead_mm, p.motor_torque, p.motor_rpm, p.ratio,
        tr(ko, "edit it in the Actuator tab or load a column below.", "액추에이터 탭에서 수정하거나 아래 열을 불러오세요.")
    ));
}

fn kind_name(ko: bool, k: Kind) -> &'static str {
    match k {
        Kind::Ball => tr(ko, "ball", "볼"),
        Kind::Roller => tr(ko, "roller", "롤러"),
        Kind::TrapBronze => tr(ko, "trap. bronze", "사다리꼴 청동"),
        Kind::TrapPom => tr(ko, "trap. POM", "사다리꼴 POM"),
    }
}

fn kind_opts(ko: bool, k: Kind) -> String {
    Kind::ALL.iter().enumerate().map(|(i, kk)| format!("<option value=\"{i}\"{}>{}</option>", if *kk == k { " selected" } else { "" }, kind_name(ko, *kk))).collect()
}

fn render_compare(a: &App) {
    let (m, ko) = (&a.m, a.ko);
    let f = plot::fmt;
    let panel = Actuator::from_inputs(&a.inp);
    let mut h = String::from("<thead><tr><th></th>");
    h.push_str(&format!("<th>{}</th>", tr(ko, "Panel", "패널")));
    for (i, c) in CFG_COLORS.iter().enumerate() {
        h.push_str(&format!("<th class=\"cfg\" data-load=\"{i}\" style=\"color:{c}\" title=\"{}\">#{} ⇢</th>", tr(ko, "Load into the screw panel", "스크루 패널로 불러오기"), i + 1));
    }
    h.push_str("</tr></thead><tbody>");
    let inp_row = |label: &str, panel_v: String, cell: &dyn Fn(usize, &Actuator) -> String| {
        let mut r = format!("<tr><td>{label}</td><td>{panel_v}</td>");
        for (i, c) in m.cfgs.iter().enumerate() {
            r.push_str(&format!("<td>{}</td>", cell(i, c)));
        }
        r + "</tr>"
    };
    let num = |i: usize, fld: &str, v: f64| format!("<input type=\"number\" step=\"any\" data-c=\"{i}\" data-f=\"{fld}\" value=\"{v}\">");
    h.push_str(&inp_row(tr(ko, "Compare", "비교"), "✓".into(), &|i, _| format!("<input type=\"checkbox\" data-c=\"{i}\" data-f=\"on\"{}>", if m.on[i] { " checked" } else { "" })));
    h.push_str(&inp_row(tr(ko, "Type", "종류"), kind_name(ko, panel.kind).into(), &|i, c| format!("<select data-c=\"{i}\" data-f=\"kind\">{}</select>", kind_opts(ko, c.kind))));
    h.push_str(&inp_row("d (mm)", f(panel.d_mm), &|i, c| num(i, "d", c.d_mm)));
    h.push_str(&inp_row(tr(ko, "Lead (mm)", "리드 (mm)"), f(panel.lead_mm), &|i, c| num(i, "lead", c.lead_mm)));
    h.push_str(&inp_row(tr(ko, "Motor N·m", "모터 N·m"), f(panel.motor_torque), &|i, c| num(i, "mt", c.motor_torque)));
    h.push_str(&inp_row(tr(ko, "Motor rpm", "모터 rpm"), f(panel.motor_rpm), &|i, c| num(i, "rpm", c.motor_rpm)));
    h.push_str(&inp_row(tr(ko, "Ratio", "감속비"), f(panel.ratio), &|i, c| num(i, "ratio", c.ratio)));
    let res: Vec<Option<&Coupled>> = std::iter::once(m.panel.as_ref()).chain(m.cmp.iter().map(|c| c.as_ref())).collect();
    let out_row = |label: &str, g: &dyn Fn(&Coupled) -> (String, Option<Status>)| {
        let mut r = format!("<tr><td>{label}</td>");
        for c in &res {
            match c {
                Some(c) => {
                    let (t, s) = g(c);
                    r.push_str(&format!("<td class=\"{}\">{t}</td>", s.map_or("", cls)));
                }
                None => r.push_str("<td>—</td>"),
            }
        }
        r + "</tr>"
    };
    h.push_str(&out_row(tr(ko, "Force available (N)", "가용 힘 (N)"), &|c| (f(c.f_avail), None)));
    h.push_str(&out_row(tr(ko, "Min torque margin", "최소 토크 여유"), &|c| (format!("×{:.2} @ {}", c.min_margin.min(999.0), deg(c.q_at_min)), Some(c.torque))));
    h.push_str(&out_row(tr(ko, "Speed margin", "속도 여유"), &|c| (format!("×{:.2}", c.speed_margin.min(999.0)), Some(c.speed))));
    h.push_str(&out_row(tr(ko, "Peak screw force (N)", "최대 스크루 힘 (N)"), &|c| (f(c.f_peak.min(1e9)), None)));
    h.push_str(&out_row(tr(ko, "Buckling SF", "좌굴 SF"), &|c| (format!("{:.1}", c.buckling_sf.min(999.0)), Some(c.buckling))));
    h.push_str(&out_row(tr(ko, "L10 (years)", "L10 (년)"), &|c| (years(c.l10_years), Some(c.life))));
    h.push_str(&out_row(tr(ko, "Back-drivable", "역구동"), &|c| ((if c.back_drivable { tr(ko, "yes", "가능") } else { tr(ko, "no", "불가") }).into(), None)));
    h.push_str(&out_row(tr(ko, "Squat time 0→90°", "스쿼트 시간 0→90°"), &|c| (squat(ko, c), None)));
    h.push_str(&out_row(tr(ko, "Overall", "종합"), &|c| (st(ko, c.overall).into(), Some(c.overall))));
    h.push_str("<tr><td></td><td></td>");
    for i in 0..4 {
        h.push_str(&format!("<td><button data-load=\"{i}\">{}</button></td>", tr(ko, "Load", "불러오기")));
    }
    h.push_str("</tr></tbody>");
    el("m-cmp").set_inner_html(&h);
}

fn canvas(id: &str) -> HtmlCanvasElement {
    el(id).unchecked_into()
}

fn render_plots(a: &mut App) {
    let colors = if a.dark { &plot::DARK } else { &plot::LIGHT };
    let (m, ko) = (&a.m, a.ko);
    let xs: Vec<f64> = m.samples.iter().map(|s| s.q.to_degrees()).collect();
    let cur = Some(m.q.to_degrees());
    let fg = colors.fg;
    let res: Vec<(Option<&Coupled>, &str, f64, String)> = std::iter::once((m.panel.as_ref(), fg, 2.6, tr(ko, "panel", "패널").to_string()))
        .chain(m.cmp.iter().enumerate().map(|(i, c)| (c.as_ref(), CFG_COLORS[i], 1.5, format!("#{}", i + 1))))
        .collect();
    let mut lines = vec![plot::Series { y: m.samples.iter().map(|s| m.setup.load.tau_req(s.q).abs()).collect(), color: C_REQ, width: 2.0, label: tr(ko, "required |τ_req|", "요구 |τ_req|").into() }];
    for (c, col, w, l) in &res {
        if let Some(c) = c {
            lines.push(plot::Series { y: c.pts.iter().map(|p| p.tau_av).collect(), color: col, width: *w, label: format!("τ_av {l}") });
        }
    }
    let xl = if is_ankle(&m.setup) { tr(ko, "pitch ° · N·m", "피치 ° · N·m") } else { tr(ko, "q ° · N·m", "q ° · N·m") };
    plot::series(&canvas("p-mtau"), &xs, &lines, cur, None, colors, xl);

    let mut lines = vec![];
    for (c, col, w, l) in &res {
        if let Some(c) = c {
            lines.push(plot::Series { y: c.pts.iter().map(|p| p.omega_max.to_degrees()).collect(), color: col, width: *w, label: format!("ω_max {l}") });
        }
    }
    let target = m.setup.load.omega.to_degrees();
    lines.push(plot::Series { y: vec![], color: C_REQ, width: 1.0, label: format!("{} {:.0}°/s", tr(ko, "target (dashed)", "목표 (점선)"), target) });
    // Cap the axis at a few times the target so one huge value does not flatten the plot.
    for l in lines.iter_mut() {
        for v in l.y.iter_mut() {
            *v = v.min(4.0 * target);
        }
    }
    plot::series(&canvas("p-momega"), &xs, &lines, cur, Some((target, C_REQ)), colors, "° · °/s");

    let lines = vec![
        plot::Series { y: m.samples.iter().map(|s| if s.valid { s.h_torque * 1000.0 } else { f64::NAN }).collect(), color: CFG_COLORS[0], width: 2.0, label: tr(ko, "moment arm h (mm)", "모멘트 암 h (mm)").into() },
        plot::Series { y: m.samples.iter().map(|s| if s.valid { s.mu_deg } else { f64::NAN }).collect(), color: CFG_COLORS[2], width: 2.0, label: tr(ko, "transmission angle μ (°), dashed 20°", "전달각 μ (°), 점선 20°").into() },
    ];
    plot::series(&canvas("p-mh"), &xs, &lines, cur, Some((mech::MU_WARN, C_REQ)), colors, "° · mm · °");

    if is_ankle(&m.setup) && !m.cells.is_empty() {
        let vmax = m.cells.iter().map(|c| c.tau_min).fold(0.0, f64::max);
        let cells: Vec<(f64, f64, f64, bool)> = m.cells.iter().map(|c| (c.pitch.to_degrees(), c.roll.to_degrees(), c.tau_min, c.singular)).collect();
        let l1 = format!("{} 0…{} N·m", tr(ko, "min(pitch, roll) torque, red → green", "최소(피치, 롤) 토크, 빨강 → 초록"), plot::fmt(vmax));
        plot::heatmap(
            &canvas("p-mws"),
            &cells,
            WS,
            (m.setup.q_min.to_degrees(), m.setup.q_max.to_degrees()),
            (-m.roll_lim.to_degrees(), m.roll_lim.to_degrees()),
            vmax,
            (m.q.to_degrees(), m.roll.to_degrees()),
            colors,
            [tr(ko, "pitch ° · roll °", "피치 ° · 롤 °"), &l1, tr(ko, "grey ╱ = singular (|det J| < 10 %)", "회색 ╱ = 특이 (|det J| < 10 %)")],
        );
    }
    a.m.drawn = Some((a.m.q, a.m.roll));
}

/// Forces both actuators need at the pose to hold the required torque (ankle: pure pitch torque).
fn forces_needed(st: &Setup, q: f64, roll: f64) -> Option<[f64; 2]> {
    let tau = st.load.tau_req(q);
    match st.mech {
        Mech::Ankle(a) => {
            let j = a.jacobian(q, roll);
            let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
            if det.abs() < 1e-12 {
                return None;
            }
            // F = J⁻ᵀ·(τ, 0).
            Some([j[1][1] * tau / det, -j[0][1] * tau / det])
        }
        _ => {
            let s = st.mech.sample(q);
            (s.valid && s.h_torque > 0.0).then(|| [tau.abs() / s.h_torque; 2])
        }
    }
}

fn hud(a: &App) -> (String, bool) {
    let (m, ko) = (&a.m, a.ko);
    let f = plot::fmt;
    let fav = m.panel.as_ref().map_or(0.0, |c| c.f_avail);
    let s = m.setup.mech.sample(m.q);
    let need = forces_needed(&m.setup, m.q, m.roll);
    let fmax = need.map_or(f64::INFINITY, |n| n[0].abs().max(n[1].abs()));
    let over = fmax > fav;
    let tau_req = m.setup.load.tau_req(m.q);
    let status = if !s.valid || over { Status::Fail } else { s.status };
    let mut out = String::new();
    if let Mech::Ankle(an) = &m.setup.mech {
        let l = an.lengths(m.q, m.roll);
        let n = need.unwrap_or([f64::INFINITY; 2]);
        out.push_str(&format!(
            "{}: <b>{:.1}°</b> {}: <b>{:.1}°</b><br>s1 / s2: <b>{:.1} / {:.1} mm</b><br>F1 / F2 {}: <b>{} / {} N</b><br>",
            tr(ko, "Pitch", "피치"), m.q.to_degrees(), tr(ko, "roll", "롤"), m.roll.to_degrees(),
            l[0] * 1000.0, l[1] * 1000.0, tr(ko, "needed", "필요"), f(n[0].min(1e9)), f(n[1].min(1e9)),
        ));
    } else {
        out.push_str(&format!(
            "q: <b>{:.1}°</b><br>s: <b>{:.1} mm</b><br>{}: <b>{} N</b><br>",
            m.q.to_degrees(), s.s * 1000.0, tr(ko, "Force needed", "필요 힘"), f(fmax.min(1e9)),
        ));
    }
    let omega = if s.h_speed > 0.0 { m.panel.as_ref().map_or(0.0, |c| c.v_lim / s.h_speed).to_degrees() } else { f64::INFINITY };
    out.push_str(&format!(
        "{}: <b>{} N</b><br>h: <b>{:.1} mm</b> μ: <b>{:.0}°</b><br>τ_req / τ_av: <b>{} / {} N·m</b><br>ω_max: <b>{} °/s</b><br>{}: <b class=\"{}\">{}</b>",
        tr(ko, "Force available", "가용 힘"), f(fav),
        s.h_torque * 1000.0, s.mu_deg,
        f(tau_req.abs()), f(fav * s.h_torque),
        f(omega.min(1e6)),
        tr(ko, "Here", "현재"), cls(status), st(ko, status),
    ));
    (out, over)
}

fn set_slider(a: &App) {
    let m = &a.m;
    let span = (m.setup.q_max - m.setup.q_min).max(1e-9);
    input("m-q").set_value(&format!("{}", ((m.q - m.setup.q_min) / span * 1000.0).round()));
    input("m-roll").set_value(&format!("{}", (m.roll / m.roll_lim * 1000.0).round()));
}

pub fn set_play_label(a: &App) {
    el("m-play").set_text_content(Some(if a.m.playing { tr(a.ko, "Pause", "일시정지") } else { tr(a.ko, "Play", "재생") }));
    el("m-play").class_list().toggle_with_force("on", a.m.playing).ok();
}

pub fn frame(a: &mut App, dt: f64) {
    if a.m.playing {
        let m = &mut a.m;
        m.phase += dt * a.slow * 1.2;
        let u = 0.5 - 0.5 * m.phase.cos();
        m.q = m.setup.q_min + (m.setup.q_max - m.setup.q_min) * u;
        m.roll = if is_ankle(&m.setup) { m.roll_lim * 0.8 * (0.5 * m.phase).sin() } else { 0.0 };
        set_slider(a);
    }
    let (h, over) = hud(a);
    el("hud").set_inner_html(&h);
    el("m-qlabel").set_text_content(Some(&if is_ankle(&a.m.setup) {
        format!("{} {:.1}° · {} {:.1}°", tr(a.ko, "pitch", "피치"), a.m.q.to_degrees(), tr(a.ko, "roll", "롤"), a.m.roll.to_degrees())
    } else {
        format!("q {:.1}°", a.m.q.to_degrees())
    }));
    if a.m.drawn != Some((a.m.q, a.m.roll)) {
        render_plots(a);
    }
    let Some(sc) = a.scene.as_mut() else { return };
    let c: HtmlCanvasElement = el("view").unchecked_into();
    let dpr = web_sys::window().unwrap().device_pixel_ratio();
    let (w, h) = ((c.client_width() as f64 * dpr) as u32, (c.client_height() as f64 * dpr) as u32);
    if w == 0 || h == 0 {
        return;
    }
    if c.width() != w || c.height() != h {
        c.set_width(w);
        c.set_height(h);
    }
    let i = &a.inp;
    let draw = ActDraw {
        d: i.d_mm as f32 / 1000.0,
        lead: i.lead_mm as f32 / 1000.0,
        dr: root_diameter(i.d_mm, i.lead_mm) as f32 / 1000.0,
        s_min: a.m.s_range.0 as f32,
        stroke: (a.m.s_range.1 - a.m.s_range.0) as f32,
        over,
    };
    a.m.view.frame(sc, w as i32, h as i32, dt as f32, &a.m.setup, a.m.q, a.m.roll, &draw);
}

/// Apply a drag from the 3D view.
pub fn drag(a: &mut App, act: MAction) {
    let m = &mut a.m;
    match act {
        MAction::Q(q) => {
            // Shortest angular jump from the current pose, then clamp to the range.
            let mut q = q;
            while q - m.q > std::f64::consts::PI {
                q -= std::f64::consts::TAU;
            }
            while m.q - q > std::f64::consts::PI {
                q += std::f64::consts::TAU;
            }
            m.q = q.clamp(m.setup.q_min, m.setup.q_max);
        }
        MAction::PitchRoll(dp, dr) => {
            m.q = (m.q + dp).clamp(m.setup.q_min, m.setup.q_max);
            m.roll = (m.roll + dr).clamp(-m.roll_lim, m.roll_lim);
        }
    }
    m.playing = false;
    set_play_label(a);
    set_slider(a);
}

/// Load preset `i`: geometry, load case, its actuator into config #1 and into the screw panel.
pub fn apply_preset(a: &mut App, i: usize) {
    let (setup, act) = preset(i);
    a.m.preset = i;
    a.m.setup = setup;
    a.m.cfgs[0] = act;
    a.m.on[0] = true;
    // The preset's actuator becomes the screw-panel actuator.
    a.inp = act.apply(&a.inp);
    crate::write_inputs(&a.inp);
    crate::recompute(a);
    a.m.q = 0.0f64.clamp(a.m.setup.q_min, a.m.setup.q_max);
    a.m.roll = 0.0;
    a.m.view.reset(&a.m.setup);
}

/// Load comparison config `i` into the screw panel.
fn load_cfg(a: &mut App, i: usize) {
    let c = a.m.cfgs[i];
    a.inp = c.apply(&a.inp);
    crate::write_inputs(&a.inp);
    crate::recompute(a);
    recompute(a);
}

fn target_el(e: &web_sys::Event) -> Option<Element> {
    e.target().and_then(|t| t.dyn_into::<Element>().ok())
}

pub fn wire() {
    crate::on_id("m-preset", "change", |_| {
        with(|a| {
            let i = select("m-preset").value().parse::<usize>().unwrap_or(0).min(mech::PRESETS - 1);
            apply_preset(a, i);
            render_inputs(a);
            recompute(a);
        });
    });
    crate::on_id("m-kind", "change", |_| {
        with(|a| {
            // Start each mechanism type from the preset of that type.
            let i = match select("m-kind").value().as_str() {
                "1" => 3,
                "2" => 1,
                _ => 0,
            };
            apply_preset(a, i);
            render_inputs(a);
            recompute(a);
        });
    });
    crate::on_id("m-dims", "change", |_| {
        with(|a| {
            let (f, old) = geo(&a.m.setup.mech);
            let v = read_fields("m-dims", "data-g", f, &old);
            set_geo(&mut a.m.setup.mech, &v);
            recompute(a);
        });
    });
    crate::on_id("m-load", "change", |_| {
        with(|a| {
            let old = load_vals(&a.m);
            let v = read_fields("m-load", "data-l", &LOAD_F, &old);
            set_load(&mut a.m, &v);
            recompute(a);
        });
    });
    crate::on_id("m-cmp", "change", |e| {
        let Some(t) = target_el(&e) else { return };
        let (Some(i), Some(fld)) = (t.get_attribute("data-c").and_then(|s| s.parse::<usize>().ok()), t.get_attribute("data-f")) else { return };
        with(|a| {
            if i >= 4 {
                return;
            }
            let c = &mut a.m.cfgs[i];
            let v = t.dyn_ref::<HtmlInputElement>().map(|x| x.value_as_number()).filter(|v| v.is_finite());
            match (fld.as_str(), v) {
                ("on", _) => a.m.on[i] = t.dyn_ref::<HtmlInputElement>().is_some_and(|x| x.checked()),
                ("kind", _) => {
                    let k = t.dyn_ref::<HtmlSelectElement>().and_then(|s| s.value().parse::<usize>().ok()).unwrap_or(0).min(3);
                    c.kind = Kind::ALL[k];
                    c.ca_kn = c.kind.preset().default_ca_kn(c.d_mm);
                }
                ("d", Some(v)) => {
                    c.d_mm = v.clamp(6.0, 40.0);
                    c.ca_kn = c.kind.preset().default_ca_kn(c.d_mm);
                }
                ("lead", Some(v)) => c.lead_mm = v.clamp(1.0, 40.0),
                ("mt", Some(v)) => c.motor_torque = v.clamp(0.001, 1e3),
                ("rpm", Some(v)) => c.motor_rpm = v.clamp(1.0, 1e5),
                ("ratio", Some(v)) => c.ratio = v.clamp(0.1, 20.0),
                _ => {}
            }
            recompute(a);
        });
    });
    crate::on_id("m-cmp", "click", |e| {
        let Some(t) = target_el(&e).and_then(|t| t.closest("[data-load]").ok().flatten()) else { return };
        let Some(i) = t.get_attribute("data-load").and_then(|s| s.parse::<usize>().ok()) else { return };
        with(|a| {
            if i < 4 {
                load_cfg(a, i);
            }
        });
    });
    for id in ["m-q", "m-roll"] {
        crate::on_id(id, "input", move |_| {
            with(|a| {
                let m = &mut a.m;
                if id == "m-q" {
                    m.q = m.setup.q_min + (m.setup.q_max - m.setup.q_min) * crate::num("m-q", 0.0) / 1000.0;
                } else {
                    m.roll = m.roll_lim * crate::num("m-roll", 0.0) / 1000.0;
                }
                m.playing = false;
                set_play_label(a);
            });
        });
    }
    crate::on_id("m-play", "click", |_| {
        with(|a| {
            a.m.playing = !a.m.playing;
            if a.m.playing {
                // Resume the sweep from the current pose.
                let m = &mut a.m;
                let u = ((m.q - m.setup.q_min) / (m.setup.q_max - m.setup.q_min).max(1e-9)).clamp(0.0, 1.0);
                m.phase = (1.0 - 2.0 * u).acos();
            }
            set_play_label(a);
        });
    });
}
