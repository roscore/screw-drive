# Task: ball / roller / lead screw linear actuator sizer for robots (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook methods only (screw torque–thrust relations with lead angle and friction, efficiency
and back-driving condition, Euler column buckling with end-fixity factors, shaft critical (whirling)
speed with end-support factors, ball/roller screw basic rating life L10 = (Ca/Fm)^3 · 10^6 rev with a
cubic-mean load over a duty cycle). Material/friction values are generic and labelled "typical" (no
vendor names, no copied catalogue tables).

## Product (SMALL scope — core + UI about 800-1200 lines of new Rust; renderer/math are reused)
A single-page web app (GitHub Pages, sub-path `/screw-drive/`): "I want a linear actuator for my
robot's knee / ankle / linear joint — which screw lead and diameter, how much motor torque and speed,
will it buckle, whirl, or wear out, and can it be back-driven?"

1. **Inputs**: screw type preset: "ball screw (typical)", "planetary roller screw (typical)",
   "trapezoidal lead screw, bronze nut (typical)", "trapezoidal lead screw, POM nut (typical)" — each
   with friction μ or efficiency, default dynamic load rating per mm² of nominal diameter (own simple
   generic estimate, editable as Ca in kN), max DN value; nominal diameter d (mm, 6-40), lead l (mm,
   1-40), screw length between supports L (mm), end-support preset (fixed-free, fixed-simple,
   simple-simple, fixed-fixed) used for both buckling and critical-speed factors; motor: torque (N·m),
   max speed (rpm), optional belt/gear ratio between motor and screw; load: moving mass (kg), axis
   angle (0° horizontal … 90° vertical), external force (N), stroke (mm), target max speed (mm/s),
   acceleration (m/s²); duty cycle: cycles per day and years.
2. **Model (core)**: lead angle λ = atan(l / (π·d)); efficiency forward η = tanλ / tan(λ+φ) (or preset
   value for ball/roller), back-drive efficiency η' = tan(λ−φ)/tanλ (≤ 0 → self-locking flag);
   required thrust per phase (accel / constant / decel, incl. gravity component and friction of the
   guide μg default 0.01); torque T = F·l / (2π·η) plus inertia torque (screw inertia as solid
   cylinder steel 7850 kg/m³ + reflected load m·(l/2π)²); motor speed needed n = v / l; available thrust
   from motor torque; cubic-mean load Fm over the trapezoidal move; L10 life in revolutions, km and
   years at the duty cycle; Euler buckling load P = k·π²·E·I / L² (root diameter ≈ d − 0.6·l capped,
   own documented estimate; E 206 GPa) with safety factor; critical speed n_c = f·(d_r/L²)·1e7 style
   formula derived from first principles (λ² / (2π) · sqrt(E·I/(ρ·A)) / L²), shown with 0.8 margin; DN
   = d·n check; motor-limited max linear speed and force; trapezoidal move time for the stroke.
   Status per check (OK / warning / fail) and an overall status.
3. **Extras (small)**: (a) "find screws" search: over d ∈ {8,10,12,14,16,20,25,32} and lead ∈
   {1,2,4,5,8,10,12,16,20,25,32} for the chosen type, keep those passing all checks with the given
   motor, list top 12 sorted by (min safety factor ≥ 1.5) then motor torque margin, click a row to load;
   (b) 2D plot of force–speed envelope (motor-limited thrust vs linear speed, critical-speed and DN
   speed limits as vertical lines, the required operating points marked).
4. **3D view (WebGL2, game-grade — see below)**: a real 3D actuator: helical-thread screw shaft
   (visible helix ridges at the actual lead, scaled), nut block that travels along the screw, end
   supports drawn per the end-support preset (fixed = double bearing block, simple = single), motor
   can at one end with optional belt pulleys when ratio ≠ 1, a carriage/payload box on the nut, axis
   tilted by the axis angle. Animated: plays the trapezoidal move back and forth with the screw
   rotating at the correct rev count per mm of travel, play/pause, slow motion (default ~0.25×), time
   slider. Colouring toggle: screw tinted by the worst check (green→yellow→red), whirl-mode ghost
   (exaggerated first-mode shape) when speed > 80 % of critical. Arrows: thrust and gravity vectors.
5. **2D plots** (canvas 2D, own small module): force–speed envelope above, and a position/velocity/
   thrust vs time plot of the move with a cursor at the current animation time.
6. Korean + English toggle (browser default). Mobile friendly. CSS prefers-color-scheme only.
Out of scope (do NOT build): preload/backlash analysis, thermal expansion, nut stiffness, multi-axis,
URL share, JSON/CSV import/export, STL export.

## 3D view requirements (game-like, reuse the existing renderer)
- `crates/sd_web/src/gl.rs` and `crates/sd_web/src/math.rs` already exist: a tool-independent WebGL2
  renderer (lit meshes, checker/grid ground with fog, sky gradient, stencil planar shadows, translucent
  volumes, lines, `prism()` extrusion) and pure math (mat4, damped orbit camera with inertia,
  ray-plane/ray-sphere picking) with unit tests. Reuse them; extend only if needed and keep them
  tool-independent and tested.
- Scene code in `view3d.rs`. Orbit (left drag) / pan (right drag, two fingers) / zoom (wheel, pinch)
  with inertia, double-click reset, `touch-action:none`. View presets: Perspective / Top / Side /
  Nut-follow (camera rides beside the nut).
- Direct manipulation: dragging the carriage along the axis scrubs the time / position; dragging the
  far end support along the axis changes the screw length L; hover highlight + cursor change.
- Toggle buttons over the view: check colouring, force arrows, whirl ghost, play/pause.
- HUD over the canvas: thrust N, motor torque N·m, motor rpm, linear speed mm/s, L10 life (years),
  buckling SF, critical-speed margin, back-drivable yes/no, overall status.
- If WebGL2 is unavailable, show a message and keep the numeric readouts and 2D plots working.
- Release wasm ≲ 350 KB. No crates besides wasm-bindgen, web-sys, js-sys.

## Core crate (pure Rust, no web deps, unit tested) — `crates/sd_core`
- Tests (hand-checkable, 6-10): lead angle for d=16, l=5 ≈ 5.68°; trapezoidal screw with μ=0.1,
  d=10, l=2 is self-locking, ball screw with η 0.9 is back-drivable; torque for F=1000 N, l=10 mm,
  η=1 equals 1.5915 N·m; L10 = 10^6 rev when Fm = Ca; doubling Fm divides life by 8; Euler buckling
  scales with 1/L² and fixed-fixed = 4× simple-simple; critical speed scales with 1/L²; cubic-mean of
  a constant load equals that load; search results all pass every check and are sorted.
- `crates/sd_web`: UI, plain web-sys. The existing math.rs/gl.rs tests must keep passing.

## Repo hygiene
- README (Korean first, then English): what it is, model and assumptions (rigid screw, Euler buckling,
  first whirling mode, L10 cubic law, no preload, generic typical values), controls, local build
  (`cargo test`, `trunk build --release`), MIT license, "educational estimates — validate with the
  screw maker's data and real tests before trusting a robot with it".
- LICENSE: MIT, copyright "screw-drive contributors" (exists). `.gitignore` for target/ dist/ (exists).
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  `Trunk.toml` exists (`filehash = false`, `public_url = "/screw-drive/"`, external `web/boot.js`
  copied via `<link data-trunk rel="copy-file" href="web/boot.js">`, which imports `./sd_web.js`).
- Root `Cargo.toml` workspace with members `crates/sd_core`, `crates/sd_web` (exists).
- `.github/workflows/pages.yml` exists (trunk 0.22.1, privacy check, cargo test, deploy-pages). Keep it.
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test` green (workspace); `trunk build --release` succeeds with no warnings;
  `sh tools/privacy_check.sh` clean.
- Commit author `screw-drive contributors <59640857+roscore@users.noreply.github.com>`.
- Commit everything with clear commits; open a PR. Do NOT create new GitHub repos or change repo settings.
