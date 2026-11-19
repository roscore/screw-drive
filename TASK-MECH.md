# Task: joint mechanisms on top of screw-drive (knee / ankle linkages driven by linear actuators)

Extend the existing screw-drive app. Keep everything already there working (all 21 tests stay green).
Original code only, textbook kinematics/statics. No vendor tables, no copied assets.

## Goal
"My humanoid's knee or ankle is driven by a linear actuator through a linkage (the way recent humanoids
do it). Over the joint's whole range, what joint torque and speed do I get, and which screw / lead /
motor makes it work?" The user picks a mechanism, then compares several screw actuators on it.

## Mechanisms (new crate module `sd_core::mech`, pure Rust, unit-tested)
All planar, angles in degrees in the UI, SI internally. Each mechanism maps actuator length s ↔ joint
angle q and gives the transmission ratio J(q) = ds/dq (m/rad) analytically or by central difference
with a test that both agree.
1. **Knee: actuator + crank (slider-crank)**. Actuator base pivot on the thigh at (a_x, a_y) from the
   knee axis, rod end on a shank lever of radius r at angle offset q0. s = |P_base − P_rod(q)|.
   Inputs: a_x, a_y, r, q0, knee range (default 0…120°). Moment arm h(q) = |J(q)|.
2. **Knee: actuator + four-bar** (rod drives a coupler link that drives the shank): base pivot, crank
   radius, coupler length, shank lever radius, ground offset. Solve with closed-form circle intersection;
   flag assembly-mode flips / singular positions (transmission angle < 20° → warning, < 10° → fail).
3. **Ankle: two parallel linear actuators** (dual-rod ankle, pitch + roll). Two actuators from the
   shank to the foot plate at ±w/2 lateral, attachment height H above the ankle axis, foot lever c.
   Pitch/roll (p, r) → (s1, s2) by vector loop; Jacobian 2×2; joint torques τ = Jᵀ·F. Show the
   pitch–roll workspace (grid) colored by min available torque, and flag singular cells (|det J| small).
Each mechanism has an editable dimension set. Presets (all approximate, clearly labelled
"approx., for comparison — edit to your design", no claim of exact OEM numbers):
- "Optimus-style knee (slider-crank, roller screw)" and "Optimus-style dual-rod ankle": geometry is
  your own plausible estimate sized for a ~1.7 m / ~60 kg humanoid; actuator force classes like
  publicly presented ones (≈ 500 N / 3.9 kN / 8 kN linear actuators) may be used as force presets.
- "Generic dual-rod ankle (humanoid)" and "Generic four-bar knee".
Do not name or state specs for any other company's robot.

## Coupling with the screw model
- For q over the range (≥ 121 samples) and for the actuator chosen in the existing screw panel:
  actuator max force F_max(v) from the existing motor/screw model (force–speed envelope) →
  joint torque available τ_av(q) = F·h(q), joint speed limit ω(q) = v_max/h(q).
- Load case: required joint torque profile τ_req(q) = editable constant + gravity term
  m_seg·g·L_com·cos(q + offset) (defaults for a squat: body mass share on one knee) and a target joint
  speed (deg/s). Required actuator force = τ_req/h, which feeds buckling / L10 / DN checks of the screw.
- **Compare mode**: up to 4 actuator configs (type, d, lead, motor) side by side on the same mechanism:
  plot τ_av(q) curves vs τ_req(q), ω_max(q), and a table (min torque margin and where, peak screw force,
  buckling SF, L10 years, back-drivable yes/no, squat time for 0→90°). Click a column to load it into
  the main screw panel.

## UI
- New tab "Mechanism" next to the current view; existing tabs unchanged. Korean / English as today.
- 3D (reuse the existing WebGL2 renderer and look): thigh/shank/foot as shaded boxes, the actuator as
  the existing animated screw + nut body placed between its pivots, rod and links. A slider and drag on
  the shank (or foot) set q; the screw rotates and the nut slides consistently with s(q). Color the
  actuator red when the required force exceeds the available force at that q. Ankle: drag the foot to
  set pitch/roll, both actuators move.
- 2D plots (reuse plot.rs): τ_av vs τ_req over q, moment arm h(q), transmission angle; ankle workspace.

## Tests (add, keep existing)
- slider-crank: s(q) vs J(q) finite-difference agreement; h at the perpendicular pose equals r.
- four-bar: loop closure error < 1e-9 over the range; transmission-angle flagging.
- ankle: pure pitch with symmetric geometry gives s1 = s2; Jacobian vs finite difference; τ = Jᵀ F
  equals virtual-work power balance.
- coupling: τ_av = F·h for a constant-force actuator.

## Delivery
Branch `mech`, one PR to main, title "Mechanism mode: knee/ankle linkages with actuator comparison".
README (ko/en): add a section with a short how-to and the "approximate presets" disclaimer.
Run cargo test for the workspace and a release trunk build before opening the PR.
