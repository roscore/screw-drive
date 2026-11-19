# screw-drive

**한국어** · [English](#english)

로봇의 무릎 / 발목 / 직동 관절용 볼 스크루 · 유성 롤러 스크루 · 사다리꼴 리드 스크루 액추에이터를 고르는
브라우저 도구입니다. "어떤 리드와 지름이 필요한가, 모터 토크와 속도는 얼마인가, 좌굴·휘돌림·마모는 괜찮은가,
역구동이 되는가?"에 답합니다. Rust → WebAssembly로 만든 정적 페이지이며 100 % 클라이언트에서 동작합니다
(분석·추적·외부 요청 없음).

### 기능
- 입력: 스크루 종류 프리셋(일반값), 호칭경 d, 리드 l, 지지 간 길이 L, 지지 방식, 동정격하중 Ca, 모터 토크·최대 속도·감속비,
  이동 질량, 축 각도, 외력, 스트로크, 목표 속도, 가속도, 일일 사이클·목표 수명.
- 결과: 리드각, 정/역 효율과 셀프 록킹 여부, 구간별 추력, 모터 토크·회전수, 좌굴 안전율, 위험 속도 여유, DN, L10 수명(회전·km·년),
  체크별 상태(OK / 주의 / 불가)와 종합 판정.
- 스크루 찾기: 선택한 종류로 d × l 조합을 검색해 상위 12개를 표시, 클릭하면 적용.
- 그래프: 힘–속도 영역(모터 한계, 0.8 × 위험 속도, DN 한계, 운전점), 위치·속도·추력 vs 시간(커서 포함).
- 3D 뷰(WebGL2): 나선 나사산, 이동 너트·캐리지, 지지 방식별 베어링 블록, 모터(감속비 ≠ 1이면 벨트 풀리), 기울어진 축,
  판정 색, 힘 화살표, 휘돌림 모드 고스트, 재생 / 일시정지 / 슬로 모션 / 시간 슬라이더. WebGL2가 없어도 수치와 그래프는 동작합니다.

### 모델과 가정
- 리드각 λ = atan(l / πd). 정효율 η = tanλ / tan(λ+φ), 역효율 η' = tan(λ−φ) / tanλ (≤ 0이면 셀프 록킹).
  사다리꼴: φ = atan(μ / cos 15°). 볼·롤러: 프리셋 η에서 φ를 역산.
- 추력 = m·a + m·g·sinθ + 외력 + 가이드 마찰(μg = 0.01). 토크 T = F·l / (2πη) + (J_screw + m(l/2π)² / η)·α.
- 사다리꼴(가속–등속–감속) 왕복, 짧은 스트로크는 삼각형 프로파일. 등가 하중 Fm은 회전수 가중 3제곱 평균.
- L10 = (Ca / Fm)³ · 10⁶ 회전. 예압 없음. Ca 기본값은 d²에 비례하는 자체 단순 추정치(카탈로그 값 아님).
- 오일러 좌굴 P = k·π²·E·I / L², 골지름 d_r ≈ d − min(0.6·l, 0.25·d), E = 206 GPa, k = 0.25 / 2.046 / 1 / 4.
- 1차 휘돌림 속도 ω = β² · √(E·I / ρ·A) / L², β = 1.875 / 3.927 / π / 4.730, 0.8 여유 적용.
- 강체 스크루, 열팽창·너트 강성·백래시 미고려. 모든 재료·마찰값은 일반적인 "typical" 값입니다.
- 3D의 나사산은 L / l이 120회전을 넘으면 리드를 늘려서(스케일) 그리며, 휘돌림 모드 형상은 시각용 근사입니다.

### 조작
- 드래그: 회전 · 우클릭 드래그 / 두 손가락: 이동 · 휠 / 핀치: 확대 · 더블클릭: 시점 초기화
- 캐리지를 축 방향으로 끌면 시간 이동, 끝 지지부를 끌면 L 변경. 시점 프리셋: 원근 / 위 / 옆 / 너트 추적.

### 로컬 빌드
```sh
rustup target add wasm32-unknown-unknown
cargo test
trunk build --release   # dist/ 생성, public_url = /screw-drive/
sh tools/privacy_check.sh
```

### 라이선스
MIT — `LICENSE` 참고.

> **교육용 추정치입니다 — 로봇에 적용하기 전에 스크루 제조사 데이터와 실제 시험으로 검증하세요.**

---

<a id="english"></a>
## English

A browser tool to size ball screw, planetary roller screw and trapezoidal lead screw actuators for a robot's
knee / ankle / linear joint: "which lead and diameter, how much motor torque and speed, will it buckle, whirl or
wear out, and can it be back-driven?" Static Rust → WebAssembly page, 100 % client-side (no analytics, trackers or
network requests).

### Features
- Inputs: screw type preset (typical values), nominal diameter d, lead l, length between supports L, end supports,
  dynamic load rating Ca, motor torque / max speed / ratio, moving mass, axis angle, external force, stroke,
  target speed, acceleration, cycles per day and target life.
- Results: lead angle, forward / back-drive efficiency and self-locking flag, thrust per phase, motor torque and speed,
  buckling safety factor, critical-speed margin, DN, L10 life (rev, km, years), per-check status (OK / warning / fail)
  and an overall status.
- Find screws: searches d × lead combinations for the chosen type, lists the top 12; click a row to load it.
- Plots: force–speed envelope (motor limit, 0.8 × critical speed, DN limit, operating points) and position / speed /
  thrust vs time with a cursor.
- 3D view (WebGL2): helical thread, travelling nut and carriage, bearing blocks per end-support type, motor (with belt
  pulleys when ratio ≠ 1), tilted axis, check colouring, force arrows, whirl-mode ghost, play / pause / slow motion /
  time slider. Without WebGL2 the numbers and plots still work.

### Model and assumptions
- Lead angle λ = atan(l / πd). Forward efficiency η = tanλ / tan(λ+φ), back-drive η' = tan(λ−φ) / tanλ (≤ 0 means
  self-locking). Trapezoidal: φ = atan(μ / cos 15°). Ball / roller: φ is solved from the preset η.
- Thrust = m·a + m·g·sinθ + external force + guide friction (μg = 0.01). Torque T = F·l / (2πη) + (J_screw + m(l/2π)² / η)·α.
- Trapezoidal move (accel – cruise – decel) out and back; short strokes become triangular. Fm is the
  revolution-weighted cubic mean.
- L10 = (Ca / Fm)³ · 10⁶ rev. No preload. The default Ca is an own simple estimate proportional to d² (not catalogue data).
- Euler buckling P = k·π²·E·I / L² on the root diameter d_r ≈ d − min(0.6·l, 0.25·d), E = 206 GPa,
  k = 0.25 / 2.046 / 1 / 4 (fixed-free / fixed-simple / simple-simple / fixed-fixed).
- First whirling mode ω = β² · √(E·I / ρ·A) / L², β = 1.875 / 3.927 / π / 4.730, with a 0.8 margin.
- Rigid screw; no thermal expansion, nut stiffness or backlash. All material / friction values are generic "typical" values.
- In 3D the thread lead is stretched when L / l exceeds 120 turns; the whirl mode shape is a visual approximation.

### Controls
- Drag: orbit · right drag / two fingers: pan · wheel / pinch: zoom · double-click: reset view.
- Drag the carriage along the axis to scrub time; drag the far end support to change L.
  View presets: Perspective / Top / Side / Nut-follow.

### Local build
```sh
rustup target add wasm32-unknown-unknown
cargo test
trunk build --release   # writes dist/, public_url = /screw-drive/
sh tools/privacy_check.sh
```

### License
MIT — see `LICENSE`.

> **Educational estimates — validate with the screw maker's data and real tests before trusting a robot with it.**
