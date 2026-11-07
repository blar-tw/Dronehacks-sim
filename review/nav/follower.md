# nav/follower.py review (branch nav-review, static: no Python was executed)

Files touched: `nav/follower.py`, `tests/test_nav_follower.py` only. Public API unchanged; two new
`FollowerParams` fields (`corner_arrive_timeout_s`, `stop_every_vertex`), both loadable via `nav/config.py fill()`.
Line numbers below are the edited file.

## Findings, by severity

### 1. Stall: a stop corner waits forever for the vehicle to come within `arrive_m` (follower.py:147-162)
The hold timer only started once `|pos - vertex| <= arrive_m` (0.15 m). The carrot is parked at the vertex, so
the vehicle converges on it, but on the real stack the map-frame position is PX4 position-hold error (a few cm
to ~0.1 m) plus aligner corrections that jump `pos_map` by up to `max_step_xy_m` 0.30 m. A vehicle that settles
0.16 m off the vertex never "arrives": the carrot stays parked until the mission's 60 s segment timeout, then
hover -> back_off 0.4 m -> replan from the same spot -> the same corner -> the same wait, and after `replans`
the mission lands. The tests never showed it because the fake vehicles track the setpoint exactly.
Fix: `corner_arrive_timeout_s = 4.0` (new). Parked that long without arrival, the hold starts anyway (then
`corner_hold_s` as usual). 0 restores the old behaviour. `t_parked` is reset with `t_corner` in `_next_stop`
and when the hold ends. Test: `test_corner_arrive_timeout_releases_a_vehicle_that_cannot_settle` (vehicle
tracks with a constant 0.3 m lateral offset; checks the 4 + 1 s hold, the release, and that 0 waits forever).

### 2. NaN setpoints from a repeated path point (follower.py:78-86)
`U = D / lens` with a zero-length segment gives NaN tangents, so `_project`, `_point` and the yaw goal all go
NaN and `Setpoint.x/y/z` are NaN: the mission would stream them to PX4. The planner can produce it:
`_leg` builds `[start] + cells + [end]` and string-pulls; after a back-off/replan from a vehicle sitting on
the gate entry, `start == entry` and the pulled polyline is `[start, start]`; `through_gate` then appends
the exit point. Rare, but the failure is total and silent. Fix: drop consecutive points closer than 1e-6 m,
pad a single survivor to a zero-length two-point path (U = 0, L = 0, yaw kept, done when the vehicle is
there). Test: `test_repeated_points_and_a_single_point_path_give_finite_setpoints`.

### 3. Minor: `d_stop` could be -1 ulp (follower.py:177)
`_project` returns `S0[i] + t[i]` with `t` clipped to `S1 - S0`; `S0 + (S1 - S0)` is not always `== S1`, so the
vehicle (and at `set_path`, the carrot) can sit 1 ulp past `L`. Then `8*a*d_stop` is a tiny negative added to
`(a dt)^2 > 0`: no NaN, `v` is a tiny negative that `max(self.s_cmd, ...)` already discards. Guarded with
`max(stop_s - s_cmd, 0.0)` anyway, so the formula's precondition `d >= 0` is explicit.

## Stop-and-go option (team decision 5)
`stop_every_vertex: bool = False` (follower.py:50-51, 103-107). True makes every interior vertex a stop corner
regardless of turn angle, so the carrot brakes to 0 on it, waits for arrival (or the timeout), holds
`corner_hold_s`, and goes on. The planner's gate entry point is a vertex (`through_gate` returns
`poly + [exit_]`, entry = `poly[-1]`), so the vehicle is still at every gate entry. Default unchanged; the
ARCHITECTURE behaviour (stop only above `corner_stop_deg`) remains the default.
Mission change: none required. To run strict stop-and-go, add to `nav/config/nav.yaml`:
```
follower:
  stop_every_vertex: true
```
Trade-off: each extra stop costs about decel 1 s (+0.5 s vs cruise) + arrival ~0.3 s + hold 1 s + accel
(+0.5 s) = ~2.3 s per vertex, ~3 s if the yaw also turns (90 deg at 60 deg/s overlaps the hold). With roughly
15 vertices per course that is +35-45 s of flight; a single segment with 6 straight vertices adds ~15 s, so
`segment_timeout_s` 60 s still fits for legs under ~15 m, but a long leg with many A* vertices gets close:
consider 90 s with stop-and-go on. The benefit is that every aligner update lands while the vehicle is
still (no motion blur / no lag between the correction and the leg it applies to), which is what decision 5
buys; the default gets the same at every corner sharper than 45 deg, which is where the camera view changes.
The path end has no hold: the carrot brakes to 0 there, `done` fires within `arrive_m`, and the mission
re-plans at once (the next path starts from speed 0). If the team wants a still period at gate exits too,
that is a mission-side timer, not a follower change. Test: `test_stop_every_vertex_holds_at_every_vertex...`.

## Verified correct (derivations)
- Discrete brake `v = (-a dt + sqrt((a dt)^2 + 8 a d)) / 2` (follower.py:173-178). Speeds v, v - a dt, ...,
  down to 0 over n = v/(a dt) steps cover d = sum (v - k a dt) dt = v (v + a dt) / 2a; solving for v gives
  the formula. Identity: v_brake(d - v_brake(d) dt) = v_brake(d) - a dt whenever d >= a dt^2, so successive
  steps drop exactly a dt and never overshoot; the first braking step also drops <= a dt because v_brake is
  monotone in d. When d < a dt^2 the step would pass the stop and `min(..., stop_s)` clips it, landing exactly
  on stop_s; that last step's drop is (1+x)(2-x)/2 * a dt for x in [0,1), at most 9/8 a dt (jerk, no overshoot).
  v >= 0 for all d >= 0 and dt > 0; dt is clamped to DT_MAX_S 0.2. Cross-checked numerically with an awk
  loop (L = 3.7137, 2.01, 0.9; dt = 0.05, 0.2): lands exactly, max drop ratio 1.1235. Test:
  `test_brake_profile_lands_exactly_on_the_stop_for_any_dt` (dt 0.05/0.1/0.2/0.5).
- `_project` (follower.py:203-212): `lo <= hi` always (both clipped to [0, L] from a window around s_veh in
  [0, L]); a segment with `S1 < lo` or `S0 > hi` is masked to inf; a partly covered segment has t clipped to
  `[max(lo - S0, 0), min(hi - S0, S1 - S0)]`, which is never inverted for an unmasked segment. Some segment
  always survives, so argmin is defined. A vehicle pushed off the path by more than `progress_window_m`
  keeps a sensible progress (the nearest point within the window) and a large cross-track; the carrot cap
  then pulls it back. At `set_path` the whole-path projection is safe because the planner starts the path at
  the vehicle's xy (distance ~0), so a parallel return leg 0.6 m away cannot win.
- `holding` when `k_stop` advances in the same step (follower.py:158-162): `stop_s` is refreshed and the carrot
  may start moving in that step from v = 0; `s_cmd == stops[k-1] < stops[k]`, so `d_stop > 0`.
- Yaw unwrapping (follower.py:91-97): a vertical segment keeps the previous heading (the vehicle yaw for a
  leading one), differences are wrapped turns in (-pi, pi]; a vertical leg has turn 0 (no stop) and bend 90 deg
  (slows). Test: `test_vertical_first_segment_keeps_the_vehicle_yaw_and_is_not_a_stop_corner`.
- `_corner_limit` (follower.py:214-222): zigzag L60/R60 within the window: |cumsum| = 60, max bend = 60 ->
  theta 60 deg (same as one 60 deg bend); two left 60s add to 120 -> corner speed. Sensible. Test:
  `test_corner_limit_zigzag_is_the_largest_bend_and_same_direction_turns_add_up`.
- `done` / deadlock: `s_cmd >= L` is reachable (cap is `s_veh + lead_max_m`, s_veh -> L). `turn_first` cannot
  deadlock: yaw_sp is rate-limited toward the goal, so the yaw error shrinks for any vehicle that follows
  yaw; a vehicle that never comes within `arrive_m` of the end is left to the mission's 60 s timeout.
- Carrot lead cap: first-order lag at 0.5 m/s gives a steady lag of v*tau = 0.175 m (0.35 s) / 0.2 m (test
  TAU 0.4), well under 0.6 m, so the cap never engages in the fakes and nothing starves. When it does engage
  the carrot speed self-limits to the vehicle's speed (sawtooth, no stall). See the tuning note below.
- Face-path yaw at a stop corner: `_seg(Sv)` picks the NEXT leg (searchsorted side="right"), so the yaw goal
  flips to the next tangent the step the carrot lands on the vertex, and the vehicle (<= 0.2 m behind)
  turns while it stops: intended (ARCHITECTURE "turn while stopped"). `turn_first` is harmless there: the
  carrot is parked anyway, and after the hold it only delays the release until the yaw error is <= 30 deg.
  Test: `test_face_path_turns_toward_the_next_leg_while_parked_and_turn_first_gates_the_release`.
- Efficiency: per step ~8 small numpy ops over <= 20 segments plus 3 scalar searchsorted calls; nothing
  added per step except two scalar compares. No change needed for 20 Hz on an Orin Nano.

## Tuning notes (not bugs, not changed)
- `nav/node.py` sends position-only setpoints (velocity NaN). PX4's position P loop (MPC_XY_P 0.95 default)
  then lags v / P = 0.53 m at 0.5 m/s, close to `lead_max_m` 0.6: expect the cap to engage intermittently on
  the real vehicle (speed sawtooth). Either raise `lead_max_m` to 0.8-1.0 in nav.yaml or add velocity
  feed-forward in node.py (owner of that file).
- After an aligner correction moves the vehicle forward along the path, the carrot can be behind the
  vehicle for ~lead/v s and the vehicle backs up by that amount (<= 0.3 m). Harmless; left alone because
  snapping the carrot forward could skip a stop corner.
- `arrive_m` 0.15 is the same gate for `done`; a steady 0.16 m error at the path end costs the 60 s timeout.
  A `done` arrive timeout is a mission decision (gate crossing is checked separately), so not added.

## Not verified (tests could not run)
- None of the 12 tests in `tests/test_nav_follower.py` (4 existing + 8 new) were executed; each new test was
  traced by hand against the code (timings with 2*DT slack, float-exact comparisons only where the code
  assigns the exact value). The e2e test with the real follower was not run either.
- The planner-side claim that `_leg` can return two equal points was read, not reproduced.
- Real PX4 lag numbers above are from defaults, not measured on the SQRL airframe.
