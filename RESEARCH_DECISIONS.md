# GPS-Denied Navigation — Design Decisions & Research Notes

**Project:** DroneHacks — GPS-denied autonomous navigation (Sky Beta, PX4 + cuVSLAM, ROS 2)
**Scope of this document:** the approaches we investigated for localization,
checkpoint passage, and landing, and the engineering reasons we accepted or
rejected each. This is a decision record, not an API reference — it explains
*why* the flight stack is built the way it is, so reviewers and future
teammates don't re-litigate paths we already closed.

---

## System context (the constraints every decision was made under)

- **Localization/mapping is our scope; the inner (attitude/rate) controller is
  not ours to tune.** We send position/yaw setpoints to PX4; PX4 owns attitude.
  Any idea that required changing how the aircraft is stabilized was out of
  scope by definition.
- **State estimate:** cuVSLAM VIO in a start-anchored FLU frame, bridged to PX4
  as external-vision odometry. PX4 fuses only VIO. Map corrections never touch
  PX4 — they move *targets* via the map↔odom transform (`T_map_odom`) maintained
  by the localizer against a known course map.
- **The course is known geometry.** Walls, tunnel, chicane/window, boulders,
  landing pad and checkpoints are surveyed into the map. Navigation against this
  is a planning + map-matching problem, not a perception-learning problem.

---

## 1. Fiducial / ArUco marker points — **rejected**

**Idea considered:** place or exploit printed ArUco/AprilTag markers as absolute
pose anchors, detect them with `solvePnP`, and fold the sighting into
localization to correct drift (especially heading).

**Why we rejected it:**

1. **No fiducials exist on the course to detect.** Inspection of the course
   model showed every surface is untextured solid geometry — there is no printed
   marker pattern anywhere for a detector to decode. The only printed ground
   feature (the launch mark) is a ~15 cm floor decal at the start, not a wall
   fiducial.
2. **Even if present, our sensor geometry can't use it.** We fly at ~0.6 m with
   forward-facing stereo. A small floor mark is below/behind the field of view in
   cruise, and we cannot rotate/tilt to hunt for it without destabilizing the
   estimator (see §2).
3. **A single marker is the *wrong* tool for the problem we actually had
   (heading).** Planar `solvePnP` estimates a marker's in-plane position well but
   its out-of-plane orientation *poorly*, especially at range or shallow angle.
   Folding marker-derived yaw into the estimate risks injecting heading noise
   into the exact axis we were trying to stabilize — i.e., it could make things
   worse, not better.
4. **It duplicates what the map already gives us.** Every "fixed point" we care
   about (start, tunnel ends, window, pad) is already a surveyed coordinate in
   the map. We already know where they are relative to the drone via
   `T_map_odom`; we don't need a camera to re-discover a location we surveyed.

**What we did instead:** rely on the surveyed map + wall-matching localization
for absolute correction. Checkpoints are map coordinates, not things we visually
search for.

---

## 2. Tilting the drone so the camera can see the hover/landing point — **rejected**

**Idea considered:** pitch the airframe a small amount (≈5–8°) on approach so the
forward camera could glimpse the landing pad / hover point, confirm it visually,
then level out and land.

**Why we rejected it:**

1. **Pitch is not an independent, commandable degree of freedom for us.** On a
   quadrotor, pitch angle *is* horizontal acceleration — they are the same
   control variable. We cannot hold a fixed tilt while hovering; the attitude is
   whatever the position controller chooses to track our setpoint, and we do not
   control that loop. "Tilt 6° to look" is simply not an action available in our
   interface.
2. **Even a small sustained tilt means motion, not a stationary look.** To
   *cause* a steady forward tilt you must command steady forward acceleration —
   which carries the drone toward/past the very point it's trying to observe.
3. **It's geometrically insufficient anyway.** At 0.6 m altitude with a
   forward camera, a few degrees of pitch shifts the ground-intersection line
   only slightly; it does not swing a downward view under the aircraft. Seeing a
   pad below you with a forward camera needs tens of degrees — which is the
   regime that endangers tracking.
4. **Aggressive attitude change is a known estimator risk for us.** We have
   direct evidence that large/fast rotation loses visual tracking and triggers a
   failsafe. Introducing a deliberate attitude maneuver right before the scored
   landing reintroduces the one failure mode we had specifically disabled.

**What we did instead:** treat landing accuracy as a *position-estimate* problem,
not a *see-the-pad* problem. The pad is a surveyed coordinate; we navigate to it
via `T_map_odom`, settle over it, and gate the landing commit on a tight
localization sigma (mirroring how the window passage waits for low sigma before
committing). No tilt, no visual search, no estimator risk. (See §6.)

---

## 3. First-tunnel strategy — **reconsidered, then changed (camera/estimator limits)**

**Idea considered:** an initial tunnel-passage strategy that relied on the camera
actively re-orienting / scanning to acquire references through the tunnel.

**Why we changed it:**

- The tunnel is a pair of near-parallel walls. Parallel walls give a strong
  *lateral* fix but leave the *along-tunnel* direction weakly constrained — one
  wall (or two parallel ones) cannot fix position along their length.
- The scanning/re-orienting behavior we first tried stressed the camera-only
  estimator exactly where features were sparse (looking down an opening rather
  than at a textured surface), which degraded tracking at the worst moment.

**What we did instead:** approach the tunnel as a **stop-and-re-align** chokepoint
rather than a scan-through. We re-localize against the available geometry at the
tunnel entrance/exit with low rotation rate, keep the aircraft level and moving
gently, and use the tunnel's known open-end coordinates from the map to pin the
along-axis degree of freedom the walls can't. This keeps the estimator healthy
through the passage and uses surveyed geometry instead of risky camera maneuvers.

---

## 4. Ground effect — **investigated, found already handled by PX4**

**Concern investigated:** near the floor, rotor downwash recirculation (ground
effect) changes lift and could disturb a precise takeoff/landing.

**Finding:** PX4's takeoff handling already manages the low-altitude regime for
us. Because the inner loop and takeoff behavior are PX4's (not ours to tune), the
transient at lift-off is absorbed by the autopilot's existing takeoff logic — the
same mechanism that manages the initial climb manages the ground-effect band. We
confirmed we did not need to add our own compensation; doing so would have meant
reaching into control authority we don't own and shouldn't duplicate.

**Decision:** no custom ground-effect handling. Rely on PX4's takeoff behavior,
which already covers it.

---

## 5. Landing touchdown transient (motor cut-off bump) — **accepted; landing handled + rangefinder**

**Concern investigated:** when the propellers are commanded off at a cut-off
height, the aircraft drops the last short distance — a possible touchdown bump /
small bounce.

**Finding / resolution:**

- Landing descent and the final cut-off are handled by PX4's landing behavior; it
  is not something we hand-fly. The residual touchdown transient is a small,
  bounded drop from the cut-off height — acceptable for our purposes and inherent
  to how auto-land terminates.
- Critically, **we have a precise rangefinder giving reliable height-above-ground**,
  which removes the altitude-estimate uncertainty that would otherwise make the
  cut-off height unreliable. With a trustworthy AGL reference, the descent and
  cut-off happen at the intended height rather than at a drifting VIO-z estimate,
  so the transient is consistent and small.

**Decision:** accept PX4-managed landing with the rangefinder providing the
absolute height reference. No custom descent controller needed.

---

## 6. How we actually land *inside* the pad (the retained approach)

For completeness, the approach we kept (contrast to §1/§2):

- The pad is a **surveyed map coordinate**; the drone always knows where it is
  relative to the pad via `T_map_odom`. No visual detection required.
- The pad is a **stop waypoint**: the drone settles over the surveyed center.
- The commit-to-land is **gated on a tight localization sigma** — we do not hand
  off to auto-land until the position estimate is confident, analogous to how the
  window passage waits for low sigma before committing. This makes touchdown
  accuracy a function of estimate tightness, which we control, rather than of a
  camera glimpse, which we don't.
- The **rangefinder** provides absolute height for a consistent descent/cut-off.

---

## 7. The one bug that actually bit us (and its fix)

Separate from the above design choices, the concrete failure we debugged was an
**initialization offset**, not an estimator or control fault: the map's assumed
start point was set slightly further forward than where the aircraft physically
began. This shifted the entire map frame from the first frame, so the drone
believed it had more clearance ahead than it did and collided.

**Fix:** correct the assumed start pose at localization init to match the true
physical placement — **both longitudinal position and start heading** — so
`T_map_odom` is unbiased from the first frame. (A start-*heading* mismatch
produces the "continues straight instead of turning at the right place" symptom,
since every map-frame setpoint is rotated by the heading error; so the start
*orientation*, not just position, must match.)

**Lesson recorded:** because the init was a silent assumption, a biased start was
invisible until contact. We treat the start pose as a measured quantity to verify,
not a value to assume, and we check that early wall-fits are being *accepted*
(so the localizer can catch residual init error instead of flying it into a wall).

---

## Summary table

| Approach | Decision | Primary reason |
|---|---|---|
| ArUco / fiducial anchors | Rejected | No markers on course; sensor geometry can't see them; single-marker yaw is unreliable; map already provides fixed points |
| Tilt to see pad/hover point | Rejected | Pitch isn't an independent command for us; geometrically insufficient; estimator risk |
| Camera-scan tunnel strategy | Changed | Parallel-wall geometry + feature-sparse view stressed camera-only estimator; switched to stop-and-re-align using surveyed tunnel ends |
| Ground effect compensation | Not needed | Already covered by PX4 takeoff handling (control authority we don't own) |
| Touchdown bump / cut-off | Accepted | PX4-managed landing + precise rangefinder give consistent cut-off height |
| Land inside pad | Retained (map + sigma gate) | Pad is surveyed; land commit gated on tight sigma, not visual detection |
| Start-point offset | Fixed | Init bias in position *and* heading; corrected to true placement |

---

*These decisions were reached through iterative analysis of the course model, the
flight stack, and the lab's hardware/estimator constraints. Where a justification
reflects engineering rationale rather than a logged measurement, it is stated as
the reasoning we acted on, to be revised if telemetry shows otherwise.*
