# nav/aligner.py review (static; no code was executed)

Files: `nav/aligner.py` (207 -> 326 lines), `tests/test_nav_aligner.py` (2 -> 18 tests). API kept;
new `AlignerParams` fields have defaults and load through `config.fill()`. Line numbers below refer
to the ORIGINAL file for findings and to the NEW file for changes.

## Findings, by severity

### Would misplace the drone

1. **No defence against a rigid frame error (depth time-alignment), orig. 194-201.** A frame rotated
   3-9 deg about the sensor (fast yaw) is a rigid transform of the walls, so the ICP absorbs it with a
   near-zero rms and the only gate was the absolute `max_step_yaw_deg = 8`: a 5 deg wrong frame with
   hundreds of points on two walls was accepted whole, moving the map yaw by ~5 deg (= 17 cm lateral
   at 2 m). Fix (new 286-301): Mahalanobis gate on the step d = (dx, dy, dyaw) at the vehicle against
   the covariance of the correction the prior and this frame allow, `P_prior - P_post` (pseudo-inverse
   over its non-zero eigen-directions, so an uninformed along-wall direction never counts); reject above
   `gate_nsigma = 4`. Measured against P_prior alone the gate would NOT work: the MAP step is already
   shrunk by the prior (a 4 deg frame makes a ~2.4 deg step at a 0.5 deg prior) and slips under 4 sigma.
   Prerequisite (new 302-307): a posterior sigma floor at the vehicle, `min_sigma_xy_m = 0.02`,
   `min_sigma_yaw_deg = 0.5`, added to the vehicle-frame diagonal (keeps PSD). Without it the information
   cap still lets sigma collapse to ~3 mm / 0.1 deg after a second on a wall (6000 F per update vs 0.7 mm
   of drift growth per frame at 0.5 m/s, 15 Hz), the gate would then fire on 0.3 deg attitude jitter,
   and 6.2's "margin = distance - 3 sigma" would trust a map whose gate positions are estimates. A real
   jump larger than the gate recovers as predict() grows sigma (the same behaviour as the old gates).
   Tests: `test_rigid_rotation_of_a_frame_is_rejected_by_the_prior_gate` (4 deg frame: step 1.5-8 deg,
   rejected, state untouched, next good frame accepted), `test_absolute_step_gate_rejects_a_large_jump`.

2. **Any numerical failure raised out of the 15 Hz callback, orig. 179 / 185 / 203.** `np.linalg.inv(cov)`
   raises `LinAlgError` on a singular covariance; `solve`/`inv` of the normal equations likewise; a NaN
   covariance (a NaN odom delta into `predict` makes `sqrt(diag)` NaN, orig. 98) propagates into T
   silently -> the mission would convert every target through NaN. Fix: `update()` never raises and never
   stores a non-finite T/cov (new 248-270, 272); `predict` ignores a non-finite distance and clamps a
   negative rounding diagonal (new 150-154); non-finite points and a non-finite sensor are dropped /
   rejected before association (new 237, 243). Tests: `test_singular_or_nan_prior_is_rejected_not_raised`,
   `test_nan_and_inf_points_are_ignored`, `test_empty_match_is_rejected_and_state_untouched`.

### Wrong but hidden by the current tests / sim

3. **Non-rigid (distorted) frames counted at full weight, orig. 203.** Time skew inside one frame or a
   mixed wall set converges with a large rms, yet the posterior information was the same as for a clean
   frame. Fix (new 258-264): when rms > `rms_trust_k * meas_sigma_m` (1.5 x 5 cm) scale the measurement
   information by `(k sigma / rms)^2` and re-solve the MAP step from the prior with that weight (exact,
   doubles the cost only on bad frames; `why` reports `trust 0.xx`). Alternative considered: reject
   above a multiple of sigma; rejected because a squeezed frame still carries correct yaw/along info.
   Test: `test_distorted_frame_is_trusted_less`.
4. **End faces are effectively never used, orig. 151.** `corner_m = 0.05` vs half-thickness 0.0508: on a
   0.1016 m end face only a point within 0.4 mm of its centre survives `d2 - d1 > corner_m`. Along-corridor
   information comes only from perpendicular walls (barrier north faces etc.), which is why the e2e
   start error in y is corrected late, not in the first corridor. Not changed (end points are also the
   stereo flying-edge artefacts); documented here. Lower `corner_m` to 0.03 if the lab shows a y lag.
5. **Pure-geometry gap, orig. 124-125 (course.py), not fixed:** the tube half-width changes with z, so
   on the real map each cm of height is a distinct face set (the dedupe cache helps only on z bands
   where no tube is present). Cost is one 52-face build (~50 us) per new cm; bounded by ~180 keys.
6. `sigma_xy()` reports the sigma at the odom origin (orig. 104); the mission logs it at tunnel entry
   while the vehicle is 5 m away (yaw 0.5 deg x 5 m = 4 cm hidden). Added `sigma_xy(odom_xy=None)`:
   with the vehicle's odom xy it includes the lever arm. Default unchanged.

### Minor

7. `params=AlignerParams()` default instance shared by every Aligner (orig. 69) -> `None` + fresh instance.
8. `np.linalg.inv(_lever(..))` at 20 Hz in predict/init (orig. 82, 101) -> closed-form shear inverse.
9. z anchor (coordinator request): `init_at_launch(..., odom_z=None, launch_z=0.0)` sets `T.z`; the
   update keeps `T.z` (new 275). Test: `test_init_at_launch_z_anchor`.

## Verified correct (derived by hand, covered by tests)

- `Box.corners` is CCW for any yaw (shoelace 8*hl*ht > 0), so `N = (Dy, -Dx)/L` is outward and the
  back-face cull `(S - A).N <= 0` is right (`test_face_normals_point_outward_for_a_yawed_box`, yaw 1.2217).
- Residual `r = n.(q - a)`, Jacobian `[nx, ny, ny*vx - nx*vy]` with `v = R p` (lever from the odom
  origin, the state's translation point), IEKF step `-(Pinv + H'WH)^-1 (Pinv e + H' w r)` from the prior
  mean, posterior `inv(Pinv + H'WH)` at the final association. Along one wall `H[:,0]` is exactly 0 for an
  axis-aligned wall and the x block stays decoupled, so the along-wall mean and sigma are untouched over
  20 update/predict cycles (`test_single_wall_never_moves_along_wall_over_many_updates`, sigma = 0.12 exactly).
- `_lever` / `_vehicle_cov_to_T` / `predict`: `cov_v = A cov_T A'`, noise added at the vehicle, back with
  `A^-1`; sigma grows linearly in distance regardless of call rate (`test_predict_is_linear_...`).
- `same` (colinear faces) excludes the two chicane window parts from the corner test; the 0.03 m tube walls
  survive only because the culled back face is already inf (with `sensor=None` all tube points are dropped).
- Outliers: boulders (r 0.23, h 0.53) are below cruise height and > 1 m from any face; net points along
  x = +-3.048 sit on culled or past-end faces. Points > `match_max_m` from every face are never associated
  and the result is bit-identical (`test_outliers_beyond_match_max_have_no_effect`).
- Information cap vs e2e: 15 eff points/face -> 1.3 cm per update per face, a 0.15 m start error is
  absorbed in one update (step 1.2-2.2 sigma of the allowed correction, under the gate), rms-trust does not
  fire on the noise-free fake. `min_points = 30` with 300 subsampled points is fine for a 72 deg FOV.
- Face count at cruise: 11 boxes (c2a parts 0,1 only) + 2 tube walls = 52 faces (`test_match_equals_einsum_reference` asserts it).

## Efficiency

`_match` (6 calls/update, n <= 300, F = 52) before: `rel (n,F,2)` 250 KB, `einsum("nfk,fk->nf")`
(not BLAS), `clip(u)[...,None]*D`, `rel - ...`, `norm` (square + sum + sqrt) -> ~5 (n,F,2) temporaries +
~8 (n,F) passes + `np.where(same[best])` copy. Rough Orin Nano estimate (A78, numpy ~1-2 GB/s elementwise):
1.5-2.5 ms per match, 9-15 ms per update at 15 Hz (15-20% of a core).
After: `perp = Q @ N.T - A.N`, `u = (Q @ D.T - A.D)/L^2` (two 300x2x52 matmuls), `dist = hypot(perp, past)`
(identical to the clipped-segment distance: `rel = perp N + uL D_hat`), `dist[same[best]] = inf` in place,
`r = perp[i, best]` reused, H filled into a preallocated (m,3); per-face `A.D`, `A.N`, `1/L^2`, `same`
precomputed once per face set. ~9 (n,F) passes of 125 KB, no (n,F,2): est. 0.5-0.9 ms per match,
3-5.5 ms per update (x2 on a frame that triggers the rms re-solve). The 3x3 eigh for the gate is ~30 us.
Both figures are estimates; nothing was timed.

## Could not verify (execution blocked)

- That every new test passes: numbers were derived by hand (yaw info ~23600 -> 0.37 deg < floor;
  rotated-frame step 2.2-2.5 deg at 6-7 sigma; squeezed rms 0.107 > 0.075). Marginal ones:
  `test_rigid_rotation...` (`dyaw > 1.5 deg`), `test_recovers_offset_against_yawed_walls` (`sigma_xy < 0.05`).
- `test_match_equals_einsum_reference`: `np.array_equal(same)` relies on no face offset within 1e-3 of the
  threshold (smallest real offset is 0.03 m); `allclose` tolerances 1e-9.
- The e2e tests with the gate and floor on: analysed (first step ~1.4 sigma, later y correction 0.8 sigma)
  but not run. If `test_aligner_absorbs_start_error` fails on "gate_nsigma", raise `gate_nsigma` to 5
  before touching anything else.
- `np.linalg.inv` on an all-NaN matrix: returns NaN or raises depending on LAPACK; both paths are handled.
- Actual timings on the Orin Nano.
