# nav/planner.py + nav/course.py review (branch nav-review, static: nothing was executed)

Files edited: `nav/planner.py`, `nav/course.py`, `tests/test_nav_planner.py`. Not committed.
Public API unchanged (Planner, through_gate, to_point, inflate_m, Path fields, PlannerParams fields,
Course/Box/Cylinder/Checkpoint/Grid/occupancy/load/default_map_path/faces_at/boxes_at/checkpoint/in_net).
Only one default changed: `PlannerParams.min_inflate_m` 0.15 -> 0.25.

## Findings, by severity

### A. Would hit a wall

1. **Fallback floor below the propeller radius** — `planner.py:33` (`min_inflate_m = 0.15`) and `_schedule`.
   Drone span 0.40 m: half-span 0.20. The fallback shrank inflation to 0.15 m, i.e. the planned centre line
   could pass 0.15 m from a wall face with 0.20 m propellers: 5 cm of prop in the wall, before any tracking
   error. Combined with finding 2 the real clearance was 0.115 m. The reduction was only mentioned in
   `Path.notes`, which `mission.py` never reads or logs, so the mission flew it at full speed.
   Change: default `min_inflate_m = 0.25` (span/2 + tilt: only the localization allowance is traded away),
   and `_schedule` clamps to `max(min_inflate_m, drone_span_m / 2)` whatever the config says. The note now
   reads "inflation reduced to X m from Y m (...): margin is thin, fly slowly".
   **Mission owner**: log `path.notes` and scale `speed_mps` by `path.inflate_m / planner.inflate_m()`.

2. **Half-cell lost by centre marking** — `course.py occupancy()/_fill_polygon()` (old lines 214-257).
   A cell is occupied iff its CENTRE lies in the grown footprint, so a free cell can extend res/2 = 2.5 cm
   (3.2 cm for the 70 deg chicane walls, (res/2)(|cos|+|sin|); 3.5 cm half-diagonal for a disc) into the
   inflated zone, and `_seg_free` accepts any point of a free cell. `Path.inflate_m` therefore overstated the
   true clearance by up to 3.5 cm: 0.33 -> 0.295 at full inflation (hidden by the test's `>= 0.2`), fatal
   together with finding 1. Change: each footprint is additionally grown by the cell's support along its own
   axes (exact for yaw 0 and 90, tight upper bound otherwise); cylinders by res/sqrt2; the net margin by
   res/2. Proof: p in cell C, |dx|,|dy| <= res/2 => in the box frame |du|,|dv| <= (res/2)(|c|+|s|); if p were
   within inflate of the box, C's centre would be within inflate + support, so C would be marked. Hence
   every point of every free cell is >= inflate_m from every real obstacle; `Path.inflate_m` is now a true
   lower bound. Cost on the real course: free widths shrink 5-6 cm (M2 south gap 0.93 -> 0.22 m free,
   window 0.19 m, gate-b line 0.21 m, C1-C2b chicane corridor 0.86 m faces -> 0.14 m free = 3 cells, tube
   0.56 m). All still open at full inflation by my cell-by-cell trace (see "Not verified").

3. **`_seg_free` point sampling could miss a corner clip** — `planner.py` old lines 193-203.
   Samples every res/2 along the edge; an edge entering an occupied cell at a corner with a chord shorter
   than 2.5 cm (penetration up to ~1.25 cm) was reported free, and A*'s no-corner-cutting rule does not
   protect string pulling. Change: exact traversal — cut the segment at every grid line crossed (vectorised:
   arange of crossings, sort, midpoints), test the cell of each piece; zero-length pieces (pure corner
   touches) are dropped, which is safe because of finding 2's growth. Same allocation count as before.

### B. Wrong but hidden by tests

4. **Tube walls placed on the OUTER oval** — `course.py tube_boxes()` old lines 109-118.
   The map's tube box is the mesh bounding box = outer shell (58 x 72 x 60 in). The walls' inner faces were
   put at the outer half-width and the 3 cm shell outside it: the interior was 6 cm too wide. Band at
   z=0.8: half-width at z=0.55 is 0.7365*sqrt(1-((0.55-0.9398)/0.9144)^2) = 0.666 m (outer) -> interior
   half-width now 0.636 m, corridor 1.272 m, 0.56 m free at full inflation (sensible: conservative versus
   the 0.698 m at z=0.8 itself). **Aligner/landmarks owners**: `faces_at(z)` inside the tube band now
   returns the tube side faces 3 cm further INWARD (x = 2.3114 -/+ inner half-width). Orientation of
   `Box.corners`/`Box.faces` is untouched (still CCW, local (+l,+t) first).

5. **Diagnostic text** said "needs > 2 x inflation"; now "2 x (inflation + half a cell)" to match the grid.

### C. Minor

6. `_pull` kept a duplicate last vertex when the goal coincided with the last cell centre (follower already
   guards zero-length edges); dedupe now keeps endpoints and guarantees >= 2 points.
7. Boulders top out at 0.5334 m, the z=0.8 band starts at 0.55 m: they are excluded by 1.7 cm. Correct per
   design but worth knowing when cruise_z or z_band_m is tuned (0.27 m vertical clearance to the body).
8. Sim x500 is 0.63 m wide but `drone_span_m` stays 0.40 in nav.yaml: in the GZ twin the 0.33 m inflation
   leaves 1.5 cm for the x500's tips. Set `planner.drone_span_m: 0.63` for sim runs (the 0.93 m gaps then
   need the fallback: 0.63/2 + 0.05 + 0.08 = 0.445 > 0.93/2 - 0.025).

## Verified correct (by reading and by hand computation)
- `Box.corners`: local corners are CCW, `local @ R.T` is a proper rotation, so CCW is preserved for any
  yaw; `_fill_polygon` picks the sign from the shoelace area, so CW input would also work; `>= 0` marks
  boundary cells occupied (conservative). Bounding-index clamps handle polygons off the grid.
- Crossing direction: `side = cross(b-a, start-a)`; `n = -left` if start is left, so n always points away
  from the start; `mission.cross_sign` is derived from the returned edge, so they agree.
- Window (yaw 1.2217): the line is along the wall, so `n` is the wall normal; entry/exit are 0.35 m either
  side of the opening centre, 0.30 m from the faces; sill (#2, z<=0.30) and lintel (#3, z>=1.22) fall
  outside the 0.55..1.05 band; z=0.8 is inside 0.40..1.12. Checked numerically: the lead edge's cells
  have |t| < 0.05 along the wall, free run is |t| < 0.095.
- Tunnel gate: start is north, so the crossing is north->south through the tube; entry 0.35 m inside the
  south face, exit outside; free run centre at x=2.31 (tube axis).
- `other_meshes` (cradles, zmax 0.2776) excluded at the 0.55..1.05 band; included automatically for any
  band below 0.53 m (none planned: pad descent is vertical in mission.py).
- Net margin: full inflation (+ half cell) from the net; first corridor keeps 0.36 m of free width.
- A*: 8-connected, both orthogonal neighbours required for a diagonal, heuristic octile x 1.001 (bounded
  0.1 % suboptimality), lazy closed set; `_snap` radius 0.30 m; `_widest` maximin diagnostic intact.
- Real route at z=0.8 traced cell by cell with the new margins: launch cell (row 79, y 0.927) is free vs
  barrier_1 grown to 0.888; corridor 1 cols 7-13; M1 north strip rows 110-114; M1-M2 cols 31-36; under M2
  rows 78-82; M2-M3 cols 53-58; gate-a rows 110-114; tube cols 102-112; gate-b free rows 58-61 and its
  lead edge row 59 clears C2b (cell centres at x -0.123/-0.073 straddle the grown face at -0.110..-0.088);
  a1-C2b gap 0.22 m for the gate-b -> pad leg; window -> pad passes south of a1.

## Efficiency (Orin Nano CPU, estimates: nothing was timed)
- Grid 122 x 122 = 14.9k cells. Occupancy build: ~25 polygons on small bounding boxes + 0 cylinders at
  cruise + net, ~1-2 ms; now broadcasting instead of meshgrid copies (small win), one shoelace per polygon.
- A* (dominant): ~3-8k expansions x 8 moves, pure Python. Before ~80-200 ms on x86, 2-3x on Orin
  (0.2-0.5 s). After: the octile heuristic is precomputed for all cells in numpy (one list lookup instead
  of divmod + 2 abs + max/min per relaxation) and heap functions are bound locally: est. 30-40 % less.
- String pull: ~n_cells (100-250) `_seg_free` calls; before linspace + floor (~25 us), after arange + sort
  + midpoints (~30 us); `_free_run` is now one vectorised lookup instead of ~60 Python `g.free` calls.
- Mission note: planning runs inside the 20 Hz step, so the setpoint stream pauses for the A* time; keep
  an eye on COM_OF_LOSS_T if Orin timings approach 0.5 s (not changed here; the test keeps `dt < 1.0`).

## Could NOT verify (tests cannot run here)
- That every new/changed test passes; in particular `test_real_map_every_edge_keeps_inflate_m...` asserts
  NO fallback on the real course with the new margins: the C1-C2b chicane corridor is 0.14 m free (3 cells
  at 70 deg, rows overlap by >= 1.6 cells so 8-connectivity holds by my analysis) and gate-b's lead edge
  clears C2b by ~1 cm of cell centre. If it fails there, the message names the segment and clearance.
- Floating-point placement of samples exactly on cell boundaries in the synthetic fallback tests
  (chosen off-boundary: 0.66 m and 0.54 m openings, so cell centres +-0.025 decide).
- Runtime figures above.

## Tests added (tests/test_nav_planner.py)
- `test_fallback_never_plans_below_half_the_drone_span` (finding 1), `test_free_cells_keep_inflate_m_from_
  straight_and_yawed_walls[yaw 0 / 1.2217]` (finding 2; offsets 0.315-0.325 flew straight at < 0.33 before),
  `test_seg_free_is_an_exact_cell_traversal` (finding 3; a 15 mm corner chord), `test_tube_walls_use_the_
  interior_of_the_oval_in_the_band` (finding 4), `test_window_in_a_yawed_wall_is_crossed_perpendicular_to_
  the_wall`, `test_real_map_every_edge_keeps_inflate_m_from_every_obstacle[chicane]` (real map, boxes +
  cylinders in the band, every edge >= inflate_m, inside the net, no fallback, no snap). Existing fallback
  test opening widened 0.60 -> 0.66 m for the new 0.25 m floor; clearance asserts tightened to inflate_m.
