# Project Updates

Date: 2026-05-25

## What we did so far
- Reviewed existing trajectory plots and confirmed there are multiple tunnel groups: a large Shield cluster, plus smaller Conventional clusters (Tunneling A and B). The Shield group includes a short conventional run (Tunneling_tunnel4) that starts at the same entrance.
- Focused on the Shield group for a clean 2D traced path image (top-down). Identified drift across runs as the source of the grey line spread.
- Generated multiple traced-path variants to compare approaches:
  - Overlay with representative run
  - Drift-corrected overlay
  - Centerline-only (median-based)
  - Mean-per-x and median-per-x centerlines
  - Best single run
  - Heavy-smoothed mean-per-x centerline
- Selected the heavy-smoothed mean-per-x line as the final schematic-style trace.

## Final output (kept)
- plots/combined_Shield_traced_path_mean_x_heavy.png
  - Heavy-smoothed mean-per-x centerline
  - PCA-rotated to make the main tunnel axis horizontal
  - Includes a 100 m scale bar

## Verification from TXT files (Shield only)
Using the raw Shield_tunnel*.txt files, with the same PCA alignment used for the final mean-per-x trace:
- Start-point spread: ~1.27 m
- End-point spread: ~0.98 m
- Lateral residuals to the heavy-mean centerline:
  - Mean absolute: ~1.85 m
  - Standard deviation: ~2.46 m
  - Max absolute: ~7.43 m

Interpretation: runs share the same physical tunnel (tight start/end spread), with measurable lateral drift across the run length.

## Cleanup and organization
- Kept only the finalized plot in the plots folder.
- Removed all other PNGs from plots to reduce clutter.

## Environment notes
- Python packages installed in the project venv: numpy, matplotlib.

## Notes
- No TXT source files were modified.
- All transformations (rotation, smoothing) were applied only for visualization.

## Added details (2026-05-25)
- Final trace parameters:
  - Shield-only data
  - PCA rotation to align the main axis horizontally
  - Mean-per-x centerline with 1 m bins
  - Heavy smoothing window: 61 samples (moving average)
  - 100 m scale bar in the final figure
- Plots folder now contains only the finalized trace image; all other PNGs were deleted.

## Tunnel dimensions (Shield data, PCA frame)
Definitions:
- Length: 2D centerline length along the final heavy-mean trace.
- Breadth: cross-track spread across all Shield runs (max minus min along the normal axis).
- Depth: Z range across all Shield runs (max minus min).

Values:
- Length: 351.13 m (centerline length). Axis span is 351.41 m.
- Breadth: 13.01 m.
- Depth range: 8.12 m.

## MuJoCo model (2026-05-25)
- Created folder: mujoco (renamed from mujoco_tunnel for clarity)
- Added generator script and README:
  - mujoco/generate_mujoco_tunnel.py
  - mujoco/README.md
- Generated outputs (from Shield_tunnel*.txt):
  - mujoco/tunnel.xml
  - mujoco/tunnel_centerline.csv
- Model characteristics:
  - Uses heavy-smoothed mean-per-x centerline (same method as final trace).
  - Preserves world XY coordinates (SHIFT_TO_ORIGIN = False by default).
  - Tunnel diameter is configurable via TUNNEL_DIAMETER_M.
  - Floor and rails are included and collidable; tunnel walls are visual-only.
  - Rails are configurable (gauge, radius, height) and can be disabled.
  - Axis marker is included at the tunnel start for reference.
  - README updated with correct generator commands (repo root vs inside folder)
    and MuJoCo viewer invocation (PATH or full binary path).

## MuJoCo model rework (2026-05-25)

Reworked `mujoco/generate_mujoco_tunnel.py` and `mujoco/tunnel.xml` after a
full audit. Decisions were taken jointly with the user; bug fixes were made
without asking.

### Decisions taken jointly
- **Wall is now a hollow circular tube mesh, not 1,400 capsule geoms.**
  Generator emits `mujoco/tunnel_wall.obj` (24-sided rings, double-sided
  triangles) and the XML references it through a single `<mesh>` asset. This
  is the chosen fidelity/cost balance: the wall reads as a true tunnel
  interior from inside, the XML is smaller (1,093 lines vs the old 1,441),
  and physics stays cheap because mesh collision in MuJoCo collapses to a
  convex hull (a solid log) — so the wall is intentionally visual-only and
  the floor + rails carry the physics.
- **Rails kept** at standard gauge (1.435 m), still toggleable via
  `RAILS_ENABLED`.
- **Diameter is now auto-derived from the data** (configurable). Default:
  `TUNNEL_DIAMETER_M = None` triggers auto-derivation as
  `2 * max(max_perp, max_vert) + 0.5 m`, where `max_perp` is the largest
  perpendicular distance from any observed pose to the centerline in XY and
  `max_vert` is the largest vertical residual. Set `TUNNEL_DIAMETER_M = X`
  to override with `X` metres.
- **Static tunnel only.** No robot body added — the model is a tunnel asset
  to be included from another scene later.

### Auto-derived diameter caveat
With the current Shield data the script reports:
- `max_perp = 7.41 m` (worst single-pose lateral residual to centerline)
- `max_vert = 4.39 m` (worst vertical residual)
- `diameter = 2 * max(7.41, 4.39) + 0.5 = 15.32 m`

A 15 m Shield bore would be among the largest in the world. The figure is
honest to the data but **inflated by cross-run localisation drift over the
~23-hour Shield session**, not a true bore width. Sources of inflation:
- Pose drift between runs accumulates laterally even when the robot stays
  inside the same physical tunnel.
- The PCA "main axis" is the best-fit straight line through *all* poses;
  any genuine curvature inflates the perpendicular residuals.

If a more realistic Shield bore is desired (typical range 5–8 m), set
`TUNNEL_DIAMETER_M = 6.0` (or similar) explicitly. Both the script's stdout
and this note record which value is in effect.

### Bug fixes (made without asking)
- **Dataset path**: was hardcoded to `/home/badrikanath/ICRA_PAPERS/rss/...`;
  now resolved relative to the script (`SCRIPT_DIR.parent / "dataset" / ...`).
  Same fix applied to the README's run instructions.
- **Camera + sun light**: the named overview camera was at world `(0, -100, 60)`
  and the sun at `(0, 0, 50)` — ~2.4 km from the tunnel at `(988, 2176, 8)`.
  The camera was removed (the free camera auto-frames via `<statistic>`); the
  sun light was moved to 80 m above the centerline midpoint.
- **Deprecated `coordinate="local"`** removed from `<compiler>`.
- **Floor pitch**: floor boxes were yawed but not pitched, producing ~2 cm
  step discontinuities per segment along the 8 m descent. Floor boxes are
  now both yawed *and* pitched along the local segment slope, and placed
  using the segment's actual normal direction (not just `mid.z - radius`).
- **Rails**: now use the same pitched normal as the floor so they sit
  consistently above the floor on slopes.
- **Visual setup**: added gradient skybox, headlight, and haze for usable
  default lighting in the viewer.

### Outputs after rerun
- `mujoco/tunnel_centerline.csv` — 353 points
- `mujoco/tunnel_wall.obj` — 8,472 verts, 33,792 tris (~836 KB)
- `mujoco/tunnel.xml` — 1 mesh wall, 352 floor segments, 704 rail segments,
  3 axis helpers (~136 KB)
- Validated by structural parse and per-vertex geometry checks (every ring
  vertex at exactly `diameter/2` from its centerline point; rings
  perpendicular to local tangent; floors at the expected depth below the
  centerline).

### Verification limitation
MuJoCo is not installed on this machine, so the XML could not be opened in
`simulate` for a final visual check. Structural validation (XML parse, geom
counts, mesh geometry, floor placement) all passed.

## MuJoCo model second-pass fixes (2026-05-25)

Driven by screenshots from the MuJoCo viewer that showed (a) the floor
protruding as a horizontal plate below the bore, (b) a shimmering /
cross-hatched pattern on the upper-middle of the tube, and (c) a question
about whether the descending slope was being captured. All three are now
fixed.

### Floor "plate" sticking out below the bore — fixed
- **Cause**: previous version placed the floor at depth `radius - thickness/2`
  below the centerline (i.e. tangent to the very bottom of the bore, where
  the local chord is only ~2.5 m wide), but the floor box width was
  `0.9 × diameter ≈ 13.8 m`. Net result: the floor protruded ~5.7 m on each
  side outside the wall.
- **Fix**: replaced `FLOOR_WIDTH_FRACTION` with a chord-based parameterisation.
  The floor top sits at `FLOOR_DEPTH_BELOW_CENTER_M = 1.5 m` below the
  centerline, and the floor width is derived as `2 * (half_chord - margin)`
  where `half_chord = sqrt(r² - depth²)` and
  `FLOOR_WIDTH_MARGIN_M = 0.3 m`. The script raises if the depth + margin
  leaves no room. For the current 15.32 m bore: floor is now 14.43 m wide
  at 1.5 m depth, and the bore chord at the floor's *bottom* (1.7 m depth)
  is 14.94 m — so the floor stays fully inside the wall with margin.

### "Weird rendering" / z-fighting on the wall — fixed
- **Cause**: previous version made the wall mesh double-sided by emitting
  both outward- and inward-facing triangles on the *same* surface (same
  three vertices, opposite winding). Those triangles were exactly coplanar,
  so the GPU couldn't resolve depth between them — visible as the
  cross-hatched shimmer in the upper-middle of the tube.
- **Fix**: replaced the single double-sided mesh with **two single-sided
  shells** separated by `WALL_THICKNESS_M = 0.15 m`:
  - `tunnel_wall_outer.obj` at `r + thickness/2`, outward-facing normals
  - `tunnel_wall_inner.obj` at `r - thickness/2`, inward-facing normals
  No two triangles are coplanar anywhere, so z-fighting is impossible by
  construction. The wall now also has a believable physical thickness
  (segmental-liner aesthetic).
- A `verify_mesh_normals()` assertion runs at generation time and raises
  if either shell's winding produces the wrong normal direction.

### Z-axis (elevation) capture — improved
- **Observation**: the previous `SMOOTH_WINDOW = 61` was applied identically
  to the cross-track (`Y`) and the elevation (`Z`) components of the
  centerline. That heavy a window is right for `Y` (kills cross-run drift),
  but it flattened ~half of the real elevation variation.
- **Numbers**:
  - Raw Shield-data Z range: 8.12 m
  - Old smoothed centerline Z range: 4.10 m
  - New smoothed centerline Z range: 4.54 m
  - Net descent (start→end): 2.31 m over 351 m horizontal = **0.66 % grade**
- **Fix**: split into `SMOOTH_WINDOW_Y = 61` (unchanged) and
  `SMOOTH_WINDOW_Z = 15` (lighter). The cross-track trace looks the same,
  the elevation profile preserves more real structure. Setting
  `SMOOTH_WINDOW_Z = 1` would keep the raw mean-per-bin Z with no
  smoothing — choose per how much elevation detail you want.

### Resulting model
- Centerline: 353 points
- Wall: outer mesh 8,472 verts / 16,896 tris (~564 KB); inner same
- Floor: 352 chord-fitted segments, pitched + yawed along the slope
- Rails: 704 capsules (2 per segment), standard gauge, 5 cm above floor
- XML: 136 KB (unchanged count of geoms, just two mesh geoms instead of one)
- Mesh-normal sanity check passes at generation time.

## MuJoCo model third-pass floor clipping (2026-05-25)

User request: keep the floor positioned at the bore bottom (same setup), but
clip off any parts that extend outside the circular bore. The floor should be
"honest" to the TXT data and not pushed upward to fit.

Changes made:
- Floor remains a bore-bottom plate with top at depth `radius - FLOOR_THICKNESS_M`.
- Added `FLOOR_WIDTH_FRACTION = 0.9` to preserve the original intended width.
- The floor is now clipped to the bore chord: the actual used half-width is
  `min(desired_half_width, chord_half_width)`.
- The resulting mesh is a circle-segment cross-section (flat top, curved
  bottom), so no corner can protrude through the wall.
- Output includes `mujoco/tunnel_floor.obj`.

Notes:
- If `FLOOR_WIDTH_FRACTION` exceeds the available chord at the bore bottom,
  the floor is automatically clipped (this is expected for the current data).
- Rails continue to sit on the floor top using the same depth reference.

## Centerline smoothing and Z range preservation (2026-05-26)

Changes made:
- Reduced cross-track smoothing to keep curvature: `SMOOTH_WINDOW_Y = 15`.
- Disabled Z smoothing: `SMOOTH_WINDOW_Z = 1`.
- Added `PRESERVE_Z_RANGE = True` so the centerline Z min/max matches the
  raw Shield TXT data range exactly.
- Generated a new path image with the lighter XY smoothing:
  - plots/combined_Shield_traced_path_light.png

## Y smoothing reverted (2026-05-26)

- Reverted cross-track smoothing to heavy: `SMOOTH_WINDOW_Y = 61`.
- Kept Z settings unchanged: `SMOOTH_WINDOW_Z = 1` and `PRESERVE_Z_RANGE = True`.
- Regenerated MuJoCo outputs with the reverted Y smoothing.
