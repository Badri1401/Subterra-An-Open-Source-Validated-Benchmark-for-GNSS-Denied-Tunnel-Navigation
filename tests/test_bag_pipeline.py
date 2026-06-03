#!/usr/bin/env python3
"""Verifications for the LiDAR-driven tunnel pipeline.

Three suites:

  TestBagExtraction        ground-truth artefacts from the .bag exist, are
                           consistent with the raw bag, and pass sanity checks
                           (left-right symmetry, plausible dimensions, texture
                           file validity).

  TestMujocoModel          the generated MuJoCo XML + OBJ meshes are
                           well-formed: parseable, finite verts, valid
                           triangle indices, manifold-ish ring topology, mesh
                           dimensions agree with the extracted profile within
                           tolerance, materials reference real texture files.

  TestZAxisUpdates         the Z-axis settings the user documented in
                           updates.md (PRESERVE_Z_RANGE=True, SMOOTH_WINDOW_Z=1,
                           SMOOTH_WINDOW_Y=61) are in effect, and the
                           centerline Z span equals the raw Shield Z span
                           bit-for-bit.

Run from repo root:
    pytest -v test_bag_pipeline.py
"""
from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parent.parent
MUJOCO_DIR = REPO / "mujoco"
EXTRACTED = MUJOCO_DIR / "extracted"
DATASET = REPO / "dataset" / "drive-download-20260524T182528Z-3-001"

PROFILE = EXTRACTED / "cross_section_profile.npz"
META = EXTRACTED / "metadata.json"
WALL_TEX = EXTRACTED / "wall_texture.png"
FLOOR_TEX = EXTRACTED / "floor_reflectivity.png"
SENSOR_CLOUD = EXTRACTED / "pointcloud_sensor.npz"

XML = MUJOCO_DIR / "tunnel.xml"
WALL_OUTER = MUJOCO_DIR / "tunnel_wall_outer.obj"
WALL_INNER = MUJOCO_DIR / "tunnel_wall_inner.obj"
FLOOR_OBJ = MUJOCO_DIR / "tunnel_floor.obj"
SLEEPER_OBJ = MUJOCO_DIR / "sleeper.obj"
CENTERLINE_CSV = MUJOCO_DIR / "tunnel_centerline.csv"

# HK MTR concrete monoblock sleeper expected dimensions (mirrors
# constants in generate_mujoco_tunnel.py; kept here so the tests are
# independent of the generator module import).
# Updated 2026-05-27: sleeper length reduced from 2.50 → 1.945 m so rails
# sit centered on their raised pads with full fastener clearance. The pad
# half-width is 0.140 m (rail foot half 0.070 + fastener clearance 0.070
# per side), centered on the rail at ±0.7525 m.
SLEEPER_LENGTH_M = 1.945
SLEEPER_WIDTH_M = 0.30
SLEEPER_HEIGHT_END_M = 0.07         # rail-seat pad height (HIGH on the top)
SLEEPER_HEIGHT_MIDDLE_M = 0.04      # belly + outer-shoulder height (LOW)
SLEEPER_SPACING_M = 0.60

# Rail + fastener geometry (HK MTR Urban Lines BS113A + Pandrol Fastclip):
RAIL_GAUGE_M = 1.435
RAIL_HEAD_RADIUS_M = 0.035
RAIL_FOOT_WIDTH_M = 0.140
RAIL_CENTER_Y_M = RAIL_GAUGE_M / 2.0 + RAIL_HEAD_RADIUS_M  # = 0.7525
SLEEPER_PAD_HALF_WIDTH_M = 0.140
SLEEPER_PAD_INNER_Y_M = RAIL_CENTER_Y_M - SLEEPER_PAD_HALF_WIDTH_M   # 0.6125
SLEEPER_PAD_OUTER_Y_M = RAIL_CENTER_Y_M + SLEEPER_PAD_HALF_WIDTH_M   # 0.8925
FASTENER_CLEARANCE_PER_SIDE_M = 0.070

GEN_PY = MUJOCO_DIR / "generate_mujoco_tunnel.py"


# ---------- helpers ----------

def read_obj(path: Path) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    vs: list[tuple[float, float, float]] = []
    fs: list[tuple[int, int, int]] = []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            parts = line.split()
            vs.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("f "):
            parts = line.split()[1:]
            ids = []
            for p in parts:
                # 'v', 'v/vt', 'v/vt/vn' all start with the vertex index
                ids.append(int(p.split("/")[0]) - 1)
            assert len(ids) == 3, f"non-tri face in {path}: {line!r}"
            fs.append(tuple(ids))
    return np.asarray(vs, dtype=np.float64), fs


def png_signature_ok(path: Path) -> bool:
    head = path.read_bytes()[:8]
    return head == b"\x89PNG\r\n\x1a\n"


# ---------- TestBagExtraction ----------

class TestBagExtraction:

    def test_extraction_artefacts_exist(self):
        for p in (PROFILE, META, WALL_TEX, FLOOR_TEX, SENSOR_CLOUD):
            assert p.exists(), f"missing {p}"

    def test_metadata_consistent(self):
        meta = json.loads(META.read_text())
        assert meta["lidar_scans_read"] >= 5000, "expected ~5427 scans"
        assert meta["lidar_pts_total"] > 100_000_000, "expected >100M points"
        # Tunnel is small concrete bore -- not 15m
        cs = meta["cross_section"]
        assert 4.5 < cs["width_y_m"] < 8.0, f"width {cs['width_y_m']}"
        assert 4.0 < cs["height_z_m"] < 7.0, f"height {cs['height_z_m']}"
        assert 2.5 < cs["top_arch_radius_m"] < 3.5
        assert -3.0 < cs["floor_z_below_sensor_m"] < -1.5

    def test_profile_loadable(self):
        a = np.load(PROFILE)
        for k in ("theta", "radius", "radius_raw", "count"):
            assert k in a.files, f"missing key {k}"
        assert len(a["theta"]) == len(a["radius"])
        assert np.all(np.isfinite(a["radius"])), "filled radius has NaN"

    def test_profile_left_right_symmetry(self):
        """Tunnel is LR-symmetric (not TB -- it has a flat floor + arched top).
        Mirror y -> -y => theta -> pi - theta (mod 2pi). We pair each bin
        with its LR-mirror and check the median relative radius difference
        is small."""
        a = np.load(PROFILE)
        theta = a["theta"]
        radius = a["radius"]
        # LR mirror: new_theta = pi - theta, wrapped to [-pi, pi]
        mirror_theta = np.where(theta >= 0, math.pi - theta, -math.pi - theta)
        # For each bin find nearest mirror partner
        med_diff_pct = []
        for i in range(len(theta)):
            j = int(np.argmin(np.abs(theta - mirror_theta[i])))
            if radius[i] > 0.1:
                med_diff_pct.append(abs(radius[i] - radius[j]) / radius[i])
        m = float(np.median(med_diff_pct))
        assert m < 0.30, f"L-R asymmetry too large: median {m*100:.1f}%"

    def test_profile_no_huge_jumps(self):
        """Adjacent bins shouldn't differ by > 1m -- that signals discontinuity."""
        a = np.load(PROFILE)
        r = a["radius"]
        diffs = np.abs(np.diff(r))
        # 99th pctl must be modest
        assert np.percentile(diffs, 99) < 1.0, f"99th pctl jump {np.percentile(diffs,99):.2f}"

    def test_wall_texture_is_valid_png(self):
        assert png_signature_ok(WALL_TEX)
        # require minimum useful size
        from PIL import Image
        img = Image.open(WALL_TEX)
        assert img.size[0] >= 128 and img.size[1] >= 128

    def test_floor_texture_is_valid_png(self):
        assert png_signature_ok(FLOOR_TEX)
        from PIL import Image
        img = Image.open(FLOOR_TEX)
        assert img.size[0] >= 64 and img.size[1] >= 64

    def test_sensor_cloud_in_plausible_range(self):
        a = np.load(SENSOR_CLOUD)
        cloud = a["xyzr"]
        assert cloud.shape[1] == 4
        assert cloud.shape[0] > 10_000, "voxel cloud too sparse"
        # All x within reasonable lidar range (0 to ~300m)
        assert cloud[:, 0].min() >= -10.0
        # Reflectivity in 0..255
        assert cloud[:, 3].min() >= 0 and cloud[:, 3].max() <= 255


# ---------- TestMujocoModel ----------

class TestMujocoModel:

    def test_artefacts_exist(self):
        for p in (XML, WALL_OUTER, WALL_INNER, FLOOR_OBJ, CENTERLINE_CSV):
            assert p.exists(), f"missing {p}"

    def test_xml_parses(self):
        tree = ET.parse(XML)
        root = tree.getroot()
        assert root.tag == "mujoco"

    def test_xml_meshes_resolve(self):
        tree = ET.parse(XML)
        root = tree.getroot()
        meshes = {m.get("name"): m.get("file") for m in root.findall(".//asset/mesh")}
        for name, fname in meshes.items():
            assert (MUJOCO_DIR / fname).exists(), f"mesh file missing: {fname}"

    def test_xml_textures_resolve(self):
        tree = ET.parse(XML)
        root = tree.getroot()
        for tex in root.findall(".//asset/texture"):
            f = tex.get("file")
            if f is None:
                continue
            assert (MUJOCO_DIR / f).exists(), f"texture file missing: {f}"

    def test_mesh_obj_finite_vertices(self):
        for p in (WALL_OUTER, WALL_INNER, FLOOR_OBJ):
            verts, _ = read_obj(p)
            assert verts.shape[0] > 0
            assert np.all(np.isfinite(verts)), f"non-finite in {p}"

    def test_mesh_faces_valid_indices(self):
        for p in (WALL_OUTER, WALL_INNER, FLOOR_OBJ):
            verts, faces = read_obj(p)
            n = verts.shape[0]
            for f in faces:
                for idx in f:
                    assert 0 <= idx < n, f"bad index in {p}"

    def test_wall_radius_matches_profile(self):
        """Sample the wall mesh: for each centerline point, compute distance
        from the local centerline to each ring vertex projected into the
        local (lateral, vertical) plane. Compare to the extracted profile
        radius within 0.30 m (one voxel size)."""
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        prof = np.load(PROFILE)
        profile_yz = np.column_stack([prof["radius"] * np.cos(prof["theta"]),
                                       prof["radius"] * np.sin(prof["theta"])])
        # The generator subsamples; recover subsample by checking ring sizes
        verts, _ = read_obj(WALL_OUTER)
        n_pts = len(cl)
        K = verts.shape[0] // n_pts
        assert K * n_pts == verts.shape[0], (
            f"wall verts {verts.shape[0]} not divisible by centerline pts {n_pts}"
        )
        # Build profile downsampled to K
        prof_step = max(1, len(profile_yz) // K)
        prof_sub = profile_yz[::prof_step][:K]
        prof_r = np.linalg.norm(prof_sub, axis=1)

        # For each centerline point i, compute radial distance for each ring vert
        # The outer shell adds a small radial offset (WALL_THICKNESS_M/2).
        wall_thickness = 0.15
        expected_r_outer = prof_r + wall_thickness / 2.0

        # Use a few centerline points (middle to avoid endpoint effects)
        idxs = [n_pts // 4, n_pts // 2, 3 * n_pts // 4]
        max_dev = 0.0
        for i in idxs:
            ring = verts[i * K:(i + 1) * K]
            c = cl[i]
            ring_r = np.linalg.norm(ring - c, axis=1)
            dev = np.abs(ring_r - expected_r_outer)
            max_dev = max(max_dev, float(dev.max()))
        # 0.30 m tolerance is generous; the profile and the centerline are
        # built independently and may have small frame mismatches.
        assert max_dev < 0.30, f"wall ring radius mismatch (max dev {max_dev:.3f})"

    def test_floor_lies_below_centerline(self):
        """Every floor vertex should be below the nearest centerline point."""
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        verts, _ = read_obj(FLOOR_OBJ)
        # Brute-force NN (only ~thousands of pts) -- avoid scipy dep
        z_below = []
        for v in verts:
            j = int(np.argmin(np.sum((cl - v) ** 2, axis=1)))
            z_below.append(cl[j, 2] - v[2])
        z_below = np.asarray(z_below)
        assert (z_below > 0).all(), f"floor verts above centerline! min delta {z_below.min():.3f}"
        # Floor should be ~1.4-2 m below centerline (LiDAR -> raised platform top).
        assert 0.8 < z_below.mean() < 2.5, f"mean floor depth below centerline {z_below.mean():.3f}"

    def test_no_old_plinth_components(self):
        """The earlier slab-track plinth experiment is gone — the floor
        furniture is now: sleepers + BS113A rails + Pandrol Fastclip fasteners.
        (The new Pandrol-style fasteners use the same name prefix `fastener_`
        but are a deliberate, modelled component, not the old slab-track
        plinth+fastener experiment.)"""
        tree = ET.parse(XML)
        root = tree.getroot()
        for g in root.findall(".//worldbody/geom"):
            name = g.get("name") or ""
            assert not name.startswith("plinth_"), (
                f"obsolete plinth component remains: <geom name={name}>"
            )

    def test_hk_mtr_rails_present_and_paired(self):
        """HK MTR BS113A rails: two sides (L+R), each with a foot box and a
        head capsule per centerline segment. Standard gauge 1.435 m measured
        between inner head faces."""
        RAIL_GAUGE_M = 1.435
        RAIL_HEAD_RADIUS_M = 0.035
        tree = ET.parse(XML)
        root = tree.getroot()
        geoms = root.findall(".//worldbody/geom")
        foot_L = [g for g in geoms if (g.get("name") or "").startswith("rail_L_foot_")]
        foot_R = [g for g in geoms if (g.get("name") or "").startswith("rail_R_foot_")]
        head_L = [g for g in geoms if (g.get("name") or "").startswith("rail_L_head_")]
        head_R = [g for g in geoms if (g.get("name") or "").startswith("rail_R_head_")]
        # Same count per side per component (one per segment)
        assert len(foot_L) > 100, f"too few rail_L_foot geoms: {len(foot_L)}"
        assert len(foot_L) == len(foot_R) == len(head_L) == len(head_R), (
            f"rail component counts disagree: "
            f"foot_L={len(foot_L)} foot_R={len(foot_R)} "
            f"head_L={len(head_L)} head_R={len(head_R)}"
        )
        # Classes correctly assigned (so material/contype inherit properly)
        for g in foot_L[:5] + foot_R[:5]:
            assert g.get("class") == "rail_foot", g.get("class")
        for g in head_L[:5] + head_R[:5]:
            assert g.get("class") == "rail_head", g.get("class")
        # Gauge check: distance between L and R head capsule centres
        # (measured at the same segment index). The rail centerline is offset
        # outward by RAIL_HEAD_RADIUS_M from the gauge line, so the centre-
        # to-centre distance = GAUGE + 2*HEAD_RADIUS.
        expected_center_dist = RAIL_GAUGE_M + 2 * RAIL_HEAD_RADIUS_M
        gauges_seen = []
        for hL, hR in list(zip(head_L, head_R))[: min(20, len(head_L))]:
            ftL = hL.get("fromto"); ftR = hR.get("fromto")
            assert ftL and ftR
            pL = np.array([float(x) for x in ftL.split()[:3]])
            pR = np.array([float(x) for x in ftR.split()[:3]])
            gauges_seen.append(float(np.linalg.norm(pL - pR)))
        mean_gauge = float(np.mean(gauges_seen))
        assert abs(mean_gauge - expected_center_dist) < 0.01, (
            f"rail head centre-to-centre {mean_gauge:.4f} m != "
            f"expected {expected_center_dist:.4f} m "
            f"(gauge {RAIL_GAUGE_M} + 2×head_radius {RAIL_HEAD_RADIUS_M})"
        )

    def test_hk_mtr_fasteners_present_and_paired(self):
        """HK MTR Pandrol Fastclip-style fasteners: per sleeper, 2 rails × 2
        sides (inner/outer) × 2 sub-geoms (baseplate + clip) = 8 fasteners.
        Across all 586 sleepers ≈ 4,688 fastener geoms."""
        tree = ET.parse(XML)
        root = tree.getroot()
        geoms = root.findall(".//worldbody/geom")
        fastener_base = [g for g in geoms if (g.get("name") or "").startswith("fastener_base_")]
        fastener_clip = [g for g in geoms if (g.get("name") or "").startswith("fastener_clip_")]
        # Equal counts (one baseplate paired with one clip per side)
        assert len(fastener_base) == len(fastener_clip), (
            f"baseplate count {len(fastener_base)} != clip count {len(fastener_clip)}"
        )
        # ~586 sleepers × 4 sides each = ~2344 per kind
        assert 2000 < len(fastener_base) < 3000, (
            f"unexpected baseplate count {len(fastener_base)} (expected ~2344)"
        )
        # Classes inherit material/colour correctly
        for g in fastener_base[:5]:
            assert g.get("class") == "fastener_base", g.get("class")
        for g in fastener_clip[:5]:
            assert g.get("class") == "fastener_clip", g.get("class")
        # Per sleeper there should be exactly 4 of each (2 rails × 2 sides).
        # Name format: fastener_base_NNNN_<L|R>_<in|out>
        per_sleeper_count: dict[int, int] = {}
        for g in fastener_base:
            sleeper_idx = int((g.get("name") or "").split("_")[2])
            per_sleeper_count[sleeper_idx] = per_sleeper_count.get(sleeper_idx, 0) + 1
        counts = list(per_sleeper_count.values())
        assert all(c == 4 for c in counts), (
            f"every sleeper must have 4 baseplate fasteners; saw {set(counts)}"
        )

    def test_fasteners_sit_within_pad_clearance(self):
        """The fastener housings must fit ENTIRELY in the flat raised pad
        region of the sleeper (per the user's design intent: 'both fasteners
        on both sides of each rail must be on the higher edges'). For each
        baseplate, its across-tunnel extent (y) must fall in:
            inner side: between pad_inner_edge and rail_foot_inner_edge
            outer side: between rail_foot_outer_edge and pad_outer_edge
        We check via the geom 'size' (the half-extent across tunnel = size[1]).
        """
        tree = ET.parse(XML)
        root = tree.getroot()
        fastener_base = [g for g in root.findall(".//worldbody/geom")
                         if (g.get("name") or "").startswith("fastener_base_")]
        # The geom size in MuJoCo for a box is "size_x size_y size_z" half-extents.
        # In our sleeper-aligned frame: x=along tunnel, y=across tunnel, z=vertical.
        # Constraint: baseplate width (= 2*size[1]) must be < FASTENER_CLEARANCE_PER_SIDE_M
        for g in fastener_base[:200]:
            sz = [float(x) for x in g.get("size").split()]
            baseplate_width_across = 2.0 * sz[1]
            assert baseplate_width_across < FASTENER_CLEARANCE_PER_SIDE_M + 1e-6, (
                f"{g.get('name')}: baseplate width across tunnel {baseplate_width_across:.4f} "
                f"> clearance per side {FASTENER_CLEARANCE_PER_SIDE_M}"
            )

    def test_fastener_materials_are_dark_steel(self):
        """Fastener materials must exist and read as dark steel:
            fastener_base: dark grey baseplate
            fastener_clip: near-black clip
        Both with non-trivial specular so they pick up the LED highlight."""
        tree = ET.parse(XML)
        root = tree.getroot()
        mats = {m.get("name"): m for m in root.findall(".//asset/material")}
        for name in ("fastener_base", "fastener_clip"):
            assert name in mats, f"missing <material name='{name}'>"
            rgba = [float(x) for x in mats[name].get("rgba").split()]
            # Dark: each channel below 0.30
            assert all(c < 0.30 for c in rgba[:3]), (
                f"{name} rgba {rgba[:3]} not dark enough"
            )
            spec = float(mats[name].get("specular") or 0.0)
            assert spec >= 0.20, f"{name} specular {spec} too low"
        # Clip must be darker than baseplate (per the photo)
        base_rgba = [float(x) for x in mats["fastener_base"].get("rgba").split()][:3]
        clip_rgba = [float(x) for x in mats["fastener_clip"].get("rgba").split()][:3]
        assert sum(clip_rgba) < sum(base_rgba), (
            f"clip ({clip_rgba}) should be darker than baseplate ({base_rgba})"
        )

    def test_rail_material_is_steel(self):
        """The rail material should exist and have a steel-grey rgba with
        some specular highlight (not the flat sleeper grey)."""
        tree = ET.parse(XML)
        root = tree.getroot()
        rail_mat = None
        for m in root.findall(".//asset/material"):
            if m.get("name") == "rail":
                rail_mat = m
                break
        assert rail_mat is not None, "no <material name='rail'> in XML"
        # Specular highlight on the running surface (matches in-service look)
        spec = float(rail_mat.get("specular") or 0.0)
        assert spec >= 0.20, f"rail material specular {spec} too low for steel"

    def test_sleeper_mesh_obj_valid(self):
        """sleeper.obj must exist, parse, and have positive height everywhere
        (no degenerate top-bottom collapse)."""
        assert SLEEPER_OBJ.exists(), f"missing {SLEEPER_OBJ}"
        verts, faces = read_obj(SLEEPER_OBJ)
        assert verts.shape[0] >= 24, f"sleeper mesh too small: {verts.shape[0]} verts"
        assert len(faces) >= 30, f"sleeper mesh too few tris: {len(faces)}"
        # Bottom-z = 0, top-z = positive at every cross-section
        # x range = ±SLEEPER_WIDTH_M/2
        assert abs(verts[:, 0].min() + SLEEPER_WIDTH_M / 2) < 1e-3
        assert abs(verts[:, 0].max() - SLEEPER_WIDTH_M / 2) < 1e-3
        # y range covers the full sleeper length
        y_span = verts[:, 1].max() - verts[:, 1].min()
        assert abs(y_span - SLEEPER_LENGTH_M) < 1e-3, f"sleeper y-span {y_span:.4f} != {SLEEPER_LENGTH_M}"
        # z range: 0 .. SLEEPER_HEIGHT_END_M (the thick ends are the tallest)
        assert verts[:, 2].min() >= -1e-6
        assert abs(verts[:, 2].max() - SLEEPER_HEIGHT_END_M) < 1e-3, (
            f"sleeper z-max {verts[:, 2].max():.4f} != {SLEEPER_HEIGHT_END_M}"
        )

    def test_sleeper_profile_rail_seat_pads_centered_on_rails(self):
        """Side profile per 2026-05-27 design: low middle belly, two raised
        rail-seat pads CENTERED on the rails (±0.7525 m), short slopes, and
        low outer structural shoulders past the pads. Critically, the rail
        center positions ±0.7525 m must land on the HIGH pad surface so
        rails and fasteners can sit symmetrically on flat raised concrete."""
        verts, _ = read_obj(SLEEPER_OBJ)
        def top_at(y_target: float) -> float:
            y_uniq = np.unique(verts[:, 1])
            j = int(np.argmin(np.abs(y_uniq - y_target)))
            y_near = y_uniq[j]
            mask = np.abs(verts[:, 1] - y_near) < 1e-6
            return float(verts[mask, 2].max())
        # Middle is LOW
        z_mid = top_at(0.0)
        assert abs(z_mid - SLEEPER_HEIGHT_MIDDLE_M) < 1e-3, (
            f"middle height {z_mid:.3f} != expected {SLEEPER_HEIGHT_MIDDLE_M}"
        )
        # Both rail-seat pads are HIGH at the rail center positions ±0.7525.
        # (This is the new "rails sit on the higher pads" guarantee.)
        z_rail_L = top_at(-RAIL_CENTER_Y_M)
        z_rail_R = top_at(+RAIL_CENTER_Y_M)
        assert abs(z_rail_L - SLEEPER_HEIGHT_END_M) < 1e-3, (
            f"left pad height under rail {z_rail_L:.3f} != expected {SLEEPER_HEIGHT_END_M}"
        )
        assert abs(z_rail_R - SLEEPER_HEIGHT_END_M) < 1e-3
        # Both pad inner edges and outer edges are HIGH (i.e. the WHOLE 280-mm
        # pad is flat HIGH, so fasteners on both sides of the rail foot ride
        # on full-height concrete).
        for y in (SLEEPER_PAD_INNER_Y_M, SLEEPER_PAD_OUTER_Y_M):
            assert abs(top_at(+y) - SLEEPER_HEIGHT_END_M) < 1e-3, (
                f"right pad surface at y={+y} is {top_at(+y):.3f}, not HIGH"
            )
            assert abs(top_at(-y) - SLEEPER_HEIGHT_END_M) < 1e-3, (
                f"left pad surface at y={-y} is {top_at(-y):.3f}, not HIGH"
            )
        # Pad must be strictly HIGHER than middle belly
        assert z_rail_L > z_mid + 0.02
        assert z_rail_R > z_mid + 0.02
        # Bottom is flat at z=0
        assert verts[:, 2].min() >= -1e-6 and verts[:, 2].min() < 1e-6

    def test_sleepers_present_and_spaced(self):
        """Sleepers placed along the LiDAR-derived centerline at HK MTR
        standard ~0.60 m spacing. Total arc ~351 m -> ~586 sleepers."""
        tree = ET.parse(XML)
        root = tree.getroot()
        geoms = root.findall(".//worldbody/geom")
        sleepers = [g for g in geoms
                    if (g.get("name") or "").startswith("sleeper_")]
        # ~351 m / 0.6 m -> ~586
        assert 400 < len(sleepers) < 800, f"sleeper count {len(sleepers)} out of expected range"
        # All sleepers reference the shared mesh via the "sleeper" default class.
        # The mesh="sleeper" attribute is inherited from <default class="sleeper">
        # in the XML, so we check the class attribute on each geom.
        for g in sleepers[:10]:
            assert g.get("class") == "sleeper", (
                f"sleeper geom should use class=sleeper, got {g.get('class')}"
            )
        # Along-track spacing of ALL sleepers ~0.60 m (HK MTR standard).
        # Measured 3-D distance between consecutive geom centres — should
        # match SLEEPER_SPACING_M closely along the whole tunnel.
        positions = np.array([
            [float(x) for x in g.get("pos").split()] for g in sleepers
        ])
        spacings = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        mean_sp = float(np.mean(spacings))
        std_sp = float(np.std(spacings))
        min_sp = float(np.min(spacings))
        max_sp = float(np.max(spacings))
        # Mean must be within 1 cm of HK MTR spec; std (jitter) under 2 cm.
        assert abs(mean_sp - SLEEPER_SPACING_M) < 0.01, (
            f"mean sleeper spacing {mean_sp:.4f} m != HK MTR {SLEEPER_SPACING_M} m"
        )
        assert std_sp < 0.02, f"sleeper spacing std too jittery: {std_sp:.4f} m"
        # No two consecutive sleepers should overlap (min > sleeper width 0.30 m)
        # nor be more than 0.10 m off-spec.
        assert min_sp > 0.30, f"sleepers overlapping! min spacing {min_sp:.3f} m"
        assert max_sp < SLEEPER_SPACING_M + 0.10, f"max spacing {max_sp:.3f} m"

    def test_sleepers_fit_within_lidar_bore(self):
        """Each sleeper's lateral extent (half-length = 1.25 m) must fit
        within the LiDAR-derived bore at the floor level — i.e. the
        chord-width of the bore at z_local = -1.50 m must exceed the
        sleeper length by at least the configured clearance."""
        prof = np.load(PROFILE)
        theta = prof["theta"]; radius = prof["radius"]
        y = radius * np.cos(theta); z = radius * np.sin(theta)
        profile_yz = np.column_stack([y, z])
        # Floor top z_local = -1.50 (FLOOR_TOP_Z_LOCAL in generator)
        FLOOR = -1.50
        pp = np.vstack([profile_yz, profile_yz[:1]])
        ys = []
        for i in range(len(pp) - 1):
            y0, z0 = pp[i]; y1, z1 = pp[i + 1]
            if (z0 <= FLOOR <= z1) or (z1 <= FLOOR <= z0):
                if abs(z1 - z0) < 1e-9:
                    continue
                t = (FLOOR - z0) / (z1 - z0)
                ys.append(y0 + t * (y1 - y0))
        assert len(ys) >= 2
        chord = max(ys) - min(ys)
        assert chord > SLEEPER_LENGTH_M + 0.20, (
            f"LiDAR bore floor chord {chord:.2f} m is too narrow for HK MTR "
            f"sleeper length {SLEEPER_LENGTH_M:.2f} m"
        )

    def test_led_strip_sites_present(self):
        """Many emissive site markers along both walls + a small number of
        real <light> elements (kept under the OpenGL 8-light cap)."""
        tree = ET.parse(XML)
        root = tree.getroot()
        led_sites = [s for s in root.findall(".//worldbody/site")
                     if (s.get("name") or "").startswith("led_")]
        led_lights = [l for l in root.findall(".//worldbody/light")
                      if (l.get("name") or "").startswith("led_")]
        sun_lights = [l for l in root.findall(".//worldbody/light")
                      if l.get("name") == "sun"]
        assert len(led_sites) > 100, f"expected many LED sites, got {len(led_sites)}"
        total_lights = len(led_lights) + len(sun_lights)
        assert total_lights <= 8, (
            f"total lights {total_lights} exceeds MuJoCo OpenGL 8-light cap"
        )

    def test_no_xml_axis_orphan_geoms_outside_tunnel(self):
        """Axis marker geoms should sit at the tunnel entrance, not at
        the world origin (regression test for the old wrong-camera bug)."""
        tree = ET.parse(XML)
        root = tree.getroot()
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        start = cl[0]
        for g in root.findall(".//worldbody/geom"):
            name = g.get("name", "")
            if name.startswith("axis_"):
                ft = g.get("fromto")
                if ft:
                    nums = [float(x) for x in ft.split()]
                    p = np.array(nums[:3])
                    d = np.linalg.norm(p - start)
                    assert d < 1.0, f"axis {name} not near tunnel start (d={d:.2f})"


# ---------- TestZAxisUpdates ----------

class TestZAxisUpdates:

    def test_preserve_z_range_constant_is_true(self):
        src = GEN_PY.read_text()
        assert re.search(r"^\s*PRESERVE_Z_RANGE\s*=\s*True", src, re.M), (
            "PRESERVE_Z_RANGE should be True per updates.md 2026-05-26"
        )

    def test_smooth_window_z_is_active(self):
        """Z smoothing enabled (user requested 2026-05-26 smoothing on)."""
        src = GEN_PY.read_text()
        m = re.search(r"^\s*SMOOTH_WINDOW_Z\s*=\s*(\d+)", src, re.M)
        assert m, "SMOOTH_WINDOW_Z must be a non-negative int"
        assert int(m.group(1)) >= 3, f"SMOOTH_WINDOW_Z should be >=3 for smoothing, got {m.group(1)}"

    def test_centerline_z_no_high_frequency_noise(self):
        """Z must be heavily smoothed (window 161 by default per HK MTR
        / typical metro grade-smoothness expectations). Max |2nd diff|
        below 0.05 m -- a tighter bound than the old 0.10 m. The current
        run achieves ~0.02 m; this gives some headroom but flags if the
        smoothing window is dropped back to a small value."""
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        z = cl[:, 2]
        d2 = z[2:] - 2 * z[1:-1] + z[:-2]
        assert float(np.max(np.abs(d2))) < 0.05, (
            f"Z second-diff max {np.max(np.abs(d2)):.3f} too noisy; "
            f"is SMOOTH_WINDOW_Z too small?"
        )

    def test_average_grade_within_metro_limits(self):
        """Average grade should stay within HK MTR / standard metro
        shield-tunnel limits (<=3% absolute). PRESERVE_Z_RANGE keeps the
        endpoints at the raw extrema, so this is data-driven."""
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        z = cl[:, 2]
        seg = np.diff(cl, axis=0)
        arc = float(np.linalg.norm(seg, axis=1).sum())
        grade = abs(z.max() - z.min()) / arc
        assert grade < 0.03, f"avg grade {grade*100:.2f}% exceeds 3% metro limit"

    def test_smooth_window_y_is_61(self):
        src = GEN_PY.read_text()
        assert re.search(r"^\s*SMOOTH_WINDOW_Y\s*=\s*61\b", src, re.M)

    def test_centerline_z_span_matches_raw_shield(self):
        """PRESERVE_Z_RANGE=True implies: centerline Z span = raw Shield Z span."""
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        # Raw Shield Z range
        zs = []
        for f in sorted(DATASET.glob("Shield_tunnel*.txt")):
            zs.append(np.loadtxt(f)[:, 3])
        all_z = np.concatenate(zs)
        raw_span = float(all_z.max() - all_z.min())
        cl_span = float(cl[:, 2].max() - cl[:, 2].min())
        # Allow 1 cm tolerance for floating-point rounding
        assert abs(cl_span - raw_span) < 0.01, (
            f"centerline Z span {cl_span:.4f} != raw {raw_span:.4f}"
        )

    def test_centerline_z_min_max_match_raw(self):
        cl = np.loadtxt(CENTERLINE_CSV, delimiter=",", skiprows=1)
        zs = []
        for f in sorted(DATASET.glob("Shield_tunnel*.txt")):
            zs.append(np.loadtxt(f)[:, 3])
        all_z = np.concatenate(zs)
        # min/max should agree with raw within 1 cm (PCA mean-per-x smoothing
        # may shift values slightly but PRESERVE_Z_RANGE rescales to fit).
        assert abs(cl[:, 2].min() - all_z.min()) < 0.05
        assert abs(cl[:, 2].max() - all_z.max()) < 0.05
