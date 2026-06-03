#!/usr/bin/env python3
"""
join_trajectories.py

Determines which trajectory runs share the same physical tunnel, joins them,
and generates a combined image per tunnel suitable for 3-D MuJoCo construction.

Joining strategy (global — not split by filename prefix):
  1. Spatial clustering  — runs whose point clouds are within SPATIAL_THRESHOLD_M
                           of each other belong to the same physical environment.
  2. Temporal ordering   — within each cluster, sort runs by start timestamp.
  3. Continuity decision — "stitch" if runs are temporally adjacent (gap <
                           TEMPORAL_GAP_S) AND endpoints are close (< ENDPOINT_M);
                           otherwise "overlay" (union of all explored positions).

Notable finding: Tunneling_tunnel4 starts at the same entrance as the Shield
tunnels (~1000, 2002), so it is spatially clustered with Shield runs even though
its name says "Tunneling". This is flagged clearly in the output.
"""

import sys
import types

# Fix mpl_toolkits shadowing — must precede matplotlib.pyplot import.
_USER_TOOLKITS = "/home/badrikanath/.local/lib/python3.10/site-packages/mpl_toolkits"
_mt_stub = types.ModuleType("mpl_toolkits")
_mt_stub.__path__ = [_USER_TOOLKITS]
_mt_stub.__package__ = "mpl_toolkits"
sys.modules["mpl_toolkits"] = _mt_stub

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

_HAS_3D = False
try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    _HAS_3D = True
except Exception:
    pass

import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import List, Dict, Tuple

from trajectory_analysis import parse_trajectory, TrajectoryData, discover_txt_files

# ── Config ─────────────────────────────────────────────────────────────────────
DATASET_DIR = Path(
    "/mnt/storage/mining_env_dataset_folder/rss/dataset/"
    "drive-download-20260524T182528Z-3-001"
)
OUTPUT_DIR = Path("/mnt/storage/mining_env_dataset_folder/rss/plots")

SPATIAL_THRESHOLD_M  = 25.0   # min inter-cloud dist (m) → same physical tunnel
ENDPOINT_THRESHOLD_M =  5.0   # end→next-start dist (m) → runs are "sequential"
TEMPORAL_GAP_S       = 600.0  # max inter-run gap (s) to qualify for "stitch"

_TAB10 = list(plt.get_cmap("tab10").colors)
_BG    = "#0d0d0d"
_PBG   = "#111111"
_GRID  = 0.15


# ── Data structure ─────────────────────────────────────────────────────────────
@dataclass
class TunnelGroup:
    name:       str                  # descriptive label, e.g. "Shield_plus_T4"
    runs:       List[TrajectoryData] # sorted by start timestamp
    method:     str                  # "stitch" | "overlay" | "single"
    note:       str                  # human-readable explanation
    positions:  np.ndarray           # (N, 3) combined positions
    boundaries: List[int]            # start index per run in positions


# ── Spatial clustering ─────────────────────────────────────────────────────────
def _min_cloud_dist(a: np.ndarray, b: np.ndarray) -> float:
    """Minimum Euclidean distance between two (N,3) point clouds (downsampled)."""
    sa = max(1, len(a) // 300)
    sb = max(1, len(b) // 300)
    da, db = a[::sa], b[::sb]
    diff = da[:, None, :] - db[None, :, :]        # (Na, Nb, 3)
    return float(np.sqrt((diff ** 2).sum(-1)).min())


def _spatial_clusters(runs: List[TrajectoryData]) -> List[List[int]]:
    """Union-find: group run indices where min inter-cloud dist < threshold."""
    n = len(runs)
    parent = list(range(n))

    def _find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for i in range(n):
        for j in range(i + 1, n):
            d = _min_cloud_dist(runs[i].positions, runs[j].positions)
            if d < SPATIAL_THRESHOLD_M:
                pi, pj = _find(i), _find(j)
                if pi != pj:
                    parent[pi] = pj

    groups: Dict[int, List[int]] = {}
    for i in range(n):
        groups.setdefault(_find(i), []).append(i)
    return list(groups.values())


# ── Temporal analysis ──────────────────────────────────────────────────────────
def _sort_and_analyse(
    runs: List[TrajectoryData],
) -> Tuple[List[TrajectoryData], List[float], List[float]]:
    """Sort by start-timestamp; return (sorted, time_gaps_s, endpoint_dists_m)."""
    srt = sorted(runs, key=lambda r: r.timestamps[0])
    gaps, ep_dists = [], []
    for i in range(len(srt) - 1):
        gaps.append(srt[i + 1].timestamps[0] - srt[i].timestamps[-1])
        ep_dists.append(
            float(np.linalg.norm(srt[i].positions[-1] - srt[i + 1].positions[0]))
        )
    return srt, gaps, ep_dists


def _decide_method(
    runs: List[TrajectoryData],
    gaps: List[float],
    ep_dists: List[float],
) -> Tuple[str, str]:
    if len(runs) == 1:
        return "single", "Single run — no joining required."

    can_stitch = (
        bool(gaps)
        and max(gaps) < TEMPORAL_GAP_S
        and max(ep_dists) < ENDPOINT_THRESHOLD_M
    )
    if can_stitch:
        return (
            "stitch",
            f"All {len(runs)} runs are temporally adjacent "
            f"(max gap {max(gaps):.0f} s) and spatially continuous "
            f"(max endpoint dist {max(ep_dists):.1f} m) → concatenated in time order.",
        )

    # Check out-and-back pattern (start ≈ end within 20 m)
    oa = sum(
        1 for r in runs
        if np.linalg.norm(r.positions[0] - r.positions[-1]) < 20.0
    )
    gap_str = ", ".join(f"{g:.0f}s" for g in gaps)
    ed_str  = ", ".join(f"{d:.1f}m" for d in ep_dists)

    parts = []
    if oa:
        parts.append(f"{oa}/{len(runs)} runs are out-and-back (start≈end).")
    parts.append("Shown as spatial union of all explored positions.")
    parts.append(f"Inter-run time gaps: [{gap_str}].")
    parts.append(f"End→start distances: [{ed_str}].")
    return "overlay", " ".join(parts)


def _build_combined(
    runs: List[TrajectoryData],
) -> Tuple[np.ndarray, List[int]]:
    parts, boundaries, offset = [], [], 0
    for r in runs:
        boundaries.append(offset)
        parts.append(r.positions)
        offset += len(r.positions)
    return np.vstack(parts), boundaries


# ── Group builder ──────────────────────────────────────────────────────────────
def _make_name(sorted_runs: List[TrajectoryData], tun_cluster_idx: int, n_tun_clusters: int) -> str:
    shield_n = sum(
        1 for r in sorted_runs if Path(r.source_file).stem.startswith("Shield")
    )
    tunnel_n = len(sorted_runs) - shield_n

    if shield_n > 0 and tunnel_n == 0:
        return "Shield"
    if shield_n > 0 and tunnel_n > 0:
        tnames = [Path(r.source_file).stem for r in sorted_runs if Path(r.source_file).stem.startswith("Tunneling")]
        return f"Shield_plus_{'_'.join(tnames)}"
    # Pure tunneling
    if n_tun_clusters > 1:
        return f"Tunneling_{chr(ord('A') + tun_cluster_idx)}"
    return "Tunneling"


def build_groups(all_runs: List[TrajectoryData]) -> List[TunnelGroup]:
    print("\nComputing pairwise spatial distances …")
    raw_clusters = _spatial_clusters(all_runs)
    # sort clusters by their earliest timestamp
    raw_clusters.sort(
        key=lambda idx_list: min(all_runs[i].timestamps[0] for i in idx_list)
    )

    # count pure-tunneling clusters for naming
    n_tun_clusters = sum(
        1 for c in raw_clusters
        if all(Path(all_runs[j].source_file).stem.startswith("Tunneling") for j in c)
    )
    tun_letter = 0

    groups: List[TunnelGroup] = []
    for idx_list in raw_clusters:
        cluster_runs = [all_runs[i] for i in idx_list]
        sorted_runs, gaps, ep_dists = _sort_and_analyse(cluster_runs)
        method, note = _decide_method(sorted_runs, gaps, ep_dists)
        combined, boundaries = _build_combined(sorted_runs)

        shield_n = sum(
            1 for r in sorted_runs if Path(r.source_file).stem.startswith("Shield")
        )
        tunnel_n = len(sorted_runs) - shield_n

        if shield_n > 0:
            name = _make_name(sorted_runs, 0, 0)
        else:
            name = _make_name(sorted_runs, tun_letter, n_tun_clusters)
            tun_letter += 1

        groups.append(TunnelGroup(
            name=name,
            runs=sorted_runs,
            method=method,
            note=note,
            positions=combined,
            boundaries=boundaries,
        ))

    return groups


# ── Plotting ───────────────────────────────────────────────────────────────────
def _style_ax(ax, title: str, xlabel: str, ylabel: str, equal: bool = False) -> None:
    ax.set_facecolor(_PBG)
    ax.tick_params(colors="white", labelsize=7)
    ax.set_xlabel(xlabel, color="white", fontsize=8)
    ax.set_ylabel(ylabel, color="white", fontsize=8)
    ax.set_title(title, color="white", fontsize=9, pad=4)
    for spine in ax.spines.values():
        spine.set_edgecolor("#444444")
    ax.grid(True, alpha=_GRID, color="white")
    if equal:
        ax.set_aspect("equal", adjustable="datalim")


def _run_slices(g: TunnelGroup):
    """Yield (positions, colour, label) per run."""
    ends = g.boundaries[1:] + [len(g.positions)]
    for i, (s, e) in enumerate(zip(g.boundaries, ends)):
        yield g.positions[s:e], _TAB10[i % 10], Path(g.runs[i].source_file).stem


def plot_group(g: TunnelGroup, out: Path) -> Path:
    fig = plt.figure(figsize=(22, 15))
    fig.patch.set_facecolor(_BG)
    gs = GridSpec(2, 3, figure=fig, hspace=0.38, wspace=0.3)

    slices = list(_run_slices(g))

    # ── Panel 1: 3-D trajectory ─────────────────────────────────────────────
    if _HAS_3D:
        ax3 = fig.add_subplot(gs[0, 0], projection="3d")
        ax3.set_facecolor(_PBG)
        for pos, col, lbl in slices:
            ax3.plot(pos[:, 0], pos[:, 1], pos[:, 2], "-", color=col, lw=0.7, alpha=0.8)
            ax3.scatter(*pos[0],  color=col, s=20, marker="o", zorder=5)
            ax3.scatter(*pos[-1], color=col, s=20, marker="s", zorder=5)
        ax3.set_xlabel("X (m)", color="white", fontsize=7)
        ax3.set_ylabel("Y (m)", color="white", fontsize=7)
        ax3.set_zlabel("Z (m)", color="white", fontsize=7)
        ax3.set_title("3-D Combined Trajectory", color="white", fontsize=9)
        ax3.tick_params(colors="white", labelsize=6)
        ax3.xaxis.pane.fill = ax3.yaxis.pane.fill = ax3.zaxis.pane.fill = False
        ax3.grid(True, alpha=0.15)
    else:
        ax3 = fig.add_subplot(gs[0, 0])
        for pos, col, _ in slices:
            ax3.plot(pos[:, 0], pos[:, 2], "-", color=col, lw=0.7, alpha=0.8)
        _style_ax(ax3, "XZ Side View (3D unavailable)", "X (m)", "Z (m)")

    # ── Panel 2: XY top-down (floor plan) ───────────────────────────────────
    ax_xy = fig.add_subplot(gs[0, 1])
    for pos, col, lbl in slices:
        ax_xy.plot(pos[:, 0], pos[:, 1], "-", color=col, lw=0.7, alpha=0.75, label=lbl)
        ax_xy.scatter(*pos[0,  :2], color=col, s=25, marker="o", zorder=5)
        ax_xy.scatter(*pos[-1, :2], color=col, s=25, marker="s", zorder=5)
    ax_xy.legend(fontsize=6, loc="best",
                 facecolor="#1a1a2e", edgecolor="#555599", labelcolor="white")
    _style_ax(ax_xy, "XY Top-Down — Floor Plan", "X (m)", "Y (m)", equal=True)

    # ── Panel 3: Z depth vs cumulative path distance ─────────────────────────
    ax_z = fig.add_subplot(gs[0, 2])
    for pos, col, _ in slices:
        cum = np.concatenate(
            [[0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
        )
        ax_z.plot(cum, pos[:, 2], "-", color=col, lw=0.7, alpha=0.75)
    _style_ax(ax_z, "Z Depth vs Path Distance", "Cumulative Distance (m)", "Z (m)")

    # ── Panel 4: XZ side view ────────────────────────────────────────────────
    ax_xz = fig.add_subplot(gs[1, 0])
    for pos, col, _ in slices:
        ax_xz.plot(pos[:, 0], pos[:, 2], "-", color=col, lw=0.7, alpha=0.75)
    _style_ax(ax_xz, "XZ Side View", "X (m)", "Z (m)")

    # ── Panel 5: YZ side view ────────────────────────────────────────────────
    ax_yz = fig.add_subplot(gs[1, 1])
    for pos, col, _ in slices:
        ax_yz.plot(pos[:, 1], pos[:, 2], "-", color=col, lw=0.7, alpha=0.75)
    _style_ax(ax_yz, "YZ Side View", "Y (m)", "Z (m)")

    # ── Panel 6: summary text ────────────────────────────────────────────────
    ax_info = fig.add_subplot(gs[1, 2])
    ax_info.set_facecolor(_PBG)
    ax_info.axis("off")

    p = g.positions
    total_path = sum(
        float(np.sum(np.linalg.norm(np.diff(r.positions, axis=0), axis=1)))
        for r in g.runs
    )

    lines = [
        f"GROUP  : {g.name}",
        f"METHOD : {g.method.upper()}",
        f"RUNS   : {len(g.runs)}",
        "",
        "Files (temporal order):",
    ]
    for i, r in enumerate(g.runs):
        stem = Path(r.source_file).stem
        dt   = r.timestamps[-1] - r.timestamps[0]
        t0   = r.timestamps[0]
        lines.append(f"  {i+1:2d}. {stem}")
        lines.append(f"       {len(r.positions)} poses  {dt:.0f}s  t={t0:.0f}")

    lines += [
        "",
        f"Total poses : {len(p):,}",
        f"Total path  : {total_path:.0f} m",
        f"X range     : {p[:,0].min():.1f} → {p[:,0].max():.1f} m",
        f"Y range     : {p[:,1].min():.1f} → {p[:,1].max():.1f} m",
        f"Z range     : {p[:,2].min():.1f} → {p[:,2].max():.1f} m",
        "",
        "Join note:",
    ]
    # word-wrap note at 44 chars
    words, cur = g.note.split(), ""
    for w in words:
        if len(cur) + len(w) + 1 > 44:
            lines.append("  " + cur.rstrip())
            cur = w + " "
        else:
            cur += w + " "
    if cur.strip():
        lines.append("  " + cur.rstrip())

    ax_info.text(
        0.04, 0.97, "\n".join(lines),
        transform=ax_info.transAxes,
        fontsize=7.2, color="white", va="top", fontfamily="monospace",
        bbox=dict(facecolor="#12122a", edgecolor="#4444aa", alpha=0.95, pad=7),
    )

    fig.suptitle(
        f"Combined Tunnel — {g.name}   [{g.method}]",
        fontsize=13, color="white", y=0.995,
    )
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=_BG)
    plt.close(fig)
    return out


# ── Main ───────────────────────────────────────────────────────────────────────
def main() -> List[TunnelGroup]:
    print("=" * 65)
    print("Tunnel trajectory join analysis")
    print("=" * 65)

    txt_files = sorted(discover_txt_files(DATASET_DIR))
    print(f"\nLoading {len(txt_files)} trajectory files …")

    all_runs: List[TrajectoryData] = []
    for f in txt_files:
        td = parse_trajectory(f)
        all_runs.append(td)
        t0, t1 = td.timestamps[0], td.timestamps[-1]
        ep = np.linalg.norm(td.positions[0] - td.positions[-1])
        print(
            f"  {f.stem:<30s}  {len(td.positions):5d} poses  "
            f"dur={t1-t0:.0f}s  "
            f"start=({td.positions[0,0]:.1f},{td.positions[0,1]:.1f})  "
            f"end=({td.positions[-1,0]:.1f},{td.positions[-1,1]:.1f})  "
            f"ep={ep:.1f}m"
        )

    groups = build_groups(all_runs)

    print(f"\n{'─'*65}")
    print(f"Found {len(groups)} physical tunnel group(s):")
    for g in groups:
        names = [Path(r.source_file).stem for r in g.runs]
        p = g.positions
        print(f"\n  ┌─ GROUP: {g.name}")
        print(f"  │  Files  ({len(g.runs)}): {names}")
        print(f"  │  Method : {g.method}")
        print(f"  │  Poses  : {len(p):,}")
        print(f"  │  X      : [{p[:,0].min():.1f}, {p[:,0].max():.1f}]")
        print(f"  │  Y      : [{p[:,1].min():.1f}, {p[:,1].max():.1f}]")
        print(f"  └  Z      : [{p[:,2].min():.1f}, {p[:,2].max():.1f}]")
        print(f"     Note   : {g.note}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"\nGenerating combined plots …")
    generated: List[Path] = []
    for g in groups:
        out = OUTPUT_DIR / f"combined_{g.name}_trajectory.png"
        plot_group(g, out)
        generated.append(out)
        print(f"  ✓  {out.name}")

    print(f"\n{'='*65}")
    print(f"Done. {len(generated)} combined image(s) saved to {OUTPUT_DIR}")
    return groups


if __name__ == "__main__":
    main()
