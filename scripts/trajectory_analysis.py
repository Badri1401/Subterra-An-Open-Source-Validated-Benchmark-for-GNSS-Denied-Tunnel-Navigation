#!/usr/bin/env python3
"""
Trajectory analysis for LiDAR/odometry data from tunnel mining environments.

File format (8 space-separated columns, no header):
    timestamp  x  y  z  qx  qy  qz  qw

    timestamp  - Unix time in seconds (float64, strictly monotonically increasing)
    x, y, z    - 3-D position in meters
    qx,qy,qz,qw - Quaternion orientation
                  (all-zero in this dataset = orientation not recorded)

Two environments present:
    Shield    - ~10 runs, starting near (1000, 2004, 10), tunnel extends ~350 m in Y
    Tunneling - ~5 runs, smaller-scale, starting near (100, 202, 11)

Usage:
    python trajectory_analysis.py [file_or_dir ...]
    python trajectory_analysis.py                   # processes default dataset dir
"""

from __future__ import annotations

import sys
import types
import argparse
import numpy as np

# ---------------------------------------------------------------------------
# Fix mpl_toolkits namespace collision BEFORE importing matplotlib.pyplot.
# On this machine the Ubuntu system package provides an old mpl_toolkits in
# /usr/lib/python3/dist-packages that shadows the pip-installed one.
# Because the system package has an __init__.py it takes priority over the
# namespace-package in ~/.local/…  We inject a synthetic namespace module so
# pyplot picks up the correct mpl_toolkits.mplot3d when it registers 3-D.
# ---------------------------------------------------------------------------
_USER_TOOLKITS = '/home/badrikanath/.local/lib/python3.10/site-packages/mpl_toolkits'
_mt_stub = types.ModuleType('mpl_toolkits')
_mt_stub.__path__ = [_USER_TOOLKITS]
_mt_stub.__package__ = 'mpl_toolkits'
sys.modules['mpl_toolkits'] = _mt_stub          # must happen before pyplot import

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path

_HAS_3D: bool = False
try:
    from mpl_toolkits.mplot3d import Axes3D      # noqa: F401
    _HAS_3D = True
except Exception as _e:
    import warnings
    warnings.warn(
        f"mpl_toolkits.mplot3d unavailable ({_e}) — 3-D panel replaced with XZ view.",
        stacklevel=1,
    )

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

EXPECTED_COLUMNS = 8

# Underground robots typically travel < 2 m/s; allow generous headroom
MAX_SPEED_THRESHOLD_M_S = 5.0

# Flag gaps larger than 2 s (sensor glitch or hold-still interval)
MAX_TIME_GAP_S = 2.0

# Quaternion norm below this ⟹ treated as "not recorded" (all-zero)
QUATERNION_ZERO_THRESHOLD = 1e-6

# Norm should be 0 (not recorded) or 1 (±1 % tolerance)
QUATERNION_UNIT_TOL = 0.01

DEFAULT_DATASET_DIR = Path(__file__).parent.parent / "dataset" / "drive-download-20260524T182528Z-3-001"
DEFAULT_OUTPUT_DIR  = Path(__file__).parent.parent / "plots"


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class TrajectoryData:
    """Parsed trajectory from a single file."""
    timestamps:  np.ndarray    # (N,)   Unix timestamps [s]
    positions:   np.ndarray    # (N, 3) columns: x, y, z [m]
    quaternions: np.ndarray    # (N, 4) columns: qx, qy, qz, qw
    source_file: str
    row_count:   int

    @property
    def x(self) -> np.ndarray: return self.positions[:, 0]
    @property
    def y(self) -> np.ndarray: return self.positions[:, 1]
    @property
    def z(self) -> np.ndarray: return self.positions[:, 2]
    @property
    def t_rel(self) -> np.ndarray: return self.timestamps - self.timestamps[0]


@dataclass
class ValidationResult:
    is_valid: bool
    warnings: List[str] = field(default_factory=list)
    errors:   List[str] = field(default_factory=list)
    stats:    dict       = field(default_factory=dict)

    def report(self) -> str:
        src = self.stats.get("source_file", "unknown")
        lines = [
            f"{'='*60}",
            f"File   : {src}",
            f"Status : {'PASS ✓' if self.is_valid else 'FAIL ✗'}",
        ]
        if self.errors:
            lines.append("Errors :")
            for e in self.errors:
                lines.append(f"  [ERROR] {e}")
        if self.warnings:
            lines.append("Warnings:")
            for w in self.warnings:
                lines.append(f"  [WARN]  {w}")
        lines.append("Stats  :")
        skip = {"source_file"}
        for k, v in self.stats.items():
            if k not in skip:
                lines.append(f"  {k:<22}: {v}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_trajectory(filepath: str | Path) -> TrajectoryData:
    """Parse a trajectory .txt file into a :class:`TrajectoryData` object.

    Raises
    ------
    FileNotFoundError  - if filepath does not exist
    ValueError         - if any row has wrong column count or non-numeric data
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {filepath}")

    rows: List[List[float]] = []
    with open(path, "r") as fh:
        for lineno, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != EXPECTED_COLUMNS:
                raise ValueError(
                    f"{path.name} line {lineno}: "
                    f"expected {EXPECTED_COLUMNS} columns, got {len(parts)}"
                )
            try:
                rows.append([float(p) for p in parts])
            except ValueError as exc:
                raise ValueError(
                    f"{path.name} line {lineno}: non-numeric value — {exc}"
                ) from exc

    if not rows:
        raise ValueError(f"Empty file: {filepath}")

    arr = np.array(rows, dtype=np.float64)   # (N, 8)
    return TrajectoryData(
        timestamps=arr[:, 0],
        positions=arr[:, 1:4],
        quaternions=arr[:, 4:8],
        source_file=str(path),
        row_count=len(rows),
    )


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_trajectory(data: TrajectoryData) -> ValidationResult:
    """Run all integrity checks and collect statistics.

    Checks
    ------
    1. Finite values (no NaN / Inf)
    2. Timestamp strict monotonicity
    3. Large time gaps
    4. Position continuity (max-speed plausibility)
    5. Quaternion norms (0 = not recorded, 1 = valid unit quaternion)
    6. Duplicate consecutive positions (robot stationary for many frames)
    """
    errors: List[str] = []
    warns:  List[str] = []
    N = data.row_count

    # 1. Finite check
    if not np.all(np.isfinite(data.timestamps)):
        errors.append("Non-finite value(s) in timestamps")
    if not np.all(np.isfinite(data.positions)):
        errors.append("Non-finite value(s) in positions")
    if not np.all(np.isfinite(data.quaternions)):
        errors.append("Non-finite value(s) in quaternions")

    dt = np.diff(data.timestamps)   # (N-1,)

    # 2. Monotonicity
    non_mono = np.where(dt <= 0)[0]
    if len(non_mono) > 0:
        errors.append(
            f"{len(non_mono)} non-positive time step(s) "
            f"at row indices {non_mono[:5].tolist()}"
        )

    # 3. Large gaps
    if len(dt) > 0:
        large_gap_idx = np.where(dt > MAX_TIME_GAP_S)[0]
        if len(large_gap_idx) > 0:
            warns.append(
                f"{len(large_gap_idx)} time gap(s) > {MAX_TIME_GAP_S} s "
                f"(largest={dt.max():.3f} s at row {dt.argmax()})"
            )

    # 4. Speed plausibility
    if N > 1 and len(dt) > 0:
        safe_dt = np.where(dt > 0, dt, np.nan)
        disp  = np.diff(data.positions, axis=0)          # (N-1, 3)
        dist  = np.linalg.norm(disp, axis=1)              # (N-1,)
        speed = dist / safe_dt                             # (N-1,) m/s
        fast  = np.where(speed > MAX_SPEED_THRESHOLD_M_S)[0]
        if len(fast) > 0:
            warns.append(
                f"{len(fast)} position jump(s) exceed {MAX_SPEED_THRESHOLD_M_S} m/s "
                f"(max={np.nanmax(speed):.2f} m/s at row {fast[0]})"
            )

    # 5. Quaternion norms
    qnorms = np.linalg.norm(data.quaternions, axis=1)
    all_zero = np.all(qnorms < QUATERNION_ZERO_THRESHOLD)
    if all_zero:
        warns.append("All quaternions are zero vectors — orientation not recorded")
    else:
        # must be 0 or unit quaternion
        invalid_q = np.where(
            (qnorms > QUATERNION_ZERO_THRESHOLD)
            & (np.abs(qnorms - 1.0) > QUATERNION_UNIT_TOL)
        )[0]
        if len(invalid_q) > 0:
            errors.append(
                f"{len(invalid_q)} quaternion(s) with norm ≠ 0 or 1 "
                f"(first at row {invalid_q[0]})"
            )

    # 6. Long static stretches (same position for > 10 consecutive rows)
    if N > 1:
        pos_delta = np.linalg.norm(np.diff(data.positions, axis=0), axis=1)
        static_mask = pos_delta == 0.0
        max_run = _max_run_length(static_mask)
        if max_run > 10:
            warns.append(
                f"Longest static stretch (zero displacement): {max_run} consecutive rows"
            )

    # --- Statistics ---
    duration = float(data.timestamps[-1] - data.timestamps[0])
    path_len = float(np.sum(np.linalg.norm(np.diff(data.positions, axis=0), axis=1)))
    net_disp = float(np.linalg.norm(data.positions[-1] - data.positions[0]))

    stats: dict = {
        "source_file":       data.source_file,
        "num_poses":         N,
        "duration_s":        f"{duration:.3f}",
        "path_length_m":     f"{path_len:.3f}",
        "net_displacement_m":f"{net_disp:.3f}",
        "start_pos_xyz":     _fmt_xyz(data.positions[0]),
        "end_pos_xyz":       _fmt_xyz(data.positions[-1]),
        "x_range_m":         _fmt_range(data.x),
        "y_range_m":         _fmt_range(data.y),
        "z_range_m":         _fmt_range(data.z),
        "mean_dt_s":         f"{float(np.mean(dt)):.4f}" if len(dt) else "N/A",
        "max_dt_s":          f"{float(np.max(dt)):.4f}"  if len(dt) else "N/A",
        "orientation":       "not recorded (all-zero)" if all_zero else "present",
    }

    return ValidationResult(
        is_valid=len(errors) == 0,
        warnings=warns,
        errors=errors,
        stats=stats,
    )


def _fmt_xyz(v: np.ndarray) -> str:
    return f"({v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f})"

def _fmt_range(v: np.ndarray) -> str:
    return f"[{v.min():.4f}, {v.max():.4f}]"

def _max_run_length(mask: np.ndarray) -> int:
    """Return length of the longest consecutive True run in a 1-D bool array."""
    if not np.any(mask):
        return 0
    max_run = cur = 0
    for val in mask:
        cur = cur + 1 if val else 0
        max_run = max(max_run, cur)
    return max_run


# ---------------------------------------------------------------------------
# Plotting (single file)
# ---------------------------------------------------------------------------

def plot_trajectory(
    data: TrajectoryData,
    result: ValidationResult,
    output_path: Optional[str | Path] = None,
) -> Path:
    """Render a 6-panel trajectory figure and save as PNG.

    Returns the output :class:`Path`.
    """
    fname = Path(data.source_file).stem
    if output_path is None:
        output_path = DEFAULT_OUTPUT_DIR / f"{fname}_trajectory.png"
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    x, y, z, t = data.x, data.y, data.z, data.t_rel

    status_str = "VALID ✓" if result.is_valid else "INVALID ✗"
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle(
        f"{fname}  —  {data.row_count} poses  "
        f"|  {result.stats.get('duration_s', '?')} s  "
        f"|  {result.stats.get('path_length_m', '?')} m path  "
        f"|  {status_str}",
        fontsize=13, fontweight="bold",
    )

    gs = GridSpec(2, 3, figure=fig, hspace=0.40, wspace=0.32)

    # ------------------------------------------------------------------
    # Panel 1 — 3-D trajectory (or XZ fallback when 3D unavailable)
    # ------------------------------------------------------------------
    if _HAS_3D:
        ax3d = fig.add_subplot(gs[0, 0], projection="3d")
        sc = ax3d.scatter(x, y, z, c=t, cmap="plasma", s=5, alpha=0.85, depthshade=True)
        ax3d.scatter(*data.positions[0],  color="limegreen", s=80, zorder=5, label="Start")
        ax3d.scatter(*data.positions[-1], color="red",       s=80, marker="X", zorder=5, label="End")
        ax3d.set_xlabel("X [m]", fontsize=8, labelpad=2)
        ax3d.set_ylabel("Y [m]", fontsize=8, labelpad=2)
        ax3d.set_zlabel("Z [m]", fontsize=8, labelpad=2)
        ax3d.set_title("3-D Trajectory", fontsize=10)
        ax3d.legend(fontsize=7)
        fig.colorbar(sc, ax=ax3d, label="Time [s]", shrink=0.55, pad=0.1)
    else:
        ax3d = fig.add_subplot(gs[0, 0])
        sc = ax3d.scatter(x, z, c=t, cmap="plasma", s=5, alpha=0.85)
        ax3d.scatter(x[0], z[0], color="limegreen", s=80, zorder=5, label="Start")
        ax3d.scatter(x[-1], z[-1], color="red", s=80, marker="X", zorder=5, label="End")
        ax3d.set_xlabel("X [m]", fontsize=9)
        ax3d.set_ylabel("Z [m]", fontsize=9)
        ax3d.set_title("XZ View (3D projection N/A)", fontsize=10)
        ax3d.legend(fontsize=7)
        ax3d.grid(True, alpha=0.3)
        fig.colorbar(sc, ax=ax3d, label="Time [s]", shrink=0.75)

    # ------------------------------------------------------------------
    # Panel 2 — XY top-down
    # ------------------------------------------------------------------
    ax_xy = fig.add_subplot(gs[0, 1])
    sc2 = ax_xy.scatter(x, y, c=t, cmap="plasma", s=4, alpha=0.85)
    ax_xy.scatter(x[0], y[0], color="limegreen", s=80, zorder=5, label="Start")
    ax_xy.scatter(x[-1], y[-1], color="red",  s=80, marker="X", zorder=5, label="End")
    # Sparse direction arrows
    _add_direction_arrows(ax_xy, x, y, n_arrows=15, color="dimgray", alpha=0.5)
    ax_xy.set_xlabel("X [m]", fontsize=9)
    ax_xy.set_ylabel("Y [m]", fontsize=9)
    ax_xy.set_title("Top View (XY)", fontsize=10)
    ax_xy.legend(fontsize=7)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.25)
    fig.colorbar(sc2, ax=ax_xy, label="Time [s]", shrink=0.75)

    # ------------------------------------------------------------------
    # Panel 3 — Depth (Z) profile vs time
    # ------------------------------------------------------------------
    ax_z = fig.add_subplot(gs[0, 2])
    ax_z.plot(t, z, color="steelblue", lw=1.5)
    ax_z.scatter(t[0],  z[0],  color="limegreen", s=60, zorder=5, label="Start")
    ax_z.scatter(t[-1], z[-1], color="red",        s=60, marker="X", zorder=5, label="End")
    ax_z.set_xlabel("Time [s]", fontsize=9)
    ax_z.set_ylabel("Z [m]", fontsize=9)
    ax_z.set_title("Depth Profile  (Z vs Time)", fontsize=10)
    ax_z.legend(fontsize=7)
    ax_z.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 4 — XZ side view
    # ------------------------------------------------------------------
    ax_xz = fig.add_subplot(gs[1, 0])
    sc4 = ax_xz.scatter(x, z, c=t, cmap="plasma", s=4, alpha=0.85)
    ax_xz.scatter(x[0], z[0], color="limegreen", s=80, zorder=5, label="Start")
    ax_xz.scatter(x[-1], z[-1], color="red", s=80, marker="X", zorder=5, label="End")
    ax_xz.set_xlabel("X [m]", fontsize=9)
    ax_xz.set_ylabel("Z [m]", fontsize=9)
    ax_xz.set_title("Side View (XZ)", fontsize=10)
    ax_xz.legend(fontsize=7)
    ax_xz.grid(True, alpha=0.3)
    fig.colorbar(sc4, ax=ax_xz, label="Time [s]", shrink=0.75)

    # ------------------------------------------------------------------
    # Panel 5 — YZ side view
    # ------------------------------------------------------------------
    ax_yz = fig.add_subplot(gs[1, 1])
    sc5 = ax_yz.scatter(y, z, c=t, cmap="plasma", s=4, alpha=0.85)
    ax_yz.scatter(y[0], z[0], color="limegreen", s=80, zorder=5, label="Start")
    ax_yz.scatter(y[-1], z[-1], color="red",      s=80, marker="X", zorder=5, label="End")
    ax_yz.set_xlabel("Y [m]", fontsize=9)
    ax_yz.set_ylabel("Z [m]", fontsize=9)
    ax_yz.set_title("Side View (YZ)", fontsize=10)
    ax_yz.legend(fontsize=7)
    ax_yz.grid(True, alpha=0.3)
    fig.colorbar(sc5, ax=ax_yz, label="Time [s]", shrink=0.75)

    # ------------------------------------------------------------------
    # Panel 6 — Statistics / validation text
    # ------------------------------------------------------------------
    ax_txt = fig.add_subplot(gs[1, 2])
    ax_txt.axis("off")
    text_lines = []
    skip = {"source_file"}
    for k, v in result.stats.items():
        if k not in skip:
            text_lines.append(f"{k}: {v}")
    if result.errors:
        text_lines += ["", "── ERRORS ──"]
        text_lines += [f"  {e}" for e in result.errors]
    if result.warnings:
        text_lines += ["", "── Warnings ──"]
        text_lines += [f"  {w}" for w in result.warnings]
    ax_txt.text(
        0.03, 0.97, "\n".join(text_lines),
        transform=ax_txt.transAxes,
        fontsize=7.2, va="top", ha="left",
        fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#fffde7", alpha=0.9),
    )
    ax_txt.set_title("Statistics & Validation", fontsize=10)

    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


# ---------------------------------------------------------------------------
# Plotting (overview — all files)
# ---------------------------------------------------------------------------

def plot_overview(
    all_data: List[Tuple[TrajectoryData, ValidationResult]],
    output_path: Optional[str | Path] = None,
) -> Path:
    """One-figure overview of all trajectory XY paths grouped by environment type."""
    if output_path is None:
        output_path = DEFAULT_OUTPUT_DIR / "overview_all_trajectories.png"
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    shield_runs    = [(d, r) for d, r in all_data if "Shield"    in Path(d.source_file).name]
    tunneling_runs = [(d, r) for d, r in all_data if "Tunneling" in Path(d.source_file).name]

    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    fig.suptitle("All Trajectories — Overview (XY top-down)", fontsize=14, fontweight="bold")

    for ax, runs, title in [
        (axes[0], shield_runs,    "Shield Tunneling"),
        (axes[1], tunneling_runs, "Conventional Tunneling"),
    ]:
        cmap = plt.get_cmap("tab10")
        for i, (d, r) in enumerate(runs):
            color = cmap(i % 10)
            label = Path(d.source_file).stem
            ax.plot(d.y, d.x, color=color, lw=1.2, alpha=0.85, label=label)
            ax.scatter(d.y[0],  d.x[0],  color=color, s=50,  marker="o", zorder=5)
            ax.scatter(d.y[-1], d.x[-1], color=color, s=60,  marker="X", zorder=5)
        ax.set_xlabel("Y [m]", fontsize=10)
        ax.set_ylabel("X [m]", fontsize=10)
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=7, loc="best")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="datalim")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _add_direction_arrows(
    ax: plt.Axes,
    x: np.ndarray,
    y: np.ndarray,
    n_arrows: int = 15,
    color: str = "gray",
    alpha: float = 0.5,
) -> None:
    """Draw sparse direction arrows along a 2-D trajectory."""
    N = len(x)
    if N < 2:
        return
    indices = np.linspace(0, N - 2, min(n_arrows, N - 1), dtype=int)
    for i in indices:
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]
        if dx == 0.0 and dy == 0.0:
            continue
        ax.annotate(
            "",
            xy=(x[i + 1], y[i + 1]),
            xytext=(x[i], y[i]),
            arrowprops=dict(
                arrowstyle="->",
                color=color,
                alpha=alpha,
                lw=1.0,
            ),
        )


def discover_txt_files(path: str | Path) -> List[Path]:
    """Return sorted list of .txt trajectory files from a file or directory."""
    p = Path(path)
    if p.is_file():
        return [p]
    if p.is_dir():
        files = sorted(p.glob("*.txt"))
        if not files:
            raise FileNotFoundError(f"No .txt files found in {p}")
        return files
    raise FileNotFoundError(f"Path does not exist: {p}")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Parse, validate, and plot tunnel trajectory files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="Trajectory .txt file(s) or directory. Defaults to the bundled dataset.",
    )
    parser.add_argument(
        "--output-dir", "-o",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory for PNG output files.",
    )
    parser.add_argument(
        "--no-overview",
        action="store_true",
        help="Skip the combined overview PNG.",
    )
    args = parser.parse_args(argv)

    # Collect files
    if args.paths:
        txt_files: List[Path] = []
        for p in args.paths:
            txt_files.extend(discover_txt_files(p))
    else:
        txt_files = discover_txt_files(DEFAULT_DATASET_DIR)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    all_results: List[Tuple[TrajectoryData, ValidationResult]] = []
    n_pass = n_fail = 0

    for tf in txt_files:
        try:
            data   = parse_trajectory(tf)
            result = validate_trajectory(data)
        except (FileNotFoundError, ValueError) as exc:
            print(f"[ERROR] {tf.name}: {exc}", file=sys.stderr)
            n_fail += 1
            continue

        print(result.report())
        print()

        out_png = output_dir / f"{tf.stem}_trajectory.png"
        saved   = plot_trajectory(data, result, out_png)
        print(f"  → saved: {saved}")
        print()

        all_results.append((data, result))
        if result.is_valid:
            n_pass += 1
        else:
            n_fail += 1

    # Overview
    if not args.no_overview and all_results:
        ov_path = output_dir / "overview_all_trajectories.png"
        saved_ov = plot_overview(all_results, ov_path)
        print(f"Overview → {saved_ov}")

    print(f"\nSummary: {n_pass} passed, {n_fail} failed out of {len(txt_files)} file(s).")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
