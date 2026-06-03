#!/usr/bin/env python3
"""
Tests for trajectory_analysis.py

Run with:
    pytest test_trajectory.py -v
    pytest test_trajectory.py -v --tb=short

Tests cover:
    - File parsing (valid, missing, malformed)
    - Validation checks (all 6 checks)
    - Statistics correctness
    - PNG generation
    - All 15 real dataset files
"""

from __future__ import annotations

import io
import os
import math
import tempfile
import textwrap
from pathlib import Path
from typing import List

import numpy as np
import pytest

# Module under test
import trajectory_analysis as ta


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

DATASET_DIR = (
    Path(__file__).parent
    / "dataset"
    / "drive-download-20260524T182528Z-3-001"
)

ALL_TXT_FILES = sorted(DATASET_DIR.glob("*.txt")) if DATASET_DIR.exists() else []

SHIELD_FILES    = [f for f in ALL_TXT_FILES if f.stem.startswith("Shield")]
TUNNELING_FILES = [f for f in ALL_TXT_FILES if f.stem.startswith("Tunneling")]


def make_txt(lines: List[str], tmp_path: Path, name: str = "test.txt") -> Path:
    """Write lines to a temp .txt file and return its path."""
    p = tmp_path / name
    p.write_text("\n".join(lines) + "\n")
    return p


def simple_trajectory(
    n: int = 10,
    dt: float = 0.1,
    t0: float = 1700000000.0,
) -> List[str]:
    """Generate a straight-line trajectory (x increases, y/z constant)."""
    rows = []
    for i in range(n):
        ts = t0 + i * dt
        x  = 100.0 + i * 0.5
        rows.append(f"{ts:.6f} {x:.4f} 200.0000 10.0000 0 0 0 0")
    return rows


# ===========================================================================
# Section 1: Parsing
# ===========================================================================

class TestParsing:
    def test_basic_parse(self, tmp_path):
        rows = simple_trajectory(5)
        path = make_txt(rows, tmp_path)
        data = ta.parse_trajectory(path)
        assert data.row_count == 5
        assert data.timestamps.shape  == (5,)
        assert data.positions.shape   == (5, 3)
        assert data.quaternions.shape == (5, 4)

    def test_correct_values(self, tmp_path):
        rows = ["1700000000.000000 1.0 2.0 3.0 0 0 0 0"]
        path = make_txt(rows, tmp_path)
        data = ta.parse_trajectory(path)
        assert data.timestamps[0]   == pytest.approx(1700000000.0)
        assert data.positions[0, 0] == pytest.approx(1.0)
        assert data.positions[0, 1] == pytest.approx(2.0)
        assert data.positions[0, 2] == pytest.approx(3.0)
        assert np.all(data.quaternions[0] == 0.0)

    def test_missing_file(self):
        with pytest.raises(FileNotFoundError):
            ta.parse_trajectory("/nonexistent/path/file.txt")

    def test_wrong_column_count_too_few(self, tmp_path):
        path = make_txt(["1700000000.0 1.0 2.0 3.0 0 0 0"], tmp_path)   # 7 cols
        with pytest.raises(ValueError, match="expected 8 columns"):
            ta.parse_trajectory(path)

    def test_wrong_column_count_too_many(self, tmp_path):
        path = make_txt(["1700000000.0 1.0 2.0 3.0 0 0 0 0 9.9"], tmp_path)  # 9 cols
        with pytest.raises(ValueError, match="expected 8 columns"):
            ta.parse_trajectory(path)

    def test_non_numeric_value(self, tmp_path):
        path = make_txt(["1700000000.0 x 2.0 3.0 0 0 0 0"], tmp_path)
        with pytest.raises(ValueError, match="non-numeric"):
            ta.parse_trajectory(path)

    def test_empty_file(self, tmp_path):
        path = make_txt([], tmp_path)
        with pytest.raises(ValueError, match="Empty file"):
            ta.parse_trajectory(path)

    def test_blank_lines_ignored(self, tmp_path):
        rows = simple_trajectory(3)
        with_blanks = ["", rows[0], "", rows[1], "  ", rows[2], ""]
        path = make_txt(with_blanks, tmp_path)
        data = ta.parse_trajectory(path)
        assert data.row_count == 3

    def test_high_precision_timestamp(self, tmp_path):
        path = make_txt(["1706671253.339999705 999.9838 2004.6362 10.1203 0 0 0 0"], tmp_path)
        data = ta.parse_trajectory(path)
        assert data.timestamps[0] == pytest.approx(1706671253.34, rel=1e-6)

    def test_property_accessors(self, tmp_path):
        rows = simple_trajectory(4)
        data = ta.parse_trajectory(make_txt(rows, tmp_path))
        np.testing.assert_array_equal(data.x, data.positions[:, 0])
        np.testing.assert_array_equal(data.y, data.positions[:, 1])
        np.testing.assert_array_equal(data.z, data.positions[:, 2])
        assert data.t_rel[0] == pytest.approx(0.0)
        assert data.t_rel[-1] > 0.0


# ===========================================================================
# Section 2: Validation — valid data
# ===========================================================================

class TestValidationPass:
    def test_valid_trajectory_passes(self, tmp_path):
        data   = ta.parse_trajectory(make_txt(simple_trajectory(20), tmp_path))
        result = ta.validate_trajectory(data)
        assert result.is_valid is True
        assert result.errors == []

    def test_stats_populated(self, tmp_path):
        data   = ta.parse_trajectory(make_txt(simple_trajectory(10), tmp_path))
        result = ta.validate_trajectory(data)
        for key in ("num_poses", "duration_s", "path_length_m", "net_displacement_m",
                    "start_pos_xyz", "end_pos_xyz",
                    "x_range_m", "y_range_m", "z_range_m"):
            assert key in result.stats, f"Missing stat: {key}"

    def test_num_poses_matches(self, tmp_path):
        n = 17
        data   = ta.parse_trajectory(make_txt(simple_trajectory(n), tmp_path))
        result = ta.validate_trajectory(data)
        assert result.stats["num_poses"] == n

    def test_path_length_straight_line(self, tmp_path):
        # 10 steps of 0.5 m each → 9 gaps × 0.5 m = 4.5 m
        data   = ta.parse_trajectory(make_txt(simple_trajectory(10, dt=0.1), tmp_path))
        result = ta.validate_trajectory(data)
        path_len = float(result.stats["path_length_m"])
        assert path_len == pytest.approx(9 * 0.5, rel=1e-4)

    def test_net_displacement_straight_line(self, tmp_path):
        n = 10
        data   = ta.parse_trajectory(make_txt(simple_trajectory(n, dt=0.1), tmp_path))
        result = ta.validate_trajectory(data)
        expected = (n - 1) * 0.5   # straight line → net == path
        net = float(result.stats["net_displacement_m"])
        assert net == pytest.approx(expected, rel=1e-4)

    def test_duration_correct(self, tmp_path):
        dt = 0.2
        n  = 10
        data   = ta.parse_trajectory(make_txt(simple_trajectory(n, dt=dt), tmp_path))
        result = ta.validate_trajectory(data)
        assert float(result.stats["duration_s"]) == pytest.approx((n - 1) * dt, rel=1e-4)

    def test_quaternion_zero_warning(self, tmp_path):
        data   = ta.parse_trajectory(make_txt(simple_trajectory(5), tmp_path))
        result = ta.validate_trajectory(data)
        has_q_warn = any("quaternion" in w.lower() for w in result.warnings)
        assert has_q_warn, "Expected quaternion-zero warning"


# ===========================================================================
# Section 3: Validation — invalid / edge data
# ===========================================================================

class TestValidationFail:
    def test_non_monotonic_timestamp(self, tmp_path):
        rows = [
            "1700000000.0 1.0 2.0 3.0 0 0 0 0",
            "1700000002.0 2.0 2.0 3.0 0 0 0 0",
            "1700000001.0 3.0 2.0 3.0 0 0 0 0",   # goes backward
            "1700000003.0 4.0 2.0 3.0 0 0 0 0",
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        assert result.is_valid is False
        assert any("non-positive" in e.lower() for e in result.errors)

    def test_duplicate_timestamp(self, tmp_path):
        rows = [
            "1700000000.0 1.0 2.0 3.0 0 0 0 0",
            "1700000000.0 1.1 2.0 3.0 0 0 0 0",   # same timestamp
            "1700000001.0 1.2 2.0 3.0 0 0 0 0",
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        assert result.is_valid is False

    def test_large_time_gap_warning(self, tmp_path):
        rows = [
            "1700000000.0 1.0 2.0 3.0 0 0 0 0",
            "1700000010.0 1.01 2.0 3.0 0 0 0 0",  # 10-second gap
            "1700000010.1 1.02 2.0 3.0 0 0 0 0",
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        has_gap_warn = any("gap" in w.lower() for w in result.warnings)
        assert has_gap_warn

    def test_speed_jump_warning(self, tmp_path):
        rows = [
            "1700000000.0 1.0   2.0 3.0 0 0 0 0",
            "1700000000.1 500.0 2.0 3.0 0 0 0 0",  # 4990 m/s — way too fast
            "1700000000.2 500.1 2.0 3.0 0 0 0 0",
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        has_speed_warn = any(
            "jump" in w.lower() or "speed" in w.lower() or "m/s" in w.lower()
            for w in result.warnings
        )
        assert has_speed_warn

    def test_invalid_quaternion_norm(self, tmp_path):
        # norm = sqrt(0.5^2 * 4) = 1.0  ← valid; use (0.6, 0.6, 0.6, 0.6) norm≈1.2 ← invalid
        rows = [
            "1700000000.0 1.0 2.0 3.0 0.6 0.6 0.6 0.6",
            "1700000001.0 2.0 2.0 3.0 0.6 0.6 0.6 0.6",
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        assert result.is_valid is False
        assert any("quaternion" in e.lower() for e in result.errors)

    def test_valid_unit_quaternion_no_error(self, tmp_path):
        # (0, 0, 0, 1) is a valid unit quaternion
        rows = [
            "1700000000.0 1.0 2.0 3.0 0 0 0 1",
            "1700000001.0 2.0 2.0 3.0 0 0 0 1",
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        assert result.is_valid is True
        assert not any("quaternion" in e.lower() for e in result.errors)

    def test_static_stretch_warning(self, tmp_path):
        # 15 identical positions → long static stretch warning
        t0 = 1700000000.0
        rows = [f"{t0 + i * 0.1:.6f} 1.0 2.0 3.0 0 0 0 0" for i in range(15)]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        has_static_warn = any("static" in w.lower() for w in result.warnings)
        assert has_static_warn


# ===========================================================================
# Section 4: Validation report
# ===========================================================================

class TestReport:
    def test_report_contains_filename(self, tmp_path):
        data   = ta.parse_trajectory(make_txt(simple_trajectory(5), tmp_path, "myfile.txt"))
        result = ta.validate_trajectory(data)
        report = result.report()
        assert "myfile.txt" in report

    def test_report_pass(self, tmp_path):
        data   = ta.parse_trajectory(make_txt(simple_trajectory(5), tmp_path))
        result = ta.validate_trajectory(data)
        assert "PASS" in result.report()

    def test_report_fail_contains_error(self, tmp_path):
        rows = [
            "1700000002.0 1.0 2.0 3.0 0 0 0 0",
            "1700000001.0 2.0 2.0 3.0 0 0 0 0",  # non-monotonic
        ]
        data   = ta.parse_trajectory(make_txt(rows, tmp_path))
        result = ta.validate_trajectory(data)
        report = result.report()
        assert "FAIL" in report
        assert "ERROR" in report


# ===========================================================================
# Section 5: PNG output
# ===========================================================================

class TestPlotOutput:
    def test_png_created(self, tmp_path):
        data   = ta.parse_trajectory(make_txt(simple_trajectory(20), tmp_path))
        result = ta.validate_trajectory(data)
        out    = tmp_path / "out.png"
        saved  = ta.plot_trajectory(data, result, out)
        assert saved == out
        assert out.exists()
        assert out.stat().st_size > 1000   # non-trivial file

    def test_default_output_naming(self, tmp_path, monkeypatch):
        monkeypatch.setattr(ta, "DEFAULT_OUTPUT_DIR", tmp_path)
        path  = make_txt(simple_trajectory(10), tmp_path, "my_tunnel.txt")
        data  = ta.parse_trajectory(path)
        result = ta.validate_trajectory(data)
        saved = ta.plot_trajectory(data, result)   # no explicit output path
        assert saved.name == "my_tunnel_trajectory.png"
        assert saved.exists()

    def test_overview_png_created(self, tmp_path):
        records = []
        for i in range(3):
            rows = simple_trajectory(10, t0=1700000000.0 + i * 1000)
            data  = ta.parse_trajectory(make_txt(rows, tmp_path, f"Shield_t{i}.txt"))
            result = ta.validate_trajectory(data)
            records.append((data, result))
        out = tmp_path / "overview.png"
        ta.plot_overview(records, out)
        assert out.exists()
        assert out.stat().st_size > 1000


# ===========================================================================
# Section 6: Real dataset files
# ===========================================================================

@pytest.mark.skipif(not ALL_TXT_FILES, reason="Dataset directory not found")
class TestRealDataset:
    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_parses_without_error(self, filepath):
        data = ta.parse_trajectory(filepath)
        assert data.row_count > 0

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_validates_as_valid(self, filepath):
        data   = ta.parse_trajectory(filepath)
        result = ta.validate_trajectory(data)
        # Print details on failure
        if not result.is_valid:
            print(result.report())
        assert result.is_valid is True

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_timestamps_monotonically_increasing(self, filepath):
        data = ta.parse_trajectory(filepath)
        dt = np.diff(data.timestamps)
        assert np.all(dt > 0), f"Non-monotonic timestamps in {filepath.name}"

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_all_columns_finite(self, filepath):
        data = ta.parse_trajectory(filepath)
        assert np.all(np.isfinite(data.timestamps))
        assert np.all(np.isfinite(data.positions))
        assert np.all(np.isfinite(data.quaternions))

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_quaternions_all_zero(self, filepath):
        """In this dataset all quaternions are zero (orientation not recorded)."""
        data   = ta.parse_trajectory(filepath)
        qnorms = np.linalg.norm(data.quaternions, axis=1)
        assert np.all(qnorms < ta.QUATERNION_ZERO_THRESHOLD), \
            f"Unexpected non-zero quaternion in {filepath.name}"

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_expected_column_count(self, filepath):
        with open(filepath) as fh:
            for lineno, line in enumerate(fh, 1):
                if not line.strip():
                    continue
                cols = line.split()
                assert len(cols) == ta.EXPECTED_COLUMNS, \
                    f"{filepath.name} line {lineno}: {len(cols)} cols"

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_positions_have_z_depth(self, filepath):
        """All trajectories should show variation in z (robot moves into depth)."""
        data = ta.parse_trajectory(filepath)
        z_range = data.z.max() - data.z.min()
        assert z_range > 0.01, f"No z variation in {filepath.name}"

    def test_shield_tunnel_count(self):
        assert len(SHIELD_FILES) == 10, \
            f"Expected 10 Shield files, found {len(SHIELD_FILES)}"

    def test_tunneling_tunnel_count(self):
        assert len(TUNNELING_FILES) == 5, \
            f"Expected 5 Tunneling files, found {len(TUNNELING_FILES)}"

    @pytest.mark.parametrize("filepath", SHIELD_FILES, ids=lambda p: p.name)
    def test_shield_starting_region(self, filepath):
        """Shield tunnels should start near (1000, 2004)."""
        data = ta.parse_trajectory(filepath)
        start = data.positions[0]
        assert 995.0 <= start[0] <= 1005.0, \
            f"{filepath.name} start X={start[0]:.2f} outside expected range"
        assert 2000.0 <= start[1] <= 2010.0, \
            f"{filepath.name} start Y={start[1]:.2f} outside expected range"

    @pytest.mark.parametrize("filepath", ALL_TXT_FILES, ids=lambda p: p.name)
    def test_generates_png(self, filepath, tmp_path):
        data   = ta.parse_trajectory(filepath)
        result = ta.validate_trajectory(data)
        out    = tmp_path / f"{filepath.stem}.png"
        saved  = ta.plot_trajectory(data, result, out)
        assert saved.exists()
        assert saved.stat().st_size > 1000


# ===========================================================================
# Section 7: Helper utilities
# ===========================================================================

class TestHelpers:
    def test_max_run_length_no_trues(self):
        assert ta._max_run_length(np.array([False, False, False])) == 0

    def test_max_run_length_all_true(self):
        assert ta._max_run_length(np.array([True, True, True, True])) == 4

    def test_max_run_length_mixed(self):
        arr = np.array([True, True, False, True, True, True, False, True])
        assert ta._max_run_length(arr) == 3

    def test_discover_txt_single_file(self, tmp_path):
        p = make_txt(simple_trajectory(3), tmp_path, "a.txt")
        files = ta.discover_txt_files(p)
        assert files == [p]

    def test_discover_txt_directory(self, tmp_path):
        for name in ["b.txt", "c.txt", "a.txt"]:
            make_txt(simple_trajectory(3), tmp_path, name)
        files = ta.discover_txt_files(tmp_path)
        assert len(files) == 3
        assert files == sorted(files)   # must be sorted

    def test_discover_txt_missing_path(self):
        with pytest.raises(FileNotFoundError):
            ta.discover_txt_files("/nonexistent/path")

    def test_discover_txt_empty_dir(self, tmp_path):
        sub = tmp_path / "empty_subdir"
        sub.mkdir()
        with pytest.raises(FileNotFoundError, match="No .txt files"):
            ta.discover_txt_files(sub)

    def test_fmt_xyz(self):
        v = np.array([1.23456789, 2.0, -3.5])
        s = ta._fmt_xyz(v)
        assert "1.2346" in s
        assert "2.0000" in s
        assert "-3.5000" in s

    def test_fmt_range(self):
        v = np.array([1.0, 5.0, 3.0])
        s = ta._fmt_range(v)
        assert "1.0000" in s
        assert "5.0000" in s
