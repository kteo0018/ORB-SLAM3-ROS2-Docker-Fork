#!/usr/bin/env python3
"""
Batch evaluation of ORB-SLAM3 runs using ground-truth TUM files.

Features:
- Computes RMSE/median/SSE/std for every orb_slam3_*.txt/.tum against ground_truth.tum.
- Aligns estimates to ground truth with a Sim(3) (Umeyama) fit to mimic `evo_ape -as`.
- Flags tracking-loss events using timestamp gaps.
- Saves optional XY plots of aligned trajectories.
- Produces per-run CSV plus aggregated stats for afeat*/ori* groups.

Dependencies: numpy, matplotlib (no direct evo import required).
Usage:
    python analyze_evo_results.py --root . --save-plots
"""
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class Trajectory:
    timestamps: np.ndarray  # shape (N,)
    positions: np.ndarray  # shape (N, 3)
    quats: np.ndarray  # shape (N, 4) in (x, y, z, w)


@dataclass
class EvalResult:
    run: str
    file: str
    group: str
    rmse: float
    median: float
    sse: float
    std: float
    n_matches: int
    lost_events: int
    lost_in_run: bool
    plot_path: Optional[str]


def load_tum(path: Path) -> Trajectory:
    times: List[float] = []
    pos: List[List[float]] = []
    quats: List[List[float]] = []
    with path.open("r", encoding="ascii", errors="ignore") as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            t, x, y, z = map(float, parts[:4])
            qx, qy, qz, qw = map(float, parts[4:8])
            times.append(t)
            pos.append([x, y, z])
            quats.append([qx, qy, qz, qw])
    if not times:
        return Trajectory(np.array([]), np.zeros((0, 3)), np.zeros((0, 4)))
    return Trajectory(
        np.array(times, dtype=float),
        np.array(pos, dtype=float),
        np.array(quats, dtype=float),
    )


def associate_indices(ref_t: np.ndarray, est_t: np.ndarray, max_delta: float) -> Tuple[np.ndarray, np.ndarray]:
    ref_idx: List[int] = []
    est_idx: List[int] = []
    for i, t in enumerate(est_t):
        j = np.searchsorted(ref_t, t, side="left")
        candidates = []
        if j < len(ref_t):
            candidates.append(j)
        if j > 0:
            candidates.append(j - 1)
        best = None
        best_diff = max_delta
        for k in candidates:
            diff = abs(ref_t[k] - t)
            if diff <= best_diff:
                best = k
                best_diff = diff
        if best is not None:
            ref_idx.append(best)
            est_idx.append(i)
    return np.array(ref_idx, dtype=int), np.array(est_idx, dtype=int)


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = True) -> Tuple[float, np.ndarray, np.ndarray]:
    # src -> estimated, dst -> ground truth
    assert src.shape == dst.shape
    n = src.shape[0]
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_demean = src - src_mean
    dst_demean = dst - dst_mean
    cov = (dst_demean.T @ src_demean) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_src = (src_demean ** 2).sum() / n
    scale = (np.trace(np.diag(D) @ S) / var_src) if with_scale else 1.0
    t = dst_mean - scale * (R @ src_mean)
    return scale, R, t


def apply_transform(points: np.ndarray, scale: float, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (scale * (points @ R.T)) + t


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    # q: (N,4) as (x,y,z,w); returns (N,3) roll/pitch/yaw in degrees
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.degrees(np.stack([roll, pitch, yaw], axis=1))


def compute_metrics(gt: Trajectory, est: Trajectory, max_delta: float) -> Tuple[Optional[EvalResult], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    if gt.timestamps.size == 0 or est.timestamps.size == 0:
        return None, None, None, None, None
    ref_idx, est_idx = associate_indices(gt.timestamps, est.timestamps, max_delta)
    if ref_idx.size < 2:
        return None, None, None, None, None
    gt_pos = gt.positions[ref_idx]
    est_pos = est.positions[est_idx]
    scale, R, t = umeyama_alignment(est_pos, gt_pos, with_scale=True)
    est_aligned = apply_transform(est_pos, scale, R, t)
    errors = np.linalg.norm(est_aligned - gt_pos, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    median = float(np.median(errors))
    sse = float(np.sum(errors ** 2))
    std = float(np.std(errors))
    return (
        EvalResult("", "", "", rmse, median, sse, std, int(ref_idx.size), 0, False, None),
        est_aligned,
        ref_idx,
        est_idx,
        errors,
    )


def plot_xy(gt: np.ndarray, est_aligned: np.ndarray, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(6, 6))
    plt.plot(gt[:, 0], gt[:, 1], label="ground truth", linewidth=1.5)
    plt.plot(est_aligned[:, 0], est_aligned[:, 1], label="estimate", linewidth=1.2)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()


def plot_xyz_time(times: np.ndarray, gt: np.ndarray, est: np.ndarray, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    labels = ["x", "y", "z"]
    for i, ax in enumerate(axes):
        ax.plot(times, gt[:, i], "k--", linewidth=1.2, label="ground_truth" if i == 0 else None)
        ax.plot(times, est[:, i], linewidth=1.2, label="estimate" if i == 0 else None)
        ax.set_ylabel(f"{labels[i]} (m)")
    axes[-1].set_xlabel("t (s)")
    axes[0].legend()
    fig.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_rpy_time(times: np.ndarray, gt_quat: np.ndarray, est_quat: np.ndarray, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    gt_rpy = quat_to_euler(gt_quat)
    est_rpy = quat_to_euler(est_quat)
    # Swap roll and pitch for estimated (indices 0 and 1)
    est_rpy[:, [0, 1]] = est_rpy[:, [1, 0]]
    # Invert all RPY for estimated
    est_rpy = -est_rpy
    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    labels = ["roll", "pitch", "yaw"]
    for i, ax in enumerate(axes):
        ax.plot(times, gt_rpy[:, i], "k--", linewidth=1.0, label="ground_truth" if i == 0 else None)
        ax.plot(times, est_rpy[:, i], linewidth=1.0, label="estimate" if i == 0 else None)
        ax.set_ylabel(f"{labels[i]} (deg)")
    axes[-1].set_xlabel("t (s)")
    axes[0].legend()
    fig.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close(fig)


def plot_3d(gt: np.ndarray, est: np.ndarray, errors: np.ndarray, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    cmap = plt.get_cmap("jet")
    norm = plt.Normalize(vmin=np.min(errors), vmax=np.max(errors))
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], "k--", linewidth=1.0, label="ground_truth")
    sc = ax.plot(est[:, 0], est[:, 1], est[:, 2], linewidth=1.0, color="gray")
    ax.scatter(est[:, 0], est[:, 1], est[:, 2], c=errors, cmap=cmap, norm=norm, s=6)
    m = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    m.set_array(errors)
    cbar = plt.colorbar(m, ax=ax, pad=0.1)
    cbar.set_label("APE (m)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=220)
    plt.close(fig)


def detect_lost_events(timestamps: np.ndarray, gap_threshold: float) -> int:
    if timestamps.size < 2:
        return 0
    gaps = np.diff(timestamps)
    return int((gaps > gap_threshold).sum())


def fmt4(x: float) -> float:
    return float(f"{x:.4f}")


def row_dict_rounded(row: EvalResult) -> Dict[str, object]:
    d = asdict(row)
    for k in ("rmse", "median", "sse", "std"):
        d[k] = fmt4(d[k])
    return d


def load_existing_metrics(csv_path: Path) -> List[EvalResult]:
    rows: List[EvalResult] = []
    if not csv_path.exists():
        return rows
    with csv_path.open("r", encoding="ascii") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append(
                    EvalResult(
                        run=r.get("run", ""),
                        file=r.get("file", ""),
                        group=r.get("group", ""),
                        rmse=float(r.get("rmse", 0.0)),
                        median=float(r.get("median", 0.0)),
                        sse=float(r.get("sse", 0.0)),
                        std=float(r.get("std", 0.0)),
                        n_matches=int(float(r.get("n_matches", 0))),
                        lost_events=int(float(r.get("lost_events", 0))),
                        lost_in_run=str(r.get("lost_in_run", "")).lower() in ("true", "1", "yes"),
                        plot_path=r.get("plot_path", None) or None,
                    )
                )
            except Exception:
                continue
    return rows


def group_label(name: str) -> str:
    if "afeat" in name:
        return "afeat"
    if "ori" in name:
        return "ori"
    return "other"


def find_ground_truth(run_dir: Path) -> Optional[Path]:
    for name in ("ground_truth.tum", "ground_truth.txt"):
        cand = run_dir / name
        if cand.exists():
            return cand
    matches = list(run_dir.glob("ground_truth*.*"))
    return matches[0] if matches else None


def collect_estimates(run_dir: Path) -> List[Path]:
    exts = ("*.txt", "*.tum")
    estimates: List[Path] = []
    for ext in exts:
        for p in run_dir.glob(ext):
            if "ground_truth" in p.name:
                continue
            if p.name.startswith("orb_slam3_"):
                estimates.append(p)
    return sorted(estimates)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate ORB-SLAM3 runs against ground truth")
    parser.add_argument("--root", type=Path, default=Path("."), help="Root directory containing run result folders")
    parser.add_argument("--runs", nargs="*", help="Optional list of run directories to process (relative to --root)")
    parser.add_argument("--gap-threshold", type=float, default=0.5, help="Seconds without poses to count as a lost-track event")
    parser.add_argument("--max-time-delta", type=float, default=0.02, help="Max time difference (s) when pairing poses")
    parser.add_argument("--save-plots", action="store_true", help="Save XY trajectory plots")
    parser.add_argument("--out", type=Path, default=Path("evo_results"), help="Subfolder (inside each run dir) for outputs")
    parser.add_argument("--skip-existing", action="store_true", help="Reuse existing per_run_metrics.csv if present and skip recomputation for that run")
    args = parser.parse_args()

    root = args.root.resolve()
    global_out_root = root / "aggregated_evo_results"
    global_out_root.mkdir(parents=True, exist_ok=True)

    per_run_rows: List[EvalResult] = []

    # Pick the run directories to process.
    if args.runs:
        candidates = []
        for run in args.runs:
            p = Path(run)
            run_path = p if p.is_absolute() else (root / p)
            if run_path.is_dir():
                candidates.append(run_path)
        run_dirs = sorted(candidates)
    else:
        run_dirs = sorted([p for p in root.iterdir() if p.is_dir()])

    for run_dir in run_dirs:
        gt_path = find_ground_truth(run_dir)
        if not gt_path:
            continue
        run_out_dir = run_dir / args.out
        details_dir = run_out_dir / "details"

        # Skip recompute if requested and existing metrics are present.
        existing_metrics = []
        if args.skip_existing:
            existing_metrics = load_existing_metrics(details_dir / "per_run_metrics.csv")
        if args.skip_existing and existing_metrics:
            per_run_rows.extend(existing_metrics)
            continue

        details_dir.mkdir(parents=True, exist_ok=True)

        gt_traj = load_tum(gt_path)
        estimates = collect_estimates(run_dir)
        if not estimates:
            continue

        for est_path in estimates:
            est_traj = load_tum(est_path)
            eval_result, est_aligned, ref_idx, est_idx, errors = compute_metrics(gt_traj, est_traj, args.max_time_delta)
            if eval_result is None or est_aligned is None or ref_idx is None or est_idx is None or errors is None:
                continue
            lost_events = detect_lost_events(est_traj.timestamps, args.gap_threshold)
            eval_result.run = run_dir.name
            eval_result.file = est_path.name
            eval_result.group = group_label(est_path.name)
            eval_result.lost_events = lost_events
            eval_result.lost_in_run = lost_events > 0
            plot_path: Optional[str] = None
            if args.save_plots:
                matched_gt_pos = gt_traj.positions[ref_idx]
                matched_est_pos = est_aligned
                matched_times = gt_traj.timestamps[ref_idx]

                plot_file_xy = details_dir / f"{est_path.stem}_xy.png"
                plot_xy(matched_gt_pos, matched_est_pos, plot_file_xy)

                plot_file_3d = details_dir / f"{est_path.stem}_3d.png"
                plot_3d(matched_gt_pos, matched_est_pos, errors, plot_file_3d)

                plot_file_xyz = details_dir / f"{est_path.stem}_xyz.png"
                plot_xyz_time(matched_times, matched_gt_pos, matched_est_pos, plot_file_xyz)

                if gt_traj.quats.shape[0] > 0 and est_traj.quats.shape[0] > 0:
                    gt_q = gt_traj.quats[ref_idx]
                    est_q = est_traj.quats[est_idx]
                    plot_file_rpy = details_dir / f"{est_path.stem}_rpy.png"
                    plot_rpy_time(matched_times, gt_q, est_q, plot_file_rpy)

                plot_path = str(plot_file_xy)
            eval_result.plot_path = plot_path
            per_run_rows.append(eval_result)

        # write per-run CSV
        run_rows = [r for r in per_run_rows if r.run == run_dir.name]
        if run_rows:
            loss_files = [r.file for r in run_rows if r.lost_in_run]
            csv_path = details_dir / "per_run_metrics.csv"
            with csv_path.open("w", newline="", encoding="ascii") as f:
                fieldnames = [k for k in row_dict_rounded(run_rows[0]).keys() if k not in ("run", "plot_path")]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in run_rows:
                    full_dict = row_dict_rounded(row)
                    filtered_dict = {k: v for k, v in full_dict.items() if k not in ("run", "plot_path")}
                    writer.writerow(filtered_dict)

            # per-run summary
            run_summary: Dict[str, Dict[str, float]] = {}
            summary_rows = []
            for group in ("afeat", "ori", "other"):
                group_rows = [r for r in run_rows if r.group == group]
                if not group_rows:
                    continue
                rmse_vals = [r.rmse for r in group_rows]
                best = min(group_rows, key=lambda r: r.rmse)
                run_summary[group] = {
                    "average_rmse": fmt4(np.mean(rmse_vals)),
                    "tracking_lost_runs": int(sum(1 for r in group_rows if r.lost_in_run)),
                    "total_tracking_lost_events": int(sum(r.lost_events for r in group_rows)),
                    "num_evaluated_runs": len(group_rows),
                    "best_file": best.file,
                    "best_rmse": fmt4(best.rmse),
                    "best_plot": best.plot_path,
                    "files_with_tracking_loss": [r.file for r in group_rows if r.lost_in_run],
                }
                summary_rows.append({
                    "group": group,
                    **run_summary[group],
                })

            summary_json = run_out_dir / "summary.json"
            with summary_json.open("w", encoding="ascii") as f:
                json.dump(run_summary, f, indent=2)

            summary_csv = run_out_dir / "summary.csv"
            with summary_csv.open("w", newline="", encoding="ascii") as f:
                fieldnames = [
                    "group",
                    "average_rmse",
                    "tracking_lost_runs",
                    "total_tracking_lost_events",
                    "num_evaluated_runs",
                    "best_file",
                    "best_rmse",
                    "files_with_tracking_loss",
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in summary_rows:
                    out_row = dict(row)
                    # Remove best_plot from CSV output
                    out_row.pop("best_plot", None)
                    if isinstance(out_row.get("files_with_tracking_loss"), list):
                        out_row["files_with_tracking_loss"] = ";".join(out_row["files_with_tracking_loss"])
                    writer.writerow(out_row)

    if not per_run_rows:
        print("No runs processed.")
        return

    # aggregated summary across all processed runs
    agg_summary: Dict[str, Dict[str, float]] = {}
    for group in ("afeat", "ori", "other"):
        group_rows = [r for r in per_run_rows if r.group == group]
        if not group_rows:
            continue
        rmse_vals = [r.rmse for r in group_rows]
        best = min(group_rows, key=lambda r: r.rmse)
        agg_summary[group] = {
            "average_rmse": fmt4(np.mean(rmse_vals)),
            "tracking_lost_runs": int(sum(1 for r in group_rows if r.lost_in_run)),
            "total_tracking_lost_events": int(sum(r.lost_events for r in group_rows)),
            "num_evaluated_runs": len(group_rows),
            "best_file": best.file,
            "best_rmse": fmt4(best.rmse),
            "best_plot": best.plot_path,
        }

    master_csv = global_out_root / "all_runs_metrics.csv"
    with master_csv.open("w", newline="", encoding="ascii") as f:
        fieldnames = [k for k in row_dict_rounded(per_run_rows[0]).keys() if k != "plot_path"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in per_run_rows:
            full_dict = row_dict_rounded(row)
            filtered_dict = {k: v for k, v in full_dict.items() if k != "plot_path"}
            writer.writerow(filtered_dict)

    summary_path = global_out_root / "aggregated_summary.json"
    with summary_path.open("w", encoding="ascii") as f:
        json.dump(agg_summary, f, indent=2)

    print("Processed runs:", len({r.run for r in per_run_rows}))
    print("All-runs metrics: ", master_csv)
    print("Aggregated summary:", summary_path)


if __name__ == "__main__":
    main()
