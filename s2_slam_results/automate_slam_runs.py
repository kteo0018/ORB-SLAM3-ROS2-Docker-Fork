#!/usr/bin/env python3
"""
Automate ORB-SLAM3 runs across datasets and branches with configurable run counts.

Workflow:
  - For each dataset:
    - Create results folder, copy ground_truth.tum inside container
    - Host: git checkout branch (master -> ori, a-feature-palm -> afeat)
    - Container: rebuild, run N times, retrieve/rename to oriN / afeatN
  - Cleans staging results/ after finishing each dataset

Usage:
  python3 automate_slam_runs.py [--runs N] <dataset> [<dataset2> ...]
Examples:
  python3 automate_slam_runs.py gz_simple_palm_path1_run1
  python3 automate_slam_runs.py --runs 3 gz_simple_palm_path1_run1 gz_simple_palm_path1_run2_noturn

Notes:
  - If --runs is omitted, it falls back to env SLAM_RUNS, else defaults to 5.
  - Prompts for sudo once (uses sudo -v) and caches credentials for the default sudo timeout.
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import List


class OrbSlamOrchestrator:
    def __init__(self, runs: int):
        self.repo_root = Path(__file__).parent.parent
        self.s2_dir = Path(__file__).parent
        self.container_name = "orb_slam3_22_humble"
        self.container_id = None
        self.service_started = False
        self.runs = runs

    def run_cmd(self, cmd: str, shell: bool = True, check: bool = True) -> subprocess.CompletedProcess:
        """Run a shell command on the host."""
        try:
            return subprocess.run(cmd, shell=shell, capture_output=True, text=True, check=check)
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Command failed: {cmd}")
            print(f"STDOUT:\n{e.stdout}")
            print(f"STDERR:\n{e.stderr}")
            raise

    def start_container(self) -> str:
        """Start a persistent container session and return container ID."""
        print("[INIT] Starting persistent container...")
        cmd = f"cd {self.repo_root} && sudo docker compose run -d --entrypoint '/bin/bash' {self.container_name} -c 'sleep infinity'"
        result = self.run_cmd(cmd, check=False)
        if result.returncode != 0:
            print("[ERROR] Failed to start container")
            print(f"STDOUT: {result.stdout}")
            print(f"STDERR: {result.stderr}")
            raise RuntimeError("docker compose run command failed")

        container_id = result.stdout.strip()
        if not container_id:
            raise RuntimeError("No container ID returned from docker compose run")

        print(f"[INIT] Container ID: {container_id[:12]}")
        print("[INIT] Waiting for container to be ready...")
        for i in range(30):  # up to ~15s
            check = self.run_cmd(f"sudo docker inspect -f '{{{{.State.Running}}}}' {container_id}", check=False)
            if check.returncode == 0 and check.stdout.strip() == "true":
                print(f"[INIT] Container ready after {i * 0.5:.1f}s")
                break
            if i % 4 == 0 and i > 0:
                print(f"[INIT] Still waiting... ({i * 0.5:.1f}s)")
            time.sleep(0.5)
        else:
            check = self.run_cmd(f"sudo docker inspect {container_id}", check=False)
            print(f"[DEBUG] Container inspect output:\n{check.stdout}")
            raise RuntimeError("Container failed to start within 15 seconds")

        self.container_id = container_id
        self.service_started = True
        print(f"[INIT] Container started: {container_id[:12]}")
        return container_id

    def stop_container(self):
        """Stop and remove the persistent container."""
        if self.container_id:
            print(f"[CLEANUP] Stopping container {self.container_id[:12]}...")
            self.run_cmd(f"sudo docker stop {self.container_id}", check=False)
            self.run_cmd(f"sudo docker rm {self.container_id}", check=False)
            self.service_started = False

    def exec_in_container(self, cmd: str) -> str:
        """Execute a command inside the persistent container. Returns stdout."""
        if not self.container_id:
            raise RuntimeError("Container not started")
        full_cmd = f"sudo docker exec {self.container_id} bash -lc '{cmd}'"
        result = self.run_cmd(full_cmd)
        return result.stdout.strip()

    def git_checkout(self, branch: str):
        """Checkout a branch in ORB_SLAM3 on the host."""
        print(f"[GIT] Checking out {branch}...")
        cmd = f"cd {self.repo_root}/ORB_SLAM3 && git checkout {branch}"
        result = self.run_cmd(cmd)
        print(result.stdout)

    def current_branch(self) -> str:
        """Return current branch name of ORB_SLAM3 on host."""
        cmd = f"cd {self.repo_root}/ORB_SLAM3 && git rev-parse --abbrev-ref HEAD"
        result = self.run_cmd(cmd)
        return result.stdout.strip()

    def prepare_dataset_folder(self, dataset_name: str) -> Path:
        """Create dataset results folder and copy helper scripts."""
        dataset_dir = self.s2_dir / f"{dataset_name}_results"
        dataset_dir.mkdir(exist_ok=True)
        (dataset_dir / "results").mkdir(exist_ok=True)
        print(f"[{dataset_name}] Creating folder: {dataset_dir}")

        helper_scripts = [
            "g8r1-copy-slam-ground-truth.sh",
            "g8r3-run-multiple-slam-rgbd-offline.sh",
            "g8r4-retrieve-from-runs-results.sh",
        ]
        for script in helper_scripts:
            src = self.s2_dir / script
            dst = dataset_dir / script
            if src.exists() and not dst.exists():
                self.run_cmd(f"cp {src} {dst}")
        return dataset_dir

    def copy_ground_truth(self, dataset_name: str):
        print(f"[{dataset_name}] Copying ground_truth.tum...")
        cmd = f"cd /root/s2_slam_results/{dataset_name}_results && ./g8r1-copy-slam-ground-truth.sh {dataset_name}"
        self.exec_in_container(cmd)

    def run_branch(self, dataset_name: str, branch: str, mode: str):
        print(f"\n[{dataset_name}] ========== {branch} ({mode}) ==========")
        self.git_checkout(branch)

        print(f"[{dataset_name}] Rebuilding ORB-SLAM3 ({branch})...")
        self.exec_in_container("/root/custom_scripts/d1a-rebuild-orb-slam3-source-and-ros2-wrapper.sh")

        print(f"[{dataset_name}] Running SLAM {self.runs}x...")
        cmd = (
            f"cd /root/s2_slam_results/{dataset_name}_results && "
            f"rm -f results/* 2>/dev/null; ./g8r3-run-multiple-slam-rgbd-offline.sh {dataset_name} {self.runs}"
        )
        self.exec_in_container(cmd)

        print(f"[{dataset_name}] Retrieving and renaming results ({mode})...")
        cmd = f"cd /root/s2_slam_results/{dataset_name}_results && ./g8r4-retrieve-from-runs-results.sh {mode}"
        self.exec_in_container(cmd)

        print(f"[{dataset_name}] Completed {branch} ({mode}).")

    def process_dataset(self, dataset_name: str):
        print(f"\n{'='*60}")
        print(f"Processing: {dataset_name}")
        print(f"{'='*60}")

        dataset_dir = self.prepare_dataset_folder(dataset_name)
        self.copy_ground_truth(dataset_name)

        # Determine branch order based on current branch to minimize swaps
        current = self.current_branch()
        target_order = ["master", "a-feature-palm"]
        if current in target_order:
            # Start with current branch, then the other
            other = "a-feature-palm" if current == "master" else "master"
            ordered = [current, other]
        else:
            ordered = target_order

        for branch in ordered:
            mode = "ori" if branch == "master" else "afeat"
            self.run_branch(dataset_name, branch, mode)

        results_dir = dataset_dir / "results"
        if results_dir.exists():
            print(f"[{dataset_name}] Cleaning up {results_dir}...")
            self.run_cmd(f"rm -rf {results_dir}")

        print(f"\n[{dataset_name}] ===== DONE =====")
        print(f"Outputs in: {dataset_dir}")
        print(f"Expected: orb_slam3_ori1..5, orb_slam3_afeat1..5, ground_truth.tum")

    def run(self, datasets: List[str]):
        print("[INIT] Authenticating with sudo...")
        try:
            subprocess.run("sudo -v", shell=True, check=True)
            print("[INIT] Sudo authenticated. Running workflow...\n")
        except subprocess.CalledProcessError:
            print("[ERROR] Failed to authenticate with sudo")
            sys.exit(1)

        try:
            self.start_container()
            for dataset in datasets:
                self.process_dataset(dataset)
            print(f"\n{'='*60}")
            print("ALL DATASETS COMPLETED SUCCESSFULLY")
            print(f"{'='*60}")
        except KeyboardInterrupt:
            print("\n[ABORT] Interrupted by user")
            sys.exit(1)
        except Exception as e:
            print(f"\n[ERROR] {e}")
            sys.exit(1)
        finally:
            self.stop_container()


def parse_args():
    parser = argparse.ArgumentParser(description="Automate ORB-SLAM3 runs across datasets and branches")
    parser.add_argument("datasets", nargs="+", help="Dataset names (e.g., gz_simple_palm_path1_run1)")
    parser.add_argument("--runs", type=int, default=None, help="Run count per branch (default from SLAM_RUNS or 5)")
    return parser.parse_args()


def main():
    args = parse_args()

    runs = args.runs
    if runs is None:
        env_runs = os.environ.get("SLAM_RUNS")
        if env_runs and env_runs.isdigit():
            runs = int(env_runs)
        else:
            runs = 5
    if runs < 1:
        print("[ERROR] --runs must be >= 1")
        sys.exit(1)

    orchestrator = OrbSlamOrchestrator(runs=runs)
    orchestrator.run(args.datasets)


if __name__ == "__main__":
    main()
