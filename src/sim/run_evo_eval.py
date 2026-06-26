#!/usr/bin/env python3
# Copyright (c) 2026 xiaofan
# SPDX-License-Identifier: MIT

import csv
import os
import subprocess
import sys
from datetime import datetime
import shutil


def run(cmd, cwd=None):
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=cwd)
    return res.returncode, res.stdout


def parse_evo_output(text):
    metrics = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) == 2:
            key, val = parts
            if key in {"max", "mean", "median", "min", "rmse", "sse", "std"}:
                try:
                    metrics[key] = float(val)
                except ValueError:
                    pass
    return metrics


HEADER = ["time", "metric", "max", "mean", "median", "min", "rmse", "sse", "std"]
OLD_HEADER = ["time", "max", "mean", "median", "min", "rmse", "sse", "std"]


def ensure_result_header(path):
    if not os.path.isfile(path):
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(HEADER)
        return

    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(HEADER)
        return

    header = rows[0]
    if header == HEADER:
        return

    if header == OLD_HEADER:
        tmp_path = path + ".tmp"
        with open(tmp_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(HEADER)
            for row in rows[1:]:
                if not row:
                    continue
                # existing results are from evo_ape
                writer.writerow([row[0], "ape"] + row[1:len(OLD_HEADER)])
        os.replace(tmp_path, path)
        return

    # unknown header format; append with new header to avoid silent mismatch
    with open(path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)


def write_result(metrics, path, metric_name):
    ensure_result_header(path)
    with open(path, "a", newline="") as f:
        writer = csv.writer(f)
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        row = [now_str, metric_name] + [metrics.get(k, "") for k in HEADER[2:]]
        writer.writerow(row)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    result_csv = os.path.join(here, "result.csv")
    config_path = os.path.join(here, "config.jsonc")
    bak_dir = os.path.join(here, "bak")

    if os.path.isfile(config_path):
        os.makedirs(bak_dir, exist_ok=True)
        time_prefix = datetime.now().strftime("%Y%m%d_%H%M%S")
        bak_path = os.path.join(bak_dir, f"{time_prefix}_config.jsonc")
        shutil.copyfile(config_path, bak_path)

    gt_path = os.path.join(here, "object_world_truth.tum")
    traj_path = os.path.join(here, "..", "..", "traj.tum")

    if not os.path.isfile(gt_path) or not os.path.isfile(traj_path):
        print("Missing gt or traj output file.")
        return 1

    code, out = run(["evo_ape", "tum", gt_path, traj_path, "-a"], cwd=here)
    if code != 0:
        if "No such file or directory" in out or "not found" in out:
            print("evo_ape not found. Please install evo first.")
            print("Suggested: pip install evo --user")
            print("If evo_ape is not on PATH, try: export PATH=$PATH:~/.local/bin")
        else:
            print(out)
        return code

    metrics_ape = parse_evo_output(out)
    if not metrics_ape:
        print("Failed to parse evo_ape output.")
        print(out)
        return 1

    code, out_rpe = run(["evo_rpe", "tum", gt_path, traj_path, "-a"], cwd=here)
    if code != 0:
        if "No such file or directory" in out_rpe or "not found" in out_rpe:
            print("evo_rpe not found. Please install evo first.")
            print("Suggested: pip install evo --user")
            print("If evo_rpe is not on PATH, try: export PATH=$PATH:~/.local/bin")
        else:
            print(out_rpe)
        return code

    metrics_rpe = parse_evo_output(out_rpe)
    if not metrics_rpe:
        print("Failed to parse evo_rpe output.")
        print(out_rpe)
        return 1

    write_result(metrics_ape, result_csv, "ape")
    write_result(metrics_rpe, result_csv, "rpe")
    print(out)
    print(out_rpe)
    print(f"Appended results to {result_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
