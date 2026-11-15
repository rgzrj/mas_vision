"""Summarize a target-machine performance.csv using only the Python standard library."""

import argparse
import csv
import json
import math
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def summarize(path, warmup_s):
    with path.open(encoding="utf-8", newline="") as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    if not rows:
        raise ValueError("performance.csv has no frames")
    first_host_s = rows[0]["host_s"]
    rows = [row for row in rows if row["host_s"] - first_host_s >= warmup_s]
    processed = [row for row in rows if math.isfinite(row["pipeline_ms"])]
    if len(processed) < 2:
        raise ValueError("Fewer than two processed frames after warmup; use more frames or shorten warmup")
    span = processed[-1]["host_s"] - processed[0]["host_s"]
    if span <= 0:
        raise ValueError("Non-increasing host timestamps")
    report = {
        "source": str(path.resolve()),
        "warmup_s": warmup_s,
        "processed_frames": len(processed),
        "observation_seconds": span,
        "observed_processing_fps": (len(processed) - 1) / span,
        "metrics_ms": {},
        "note": "Includes profiling overhead. Does not measure serial delivery or Viewer presentation.",
    }
    for field in processed[0]:
        if not field.endswith("_ms"):
            continue
        values = [row[field] for row in processed if math.isfinite(row[field])]
        if values:
            report["metrics_ms"][field] = {
                "count": len(values),
                "mean": sum(values) / len(values),
                "p50": percentile(values, 0.50),
                "p95": percentile(values, 0.95),
                "p99": percentile(values, 0.99),
                "max": max(values),
            }
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--warmup-s", type=float, default=3.0)
    args = parser.parse_args()
    if not math.isfinite(args.warmup_s) or args.warmup_s < 0:
        parser.error("--warmup-s must be finite and nonnegative")
    try:
        result = summarize(args.csv, args.warmup_s)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Cannot summarize performance: {error}\n")
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
