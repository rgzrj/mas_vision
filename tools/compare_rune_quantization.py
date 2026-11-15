"""Unlabelled FP32/INT8 drift check, NOT an accuracy or hit-rate evaluation."""
import argparse
from collections import Counter
import gc
import json
from pathlib import Path
import time

import cv2
import numpy as np
import openvino as ov


def decode(output, row):
    values = output.reshape(18, 6300)
    classes = np.argmax(values[:3], axis=0)
    scores = np.max(values[:3], axis=0)
    kept = []
    for index in np.argsort(-scores, kind="stable"):
        if scores[index] < .25:
            break
        if not np.isfinite(scores[index]):
            continue
        points = values[3:, index].reshape(5, 3).copy()
        points[:, :2] = (points[:, :2] - np.asarray(row["pad"])) / row["scale"]
        valid = (np.isfinite(points).all(axis=1) & (points[:, 2] >= .8) &
                 (points[:, 0] >= 0) & (points[:, 0] < row["width"]) &
                 (points[:, 1] >= 0) & (points[:, 1] < row["height"]))
        if valid.sum() < 3:
            continue
        tips = points[[0, 1, 3, 4], :2]
        center = tips[np.isfinite(tips).all(axis=1)].mean(axis=0)
        state = int(classes[index])
        gate = 10 if state == 2 else 30
        if any((other["state"] == 2) == (state == 2) and
               np.linalg.norm(center - other["center"]) < gate for other in kept):
            continue
        kept.append({"state": state, "score": float(scores[index]), "center": center,
                     "points": points[:, :2], "valid": valid})
    return kept


def runtime_stats(compiled):
    # Keep the parent compiled model alive while temporary RTMap views are destroyed.
    precisions = Counter()
    convolution_rows = []
    runtime_model = compiled.get_runtime_model()
    for op in runtime_model.get_ordered_ops():
        info = op.get_rt_info()
        def value(key):
            return str(info[key].value) if key in info else ""
        kind, precision = value("layerType"), value("runtimePrecision")
        precisions[f"{kind}:{precision}"] += 1
        if kind in ("Convolution", "FullyConnected", "MatMul"):
            convolution_rows.append({"name": op.friendly_name, "type": kind,
                                     "runtimePrecision": precision, "execType": value("execType")})
    return dict(precisions), convolution_rows


def run_model(path, directory, rows, output_prefix):
    print("Checking:", output_prefix, flush=True)
    core = ov.Core()
    compiled = core.compile_model(str(path), "CPU", {
        "PERFORMANCE_HINT": "LATENCY", "NUM_STREAMS": 1, "INFERENCE_NUM_THREADS": 6,
        "ENABLE_CPU_PINNING": True, "ENABLE_HYPER_THREADING": False,
    })
    request = compiled.create_infer_request()
    predictions = []
    for row in rows:
        image = cv2.imdecode(np.fromfile(directory / row["path"], np.uint8), cv2.IMREAD_COLOR)
        tensor = np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.
        request.get_input_tensor().data[:] = tensor
        request.infer()
        output = request.get_output_tensor().data.copy()
        if output.shape != (1, 18, 6300) or not np.isfinite(output).all():
            raise ValueError("Invalid output shape or nonfinite predictions")
        predictions.append(decode(output, row))
    precisions, convolution_rows = runtime_stats(compiled)
    for _ in range(20):
        request.infer()
    timings = []
    for _ in range(200):
        start = time.perf_counter_ns()
        request.infer()
        timings.append((time.perf_counter_ns() - start) / 1e6)
    stats = {"model": str(path), "mean_infer_ms": float(np.mean(timings)),
             "p95_infer_ms": float(np.percentile(timings, 95)),
             "p99_infer_ms": float(np.percentile(timings, 99)),
             "runtime_nodes": dict(precisions), "convolution_rows": convolution_rows,
             "candidate_counts": dict(Counter(p["state"] for frame in predictions for p in frame))}
    # Write enough decoded data to audit the matching; no production center geometry is claimed.
    serializable = [[{key: value.tolist() if isinstance(value, np.ndarray) else value
                      for key, value in pose.items()} for pose in frame] for frame in predictions]
    (directory / f"{output_prefix}_predictions.json").write_text(json.dumps(serializable), encoding="utf-8")
    return predictions, stats


def describe(values):
    return {"count": len(values), "mean": float(np.mean(values)) if values else None,
            "p95": float(np.percentile(values, 95)) if values else None, "max": max(values) if values else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fp32", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.directory / "manifest.json").read_text("utf-8"))
    rows = manifest["validation"]
    fp32, first = run_model(args.fp32, args.directory, rows, "fp32")
    gc.collect()
    quantized, second = run_model(args.directory / "rune_szu_public_int8.xml", args.directory, rows, "int8")
    count = matched = changed = extra = 0
    point_errors = [[] for _ in range(5)]
    center_errors = []
    per_class = {str(c): {"reference": 0, "matched": 0} for c in range(3)}
    for reference, candidate in zip(fp32, quantized):
        unused = list(range(len(candidate)))
        for pose in reference:
            count += 1
            per_class[str(pose["state"])]["reference"] += 1
            if not unused:
                continue
            index = min(unused, key=lambda i: np.linalg.norm(pose["center"] - candidate[i]["center"]))
            other = candidate[index]
            distance = float(np.linalg.norm(pose["center"] - other["center"]))
            if distance > 30:
                continue
            unused.remove(index)
            matched += 1
            per_class[str(pose["state"])]["matched"] += 1
            changed += other["state"] != pose["state"]
            center_errors.append(distance)
            for k in range(5):
                if pose["valid"][k] and other["valid"][k]:
                    point_errors[k].append(float(np.linalg.norm(pose["points"][k] - other["points"][k])))
        extra += len(unused)
    report = {
        "note": "Same-video unlabelled drift, not accuracy. Center=mean of four network tips, "
                "NOT production lamp-to-board center. Pixel units restored to source video. "
                "Greedy matching gate 30 px; unmatched and invalid points are excluded from drift averages. "
                "Timings are this computer only, 200 sync inferences, not live FPS; no ABBA timing control.",
        "validation_frames": len(rows), "fp32": first, "int8": second,
        "reference_candidates": count, "matched": matched, "unmatched_fp32": count - matched,
        "unmatched_int8": extra, "class_changes_among_matched": changed, "by_reference_class": per_class,
        "tip_mean_center_drift_px": describe(center_errors),
        "keypoint_drift_px": {str(k): describe(values) for k, values in enumerate(point_errors)},
    }
    (args.directory / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("fp32", "int8")}, indent=2))
    for name, stats in (("FP32", first), ("INT8", second)):
        print(name, "infer mean/p95/p99:", stats["mean_infer_ms"], stats["p95_infer_ms"], stats["p99_infer_ms"])
        print(name, "runtime nodes:", stats["runtime_nodes"])


if __name__ == "__main__":
    main()
