"""Compare OpenVINO CPU scheduling without changing the live application."""
import argparse
import gc
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import openvino as ov


def measure(path, policy, iterations):
    core = ov.Core()
    compiled = core.compile_model(str(path), "CPU", {
        "PERFORMANCE_HINT": "LATENCY",
        "NUM_STREAMS": 1,
        "INFERENCE_NUM_THREADS": 6,
        "SCHEDULING_CORE_TYPE": getattr(ov.properties.hint.SchedulingCoreType, policy),
        "ENABLE_CPU_PINNING": True,
        "ENABLE_HYPER_THREADING": False,
    })
    if len(compiled.inputs) != 1:
        raise ValueError("Expected one model input")
    request = compiled.create_infer_request()
    tensor = request.get_input_tensor()
    if tensor.element_type != ov.Type.f32:
        raise ValueError("This comparison expects the original FP32 model input")
    tensor.data[:] = .5  # Same FP32 input each time; preparation is outside timing.
    properties = {key: str(compiled.get_property(key)) for key in (
        "INFERENCE_NUM_THREADS", "NUM_STREAMS", "SCHEDULING_CORE_TYPE",
        "ENABLE_CPU_PINNING", "ENABLE_HYPER_THREADING", "INFERENCE_PRECISION_HINT",
    )}
    for _ in range(20):
        request.infer()
    times = []
    for _ in range(iterations):
        start = time.perf_counter_ns()
        request.infer()
        times.append((time.perf_counter_ns() - start) / 1e6)
    return {
        "requested": policy, "properties": properties, "iterations": iterations,
        "mean_ms": float(np.mean(times)), "p50_ms": float(np.percentile(times, 50)),
        "p95_ms": float(np.percentile(times, 95)), "p99_ms": float(np.percentile(times, 99)),
        "max_ms": max(times), "inference_per_second": 1000 / float(np.mean(times)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "output/profiling" /
                        time.strftime("%Y%m%d_%H%M%S") / "cpu_scheduling_comparison.json")
    args = parser.parse_args()
    if not args.model.is_file() or args.iterations < 1:
        parser.error("Provide an existing model and positive iteration count")
    report = {
        "openvino": ov.__version__, "available_devices": ov.Core().available_devices,
        "model": str(args.model.resolve()),
        "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
        "note": "Synchronous inference only, fixed FP32 input, 20 warmups/run. "
                "Excludes camera, preprocessing, tracking, display and serial. "
                "Precision hint is not per-operation execution precision.",
        "runs": [],
    }
    print("OpenVINO:", report["openvino"], "Devices:", report["available_devices"], flush=True)
    print("Order: ANY / PCORE / PCORE / ANY; live application must be stopped.", flush=True)
    for policy in ("ANY_CORE", "PCORE_ONLY", "PCORE_ONLY", "ANY_CORE"):
        result = measure(args.model, policy, args.iterations)
        report["runs"].append(result)
        print(f"{policy:10s} mean={result['mean_ms']:.3f} ms "
              f"p95={result['p95_ms']:.3f} ms p99={result['p99_ms']:.3f} ms "
              f"infer/s={result['inference_per_second']:.2f} "
              f"actual={result['properties']['SCHEDULING_CORE_TYPE']}", flush=True)
        gc.collect()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Saved:", args.output.resolve())


if __name__ == "__main__":
    main()
