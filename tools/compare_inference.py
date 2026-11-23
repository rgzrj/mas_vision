"""Collect Linux environment, then compare CPU/GPU inference without changing the application."""
import argparse
from datetime import datetime
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
CASES = [
    ("gpu_sync", "GPU", 0, 1, 0),
    ("gpu_async_1", "GPU", 0, 1, 1),
    ("gpu_async_2", "GPU", 0, 2, 2),
    ("cpu_2", "CPU", 2, 1, 0),
    ("cpu_3", "CPU", 3, 1, 0),
    ("cpu_4", "CPU", 4, 1, 0),
]


def command(*args):
    if not shutil.which(args[0]):
        return {"error": f"{args[0]} not installed"}
    try:
        result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=15)
        return {"exit": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def fingerprint(path):
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def sensors():
    # Optional sysfs readings; no sudo, driver changes or monitoring subprocesses.
    paths = list(Path("/sys/class/thermal").glob("thermal_zone*/temp"))
    paths += list(Path("/sys/class/drm").glob("card*/gt_cur_freq_mhz"))
    paths += list(Path("/sys/devices/system/cpu/cpufreq").glob("policy*/scaling_governor"))
    readings = {}
    for path in paths:
        try:
            readings[str(path)] = path.read_text().strip()
        except OSError:
            pass
    return readings


def collect(core, ov, model_path, model):
    properties = {}
    for device in core.available_devices:
        supported = {str(prop) for prop in core.get_property(device, "SUPPORTED_PROPERTIES")}
        properties[device] = {}
        for key in ("FULL_DEVICE_NAME", "DRIVER_VERSION", "OPTIMIZATION_CAPABILITIES", "RANGE_FOR_STREAMS"):
            if key in supported:
                properties[device][key] = str(core.get_property(device, key))

    paths = [model_path]
    if model_path.suffix == ".xml":
        paths.append(model_path.with_suffix(".bin"))
    sources = ("auto_buff/buff_detect/yolo/yolo.cpp", "apps/main.cpp", "threads/buff_thread/buff_thread.cpp")
    cache = ROOT / "build/CMakeCache.txt"
    cache_keys = ("OpenVINO_DIR:", "OpenCV_DIR:", "CMAKE_BUILD_TYPE:", "CMAKE_CXX_COMPILER:", "BUILD_TESTING:")
    return {
        "platform": platform.platform(), "python": sys.executable,
        "openvino": ov.__version__, "openvino_path": ov.__file__,
        "affinity": sorted(os.sched_getaffinity(0)),
        "session_type": os.environ.get("XDG_SESSION_TYPE"),
        "devices": properties, "model_files": [fingerprint(path) for path in paths],
        "inputs": [str(port) for port in model.inputs], "outputs": [str(port) for port in model.outputs],
        "sources": [fingerprint(ROOT / path) for path in sources if (ROOT / path).is_file()],
        "config": {path: (ROOT / path).read_text() for path in
                   ("config/auto_buff.yaml", "config/hikcamera.yaml") if (ROOT / path).is_file()},
        "git": command("git", "rev-parse", "HEAD"),
        "working_tree": command("git", "status", "--short"),
        "cmake_cache": [line for line in cache.read_text().splitlines() if line.startswith(cache_keys)]
                       if cache.is_file() else [],
        "cpu": command("lscpu"), "opencl": command("clinfo", "-l"),
        "libraries": command("ldd", str(ROOT / "build/mas_vision")),
        "drivers": command("dpkg-query", "-W", "intel-opencl*", "libigc*", "libigdgmm*", "ocl-icd*"),
        "power_profile": command("powerprofilesctl", "get"),
        "gpu_monitor": shutil.which("intel_gpu_top"),
        "processes": command("ps", "-eo", "pid,comm,pcpu", "--sort=-pcpu"),
        "sensors": sensors(),
    }


def load_input(model, image_path, np):
    shape = tuple(model.input().shape)
    if len(shape) != 4 or shape[0:2] != (1, 3):
        raise ValueError("Expected static NCHW input [1,3,H,W]")
    if image_path is None:
        return np.zeros(shape, dtype=np.float32), {"kind": "zeros", "shape": shape}

    import cv2
    cv2.setNumThreads(1)
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"Cannot read image: {image_path}")
    height, width = shape[2:]
    scale = min(width / image.shape[1], height / image.shape[0])
    size = (round(image.shape[1] * scale), round(image.shape[0] * scale))
    left, top = (width - size[0]) // 2, (height - size[1]) // 2
    padded = np.full((height, width, 3), 114, dtype=np.uint8)
    padded[top:top + size[1], left:left + size[0]] = cv2.resize(image, size)
    data = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)
    data /= np.float32(255)
    return data, {"kind": "image", **fingerprint(image_path), "opencv": cv2.__version__}


def measure(core, ov, np, resource, model, data, case, seconds, warmup):
    name, device, threads, streams, jobs = case
    settings = {"PERFORMANCE_HINT": "LATENCY", "NUM_STREAMS": ov.properties.streams.Num(streams)}
    if device == "CPU":
        settings["INFERENCE_NUM_THREADS"] = threads
    compiled = core.compile_model(model, device, settings)
    actual = {key: str(compiled.get_property(key)) for key in
              ("EXECUTION_DEVICES", "NUM_STREAMS", "PERFORMANCE_HINT", "INFERENCE_PRECISION_HINT")}
    if device == "CPU":
        actual["INFERENCE_NUM_THREADS"] = str(compiled.get_property("INFERENCE_NUM_THREADS"))

    latencies = []
    inputs = {0: data}
    if jobs:
        requests = ov.AsyncInferQueue(compiled, jobs=jobs)
        requests.set_callback(lambda request, _: latencies.append(request.latency))
        infer = lambda: requests.start_async(inputs, share_inputs=True)
        finish = requests.wait_all
    else:
        request = compiled.create_infer_request()

        def infer():
            request.infer(inputs, share_inputs=True)
            latencies.append(request.latency)

        finish = lambda: None

    for _ in range(warmup):
        infer()
    finish()
    latencies.clear()

    before = sensors()
    usage = resource.getrusage(resource.RUSAGE_SELF)
    cpu_start = usage.ru_utime + usage.ru_stime
    started = time.perf_counter()
    while time.perf_counter() - started < seconds:
        infer()
    finish()
    elapsed = time.perf_counter() - started
    usage = resource.getrusage(resource.RUSAGE_SELF)
    cpu_seconds = usage.ru_utime + usage.ru_stime - cpu_start

    return {
        "case": name, "actual": actual, "jobs": jobs, "frames": len(latencies),
        "elapsed_s": elapsed, "fps": len(latencies) / elapsed,
        "cpu_percent": cpu_seconds / elapsed * 100,
        "cpu_ms_per_frame": cpu_seconds / len(latencies) * 1000,
        "request_ms": {key: float(value) for key, value in zip(
            ("mean", "p50", "p95", "p99"),
            (np.mean(latencies), *np.percentile(latencies, [50, 95, 99])))},
        "sensors_before": before, "sensors_after": sensors(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "auto_buff/buff_detect/yolo/model/rune_szu_public_int8.xml")
    parser.add_argument("--image", type=Path, help="Optional fixed image; preprocessing is outside inference timing")
    parser.add_argument("--device", choices=("GPU", "CPU", "ALL"), default="GPU")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--collect-only", action="store_true", help="Read environment/model only; do not compile or infer")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or args.repeats < 1 or args.warmup < 1:
        parser.error("seconds, repeats and warmup must be positive")
    if sys.platform != "linux":
        parser.error("Run on the target Linux machine")

    import resource
    import numpy as np
    import openvino as ov

    output = args.output or ROOT / "output/profiling" / datetime.now().strftime("inference_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=True)
    core = ov.Core()
    model = core.read_model(str(args.model))
    report = {
        "environment": collect(core, ov, args.model.resolve(), model), "runs": [],
        "settings": {"device": args.device, "seconds": args.seconds,
                     "repeats": args.repeats, "warmup": args.warmup},
    }
    path = output / "summary.json"

    def save():
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    save()
    print(f"Environment saved: {path}", flush=True)
    if args.collect_only:
        return
    if model.input().element_type != ov.Type.f32:
        raise ValueError("This comparison expects FP32 model input")
    data, report["input"] = load_input(model, args.image, np)
    report["note"] = "Inference only; excludes camera, preparation, decoding, refinement, tracking and display. " \
                     "request_ms is OpenVINO request latency; CPU is process usage, not temperature or power."
    cases = [case for case in CASES if args.device == "ALL" or case[1] == args.device]
    print(f"Input: {report['input']}; stop the live application before testing.", flush=True)

    for repeat in range(args.repeats):
        # Reverse alternate rounds to reduce fixed-order thermal bias.
        for case in cases if repeat % 2 == 0 else reversed(cases):
            print(f"Round {repeat + 1}: {case[0]}...", flush=True)
            try:
                run = measure(core, ov, np, resource, model, data, case, args.seconds, args.warmup)
                run["round"] = repeat + 1
                report["runs"].append(run)
                print(f"  {run['fps']:.1f} infer/s, CPU={run['cpu_percent']:.1f}%, "
                      f"CPU/frame={run['cpu_ms_per_frame']:.2f}ms, P99={run['request_ms']['p99']:.2f}ms", flush=True)
            except RuntimeError as error:
                report["runs"].append({"case": case[0], "round": repeat + 1, "error": str(error)})
                save()
                raise
            save()
            gc.collect()

    report["comparison"] = []
    print("\nComparison: median throughput / median CPU / worst round P99", flush=True)
    for case in cases:
        runs = [run for run in report["runs"] if run["case"] == case[0]]
        row = {"case": case[0], "median_fps": float(np.median([run["fps"] for run in runs])),
               "median_cpu_percent": float(np.median([run["cpu_percent"] for run in runs])),
               "worst_round_p99_ms": max(run["request_ms"]["p99"] for run in runs)}
        report["comparison"].append(row)
        print(f"{case[0]}: {row['median_fps']:.1f} infer/s, CPU={row['median_cpu_percent']:.1f}%, "
              f"P99={row['worst_round_p99_ms']:.2f}ms")
    save()
    print(f"Report: {path}")


if __name__ == "__main__":
    main()
