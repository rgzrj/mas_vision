"""Run the official OpenVINO benchmark_app, keeping latency/throughput/layer runs separate."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--threads", type=int, default=4, help="Match detect.threads")
    parser.add_argument("--seconds", type=int, default=10, help="Measurement duration per mode; excludes model loading")
    parser.add_argument("--mode", choices=["latency", "throughput", "layers", "all"], default="latency")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.threads <= 0 or args.seconds <= 0:
        parser.error("threads and seconds must be positive")
    import openvino as ov

    model = args.model.resolve(strict=True)
    output = args.output or Path(__file__).resolve().parents[2] / "output/profiling" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    modes = ["latency", "throughput", "layers"] if args.mode == "all" else [args.mode]
    for mode in modes:
        folder = output / mode
        folder.mkdir(parents=True, exist_ok=True)
        # Use the official console entry point rather than implementing another inference loop.
        command = [sys.executable, "-c", "from openvino.tools.benchmark.main import main; main()",
                   "-m", str(model), "-d", "CPU", "-b", "1", "-t", str(args.seconds),
                   "-nthreads", str(args.threads), "-report_folder", str(folder.resolve())]
        if mode == "throughput":
            command += ["-hint", "none", "-api", "async", "-nstreams", "AUTO", "-report_type", "no_counters"]
        else:
            # The benchmark CLI requires hint=none when manually selecting a stream count.
            command += ["-hint", "none", "-api", "sync", "-nstreams", "1", "-nireq", "1",
                        "-report_type", "detailed_counters" if mode == "layers" else "no_counters"]
        if mode == "layers":
            command += ["-pc"]
        (folder / "command.json").write_text(json.dumps(
            {"model": str(model), "openvino": ov.__version__, "mode": mode, "command": command},
            ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Running {mode}: {folder.resolve()}", flush=True)
        with (folder / "benchmark.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
        if result.returncode:
            print((folder / "benchmark.log").read_text(encoding="utf-8"), file=sys.stderr)
            return result.returncode
        for line in (folder / "benchmark.log").read_text(encoding="utf-8").splitlines():
            if any(key in line for key in ["Median:", "Average:", "Throughput:"]):
                print(line)
    print(f"Reports: {output.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
