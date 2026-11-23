"""Read back a recording written by the production C++ Rerun backend (SDK 0.41)."""

import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path, help="Path to rerun_scalar_test")
    args = parser.parse_args()

    # Use the same Python SDK as the project's blueprint generator.
    from rerun.chunk import RrdReader

    with tempfile.TemporaryDirectory(prefix="mas_rerun_scalar_") as directory:
        recording = Path(directory) / "scalar.rrd"
        subprocess.run([str(args.executable.resolve()), str(recording)], check=True)
        events = {}
        for chunk in RrdReader(recording).stream():
            path = chunk.entity_path.lstrip("/")
            if path not in ("test/value", "test/value/child"):
                continue
            batch = chunk.to_record_batch()
            times = batch.column("exposure").cast("int64").to_pylist()
            for name in ("Scalars:scalars", "Clear:is_recursive"):
                if name not in batch.schema.names:
                    continue
                for time, values in zip(times, batch.column(name).to_pylist()):
                    if values is not None:
                        key = (path, time, name)
                        if key in events:
                            raise AssertionError(f"Duplicate event: {key}")
                        events[key] = values

        expected = {
            ("test/value", 0, "Scalars:scalars"): [1.25],
            ("test/value", 1_000_000_000, "Clear:is_recursive"): [False],
            ("test/value", 2_000_000_000, "Clear:is_recursive"): [False],
            ("test/value", 3_000_000_000, "Scalars:scalars"): [2.5],
            ("test/value/child", 0, "Scalars:scalars"): [10.0],
        }
        if events != expected:
            raise AssertionError(f"Unexpected scalar/clear events:\n{events}")
    print("Finite samples, NaN/Inf clears, recovery and non-recursive scope passed.")


if __name__ == "__main__":
    main()
