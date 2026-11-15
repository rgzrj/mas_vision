"""Check a recorded session using the production C++ paired reader, without algorithms."""
import argparse
from pathlib import Path
import subprocess

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Session directory or replay.yaml")
    args = parser.parse_args()
    config = args.session / "replay.yaml" if args.session.is_dir() else args.session
    executable = Path(__file__).resolve().parents[1] / "build/mas_vision_replay"
    if not executable.is_file():
        parser.error("Build mas_vision_replay in build/ first")
    return subprocess.run([str(executable), str(config.resolve(strict=True)), "--validate"], check=False).returncode

if __name__ == "__main__":
    raise SystemExit(main())
