"""Check explicitly selected project files without rewriting them. Python stdlib only."""

import argparse
import json
from pathlib import Path
import shutil
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OWNED = {"apps", "auto_buff", "threads", "rm_utils", "tests"}


def find_tool(name):
    executable = shutil.which(name)
    if executable:
        return executable
    raise RuntimeError(f"{name} is not installed or is not in PATH")


def run(command):
    print(shlex.join(str(item) for item in command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tool", choices=("format", "tidy", "cppcheck", "all"), default="format")
    parser.add_argument("--build", type=Path, default=ROOT / "build")
    parser.add_argument("files", nargs="+", type=Path, help="Explicit .cpp/.hpp files; third_party is excluded")
    args = parser.parse_args()
    files = [path.resolve() for path in args.files]
    for path in files:
        if not path.is_relative_to(ROOT) or path.relative_to(ROOT).parts[0] not in OWNED:
            parser.error(f"Not an owned project file: {path}")
        if not path.is_file() or path.suffix not in {".cpp", ".hpp"}:
            parser.error(f"Expected an existing .cpp or .hpp: {path}")

    if args.tool in {"format", "all"}:
        run([find_tool("clang-format"), "--dry-run", "--Werror", *files])
    if args.tool in {"tidy", "cppcheck", "all"}:
        database = args.build.resolve() / "compile_commands.json"
        commands = json.loads(database.read_text(encoding="utf-8-sig"))
        selected = [item for item in commands if Path(item["file"]).resolve() in files]
        matched = {Path(item["file"]).resolve() for item in selected}
        missing = [str(path) for path in files if path.suffix == ".cpp" and path not in matched]
        if missing:
            parser.error("Files missing from compile_commands.json: " + ", ".join(missing))
        if not selected:
            parser.error("Static analysis requires at least one compiled .cpp file")
        if args.tool in {"tidy", "all"}:
            command = [find_tool("clang-tidy"), "-p", str(args.build.resolve())]
            run([*command, *sorted(matched)])
        if args.tool in {"cppcheck", "all"}:
            # Limit the compilation database to the requested units, including their actual defines/includes.
            with tempfile.TemporaryDirectory(prefix="buff_cppcheck_") as temporary:
                project = Path(temporary) / "compile_commands.json"
                project.write_text(json.dumps(selected, ensure_ascii=False), encoding="utf-8")
                run([find_tool("cppcheck"), f"--project={project}", "--enable=warning,performance,portability",
                     "--suppress=missingIncludeSystem", "--inline-suppr", "--error-exitcode=1", "--std=c++23"])
    print("Selected code checks passed.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError, ValueError) as error:
        raise SystemExit(str(error)) from error
