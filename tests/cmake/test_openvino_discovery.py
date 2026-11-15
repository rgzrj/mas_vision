"""Test CMake discovery using package fixtures, without installing or loading OpenVINO."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def quoted(path):
    return "[==[" + Path(path).as_posix() + "]==]"


def run(cmake):
    with tempfile.TemporaryDirectory(prefix="mas_openvino_") as directory:
        temporary = Path(directory)
        for case in ("local", "shared", "xdg", "local_priority", "manual", "environment", "notfound_cache", "prefix", "missing"):
            folder = temporary / case
            project = folder / "project with spaces"
            home = folder / "home"
            data = folder / "xdg data"
            local = project / ".venv-openvino"
            shared = home / ".local/share/mas-vision/venv"
            xdg = data / "mas-vision/venv"
            project.mkdir(parents=True)
            paths = {}
            for name, venv in (("local", local), ("shared", shared), ("xdg", xdg), ("manual", folder / "sdk")):
                package = venv / "lib/python3.12/site-packages/openvino/cmake"
                paths[name] = package
            def install(name):
                package = paths[name]
                package.mkdir(parents=True, exist_ok=True)
                (package / "OpenVINOConfig.cmake").write_text("set(OpenVINO_FOUND TRUE)\n", encoding="utf-8")
            body = ["cmake_minimum_required(VERSION 3.22)",
                    "set(PROJECT_SOURCE_DIR " + quoted(project) + ")",
                    "set(ENV{HOME} " + quoted(home) + ")",
                    'set(ENV{XDG_DATA_HOME} "")', 'set(ENV{OpenVINO_DIR} "")',
                    'set(ENV{OpenVINO_ROOT} "")', 'set(ENV{CMAKE_PREFIX_PATH} "")',
                    "set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH FALSE)",
                    "set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH FALSE)",
                    "set(CMAKE_FIND_USE_PACKAGE_REGISTRY FALSE)",
                    "set(CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY FALSE)"]
            expected = "local"
            if case in ("local", "local_priority", "manual", "environment"):
                install("local")
            if case in ("shared", "local_priority", "manual", "environment", "notfound_cache", "xdg"):
                install("shared")
                expected = "shared"
            if case == "local_priority":
                expected = "local"
            if case == "xdg":
                install("xdg")
                body += ["set(ENV{XDG_DATA_HOME} " + quoted(data) + ")"]
                expected = "xdg"
            if case in ("manual", "environment", "prefix"):
                install("manual")
                expected = "manual"
                if case == "manual":
                    body += ["set(OpenVINO_DIR " + quoted(paths["manual"]) + ' CACHE PATH "")']
                elif case == "environment":
                    body += ["set(ENV{OpenVINO_DIR} " + quoted(paths["manual"]) + ")"]
                else:
                    body += ["set(CMAKE_PREFIX_PATH " + quoted(paths["manual"]) + ")"]
            if case == "notfound_cache":
                body += ['set(OpenVINO_DIR OpenVINO_DIR-NOTFOUND CACHE PATH "")']
            body += ["include(" + quoted(ROOT / "cmake/OpenVINO.cmake") + ")"]
            if case != "missing":
                body += ["get_property(cached CACHE OpenVINO_DIR PROPERTY VALUE)",
                         "if(NOT OpenVINO_DIR STREQUAL " + quoted(paths[expected]) + ")",
                         'message(FATAL_ERROR "Wrong discovered SDK: ${OpenVINO_DIR}")', "endif()",
                         "if(NOT cached STREQUAL " + quoted(paths[expected]) + ")",
                         'message(FATAL_ERROR "SDK directory was not cached")', "endif()"]
            script = folder / "case.cmake"
            script.write_text("\n".join(body), encoding="utf-8")
            result = subprocess.run([cmake, "-P", str(script)], text=True, encoding="utf-8", capture_output=True)
            output = result.stdout + result.stderr
            if case == "missing":
                assert result.returncode != 0 and "setup_linux.sh" in output, output
            else:
                assert result.returncode == 0, output
            print(case + ": passed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmake", default=shutil.which("cmake"))
    args = parser.parse_args()
    if not args.cmake:
        parser.error("cmake is not in PATH; provide --cmake")
    run(args.cmake)
