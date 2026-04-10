#!/usr/bin/env python3
# Usage:
#   python3 ./build.py
#   python3 ./build.py debug
#   python3 ./build.py release linux
#   python3 ./build.py release linux ./out/release
#   py .\build.py debug windows .\cmake-build-debug-win-clangcl

from pathlib import Path
import os, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parent
CONFIGS = {"debug": "Debug", "release": "Release", "reldeb": "RelWithDebInfo", "relwithdebinfo": "RelWithDebInfo"}
TOOLS = {
    "linux": ("Unix Makefiles", ("clang", "clang++", "make"), ("-DCMAKE_C_COMPILER=clang", "-DCMAKE_CXX_COMPILER=clang++"), "linux-clang"),
    "windows": ("Ninja", ("clang-cl", "lld-link", "llvm-rc", "ninja"), ("-DCMAKE_C_COMPILER=clang-cl", "-DCMAKE_CXX_COMPILER=clang-cl", "-DCMAKE_LINKER=lld-link", "-DCMAKE_RC_COMPILER=llvm-rc"), "win-clangcl"),
}


def fail(message: str) -> None:
    raise SystemExit(f"ERROR: {message}")


def run(cmd: list[str]) -> None:
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=ROOT, check=True)


def main() -> int:
    config = CONFIGS.get((sys.argv[1] if len(sys.argv) > 1 else "release").lower())
    target = (sys.argv[2] if len(sys.argv) > 2 else ("windows" if os.name == "nt" else "linux")).lower()
    if not config:
        fail("config must be debug, release, or reldeb")
    if target not in TOOLS:
        fail("platform must be linux or windows")

    generator, required, defines, suffix = TOOLS[target]
    missing = [cmd for cmd in ("cmake", *required) if not shutil.which(cmd)]
    if missing:
        fail("missing commands: " + ", ".join(missing))

    if len(sys.argv) > 3:
        build_dir = Path(sys.argv[3]).expanduser()
        build_dir = build_dir if build_dir.is_absolute() else ROOT / build_dir
    else:
        build_dir = ROOT / f"cmake-build-{config.lower()}-{suffix}"
    try:
        run(["cmake", "-S", str(ROOT), "-B", str(build_dir), "-G", generator, f"-DCMAKE_BUILD_TYPE={config}", *defines])
        run(["cmake", "--build", str(build_dir), "--target", "vulkan_engine", "--parallel", str(max(1, os.cpu_count() or 1))])
    except subprocess.CalledProcessError as exc:
        return exc.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
