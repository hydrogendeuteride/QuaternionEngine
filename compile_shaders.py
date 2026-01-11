#!/usr/bin/env python3
import argparse, os, subprocess, sys
from pathlib import Path
from shutil import which

SHADER_EXTS = {
    ".frag", ".vert", ".comp", ".geom", ".tesc", ".tese",
    ".mesh", ".task",
    ".rgen", ".rint", ".rahit", ".rchit", ".rmiss", ".rcall",
}

PROFILE_FLAGS = {
    "Debug": ["-g"],
    "Release": ["-O"],
    "RelWithDebInfo": ["-O", "-g"],
}

STAGE_FLAGS_BY_EXT = {
    ".mesh": ["-fshader-stage=mesh"],
    ".task": ["-fshader-stage=task"],
}


def find_glslc(user_path):
    if user_path:
        return user_path

    env_glslc = os.environ.get("GLSLC")
    if env_glslc:
        return env_glslc

    vulkan_sdk = os.environ.get("VULKAN_SDK")
    if vulkan_sdk:
        sdk = Path(vulkan_sdk)
        candidates = [
            sdk / "Bin" / "glslc.exe",
            sdk / "Bin" / "glslc",
            sdk / "bin" / "glslc.exe",
            sdk / "bin" / "glslc",
        ]
        for c in candidates:
            if c.exists():
                return str(c)

    for name in ("glslc", "glslc.exe"):
        p = which(name)
        if p:
            return p

    sys.exit(
        "ERROR: `glslc` not found. Install the Vulkan SDK and ensure `glslc` is on PATH,\n"
        "       or set `VULKAN_SDK`, `GLSLC`, or pass `--glslc /path/to/glslc`."
    )


def parse_args():
    p = argparse.ArgumentParser(description="Compile GLSL shaders under shaders/ to .spv next to sources.")
    cfg = p.add_mutually_exclusive_group()
    cfg.add_argument("--debug", action="store_true", help="Debug profile (adds -g, no -O).")
    cfg.add_argument("--release", action="store_true", help="Release profile (adds -O, no -g).")
    p.add_argument(
        "--config",
        choices=list(PROFILE_FLAGS.keys()),
        default="RelWithDebInfo",
        help="Compilation profile (default: RelWithDebInfo).",
    )
    p.add_argument("--shader-dir", default="shaders", help="Shader source directory (default: shaders).")
    p.add_argument("--glslc", default=None, help="Path to glslc (default: auto-detect).")
    p.add_argument("--target-env", default="vulkan1.3", help="Vulkan target env for glslc (default: vulkan1.3).")
    p.add_argument("--no-werror", action="store_true", help="Do not pass -Werror.")
    p.add_argument("--quiet", action="store_true", help="Only print errors.")
    return p.parse_args()


def collect_sources(shader_dir: Path):
    return sorted([p for p in shader_dir.rglob("*") if p.is_file() and p.suffix.lower() in SHADER_EXTS])


def compile_one(glslc: str, src: Path, shader_dir: Path, args) -> int:
    out_path = src.parent / f"{src.name}.spv"
    stage_flags = STAGE_FLAGS_BY_EXT.get(src.suffix.lower(), [])

    cmd = [
        glslc,
        str(src),
        f"--target-env={args.target_env}",
        "-I", str(shader_dir),
        *PROFILE_FLAGS[args.config],
        *stage_flags,
    ]
    if not args.no_werror:
        cmd.append("-Werror")
    cmd += ["-o", str(out_path)]

    if not args.quiet:
        print(subprocess.list2cmdline(cmd))

    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.stdout and not args.quiet:
        sys.stdout.write(proc.stdout)
    if proc.returncode != 0:
        if proc.stderr:
            sys.stderr.write(proc.stderr)
    elif proc.stderr and not args.quiet:
        sys.stderr.write(proc.stderr)
    return proc.returncode


def main() -> int:
    args = parse_args()

    if args.debug:
        args.config = "Debug"
    elif args.release:
        args.config = "Release"

    repo_root = Path(__file__).resolve().parent
    shader_dir = (repo_root / args.shader_dir).resolve()
    if not shader_dir.exists():
        sys.stderr.write(f"ERROR: shader dir not found: {shader_dir}\n")
        return 2

    glslc = find_glslc(args.glslc)

    sources = collect_sources(shader_dir)
    if not sources:
        if not args.quiet:
            print(f"No shaders found under {shader_dir}")
        return 0

    failed = 0
    for src in sources:
        rc = compile_one(glslc, src, shader_dir, args)
        if rc != 0:
            failed += 1

    if failed:
        sys.stderr.write(f"ERROR: {failed} shader(s) failed to compile.\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
