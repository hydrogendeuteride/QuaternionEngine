#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image


FACE_NAMES = ("nx", "ny", "nz", "px", "py", "pz")
NUMBERED_FACE_NAMES = {
    1: "nx",
    2: "ny",
    3: "nz",
    4: "px",
    5: "py",
    6: "pz",
}


def repo_root() -> Path:
    return Path(__file__).resolve().parent


def default_source() -> Path:
    return repo_root() / "assets/planets/earth/specular/earth_landocean_16K.png"


def default_output() -> Path:
    return repo_root() / "assets/planets/earth/specular/L0"


def run(cmd: list[str]) -> None:
    subprocess.run(cmd, check=True)


def require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise SystemExit(f"required tool not found: {name}")


def build_strip(source: Path, strip_path: Path) -> None:
    run([
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-i",
        str(source),
        "-vf",
        "v360=input=equirect:output=c6x1:out_forder=rludfb",
        "-frames:v",
        "1",
        "-update",
        "1",
        str(strip_path),
    ])


def untile_strip(strip_path: Path, numbered_dir: Path) -> None:
    run([
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-i",
        str(strip_path),
        "-vf",
        "untile=6x1",
        str(numbered_dir / "%d.png"),
    ])


def load_initial_faces(numbered_dir: Path) -> dict[str, Image.Image]:
    loaded: dict[str, Image.Image] = {}
    for index, face_name in NUMBERED_FACE_NAMES.items():
        with Image.open(numbered_dir / f"{index}.png") as image:
            loaded[face_name] = image.convert("L").copy()
    return loaded


def remap_faces(initial: dict[str, Image.Image]) -> dict[str, Image.Image]:
    rotate180 = Image.Transpose.ROTATE_180
    return {
        "nx": initial["nx"].copy(),
        "ny": initial["px"].transpose(rotate180),
        "nz": initial["py"].copy(),
        "px": initial["ny"].copy(),
        "py": initial["nz"].transpose(rotate180),
        "pz": initial["pz"].copy(),
    }


def write_faces(output_dir: Path, faces: dict[str, Image.Image]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for face_name in FACE_NAMES:
        png_path = output_dir / f"{face_name}.png"
        ktx_path = output_dir / f"{face_name}.ktx2"
        if png_path.exists():
            png_path.unlink()
        if ktx_path.exists():
            ktx_path.unlink()
        faces[face_name].save(png_path, optimize=True)


def encode_ktx2(output_dir: Path) -> None:
    for face_name in FACE_NAMES:
        png_path = output_dir / f"{face_name}.png"
        ktx_path = output_dir / f"{face_name}.ktx2"
        run([
            "toktx",
            "--t2",
            "--genmipmap",
            "--assign_oetf",
            "linear",
            "--target_type",
            "R",
            "--encode",
            "uastc",
            "--uastc_quality",
            "2",
            "--zcmp",
            "3",
            str(ktx_path),
            str(png_path),
        ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build the Earth terrain specular cube faces and KTX2 outputs.",
    )
    parser.add_argument(
        "--source",
        type=Path,
        default=default_source(),
        help="Source equirect grayscale texture.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output(),
        help="Output directory for px/nx/py/ny/pz/nz PNG + KTX2 files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.source.resolve()
    output_dir = args.output.resolve()

    if not source.is_file():
        raise SystemExit(f"source file not found: {source}")

    require_tool("ffmpeg")
    require_tool("toktx")

    with tempfile.TemporaryDirectory(prefix="earth-specular-") as temp_dir_str:
        temp_dir = Path(temp_dir_str)
        strip_path = temp_dir / "earth_specular_c6x1.png"
        numbered_dir = temp_dir / "faces"
        numbered_dir.mkdir(parents=True, exist_ok=True)

        build_strip(source, strip_path)
        untile_strip(strip_path, numbered_dir)
        initial = load_initial_faces(numbered_dir)
        faces = remap_faces(initial)
        write_faces(output_dir, faces)
        encode_ktx2(output_dir)

    return 0


if __name__ == "__main__":
    sys.exit(main())
