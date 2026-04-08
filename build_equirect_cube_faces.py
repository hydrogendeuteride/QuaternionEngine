#!/usr/bin/env python3
# Usage: python3 build_equirect_cube_faces.py --source path/to/input.png
#        [--name job] [--output path/to/L0] [--height-mode] [--write-r16-ktx2]
#        [--name job] [--output path/to/L0] [--height-mode] [--write-ktx2]

import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image

Image.MAX_IMAGE_PIXELS = None

FACE_NAMES = ("nx", "ny", "nz", "px", "py", "pz")
REMAP = {
    "nx": ("nx", False),
    "ny": ("px", True),
    "nz": ("py", False),
    "px": ("ny", False),
    "py": ("nz", True),
    "pz": ("pz", False),
}


def main() -> int:
    name = None
    source = None
    output_dir = None
    height_mode = False
    write_r16_ktx2 = False

    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--name":
            name = next(args, "").strip()
        elif arg == "--source":
            source = Path(next(args, ""))
        elif arg == "--output":
            output_dir = Path(next(args, ""))
        elif arg == "--height-mode":
            height_mode = True
        elif arg == "--write-r16-ktx2":
            write_r16_ktx2 = True
        else:
            raise SystemExit(f"unknown argument: {arg}")

    if source is None:
        raise SystemExit("--source is required")

    source = source.expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"source file not found: {source}")
    name = (name or "faces").strip() or "faces"
    output_dir = (output_dir or source.parent / "L0").expanduser().resolve()

    with tempfile.TemporaryDirectory(prefix=f"{name.lower()}-specular-") as temp_dir:
        temp_dir = Path(temp_dir)
        strip_path = temp_dir / "specular_c6x1.png"
        numbered_dir = temp_dir / "faces"
        numbered_dir.mkdir(parents=True, exist_ok=True)

        subprocess.run([
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-i",
            str(source),
            "-vf",
            "v360=input=equirect:output=c6x1:out_forder=rludfb",
            *([] if not height_mode else ["-pix_fmt", "gray16be"]),
            "-frames:v",
            "1",
            "-update",
            "1",
            str(strip_path),
        ], check=True)
        subprocess.run([
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-i",
            str(strip_path),
            "-vf",
            "untile=6x1",
            *([] if not height_mode else ["-pix_fmt", "gray16be"]),
            str(numbered_dir / "%d.png"),
        ], check=True)

        numbered_faces = {}
        for index, face_name in enumerate(FACE_NAMES, start=1):
            with Image.open(numbered_dir / f"{index}.png") as image:
                if height_mode:
                    if image.mode in ("I;16", "I;16B", "I;16L"):
                        numbered_faces[face_name] = image.copy()
                    else:
                        numbered_faces[face_name] = image.convert("I;16").copy()
                else:
                    numbered_faces[face_name] = image.convert("L").copy()

        rotate180 = Image.Transpose.ROTATE_180
        output_dir.mkdir(parents=True, exist_ok=True)
        for face_name, (source_face, rotate) in REMAP.items():
            png_path = output_dir / f"{face_name}.png"
            png_path.unlink(missing_ok=True)
            if write_r16_ktx2:
                png_path.with_suffix(".ktx2").unlink(missing_ok=True)

            face_image = numbered_faces[source_face].transpose(rotate180) if rotate else numbered_faces[source_face]
            face_image.save(png_path, optimize=True)
            if write_r16_ktx2:
                subprocess.run([
                    "toktx",
                    "--t2",
                    "--genmipmap",
                    "--assign_oetf", "linear",
                    "--target_type", "R",
                    str(png_path.with_suffix(".ktx2")),
                    str(png_path),
                ], check=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
