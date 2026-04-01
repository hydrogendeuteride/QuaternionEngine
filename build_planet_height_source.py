#!/usr/bin/env python3
# Usage:
#   python3 build_planet_height_source.py --source in.tif --zero-radius-m 1727400 --step-m 0.5

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Convert uint16 planet DEM TIFF into 8-bit engine height source.")
    p.add_argument("--source", required=True, help="Input grayscale uint16 TIFF")
    p.add_argument("--output", help="Output grayscale PNG")
    p.add_argument("--metadata", help="Output metadata JSON")
    p.add_argument("--zero-radius-m", type=float, required=True, help="Radius represented by source value 0")
    p.add_argument("--step-m", type=float, required=True, help="Meters per uint step")
    p.add_argument("--base-mode", choices=("actual-min", "zero"), default="actual-min")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    source = Path(args.source).expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"source file not found: {source}")

    output = Path(args.output).expanduser().resolve() if args.output else source.with_name(f"{source.stem}_r8.png")
    metadata = Path(args.metadata).expanduser().resolve() if args.metadata else source.with_name(f"{source.stem}_r8.json")

    with Image.open(source) as image:
        pixels = np.asarray(image)

    if pixels.ndim != 2 or pixels.dtype != np.uint16:
        raise SystemExit(f"expected single-channel uint16 image, got shape={pixels.shape} dtype={pixels.dtype}")

    lo = int(pixels.min())
    hi = int(pixels.max())
    if hi <= lo:
        raise SystemExit("input has no dynamic range")

    base_uint = lo if args.base_mode == "actual-min" else 0
    scale = 255.0 / (hi - base_uint)
    normalized = np.clip(np.rint((pixels.astype(np.float32) - base_uint) * scale), 0.0, 255.0).astype(np.uint8)

    radius_m = args.zero_radius_m + base_uint * args.step_m
    height_max_m = (hi - base_uint) * args.step_m

    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(normalized, mode="L").save(output, optimize=True)

    metadata.parent.mkdir(parents=True, exist_ok=True)
    metadata.write_text(
        json.dumps(
            {
                "source": str(source),
                "output": str(output),
                "width": int(pixels.shape[1]),
                "height": int(pixels.shape[0]),
                "source_min_uint": lo,
                "source_max_uint": hi,
                "zero_radius_m": args.zero_radius_m,
                "step_m": args.step_m,
                "base_mode": args.base_mode,
                "recommended_radius_m": radius_m,
                "recommended_height_max_m": height_max_m,
            },
            indent=2,
        )
        + "\n"
    )

    print(f"wrote {output}")
    print(f"wrote {metadata}")
    print(f"recommended radius_m={radius_m:.3f}")
    print(f"recommended height_max_m={height_max_m:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
