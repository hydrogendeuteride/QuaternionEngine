#!/usr/bin/env python3

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None


FACE_NAMES = ("px", "nx", "py", "ny", "pz", "nz")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate cube-face detail normal and cavity maps from cube-face height PNGs."
    )
    parser.add_argument("--height-dir", required=True, help="Input face directory containing px/nx/py/ny/pz/nz.png")
    parser.add_argument("--detail-output", required=True, help="Output directory for detail normal faces")
    parser.add_argument("--cavity-output", required=True, help="Output directory for cavity faces")
    parser.add_argument("--radius-m", type=float, required=True, help="Planet radius in meters")
    parser.add_argument("--height-max-m", type=float, required=True, help="Height range represented by input faces")
    parser.add_argument("--detail-scale", type=float, default=1.0, help="Residual height multiplier for detail normals")
    parser.add_argument("--detail-blur-radius", type=int, default=12, help="High-pass blur radius for detail normals")
    parser.add_argument("--cavity-small-radius", type=int, default=3, help="Small blur radius for cavity extraction")
    parser.add_argument("--cavity-large-radius", type=int, default=12, help="Large blur radius for cavity extraction")
    parser.add_argument("--cavity-gain", type=float, default=0.9, help="Cavity darkening multiplier")
    parser.add_argument("--edge-fade-px", type=int, default=6, help="Fade normals/cavity to neutral near face edges")
    parser.add_argument(
        "--write-ktx2",
        action="store_true",
        help="Also write .ktx2 files next to the generated PNGs using toktx + ktx",
    )
    return parser.parse_args()


def load_height_image(image: Image.Image) -> np.ndarray:
    if image.mode in ("I;16", "I;16B", "I;16L"):
        pixels = np.array(image, dtype=np.uint16).astype(np.float32)
        return pixels / 65535.0

    if image.mode == "I":
        pixels = np.array(image, dtype=np.int32).astype(np.float32)
        return np.clip(pixels, 0.0, 65535.0) / 65535.0

    return np.asarray(image.convert("L"), dtype=np.float32) / 255.0


def load_height_faces(height_dir: Path) -> dict[str, np.ndarray]:
    faces: dict[str, np.ndarray] = {}
    for face in FACE_NAMES:
        path = height_dir / f"{face}.png"
        if not path.is_file():
            raise SystemExit(f"missing height face: {path}")
        with Image.open(path) as image:
            faces[face] = load_height_image(image)
    return faces


def box_blur(image: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return image.copy()

    window = radius * 2 + 1

    padded_x = np.pad(image, ((0, 0), (radius, radius)), mode="edge")
    csum_x = np.pad(np.cumsum(padded_x, axis=1, dtype=np.float32), ((0, 0), (1, 0)))
    blurred_x = (csum_x[:, window:] - csum_x[:, :-window]) / float(window)

    padded_y = np.pad(blurred_x, ((radius, radius), (0, 0)), mode="edge")
    csum_y = np.pad(np.cumsum(padded_y, axis=0, dtype=np.float32), ((1, 0), (0, 0)))
    return (csum_y[window:, :] - csum_y[:-window, :]) / float(window)


def face_direction_grid(face: str, height: int, width: int) -> np.ndarray:
    u = np.linspace(0.0, 1.0, width, dtype=np.float32)
    v = np.linspace(0.0, 1.0, height, dtype=np.float32)
    uu, vv = np.meshgrid(u, v, indexing="xy")
    cu = uu * 2.0 - 1.0
    cv = vv * 2.0 - 1.0

    if face == "px":
        direction = np.stack((np.ones_like(cu), -cv, -cu), axis=-1)
    elif face == "nx":
        direction = np.stack((-np.ones_like(cu), -cv, cu), axis=-1)
    elif face == "py":
        direction = np.stack((cu, np.ones_like(cu), cv), axis=-1)
    elif face == "ny":
        direction = np.stack((cu, -np.ones_like(cu), -cv), axis=-1)
    elif face == "pz":
        direction = np.stack((cu, -cv, np.ones_like(cu)), axis=-1)
    elif face == "nz":
        direction = np.stack((-cu, -cv, -np.ones_like(cu)), axis=-1)
    else:
        raise ValueError(f"unsupported face: {face}")

    norm = np.linalg.norm(direction, axis=-1, keepdims=True)
    return direction / np.maximum(norm, 1e-8)


def normalize_vectors(vectors: np.ndarray) -> np.ndarray:
    length = np.linalg.norm(vectors, axis=-1, keepdims=True)
    return vectors / np.maximum(length, 1e-8)


def edge_fade_mask(height: int, width: int, edge_fade_px: int) -> np.ndarray:
    if edge_fade_px <= 0:
        return np.ones((height, width), dtype=np.float32)

    yy = np.arange(height, dtype=np.float32)[:, None]
    xx = np.arange(width, dtype=np.float32)[None, :]
    dist = np.minimum(xx, yy)
    dist = np.minimum(dist, (width - 1.0) - xx)
    dist = np.minimum(dist, (height - 1.0) - yy)
    return np.clip(dist / float(edge_fade_px), 0.0, 1.0)


def compute_detail_normal(face: str,
                          height_face: np.ndarray,
                          radius_m: float,
                          height_max_m: float,
                          detail_scale: float,
                          blur_radius: int,
                          edge_fade_px: int) -> np.ndarray:
    height_px, width_px = height_face.shape
    direction = face_direction_grid(face, height_px, width_px)
    low = box_blur(height_face, blur_radius)
    residual_m = (height_face - low) * height_max_m * detail_scale

    position = direction * (radius_m + residual_m)[..., None]
    padded = np.pad(position, ((1, 1), (1, 1), (0, 0)), mode="edge")
    p_left = padded[1:-1, :-2]
    p_right = padded[1:-1, 2:]
    p_up = padded[:-2, 1:-1]
    p_down = padded[2:, 1:-1]

    normal = np.cross(p_down - p_up, p_right - p_left)
    normal = normalize_vectors(normal)

    outward = np.sum(normal * direction, axis=-1, keepdims=True)
    normal = np.where(outward < 0.0, -normal, normal)

    fade = edge_fade_mask(height_px, width_px, edge_fade_px)[..., None]
    normal = normalize_vectors(direction * (1.0 - fade) + normal * fade)
    return normal


def compute_cavity(face: np.ndarray,
                   small_radius: int,
                   large_radius: int,
                   cavity_scale: float,
                   cavity_gain: float,
                   edge_fade_px: int) -> np.ndarray:
    small = box_blur(face, small_radius)
    large = box_blur(face, large_radius)
    concavity = np.maximum(small - face, 0.0) + 0.5 * np.maximum(large - face, 0.0)

    cavity = 1.0 - np.clip((concavity / max(cavity_scale, 1e-6)) * cavity_gain, 0.0, 1.0)
    fade = edge_fade_mask(face.shape[0], face.shape[1], edge_fade_px)
    return 1.0 * (1.0 - fade) + cavity * fade


def save_rgb_png(path: Path, rgb_linear: np.ndarray) -> None:
    encoded = np.clip(np.rint((rgb_linear * 0.5 + 0.5) * 255.0), 0.0, 255.0).astype(np.uint8)
    Image.fromarray(encoded, mode="RGB").save(path, optimize=True)


def save_gray_png(path: Path, gray: np.ndarray) -> None:
    encoded = np.clip(np.rint(gray * 255.0), 0.0, 255.0).astype(np.uint8)
    Image.fromarray(encoded, mode="L").save(path, optimize=True)


def write_ktx2_from_png(png_path: Path, target: str) -> None:
    with tempfile.TemporaryDirectory(prefix=f"{png_path.stem}-ktx2-") as temp_dir_str:
        temp_dir = Path(temp_dir_str)
        temp_ktx = temp_dir / f"{png_path.stem}.uastc.ktx2"
        subprocess.run(
            [
                "toktx",
                "--t2",
                "--encode", "uastc",
                "--uastc_quality", "2",
                "--assign_oetf", "linear",
                "--genmipmap",
                str(temp_ktx),
                str(png_path),
            ],
            check=True,
        )
        subprocess.run(
            [
                "ktx",
                "transcode",
                "--target", target,
                str(temp_ktx),
                str(png_path.with_suffix(".ktx2")),
            ],
            check=True,
        )


def main() -> int:
    args = parse_args()
    height_dir = Path(args.height_dir).expanduser().resolve()
    detail_output = Path(args.detail_output).expanduser().resolve()
    cavity_output = Path(args.cavity_output).expanduser().resolve()

    detail_output.mkdir(parents=True, exist_ok=True)
    cavity_output.mkdir(parents=True, exist_ok=True)

    height_faces = load_height_faces(height_dir)

    cavity_metric = []
    for face_name in FACE_NAMES:
        face = height_faces[face_name]
        small = box_blur(face, args.cavity_small_radius)
        large = box_blur(face, args.cavity_large_radius)
        cavity_metric.append(np.maximum(small - face, 0.0) + 0.5 * np.maximum(large - face, 0.0))

    cavity_values = np.concatenate([metric.reshape(-1) for metric in cavity_metric])
    cavity_scale = float(np.percentile(cavity_values, 99.5))
    cavity_scale = max(cavity_scale, 1e-5)

    for face_name in FACE_NAMES:
        face = height_faces[face_name]
        detail_normal = compute_detail_normal(
            face_name,
            face,
            radius_m=args.radius_m,
            height_max_m=args.height_max_m,
            detail_scale=args.detail_scale,
            blur_radius=args.detail_blur_radius,
            edge_fade_px=args.edge_fade_px,
        )
        cavity = compute_cavity(
            face,
            small_radius=args.cavity_small_radius,
            large_radius=args.cavity_large_radius,
            cavity_scale=cavity_scale,
            cavity_gain=args.cavity_gain,
            edge_fade_px=args.edge_fade_px,
        )

        detail_png = detail_output / f"{face_name}.png"
        cavity_png = cavity_output / f"{face_name}.png"
        save_rgb_png(detail_png, detail_normal)
        save_gray_png(cavity_png, cavity)

        if args.write_ktx2:
            write_ktx2_from_png(detail_png, "bc7")
            write_ktx2_from_png(cavity_png, "bc7")

        print(f"wrote {detail_png}")
        print(f"wrote {cavity_png}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
