import argparse, json, os, re, shlex, subprocess, sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

try:
    from PIL import Image
    PIL_OK = True
except Exception:
    PIL_OK = False

DEFAULT_SUFFIX = {
    "albedo":    ["_albedo", "_basecolor", "_base_colour", "_base_color", "_base", "baseColor", "BaseColor"],
    "mr":        ["_mr", "_orm", "_metalrough", "_metallicroughness", "metallicRoughness", "Metallic", "Metalness"],
    "normal":    ["_normal", "_norm", "_nrm", "_normalgl", "Normal"],
    "occlusion": ["_occlusion", "_occ", "_ao"],
    "emissive":  ["_emissive", "_emission", "_emit"],
}

SUPPORTED_IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".tga", ".tif", ".tiff"}

def which_or_die(cmd):
    from shutil import which
    if which(cmd) is None:
        sys.exit(f"ERROR: `{cmd}` not found in PATH")

def build_suffix_regex(tokens):
    # ex) Foo_BaseColor -> _basecolor
    alt = "|".join([re.escape(t.lower()) for t in tokens])
    return re.compile(rf"(^|[_\-.])({alt})$", re.IGNORECASE)

def detect_role_by_suffix(stem, rx):
    s = stem.lower()
    for role, r in rx.items():
        if r.search(s):
            return role
    return None

def parse_gltf_roles(gltf_path: Path):
    """glTF(.gltf JSON) to get image role (albedo/mr/normal/occlusion/emissive)"""
    roles = {}  # uri -> role
    if not gltf_path.exists():
        return roles
    if gltf_path.suffix.lower() == ".gltf":
        data = json.loads(gltf_path.read_text(encoding="utf-8"))
    else:
        return roles

    images = data.get("images", [])
    textures = data.get("textures", [])
    materials = data.get("materials", [])

    # texture index -> image uri
    tex_to_uri = {}
    for i, tex in enumerate(textures):
        src = tex.get("source")
        if src is not None and 0 <= src < len(images):
            uri = images[src].get("uri")
            if uri:
                tex_to_uri[i] = uri

    def mark(uri, role):
        if not uri:
            return

        prio = {
            "normal":    4,
            "albedo":    3,
            "emissive":  3,
            "mr":        2,
            "occlusion": 2,
        }
        new_prio = prio.get(role, 0)
        old = roles.get(uri)
        if old is None or new_prio > prio.get(old, 0):
            roles[uri] = role

    for mat in materials:
        pbr = mat.get("pbrMetallicRoughness", {})
        base = pbr.get("baseColorTexture", {})
        mr   = pbr.get("metallicRoughnessTexture", {})
        nor  = mat.get("normalTexture", {})
        occ  = mat.get("occlusionTexture", {})
        emis = mat.get("emissiveTexture", {})

        if "index" in base and base["index"] in tex_to_uri:
            mark(tex_to_uri[base["index"]], "albedo")
        if "index" in mr and mr["index"] in tex_to_uri:
            mark(tex_to_uri[mr["index"]], "mr")
        if "index" in nor and nor["index"] in tex_to_uri:
            mark(tex_to_uri[nor["index"]], "normal")
        if "index" in occ and occ["index"] in tex_to_uri:
            mark(tex_to_uri[occ["index"]], "occlusion")
        if "index" in emis and emis["index"] in tex_to_uri:
            mark(tex_to_uri[emis["index"]], "emissive")

    return roles

def has_meaningful_alpha(img_path: Path) -> bool:
    if not PIL_OK:
        return False
    try:
        with Image.open(img_path) as im:
            if ("A" in im.getbands()) or ("transparency" in im.info):
                im = im.convert("RGBA")
                alpha = im.getchannel("A")
                extrema = alpha.getextrema()
                return bool(extrema and extrema != (255, 255))
            return False
    except Exception:
        return False
    return False

def decide_targets(role, albedo_target, img_path):
    """ return transcode target(BCn), OETF(srgb/linear)"""
    if role == "normal":
        return "bc5", "linear"
    if role == "mr":
        return "bc7", "linear"
    if role == "occlusion":
        # AO is data, not color
        return "bc7", "linear"
    # albedo
    if albedo_target == "auto":
        if has_meaningful_alpha(img_path):
            return "bc3", "srgb"
        else:
            return "bc1", "srgb"
    elif albedo_target in ("bc1", "bc3", "bc7"):
        return albedo_target, "srgb"
    else:
        return "bc7", "srgb"

def run_cmd(args_list, dry_run=False):
    cmd = " ".join(shlex.quote(a) for a in args_list)
    if dry_run:
        print(f"[DRY] {cmd}")
        return 0
    try:
        subprocess.run(args_list, check=True)
        return 0
    except subprocess.CalledProcessError as e:
        print(f"[ERR] {cmd}\n  -> exit {e.returncode}", file=sys.stderr)
        return e.returncode

def process_one(img_path: Path, out_dir: Path, role, opts):
    stem = img_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp_dir = out_dir / ".intermediate"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    tmp_ktx2 = tmp_dir / f"{stem}.uastc.ktx2"

    # 1) PNG -> KTX2(UASTC)
    target_bc, oetf = decide_targets(role, opts.albedo_target, img_path)
    toktx = [
        "toktx",
        "--t2",
        "--encode", "uastc",
        "--uastc_quality", str(opts.uastc_quality),
    ]
    if role == "normal":
        toktx += ["--normal_mode", "--normalize"]
    if opts.mipmaps:
        toktx += ["--genmipmap"]
    if opts.flip_y:
        toktx += ["--lower_left_maps_to_s0t0"]
    # albedo: srgb, else linear
    toktx += ["--assign_oetf", oetf]
    toktx += [str(tmp_ktx2), str(img_path)]
    rc = run_cmd(toktx, dry_run=opts.dry_run)
    if rc != 0: return rc

    # 2) UASTC KTX2 -> BCn KTX2
    out_ktx2 = out_dir / f"{stem}.ktx2"
    ktx_trans = [
        "ktx", "transcode",
        "--target", target_bc,
        str(tmp_ktx2), str(out_ktx2)
    ]
    rc = run_cmd(ktx_trans, dry_run=opts.dry_run)
    if rc != 0: return rc

    if not opts.keep_temp and not opts.dry_run:
        try:
            tmp_ktx2.unlink()
        except Exception:
            pass
    return 0

def main():
    p = argparse.ArgumentParser(description="Image → KTX2(BCn) encoder (toktx + ktx transcode)")
    p.add_argument("-i", "--input", required=True, help="Input folder(recursive) or image file")
    p.add_argument("-o", "--output", required=True, help="Output folder")
    p.add_argument("--gltf", help=".gltf File path (optional). glTF first, suffix last")
    p.add_argument("--suffix-albedo", default=",".join(DEFAULT_SUFFIX["albedo"]),
                   help="albedo suffix CSV (Base: %s)" % ",".join(DEFAULT_SUFFIX["albedo"]))
    p.add_argument("--suffix-mr", default=",".join(DEFAULT_SUFFIX["mr"]))
    p.add_argument("--suffix-normal", default=",".join(DEFAULT_SUFFIX["normal"]))
    p.add_argument("--suffix-occlusion", default=",".join(DEFAULT_SUFFIX["occlusion"]))
    p.add_argument("--suffix-emissive", default=",".join(DEFAULT_SUFFIX["emissive"]))
    p.add_argument("--albedo-target", choices=["auto", "bc1", "bc3", "bc7"], default="bc7",
                   help="albedo BC format(auto=non alpha BC1, alpha BC3)")
    p.add_argument("--uastc-quality", type=int, default=2, help="UASTC quality(0~4)")
    p.add_argument("--mipmaps", action="store_true", help="mipmap generation")
    p.add_argument("--flip-y", action="store_true", help="Y flip(t0 to bottom)")
    p.add_argument("--keep-temp", action="store_true", help="Preserve temporal UASTC")
    p.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4, help="Concurrent task size")
    p.add_argument("--dry-run", action="store_true", help="Don't execute")
    opts = p.parse_args()

    which_or_die("toktx")
    which_or_die("ktx")

    rx = {
        "albedo": build_suffix_regex([s.strip() for s in opts.suffix_albedo.split(",") if s.strip()]),
        "mr":     build_suffix_regex([s.strip() for s in opts.suffix_mr.split(",") if s.strip()]),
        "normal": build_suffix_regex([s.strip() for s in opts.suffix_normal.split(",") if s.strip()]),
        "occlusion": build_suffix_regex([s.strip() for s in opts.suffix_occlusion.split(",") if s.strip()]),
        "emissive":  build_suffix_regex([s.strip() for s in opts.suffix_emissive.split(",") if s.strip()]),
    }

    gltf_roles = {}
    if opts.gltf:
        gltf_roles = parse_gltf_roles(Path(opts.gltf))

    in_path = Path(opts.input)
    img_files = []
    if in_path.is_file() and in_path.suffix.lower() in SUPPORTED_IMAGE_EXTS:
        img_files = [in_path]
    else:
        for ext in SUPPORTED_IMAGE_EXTS:
            img_files.extend(in_path.rglob(f"*{ext}"))
    if not img_files:
        sys.exit("No input images (supported: .png .jpg .jpeg .tga .tif .tiff).")

    out_dir = Path(opts.output)

    def decide_role_for_path(p: Path):
        if gltf_roles:
            for uri, role in gltf_roles.items():
                if Path(uri).name.lower() == p.name.lower():
                    return role

        by_suffix = detect_role_by_suffix(p.stem, rx)
        return by_suffix or "albedo"

    tasks = []
    with ThreadPoolExecutor(max_workers=opts.jobs) as ex:
        futs = {}
        for img in img_files:
            role = decide_role_for_path(img)
            fut = ex.submit(process_one, img, out_dir, role, opts)
            futs[fut] = (img, role)
        any_err = False
        for fut in as_completed(futs):
            img, role = futs[fut]
            rc = fut.result()
            status = "OK" if rc == 0 else f"ERR({rc})"
            print(f"[{status}] {img.name} -> role={role}")
            if rc != 0:
                any_err = True
        if any_err:
            sys.exit(2)

if __name__ == "__main__":
    main()
