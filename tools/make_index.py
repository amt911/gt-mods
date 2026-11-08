#!/usr/bin/env python3
"""Pack every mod into a release archive and write index.json (stdlib only, deterministic).

    tools/make_index.py --build-dir build --platform linux-x86_64 \
        --base-url https://github.com/amt911/gt-mods/releases/download/v2026.11.08 --out-dir dist

For each <repo>/<dir>/mod.toml it takes the built library from <build-dir>/<dir>/<library> and writes
<out-dir>/<id>-<version>-<platform>.tar.gz (folder <id>/ with mod.toml and the library), then index.json.
Same inputs give byte-identical outputs: sorted entries, fixed mtime/uid/gid/mode, gzip mtime 0.
"""
import argparse
import gzip
import hashlib
import io
import json
import re
import sys
import tarfile
import tomllib
from pathlib import Path

SCHEMA = 1
ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]*$")
VERSION_RE = re.compile(r"^\d+\.\d+\.\d+$")
PLATFORM_RE = re.compile(r"^[a-z0-9_]+-[a-z0-9_]+$")
MTIME = 0


class IndexError_(Exception):
    pass


def load_descriptions(path: Path) -> dict:
    with open(path, "rb") as f:
        return dict(tomllib.load(f).get("descriptions", {}))


def discover(repo: Path, build_dir: Path, descriptions: dict) -> list:
    """Return the mods found under repo (sorted by id), validated, with their built library path."""
    mods = []
    for toml in sorted(repo.glob("*/mod.toml")):
        with open(toml, "rb") as f:
            data = tomllib.load(f)
        try:
            mid, name, version = data["id"], data["name"], data["version"]
            library, api = data["library"], data["api"]
        except KeyError as e:
            raise IndexError_(f"{toml}: missing key {e}") from None
        if not ID_RE.match(mid):
            raise IndexError_(f"{toml}: bad id {mid!r}")
        if not VERSION_RE.match(version):
            raise IndexError_(f"{toml}: version {version!r} is not x.y.z")
        if not isinstance(api, int):
            raise IndexError_(f"{toml}: api must be an integer")
        if mid not in descriptions:
            raise IndexError_(f"no description for mod {mid!r} (tools/descriptions.toml)")
        lib = build_dir / toml.parent.name / library
        if not lib.is_file():
            raise IndexError_(f"{mid}: built library not found: {lib}")
        mods.append({"id": mid, "name": name, "version": version, "api": api,
                     "description": descriptions[mid], "toml": toml, "lib": lib, "library": library})
    if not mods:
        raise IndexError_(f"no mod.toml under {repo}")
    ids = [m["id"] for m in mods]
    if len(set(ids)) != len(ids):
        raise IndexError_("duplicate mod id")
    return sorted(mods, key=lambda m: m["id"])


def pack(mod: dict) -> bytes:
    """Deterministic .tar.gz bytes: <id>/ , <id>/mod.toml, <id>/<library>."""
    raw = io.BytesIO()
    with tarfile.open(fileobj=raw, mode="w", format=tarfile.USTAR_FORMAT) as tar:
        def meta(name, mode, typ=tarfile.REGTYPE, size=0):
            ti = tarfile.TarInfo(name)
            ti.type, ti.mode, ti.size = typ, mode, size
            ti.mtime, ti.uid, ti.gid, ti.uname, ti.gname = MTIME, 0, 0, "", ""
            return ti
        tar.addfile(meta(mod["id"] + "/", 0o755, tarfile.DIRTYPE))
        files = {"mod.toml": (mod["toml"].read_bytes(), 0o644), mod["library"]: (mod["lib"].read_bytes(), 0o755)}
        for fname in sorted(files):
            blob, mode = files[fname]
            tar.addfile(meta(f"{mod['id']}/{fname}", mode, size=len(blob)), io.BytesIO(blob))
    out = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=out, compresslevel=9, mtime=0) as gz:
        gz.write(raw.getvalue())
    return out.getvalue()


def build(repo: Path, build_dir: Path, platform: str, base_url: str, out_dir: Path, descriptions: dict) -> dict:
    if not PLATFORM_RE.match(platform):
        raise IndexError_(f"bad platform tag {platform!r} (e.g. linux-x86_64)")
    base_url = base_url.rstrip("/")
    out_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for mod in discover(repo, build_dir, descriptions):
        blob = pack(mod)
        fname = f"{mod['id']}-{mod['version']}-{platform}.tar.gz"
        (out_dir / fname).write_bytes(blob)
        entries.append({"id": mod["id"], "name": mod["name"], "version": mod["version"], "api": mod["api"],
                        "description": mod["description"],
                        "platforms": {platform: {"url": f"{base_url}/{fname}",
                                                 "sha256": hashlib.sha256(blob).hexdigest(), "size": len(blob)}}})
    index = {"schema": SCHEMA, "mods": entries}
    (out_dir / "index.json").write_text(json.dumps(index, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return index


def main(argv=None) -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", type=Path, default=here.parent, help="gt-mods checkout (default: this repo)")
    ap.add_argument("--build-dir", type=Path, required=True)
    ap.add_argument("--platform", required=True)
    ap.add_argument("--base-url", required=True, help="download URL prefix of the release assets")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--descriptions", type=Path, default=here / "descriptions.toml")
    a = ap.parse_args(argv)
    try:
        idx = build(a.repo, a.build_dir, a.platform, a.base_url, a.out_dir, load_descriptions(a.descriptions))
    except IndexError_ as e:
        print(f"make_index: {e}", file=sys.stderr)
        return 1
    print(f"make_index: {len(idx['mods'])} mods -> {a.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
