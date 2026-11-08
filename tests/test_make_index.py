import gzip
import hashlib
import json
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import make_index as mi  # noqa: E402

TOML = 'id = "{id}"\nname = "{name}"\nversion = "{v}"\nlibrary = "lib{id}.so"\napi = {api}\nenabled = false\n\n[settings]\nx = 1\n'


def make_repo(tmp_path, mods=(("alpha", "1.2.3", 1), ("beta", "0.1.0", 2))):
    repo, bld = tmp_path / "repo", tmp_path / "build"
    for mid, v, api in mods:
        (repo / mid).mkdir(parents=True)
        (repo / mid / "mod.toml").write_text(TOML.format(id=mid, name=mid.title(), v=v, api=api))
        (bld / mid).mkdir(parents=True)
        (bld / mid / f"lib{mid}.so").write_bytes(b"\x7fELF" + mid.encode() * 100)
    return repo, bld, {m[0]: f"desc {m[0]}" for m in mods}


def run(tmp_path, out="out", **kw):
    repo, bld, desc = make_repo(tmp_path) if "repo" not in kw else kw.pop("repo")
    return mi.build(repo, bld, "linux-x86_64", "https://example.test/dl/", tmp_path / out, desc)


def test_index_schema_and_hash(tmp_path):
    idx = run(tmp_path)
    assert idx["schema"] == 1 and [m["id"] for m in idx["mods"]] == ["alpha", "beta"]
    a = idx["mods"][0]
    assert (a["name"], a["version"], a["api"], a["description"]) == ("Alpha", "1.2.3", 1, "desc alpha")
    p = a["platforms"]["linux-x86_64"]
    arc = tmp_path / "out" / "alpha-1.2.3-linux-x86_64.tar.gz"
    assert p["url"] == "https://example.test/dl/alpha-1.2.3-linux-x86_64.tar.gz"
    assert p["sha256"] == hashlib.sha256(arc.read_bytes()).hexdigest() and p["size"] == arc.stat().st_size
    assert json.loads((tmp_path / "out" / "index.json").read_text()) == idx


def test_archive_layout_and_metadata(tmp_path):
    run(tmp_path)
    with tarfile.open(tmp_path / "out" / "beta-0.1.0-linux-x86_64.tar.gz") as t:
        ms = t.getmembers()
        assert [m.name for m in ms] == ["beta", "beta/libbeta.so", "beta/mod.toml"]
        assert all(m.mtime == 0 and m.uid == 0 and m.gid == 0 and m.uname == "" for m in ms)
        assert t.extractfile("beta/mod.toml").read() == (tmp_path / "repo/beta/mod.toml").read_bytes()
        assert t.extractfile("beta/libbeta.so").read() == (tmp_path / "build/beta/libbeta.so").read_bytes()


def test_gzip_header_fixed(tmp_path):
    run(tmp_path)
    head = (tmp_path / "out" / "alpha-1.2.3-linux-x86_64.tar.gz").read_bytes()[:10]
    assert head[3] == 0 and head[4:8] == b"\0\0\0\0"  # no FNAME flag, mtime 0
    gzip.decompress((tmp_path / "out" / "alpha-1.2.3-linux-x86_64.tar.gz").read_bytes())


def test_deterministic_across_runs_and_mtimes(tmp_path):
    repo = make_repo(tmp_path)
    run(tmp_path, out="o1", repo=repo)
    import os
    for f in (tmp_path / "build").rglob("*.so"):
        os.utime(f, (12345, 12345))
    run(tmp_path, out="o2", repo=repo)
    for f in sorted((tmp_path / "o1").iterdir()):
        assert f.read_bytes() == (tmp_path / "o2" / f.name).read_bytes()


def test_content_change_changes_hash(tmp_path):
    repo = make_repo(tmp_path)
    a = run(tmp_path, out="o1", repo=repo)
    (tmp_path / "build/alpha/libalpha.so").write_bytes(b"other")
    b = run(tmp_path, out="o2", repo=repo)
    assert a["mods"][0]["platforms"]["linux-x86_64"]["sha256"] != b["mods"][0]["platforms"]["linux-x86_64"]["sha256"]


@pytest.mark.parametrize("bad", ['version = "1.0"', 'version = "v1.0.0"'])
def test_bad_version_rejected(tmp_path, bad):
    repo, bld, desc = make_repo(tmp_path)
    t = repo / "alpha" / "mod.toml"
    t.write_text(t.read_text().replace('version = "1.2.3"', bad))
    with pytest.raises(mi.IndexError_, match="version"):
        mi.build(repo, bld, "linux-x86_64", "u", tmp_path / "o", desc)


def test_missing_key_description_library_platform(tmp_path):
    repo, bld, desc = make_repo(tmp_path)
    with pytest.raises(mi.IndexError_, match="no description"):
        mi.build(repo, bld, "linux-x86_64", "u", tmp_path / "o", {"alpha": "x"})
    (bld / "beta" / "libbeta.so").unlink()
    with pytest.raises(mi.IndexError_, match="not found"):
        mi.build(repo, bld, "linux-x86_64", "u", tmp_path / "o", desc)
    with pytest.raises(mi.IndexError_, match="platform"):
        mi.build(repo, bld, "Linux", "u", tmp_path / "o", desc)
    t = repo / "alpha" / "mod.toml"
    t.write_text(t.read_text().replace('name = "Alpha"\n', ""))
    with pytest.raises(mi.IndexError_, match="missing key"):
        mi.discover(repo, bld, desc)


def test_bad_id_api_empty_duplicate(tmp_path):
    repo, bld, desc = make_repo(tmp_path)
    t = repo / "alpha" / "mod.toml"
    orig = t.read_text()
    t.write_text(orig.replace('id = "alpha"', 'id = "Al pha"'))
    with pytest.raises(mi.IndexError_, match="bad id"):
        mi.discover(repo, bld, desc)
    t.write_text(orig.replace("api = 1", 'api = "1"'))
    with pytest.raises(mi.IndexError_, match="api"):
        mi.discover(repo, bld, desc)
    t.write_text(orig.replace('id = "alpha"', 'id = "beta"'))
    desc["beta"] = "d"
    (bld / "alpha" / "libbeta.so").write_bytes(b"x")
    with pytest.raises(mi.IndexError_, match="duplicate"):
        mi.discover(repo, bld, desc)
    with pytest.raises(mi.IndexError_, match="no mod.toml"):
        mi.discover(tmp_path / "empty", bld, desc)


def test_real_descriptions_cover_every_mod():
    root = Path(__file__).resolve().parent.parent
    desc = mi.load_descriptions(root / "tools" / "descriptions.toml")
    import tomllib
    for toml in root.glob("*/mod.toml"):
        data = tomllib.loads(toml.read_text())
        assert data["id"] in desc and mi.VERSION_RE.match(data["version"])


def test_main_cli(tmp_path, capsys):
    repo, bld, desc = make_repo(tmp_path)
    d = tmp_path / "d.toml"
    d.write_text('[descriptions]\nalpha = "a"\nbeta = "b"\n')
    args = ["--repo", str(repo), "--build-dir", str(bld), "--platform", "linux-x86_64", "--base-url", "u",
            "--out-dir", str(tmp_path / "o"), "--descriptions", str(d)]
    assert mi.main(args) == 0 and "2 mods" in capsys.readouterr().out
    assert mi.main(args[:5] + ["Bad"] + args[6:]) == 1
    assert "make_index" in capsys.readouterr().err
