#!/usr/bin/env python3
"""Stage device modules with the filenames expected by the vendor image."""
from pathlib import Path
import shutil
import sys

root = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
dest = out / "dist" / "modules"
dest.mkdir(parents=True, exist_ok=True)

def key(name):
    return name.removesuffix(".ko").replace("-", "_")

candidates = {}
for base in (out / "kernel", out / "vendor-modules"):
    for path in base.rglob("*.ko"):
        candidates.setdefault(key(path.name), []).append(path)

for name in (root / "modules.xaga").read_text().splitlines():
    found = candidates.get(key(name), [])
    external = [p for p in found if p.is_relative_to(out / "vendor-modules")]
    found = external or found
    if len(found) != 1:
        raise SystemExit(f"Expected one build result for {name}, found {found}")
    shutil.copyfile(found[0], dest / name)

shutil.copyfile(out / "kernel" / "arch/arm64/boot/Image.gz", out / "dist/Image.gz")
print(f"Staged {len(list(dest.glob('*.ko')))} modules in {dest}")
