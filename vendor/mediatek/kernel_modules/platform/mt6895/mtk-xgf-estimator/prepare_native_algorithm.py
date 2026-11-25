#!/usr/bin/env python3
"""Compile the pinned upstream XGF bitcode and adapt its kernel boundary."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import struct
import subprocess

from elftools.elf.elffile import ELFFile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cc", default="clang")
args = parser.parse_args()
root = Path(__file__).resolve().parent
source = root / "prebuilt/xgf_v3_ko_quark.o"
expected = "22d2d39d377ce582aa9fbc943ddf60b9254db3a1eceae389d3dafeb85c7b852a"
data = source.read_bytes()
if hashlib.sha256(data).hexdigest() != expected or data[:4] != b"BC\xc0\xde":
    raise SystemExit("Unexpected upstream XGF bitcode input")
raw = root / "native_algorithm.raw.o"
subprocess.run(shlex.split(args.cc) + ["--target=aarch64-linux-gnu", "-x", "ir",
               "-c", "-O2", "-fno-lto", str(source), "-o", str(raw)], check=True)
rename = {}
with raw.open("rb") as stream:
    elf = ELFFile(stream)
    assert elf["e_machine"] == "EM_AARCH64" and elf["e_type"] == "ET_REL"
    original_text = elf.get_section_by_name(".text").data()
    for symbol in elf.get_section_by_name(".symtab").iter_symbols():
        if symbol.name and symbol["st_shndx"] != "SHN_UNDEF" and symbol["st_info"]["type"] not in {"STT_SECTION", "STT_FILE"}:
            rename[symbol.name] = "xgf_blob_" + symbol.name.replace(".", "_")
    alternatives = elf.get_section_by_name(".altinstructions")
    alternative_count = 0
    if alternatives is not None:
        table = alternatives.data()
        assert len(table) % 12 == 0
        for offset in range(0, len(table), 12):
            _, _, capability, old_size, new_size = struct.unpack_from("<iiHBB", table, offset)
            assert (capability, old_size, new_size) == (59, 4, 4)
            alternative_count += 1
rename.update({"kmalloc_caches": "xgf_legacy_caches",
               "kmem_cache_alloc_trace": "xgf_legacy_alloc",
               "__stack_chk_guard": "xgf_legacy_canary"})
rename_file = root / "rename.txt"
rename_file.write_text("\n".join(a + " " + b for a, b in sorted(rename.items())) + "\n")
command = ["llvm-objcopy", "--redefine-syms=" + str(rename_file)]
for section in [".modinfo", "__versions", ".gnu.linkonce.this_module",
                ".rela.gnu.linkonce.this_module", ".note.gnu.build-id", ".note.Linux",
                ".altinstructions", ".rela.altinstructions", ".altinstr_replacement"]:
    command.append("--remove-section=" + section)
output = root / "native_algorithm.o"
subprocess.run(command + [str(raw), str(output)], check=True)
with output.open("rb") as stream:
    elf = ELFFile(stream)
    assert elf.get_section_by_name(".text").data() == original_text
    assert all(elf.get_section_by_name(name) is None for name in
               ("__versions", ".modinfo", ".altinstructions"))
(root / "algorithm-preflight.json").write_text(json.dumps({
    "result": "PASS", "input_sha256": expected,
    "upstream_commit": "47bb376e531f9688ebeb76c1c6714ee8944a4e76",
    "input_format": "LLVM bitcode", "compiled_text_preserved_by_adapter": True,
    "legacy_alternatives_removed": alternative_count,
    "compiler": subprocess.check_output(shlex.split(args.cc) + ["--version"], text=True).splitlines()[0]
}, indent=2) + "\n")
