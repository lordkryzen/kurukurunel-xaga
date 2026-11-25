# KuruKurunel for Xiaomi xaga

Linux 6.12.38 for MediaTek MT6895.

## Build

Use Linux, Clang/LLD 23, GNU make, Python 3, flex, bison, bc, Perl, OpenSSL and
libelf development packages, pahole, dtc, and Python pyelftools. Put the LLVM tools on `PATH`.

```sh
JOBS=8 ./build.sh
```

The kernel is built from `arch/arm64/configs/xaga_defconfig`.
The kernel and vendor-compatible module filenames are staged in `out/dist/`.
Output is written to `out/`; set `OUT_DIR` to use another directory.
`./build.sh kernel`, `./build.sh modules`, and `./build.sh dtbs` build each component separately.

Licenses and copyright notices are retained in their respective source trees.

The XGF estimator uses the pinned MiCode LLVM bitcode object included under its
`prebuilt/` directory. The current Clang compiles it before linking the kernel
adapter. Upstream attribution, revision and checksum accompany that input.
