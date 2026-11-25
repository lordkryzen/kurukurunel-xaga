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

> [!CAUTION]
> **This kernel is designed to be used with my custom vendor image, not the stock vendor.**
>
> The vendor image has been heavily modified, particularly around the display and graphics stack. This includes changes to AIDL Composer and Allocator, PQ, DisplayFeature, and other components designed to work together with this kernel.
>
> Using this kernel with the stock vendor or another vendor image is likely to cause problems, ranging from broken graphics or a blank display to bootloops. Other vendor configurations are not supported.

## Known bugs

- **Fuel gauge:** The reported battery percentage can become inaccurate after the phone has been idle for a long time. For example, leaving the phone at 100% overnight (around 12 hours) can result in it reporting roughly 85%, even though actual battery drain appears to be closer to 3%. Connecting the phone to a PC over USB 2.0 seems to make the gauge recalculate its estimate, causing the reported capacity to jump to around 97% within 1–2 minutes. This appears to be a reporting issue rather than actual battery drain. For now, don't rely too heavily on the reported percentage after extended idle periods.
