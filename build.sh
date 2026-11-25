#!/usr/bin/env bash
set -euo pipefail
export XAGA_ROOT=$(cd -- "$(dirname -- "$0")" && pwd)
export ARCH=arm64 LLVM=1 LLVM_IAS=1
export CROSS_COMPILE=aarch64-linux-gnu- CROSS_COMPILE_COMPAT=arm-linux-gnueabi-
export KBUILD_BUILD_USER=${KBUILD_BUILD_USER:-lordkryzen}
export KBUILD_BUILD_HOST=${KBUILD_BUILD_HOST:-kurukurunel}
export KBUILD_BUILD_VERSION=${KBUILD_BUILD_VERSION:-17}
export LOCALVERSION=${LOCALVERSION:--KuruKurunel4Xaga-Alpha}
export KCONFIG_EXT_PREFIX="$XAGA_ROOT/"
export MALI_KCONFIG_EXT_PREFIX="$XAGA_ROOT/vendor/mediatek/kernel_modules/gpu/gpu_mali/mali_avalon/a16w/"
OUT_DIR=${OUT_DIR:-"$XAGA_ROOT/out"}
JOBS=${JOBS:-$(nproc)}
mkdir -p "$OUT_DIR"
OUT_DIR=$(cd "$OUT_DIR" && pwd)
KERNEL_OUT="$OUT_DIR/kernel"
MODULES_SRC="$XAGA_ROOT/vendor/mediatek/kernel_modules/platform/mt6895"
MODULES_OUT="$OUT_DIR/vendor-modules"
args=(-C "$XAGA_ROOT" O="$KERNEL_OUT" HOSTCC='clang -std=gnu11' HOSTCXX=clang++ LOCALVERSION="$LOCALVERSION" DEVICE_MODULES_PATH="$XAGA_ROOT" DEVICE_MODULES_REL_DIR="$XAGA_ROOT" MTK_PLATFORM=mt6895)
if [[ ! -f "$KERNEL_OUT/.config" ]]; then
    make "${args[@]}" xaga_defconfig
fi
case "${1:-all}" in
    config) make "${args[@]}" olddefconfig ;;
    prepare) make "${args[@]}" -j"$JOBS" olddefconfig modules_prepare ;;
    kernel) make "${args[@]}" -j"$JOBS" Image.gz modules ;;
    modules)
        [[ -s "$KERNEL_OUT/Module.symvers" ]] || { echo 'Build the kernel first: ./build.sh kernel' >&2; exit 1; }
        mkdir -p "$MODULES_OUT"
        cp -aL "$MODULES_SRC/." "$MODULES_OUT/"
        make "${args[@]}" -j"$JOBS" M="$MODULES_OUT" KBUILD_EXTRA_SYMBOLS= modules
        python3 "$XAGA_ROOT/scripts/stage_modules.py" "$OUT_DIR"
        ;;
    dtbs)
        mkdir -p "$OUT_DIR/dtb"
        dtc -I dts -O dtb -o "$OUT_DIR/dtb/xaga.dtb" "$XAGA_ROOT/arch/arm64/boot/dts/mediatek/xaga.dts"
        ;;
    all)
        "$0" kernel
        "$0" modules
        "$0" dtbs
        ;;
    *) echo "Usage: $0 [config|prepare|kernel|modules|dtbs|all]" >&2; exit 2 ;;
esac
