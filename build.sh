#!/usr/bin/env bash
# ============================================================
# build.sh — clean CMake configure + build for LtmuTracker.
#
# Same shape as the original repo's build.sh: auto-detect the Jetson
# board from the device tree, wipe build/ for a fully clean build each
# time (stale CMakeCache.txt after a toolchain/flag change is a classic
# silent-wrong-binary trap), then grant cap_sys_nice so the tracker's
# capture/tracker/streaming threads can request real-time scheduling
# without needing to run the whole binary as root.
#
# LTMU has one engine per branch (no constant_robotics/cuda_library/
# lockon choice to make), so there's no interactive engine picker here —
# just ONNXRUNTIME_ROOT, which the original repo's engines didn't need
# either. Override paths via env vars; every one has a sane default.
# ============================================================
set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"
ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-/opt/onnxruntime}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"

BRANCH="$(git -C "$SCRIPT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")"

echo ""
echo -e "${CYAN}==========================================${NC}"
echo -e "${CYAN}   LTMU Tracker Build${NC}"
echo -e "${CYAN}==========================================${NC}"
echo -e "${CYAN}   branch          : ${BRANCH}${NC}"

# WHY: auto-detect Jetson board from device tree — same probe as the
# original repo's build.sh. LTMU's CMakeLists doesn't currently branch
# on board, but the label is useful in build logs and for a future
# per-board tuning knob (e.g. INT8 vs FP16 TensorRT engine choice).
COMPATIBLE=$(cat /proc/device-tree/compatible 2>/dev/null | tr '\0' '\n')
if echo "$COMPATIBLE" | grep -qiE "p3767|tegra234"; then
  BOARD="orin"
  echo -e "${GREEN}   board detected  : Jetson Orin NX${NC}"
elif echo "$COMPATIBLE" | grep -qiE "p3668|tegra194"; then
  BOARD="xavier"
  echo -e "${GREEN}   board detected  : Jetson Xavier NX${NC}"
else
  BOARD="unknown"
  echo -e "${YELLOW}   board not detected — building generic (not Jetson, or unrecognised device tree)${NC}"
fi

echo -e "${CYAN}   onnxruntime root: ${ONNXRUNTIME_ROOT}${NC}"
echo -e "${CYAN}   build type      : ${BUILD_TYPE}${NC}"
echo -e "${CYAN}==========================================${NC}"
echo ""

if [ ! -d "$ONNXRUNTIME_ROOT" ]; then
  echo -e "${RED}error: ONNXRUNTIME_ROOT ($ONNXRUNTIME_ROOT) does not exist.${NC}"
  echo "  Download a release (see README.md 'ONNX Runtime') or set ONNXRUNTIME_ROOT=/your/path"
  exit 1
fi

echo -e "${YELLOW}Clearing $BUILD_DIR for a fully clean build...${NC}"
rm -rf "$BUILD_DIR"

cmake -B "$BUILD_DIR" \
      -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
      -DJETSON_BOARD:STRING="$BOARD" \
      "$SCRIPT_DIR"

cmake --build "$BUILD_DIR" -j"$JOBS"

echo ""
echo -e "${CYAN}Built binaries:${NC}"
ls "$BUILD_DIR/bin/" 2>/dev/null || echo "  (none — build failed before producing bin/)"

# Real-time thread priority for capture/tracker/streaming without running
# the whole process as root. Best-effort: no sudo password prompt here —
# if it fails (no sudo, no setcap installed), the binary still runs, just
# without RT scheduling capability.
BIN="$BUILD_DIR/bin/LtmuTracker"
if [ -f "$BIN" ] && command -v setcap >/dev/null 2>&1; then
  if sudo -n setcap cap_sys_nice+ep "$BIN" 2>/dev/null; then
    echo -e "${GREEN}cap_sys_nice set on LtmuTracker${NC}"
  else
    echo -e "${YELLOW}skipped cap_sys_nice (needs sudo) — run manually if RT scheduling is needed:${NC}"
    echo "  sudo setcap cap_sys_nice+ep $BIN"
  fi
fi
