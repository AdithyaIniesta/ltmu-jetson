#!/usr/bin/env bash
# ============================================================
# run_jp5.sh — interactive launcher, uav-dataset branch (JetPack 5).
#
# Prompts for the handful of settings that actually change between
# runs; Enter accepts the default shown in [brackets] for a fast
# standard-setup boot, matching the operator experience described for
# the original repo's run scripts.
#
# Motion model (CV/CA/CTRV/IMM) is a BOOT-TIME choice only — see
# src/tracker/motion.h for why: no single fixed model wins on both a
# turning target and a falling one, and the ground station GUI is
# deliberately left untouched, so switching models mid-mission means
# restarting with a different choice here, not a GUI toggle.
# ============================================================
set -euo pipefail

BIN="./build/bin/LtmuTracker"
if [ ! -x "$BIN" ]; then
  echo "error: $BIN not found — build first:"
  echo "  cmake -B build -DONNXRUNTIME_ROOT=/opt/onnxruntime -DCMAKE_BUILD_TYPE=Release"
  echo "  cmake --build build -j\$(nproc)"
  exit 1
fi

ask() {
  # ask VAR "prompt" "default"
  local __var="$1" __prompt="$2" __default="$3" __reply
  read -r -p "$__prompt [$__default]: " __reply || true
  printf -v "$__var" '%s' "${__reply:-$__default}"
}

echo "=== LTMU tracker — uav-dataset branch ==="
ask CLIENT_IP     "Ground station IP"           "192.168.0.20"
ask LEFT_VPORT    "Left video port"             "5000"
ask LEFT_CPORT    "Left ctrl/telem port"        "5001"
ask RIGHT_VPORT   "Right video port"            "5002"
ask RIGHT_CPORT   "Right ctrl/telem port"       "5003"
ask RES_W         "Tracker width"               "1280"
ask RES_H         "Tracker height"              "720"
ask FPS           "Frame rate"                  "30"
ask LEFT_SEQ      "Left sequence dir"           "/data/UAV123/data_seq/UAV123/bike1"
ask RIGHT_SEQ     "Right sequence dir"          "/data/UAV123/data_seq/UAV123/person1"
ask ONNX_PATH     "ONNX embedder model"         "models/resnet18_embedder.onnx"
ask UART_DEV      "UART device (blank=off)"     ""
ask REC_PATH      "Recording base path (blank=off)" ""
ask TARGET_DEPTH  "Target plane depth, mm (0=handoff off)" "0"

echo
echo "Motion model — pick the one matching your mission profile:"
echo "  cv    constant velocity   — straight-line, steady-speed targets (default, cheapest)"
echo "  ca    constant accel.     — free-fall / dropped / ballistic targets (gravity is const. accel)"
echo "  ctrv  const. turn-rate    — loitering, orbiting, banking targets"
echo "  imm   blend of all three  — unsure which regime you'll see; adapts automatically, ~3x the compute"
ask MOTION_MODEL  "Motion model (cv/ca/ctrv/imm)" "cv"

case "$MOTION_MODEL" in
  cv|ca|ctrv|imm) ;;
  *) echo "warning: unrecognised motion model '$MOTION_MODEL', falling back to cv"; MOTION_MODEL="cv" ;;
esac

echo
echo "Launching:"
echo "  $BIN $CLIENT_IP $LEFT_VPORT $LEFT_CPORT $RIGHT_VPORT $RIGHT_CPORT \\"
echo "       $RES_W $RES_H $FPS \\"
echo "       $LEFT_SEQ $RIGHT_SEQ $ONNX_PATH \\"
echo "       \"$UART_DEV\" \"$REC_PATH\" $TARGET_DEPTH $MOTION_MODEL"
echo

exec "$BIN" \
  "$CLIENT_IP" "$LEFT_VPORT" "$LEFT_CPORT" "$RIGHT_VPORT" "$RIGHT_CPORT" \
  "$RES_W" "$RES_H" "$FPS" \
  "$LEFT_SEQ" "$RIGHT_SEQ" "$ONNX_PATH" \
  "$UART_DEV" "$REC_PATH" "$TARGET_DEPTH" "$MOTION_MODEL"
