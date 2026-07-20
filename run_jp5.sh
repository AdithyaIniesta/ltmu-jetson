#!/usr/bin/env bash
# ============================================================
# run_jp5.sh — interactive launcher, uav-dataset branch (JetPack 5).
#
# Two workflows, chosen at the first prompt:
#
#   local  on-Jetson OpenCV window — single sequence, SPACE play/pause,
#          draw the ROI with the mouse, watch the overlay. No ground
#          station, no streaming. This is the dataset demo/benchmark path.
#   gui    stream H.264 to the ground-station GUI and take CAPTURE over
#          UDP — same architecture as econ-cameras, dual sequence.
#
# Only the prompts that matter for the chosen mode are shown. Enter
# accepts the default in [brackets].
#
# Motion model (CV/CA/CTRV/IMM) is a BOOT-TIME choice only — see
# src/tracker/motion.h for why: no single fixed model wins on both a
# turning target and a falling one.
# ============================================================
set -euo pipefail

BIN="./build/bin/LtmuTracker"
if [ ! -x "$BIN" ]; then
  echo "error: $BIN not found — build first:"
  echo "  ./build.sh"
  exit 1
fi

ask() {
  # ask VAR "prompt" "default"
  local __var="$1" __prompt="$2" __default="$3" __reply
  read -r -p "$__prompt [$__default]: " __reply || true
  printf -v "$__var" '%s' "${__reply:-$__default}"
}

# Default sequence for the local demo — a single UAV123 sequence.
DEFAULT_SEQ="/home/nvidia/Downloads/Dataset_UAV123/UAV123/data_seq/UAV123/bike1"

echo "=== LTMU tracker — uav-dataset branch ==="
echo
echo "Display mode:"
echo "  local  on-Jetson OpenCV window — SPACE play/pause, draw ROI with the"
echo "         mouse, watch the tracker overlay. No ground-station GUI needed."
echo "  gui    stream H.264 to the ground-station GUI and take CAPTURE over UDP"
echo "         (same as econ-cameras)."
ask DISPLAY_MODE  "Display mode (local/gui)"     "local"
case "$DISPLAY_MODE" in
  local|gui) ;;
  *) echo "warning: unrecognised display mode '$DISPLAY_MODE', using local"; DISPLAY_MODE="local" ;;
esac
echo

# Shared prompts.
ask RES_W         "Tracker width"               "1280"
ask RES_H         "Tracker height"              "720"
ask FPS           "Frame rate"                  "30"
ask ONNX_PATH     "ONNX embedder model"         "models/resnet18_embedder.onnx"

if [ "$DISPLAY_MODE" = "local" ]; then
  # ── Local demo: one sequence, no network, no handoff. ──
  export LTMU_LOCAL_DISPLAY=1
  ask LEFT_SEQ    "Sequence dir"                "$DEFAULT_SEQ"
  # The binary's arg list is positional and shared with gui mode; fill the
  # network/dual/handoff slots with harmless defaults the local viewer
  # ignores (right sequence reuses the single one so capture threads start).
  CLIENT_IP="127.0.0.1"
  LEFT_VPORT=5000; LEFT_CPORT=5001; RIGHT_VPORT=5002; RIGHT_CPORT=5003
  RIGHT_SEQ="$LEFT_SEQ"
  UART_DEV=""; REC_PATH=""; TARGET_DEPTH=0

  # An OpenCV window needs an X display. Over a plain SSH session DISPLAY is
  # unset and the window silently never appears — default it to the Jetson's
  # local desktop (:0) so the window shows on the monitor attached to the
  # Jetson. Override by exporting DISPLAY yourself (e.g. ssh -X).
  if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
    echo "note: DISPLAY was unset — using :0 (the Jetson's own monitor)."
    echo "      For the window over SSH instead, reconnect with 'ssh -X' and re-run."
  fi
  echo "  local display on DISPLAY=$DISPLAY"
else
  # ── GUI: stream to ground station, dual sequence. ──
  export LTMU_LOCAL_DISPLAY=0
  ask CLIENT_IP   "Ground station IP"           "192.168.0.20"
  ask LEFT_VPORT  "Left video port"             "5000"
  ask LEFT_CPORT  "Left ctrl/telem port"        "5001"
  ask RIGHT_VPORT "Right video port"            "5002"
  ask RIGHT_CPORT "Right ctrl/telem port"       "5003"
  ask LEFT_SEQ    "Left sequence dir"           "$DEFAULT_SEQ"
  ask RIGHT_SEQ   "Right sequence dir"          "/home/nvidia/Downloads/Dataset_UAV123/UAV123/data_seq/UAV123/person1"
  ask UART_DEV    "UART device (blank=off)"     ""
  ask REC_PATH    "Recording base path (blank=off)" ""
  ask TARGET_DEPTH "Target plane depth, mm (0=handoff off)" "0"
fi

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
echo "Launching ($DISPLAY_MODE):"
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
