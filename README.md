# LTMU Jetson tracker

Long-term tracker (short-term CSRT + ONNX ResNet-18 verifier + Kalman
motion prior + batched redetector), packaged as a drop-in alternate
engine for the ground station GUI used by `jetson-tracking-perception` —
same UDP command protocol, same telemetry wire format, same UART angle
packet. This is a **separate repository**; nothing in the original repo
is modified.

C++17 + OpenCV (CSRT) + ONNX Runtime (CUDA EP on Orin, CPU fallback
elsewhere) + GStreamer (RTP H.264 out). No Python at runtime.

## Branches

- **`uav-dataset`** — capture reads paced UAV123-style image sequences
  instead of live cameras. Validates the full pipeline (control,
  telemetry, streaming, recording, handoff) against ground truth before
  any hardware is involved.
- **`econ-cameras`** — capture reads real dual e-Con V4L2 cameras via
  GStreamer, hardware H.264 encode (`nvv4l2h264enc`). Everything else
  (tracker, control, telemetry, streaming, recorder) is unchanged — only
  `src/capture/*` and the encoder element differ between branches.

## Build (Orin NX, JetPack 5)

### 1. ONNX Runtime

No apt package ships the C++ SDK for Jetson. Two options:

**A — prebuilt (fast, do this first):**
```bash
# check https://github.com/microsoft/onnxruntime/releases for the latest
# aarch64 CUDA build compatible with JetPack 5 / CUDA 11.4
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-aarch64-1.17.1.tgz
tar xzf onnxruntime-linux-aarch64-1.17.1.tgz
sudo mv onnxruntime-linux-aarch64-1.17.1 /opt/onnxruntime
```
If that build doesn't have the CUDA execution provider (some releases
are CPU-only for aarch64), fall back to option B, or run CPU-only —
the embedder auto-detects and degrades gracefully (see `usingCuda()`).

**B — build from source** (only if A lacks CUDA EP):
```bash
git clone --recursive https://github.com/microsoft/onnxruntime
cd onnxruntime
./build.sh --config Release --build_shared_lib --parallel \
  --use_cuda --cuda_home /usr/local/cuda --cudnn_home /usr/lib/aarch64-linux-gnu \
  --skip_tests
sudo cp -r build/Linux/Release/install/* /opt/onnxruntime  # adjust path
```

### 2. Export the embedder

Needs PyTorch once, on any machine (not the Orin — do this on the dev
laptop and copy the `.onnx` over, or install torch CPU on the Orin):
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 scripts/export_onnx.py --out models/resnet18_embedder.onnx
```

### 3. Build

```bash
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-app1.0-dev \
                 libgstreamer-plugins-base1.0-dev
cmake -B build -DONNXRUNTIME_ROOT=/opt/onnxruntime -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

`libopencv-dev` must include `opencv_contrib`'s tracking module
(`cv::TrackerCSRT`) — the Jetson JetPack OpenCV build normally does;
if `TrackerCSRT` is missing, build OpenCV with `-DOPENCV_EXTRA_MODULES_PATH`
pointed at opencv_contrib.

## Dual-lock + geometric handoff

Both cameras can track independently and simultaneously — CAPTURE on
either camera starts/updates that camera's own tracker; there is no
exclusive lock. `g_selected_camera` is only the **primary** pointer that
drives UART (the gimbal has one physical setpoint). Both S1/S2 GUI
panels show live, independent tracking state at all times.

`CMD_HANDOFF_MANUAL` adds *geometric* handoff on top of this: given a
stereo calibration and an assumed target-plane depth, the tracker
computes the destination-camera pixel automatically (via a plane-induced
homography) and re-locks there — no blind re-click needed, and the
source camera keeps tracking. Pass a target depth (mm) as argv[14] and
drop a `stereo_calib_{W}x{H}.json` (or `stereo_calib.json`) — see
`config/*.sample.json` for the shape — to enable it; omit both to run
with `CMD_HANDOFF_MANUAL` disabled (plain `CMD_HANDOFF` still works).
Full geometry writeup: `docs/protocol.md`.

## Run — uav-dataset branch

```bash
./build/bin/LtmuTracker \
  192.168.0.20 \
  5000 5001 5002 5003 \
  1280 720 30 \
  /data/UAV123/data_seq/UAV123/bike1 \
  /data/UAV123/data_seq/UAV123/person1 \
  models/resnet18_embedder.onnx \
  ""            `# uart dev, empty = disabled` \
  ""            `# recording base path, empty = disabled` \
  2000          `# target plane depth in mm, 0 disables HANDOFF_MANUAL`
```

Left and right ring buffers are fed from two independent sequence
folders (pass the same folder twice to mirror one camera onto both
streams). Point the ground station GUI at the Orin's IP with the
matching ports — CAPTURE/RESET/HANDOFF/HANDOFF_MANUAL/CONFIRM all work
exactly as they do against the original repo's binary.

## Run — econ-cameras branch

```bash
git checkout econ-cameras
cmake -B build -DONNXRUNTIME_ROOT=/opt/onnxruntime -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/bin/LtmuTracker \
  192.168.0.20 5000 5001 5002 5003 1280 720 30 \
  /dev/video0 /dev/video1 \
  models/resnet18_embedder.onnx \
  /dev/ttyTHS0 \
  /data/recordings/session1 \
  UYVY          `# pixel format: UYVY | YUYV | MJPG` \
  2000          `# target plane depth in mm, 0 disables HANDOFF_MANUAL`
```

## Ground station GUI

The GUI is unmodified except for one addition: an `ENGINE_LTMU = 3`
profile entry so ids 16-23 render with LTMU's own labels instead of the
cuda_library defaults. See `docs/gui_engine_ltmu_patch.md` for the exact
diff — the GUI auto-detects the engine from telemetry and applies it,
no manual selection needed.

## Full protocol reference

`docs/protocol.md` — every command, every telemetry field, exactly what
LTMU does with each.

## Status

Built and code-reviewed on a laptop (no CUDA, no Orin) — not yet
compiled or run. First real build + smoke test happens on Orin NX,
JetPack 5, tomorrow. Report back any build errors and they get fixed
same-day.
