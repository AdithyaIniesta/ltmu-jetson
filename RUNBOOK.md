# Runbook — first run on Orin NX

Exact command sequence, in order. Nothing here has been run end-to-end
yet (see `PROJECT_STATUS.md`) — if a step fails, that's expected on the
first pass; report the error back and it gets fixed same-day.

## 1. Clone

```bash
git clone git@github.com:AdithyaIniesta/ltmu-jetson.git
cd ltmu-jetson
git checkout uav-dataset      # start here — no camera hardware needed
# git checkout econ-cameras   # once uav-dataset is proven working
```

## 2. Install ONNX Runtime (one-time)

No apt package for Jetson's C++ SDK.

```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-aarch64-1.17.1.tgz
tar xzf onnxruntime-linux-aarch64-1.17.1.tgz
sudo mv onnxruntime-linux-aarch64-1.17.1 /opt/onnxruntime
```

If that release has no CUDA execution provider for aarch64, see
README.md "ONNX Runtime" option B (build from source). The tracker
still runs CPU-only either way — slower, not broken.

## 3. Get the ONNX embedder model

The tracker needs `models/resnet18_embedder.onnx`. Two ways to get it —
pick one:

**A — export on the Jetson:**
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 scripts/export_onnx.py --out models/resnet18_embedder.onnx
```

**B — copy a pre-exported file from the dev laptop:**
```bash
# on the laptop:
scp models/resnet18_embedder.onnx <jetson-user>@<jetson-ip>:~/ltmu-jetson/models/
```

## 4. Install build dependencies (one-time)

```bash
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-app1.0-dev \
                 libgstreamer-plugins-base1.0-dev
```

`libopencv-dev` must include `opencv_contrib`'s tracking module
(`cv::TrackerCSRT`). If the build fails on `TrackerCSRT` not found, see
README.md's note on `-DOPENCV_EXTRA_MODULES_PATH`.

## 5. Build

```bash
./build.sh
```

Auto-detects Orin NX vs Xavier NX, does a clean configure+build. Repeat
this step after every `git pull` or source change — do not `cmake
--build` directly without first re-running `./build.sh` unless you know
the cache is still valid.

## 6. Run

```bash
./run_jp5.sh
```

Interactive — Enter accepts the default shown in `[brackets]` for every
prompt. Includes the motion-model choice (cv/ca/ctrv/imm — see
`docs/protocol.md` for what each means and when to pick it).

Point the ground station GUI at the Jetson's IP (same one entered at
the first prompt) and the matching video/ctrl ports. CAPTURE, RESET,
HANDOFF, HANDOFF_MANUAL, CONFIRM TARGET all work exactly as they do
against the original repo's binary — the GUI needs no changes.

## If something fails

- **`./build.sh` errors "ONNXRUNTIME_ROOT does not exist"** → step 2
  didn't finish, or used a different path — pass
  `ONNXRUNTIME_ROOT=/your/path ./build.sh`.
- **`cv::TrackerCSRT` not found** → OpenCV build is missing
  opencv_contrib's tracking module (see step 4 note).
- **`./run_jp5.sh` says model file not found** → step 3 didn't produce
  `models/resnet18_embedder.onnx`.
- **GUI shows no telemetry / video** → check the ports entered in
  `run_jp5.sh` match what the GUI is listening on, and that CAPTURE was
  sent (tracker is idle until the first CAPTURE).
- **Anything else** → copy the terminal output and send it back.
