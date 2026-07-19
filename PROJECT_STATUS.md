# Project status — read this first

Self-contained handoff note for whoever (human or Claude) picks this up
next, on a different machine. Assumes zero prior context.

## Goal

Build a long-term UAV tracker — occlusion recovery + redetection —
packaged so it can eventually be added to `jetson-tracking-perception`
(a private repo, `git@github.com:AdithyaIniesta/jetson-tracking-perception`)
as an additional tracker engine, once the manager approves. **That repo
is never modified directly** — everything lives in this separate repo,
`ltmu-jetson`, until it's proven and cleared to merge.

## Papers referenced

- **LTMU** — Dai, K. et al., *"High-Performance Long-Term Tracking with
  Meta-Updater"*, CVPR 2020. The core framework this whole project
  implements: short-term tracker + verifier + meta-updater (update-gate)
  + global re-detector. Component-level port, not weight-level — see
  "Stack" below for which components are original vs. substituted.
- **CSR-DCF** — Lukežič, A. et al., *"Discriminative Correlation Filter
  with Channel and Spatial Reliability"*, IJCV 2017. Considered first,
  rejected: doesn't handle occlusion/redetection at all, only spatial
  reliability + channel weighting within a short-term DCF. Not
  implemented anywhere in this project.
- **ODTrack** — Zheng, Y. et al., *"ODTrack: Online Dense Temporal
  Token Learning for Visual Tracking"*, AAAI 2024. Discussed as a
  candidate short-term-tracker upgrade (replacing CSRT for better
  per-frame accuracy at ~10x the compute cost). Not implemented — noted
  as a future option if CSRT-based short-term tracking proves
  insufficient after validation.

## Two repos

### 1. `~/Documents/ltmu-tracker` (Python prototype, **local only, no git repo**)

Built first, on this laptop, to validate the framework logic fast
before committing to a C++ port. Never pushed anywhere — if you're on a
different machine, this code doesn't exist there; treat the findings
below as already proven and go straight to the C++ port.

- `ltmu/` — CSRT (OpenCV) short-term + ResNet-18 (ImageNet, ONNX-able)
  verifier with positive/negative embedding banks + Kalman motion prior
  + batched motion-gated redetector + rule-based meta-updater.
- Validated on UAV123 sequences (`bike1`, `group1/2`, `person1`, `uav1`)
  via `demo/sequence.py` — confirmed occlusion → LOST → redetect →
  reacquire cycle works, including recovering the *original* target
  after a confusable distractor (another person) crossed through frame,
  via the hard-negative bank added in `ltmu/verifier.py`.
- `scripts/compare_motion_models.py` + `compare_motion_models_freefall.py`
  — the empirical basis for the motion-model work ported to C++ (see
  below). Synthetic ground-truth trajectories (circular orbit, free
  fall), three Kalman variants (CV/CA/CTRV) run prediction-only through
  a simulated occlusion gap, RMSE compared:

  | scenario | CV | CA | CTRV |
  |---|---|---|---|
  | circular orbit (gap RMSE) | 130.0 px | 132.7 px | **1.4 px** |
  | free fall (gap RMSE) | 8.0 px | **0.0 px** | 11.1 px |

  Conclusion: no fixed model wins both. CTRV is right for
  loitering/orbiting targets, CA is right for dropped/ballistic targets
  (gravity *is* constant acceleration), CV is worst-of-three almost
  everywhere but cheapest. This is why the C++ port ended up with all
  four (CV/CA/CTRV/IMM) selectable, not just one.
- `scripts/make_circular_sequence.py` — synthetic red-disc-on-white
  circular-motion image sequence generator (with a mid-run occlusion
  gap) used to visually sanity-check the real tracker pipeline, not
  just the isolated motion-model math.

### 2. `~/Documents/ltmu-jetson` (C++ port, **this repo**, pushed to GitHub)

`git@github.com:AdithyaIniesta/ltmu-jetson`, branches `uav-dataset` and
`econ-cameras`. C++17, OpenCV (CSRT), ONNX Runtime (CUDA EP on Orin, CPU
fallback), GStreamer (RTP H.264 out). No Python at runtime.

**Protocol compatibility is the load-bearing design constraint**: this
binary speaks the *exact* UDP command/ACK/telemetry wire format and
UART angle packet that `jetson-tracking-perception`'s ground-station GUI
already expects (see `src/common/protocol.h`, `docs/protocol.md`). The
GUI needs zero code changes to drive this tracker — it self-identifies
via an engine tag in the telemetry `reserved` field (`ENGINE_LTMU = 3`).
**Explicit instruction: do not modify the ground station GUI at all**
for this project — every feature (motion model, geometric handoff) is
wired through argv/boot-time config instead of `SET_PARAM`.

**Branches:**
- `uav-dataset` — capture reads paced UAV123-style image sequences.
  Built first, for validating the whole pipeline (control, telemetry,
  streaming, recording, handoff) against known ground truth before any
  camera hardware is involved.
- `econ-cameras` — capture reads real dual e-Con V4L2 cameras, hardware
  H.264 encode (`nvv4l2h264enc`). Only `src/capture/*` and the encoder
  element differ from `uav-dataset` — every other module is identical
  code, merged from `uav-dataset` at each step (see git log — every
  `uav-dataset` commit has a matching "Merge uav-dataset: ..." commit on
  `econ-cameras`).

**What's built, in commit order (see `git log --oneline --all --graph`):**

1. **Core LTMU engine** — `src/tracker/`: `shortterm.h` (CSRT wrapper),
   `embedder_onnx.h/cpp` (ONNX Runtime ResNet-18, CUDA EP with CPU
   fallback), `verifier.h` (pos/neg embedding banks, discriminative
   score), `redetector.h` (batched multi-scale motion-gated search),
   `motion.h` (see below), `ltmu.h` (orchestrator — TRACKING/LOST state
   machine, meta-updater gate, `CONFIRM_TARGET` long-mission
   persistence).
2. **Full pipeline** matching the original repo's thread topology:
   capture → tracker → control (UDP CAPTURE/RESET/SET_PARAM/etc.) →
   telemetry (UDP + UART) → streaming (RTP H.264) → recorder (MP4 +
   events log). `src/common/protocol.h` has the exact wire structs.
3. **Dual-lock + geometric handoff** — studied a separate branch of the
   original repo (`origin/handoff`, cloned read-only, studied, deleted —
   never merged into the original repo) that had evolved beyond simple
   single-camera-lock into: both cameras tracking simultaneously
   (`g_selected_camera` is just the UART "primary" pointer now, not an
   exclusive gate), plus `CMD_HANDOFF_MANUAL` — computes the
   destination-camera pixel via a stereo-calibration-based plane-induced
   homography (`src/handoff/`) instead of requiring the operator to
   blindly re-click the target on the other camera. This directly
   answers "object free-falls from boresight into depression FOV" — the
   crossing point is computed from geometry. Ported near-verbatim
   (self-contained OpenCV math). Sample calibration files in
   `config/*.sample.json` — **replace with the real airframe
   calibration before relying on `CMD_HANDOFF_MANUAL`.**
4. **Selectable motion model (CV/CA/CTRV/IMM)** — `src/tracker/motion.h`,
   direct C++ port of the Python comparison-script filters, plus a
   lightweight IMM (runs all three per frame, blends the position
   estimate by measurement-likelihood-weighted mode probability). Fixed
   at boot via `run_jp5.sh` → argv, **not** exposed as a live
   `SET_PARAM` — per the no-GUI-changes constraint.
5. **`build.sh`** — studied the original repo's `build.sh` (same
   `origin/feature/dual-camera-v2` branch), kept the useful pattern
   (auto-detect Jetson board from device tree, wipe `build/` before
   every configure, grant `cap_sys_nice` for real-time thread
   scheduling), dropped the multi-engine picker (LTMU has one engine
   per branch, nothing to choose between).

**Status: not yet compiled.** No OpenCV/GStreamer/ONNX Runtime dev
headers on this laptop — everything above is manually reviewed for
correctness (types, include order, member-initialization order, wire
struct layouts double-checked against the GUI's Python parsing code)
but never run through a real compiler. **First build is on Orin NX,
JetPack 5** — flag any build errors back and they get fixed same-day.

## To run (once ONNX Runtime + the exported embedder model exist — see README.md)

```bash
git clone git@github.com:AdithyaIniesta/ltmu-jetson.git
cd ltmu-jetson
git checkout uav-dataset   # or econ-cameras once uav-dataset is proven
./build.sh
./run_jp5.sh
```

`README.md` has full setup (ONNX Runtime install, embedder export).
`docs/protocol.md` has every command/telemetry field and exactly what
LTMU does with each. `docs/gui_engine_ltmu_patch.md` has the one
optional GUI addition (a nicer `ENGINE_LTMU` parameter-label profile) —
optional, the GUI works with zero changes either way.

## Known gaps / honest limitations

- Not compiled yet (see above) — expect some build-fix iteration.
- IMM mixing is a simplified variant (documented in `motion.h`): mode
  probabilities and blended position are combined, but sub-filter
  covariances aren't cross-mixed every step like textbook IMM. Good
  enough to adapt between regimes; worth upgrading if it doesn't hold up
  in the field.
- `CMD_HANDOFF_MANUAL` needs a real stereo calibration for your actual
  airframe — the sample files are from the studied repo's rig, not
  yours.
- Recording is simplified vs. the original repo's flight recorder (MP4
  sidecar + JSON events log, not the full raw+annotated dual-MKV +
  command-replay system).
- ONNX Runtime has no apt package for Jetson — manual download/build
  step, documented in README, untested end-to-end.
