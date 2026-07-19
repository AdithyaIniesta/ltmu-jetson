# Wire protocol

Copied byte-for-byte from `jetson-tracking-perception`'s ground-station GUI
and `src/common/globals.h`, so the GUI drives this tracker with zero
changes to its own code. See `src/common/protocol.h` for the canonical C++
structs.

## Engine tag

`TelemetryPacket.reserved` high byte identifies which parameter profile the
GUI should use:

| Value | Engine |
|---|---|
| `0xE0000000` | constant_robotics (v1) |
| `0xE0000001` | cuda_library (v2) |
| `0xE0000002` | lockon |
| `0xE0000003` | **LTMU (this repo)** |

Bit 8 (`0x00000100`) echoes `CMD_CONFIRM_TARGET` state.

## Parameter map (ids 1-23)

Ids 1-15 are the shared block every engine profile already renders in the
GUI (`SHARED_ROWS` in the ground station script) — same names, same GUI
rows, no GUI changes needed:

| id | name | LTMU meaning |
|---|---|---|
| 1 | SEARCH_WINDOW_WIDTH | redetector grid half-extent (px) |
| 2 | SEARCH_WINDOW_HEIGHT | " |
| 3 | RECT_WIDTH | init bbox width (px), set by GUI drag |
| 4 | RECT_HEIGHT | init bbox height (px) |
| 5 | LOST_MODE_OPTION | unused, kept for GUI compat |
| 6 | FRAME_BUFFER_SIZE | ring buffer depth |
| 7 | MAX_FRAMES_IN_LOST_MODE | cap on consecutive LOST frames |
| 8 | RECT_AUTO_SIZE | 0/1 — CSRT's own scale filter |
| 9 | RECT_AUTO_POSITION | unused |
| 10 | MULTIPLE_THREADS | unused |
| 11 | NUM_CHANNELS | unused |
| 12 | TYPE | unused |
| 13 | CUSTOM_1 | = VERIFIER_LOST_THRESH (aliases id 16) |
| 14 | CUSTOM_2 | = VERIFIER_UPDATE_THRESH (aliases id 17) |
| 15 | CUSTOM_3 | = REDETECT_ACCEPT_SCORE (aliases id 18) |

Ids 16-23 are LTMU-specific — add this profile to the ground station's
`ENGINE_PROFILES` dict (see `docs/gui_engine_ltmu_patch.md`):

| id | name | meaning |
|---|---|---|
| 16 | VERIFIER_LOST_THRESH | cosine similarity below which LTMU declares LOST |
| 17 | VERIFIER_UPDATE_THRESH | cosine similarity above which the template bank updates |
| 18 | REDETECT_ACCEPT_SCORE | combined (appearance × motion) score to accept a redetection |
| 19 | REDETECT_INTERVAL | run the redetector every N LOST frames |
| 20 | MOTION_SIGMA | Kalman gate width (px) |
| 21 | NEG_BANK_MAX | hard-negative bank capacity |
| 22 | NEG_MARGIN_WEIGHT | how strongly negatives suppress the score |
| 23 | ENABLE_MOTION_PRIOR | 0/1 toggle |

## Command → LTMU behavior

| CMD | LTMU behavior |
|---|---|
| CAPTURE(x, y, w=-1) | if this control port's camera matches (or none locked), stash `(x - RECT_WIDTH/2, y - RECT_HEIGHT/2, RECT_WIDTH, RECT_HEIGHT)` for the tracker thread to init on the next frame |
| RESET | `deactivate_all_cameras()` — unlock camera, clear target-confirmed |
| CHANGE_SIZE(dw, dh) | bump RECT_WIDTH/HEIGHT, applied on next CAPTURE |
| SET_PARAM(id, val) | live-apply; verifier/redetector thresholds take effect next frame |
| SAVE_PARAMS | write all 23 params to `ltmu_params_<branch>.cfg` |
| GET_PARAMS | ACK-stream all 23 current values (GUI boot sync) |
| HANDOFF | unlock camera selection, tracker cleared |
| SET_CAMERA_PARAM | V4L2 ioctl on the target camera device (econ-cameras branch only; no-op on uav-dataset) |
| CONFIRM_TARGET | freezes the frame-0 anchor embedding as authoritative; redetector keeps running through LOST indefinitely rather than accepting a plausible substitute |

## Telemetry field mapping

`det_prob` and `dnn_similarity` both carry the verifier's cosine score
(LTMU has no separate DNN pass — the ONNX embedder *is* the verifier).
`dnn_veto_threshold` / `dnn_accept_threshold` mirror `VERIFIER_LOST_THRESH`
/ `VERIFIER_UPDATE_THRESH`. `ekf_sigma_*` fields are always zero (LTMU has
no EKF — motion is a separate Kalman filter feeding the redetector only,
not the primary state estimate).
