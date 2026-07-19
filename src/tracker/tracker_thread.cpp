#include "tracker_thread.h"

#include <chrono>
#include <cstdio>
#include <thread>

#include "../common/globals.h"
#include "pending_init.h"

TrackerResultState g_resultL;
TrackerResultState g_resultR;

namespace {

// One camera's tracking step: check for a pending init, run the tracker
// if initialized, publish the result. Identical logic for L and R —
// dual-lock means both run this every tick, independently.
void stepCamera(LtmuTracker &ltmu, RingBuffer &ring, TrackerResultState &result,
                PendingInit &pending, int cameraId, const char *label) {
  cv::Mat frame;
  int frameId;
  if (!ring.latest(frame, frameId)) return;

  {
    std::lock_guard<std::mutex> lk(pending.mtx);
    if (pending.resetRequested) {
      result.initialized = false;
      pending.resetRequested = false;
      std::lock_guard<std::mutex> rlk(result.mtx);
      result.haveFrame = false;
      result.state = LtmuState::LOST;
      printf("[TRACK-%s] reset\n", label);
    }
    if (pending.pending) {
      ltmu.applyParams(g_params);
      ltmu.init(frame, pending.bbox);
      result.initialized = true;
      pending.pending = false;
      // A fresh CAPTURE on either camera clears the shared confirmation
      // flag — same semantics as the single-tracker branch, since
      // CONFIRM_TARGET is a tracker-wide operator assertion in this
      // protocol (see docs/protocol.md), not per-camera.
      g_target_confirmed.store(0);
      printf("[TRACK-%s] init @ (%d,%d,%d,%d)\n", label, pending.bbox.x, pending.bbox.y,
             pending.bbox.width, pending.bbox.height);
    }
  }

  if (!result.initialized) return;

  ltmu.applyParams(g_params);
  // Only the primary camera's confirmation is meaningful for UART, but
  // CONFIRM_TARGET is tracker-wide, so both engines see the same flag —
  // a confirmed target on the primary also makes the secondary persist
  // through LOST rather than give up, which is the useful behaviour
  // during a handoff in progress.
  ltmu.setConfirmed(g_target_confirmed.load() != 0);
  LtmuResult res = ltmu.update(frame);

  {
    std::lock_guard<std::mutex> lk(result.mtx);
    result.bbox = res.bbox;
    result.frameW = frame.cols;
    result.frameH = frame.rows;
    result.state = res.state;
    result.trackerScore = res.trackerScore;
    result.verifierScore = res.verifierScore;
    result.frameId = frameId;
    result.haveFrame = true;
  }

  if (g_selected_camera.load() == cameraId) {
    g_tracker_mode.store(res.state == LtmuState::TRACKING ? 1 : 2);
    g_frameId.store(frameId);
  }
}

}  // namespace

void trackerThread(RingBuffer &leftRing, RingBuffer &rightRing, Embedder &embedder) {
  LtmuTracker ltmuL(embedder);
  LtmuTracker ltmuR(embedder);

  printf(LOG_CYAN "[TRACK]" LOG_RESET " thread started, dual-lock (embedder: %s)\n",
         embedder.usingCuda() ? "CUDA" : "CPU");

  while (g_running.load()) {
    stepCamera(ltmuL, leftRing, g_resultL, g_pendingInitL, 1, "L");
    stepCamera(ltmuR, rightRing, g_resultR, g_pendingInitR, 2, "R");

    if (!g_resultL.initialized && !g_resultR.initialized)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  printf("[TRACK] thread exiting\n");
}
