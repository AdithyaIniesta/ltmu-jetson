#include "tracker_thread.h"

#include <chrono>
#include <cstdio>
#include <thread>

#include "../common/globals.h"
#include "../dual/tracker_state.h"
#include "pending_init.h"

TrackerResultState g_result;

void trackerThread(RingBuffer &leftRing, RingBuffer &rightRing, Embedder &embedder) {
  LtmuTracker ltmu(embedder);
  bool initialized = false;
  int lastSelected = 0;
  cv::Size lastFrameSize;

  printf(LOG_CYAN "[TRACK]" LOG_RESET " thread started (embedder: %s)\n",
         embedder.usingCuda() ? "CUDA" : "CPU");

  while (g_running.load()) {
    int selected = g_selected_camera.load();

    if (selected == 0) {
      // Not locked to a camera yet — idle, low CPU.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    if (selected != lastSelected) {
      // Camera changed (fresh CAPTURE or HANDOFF+re-CAPTURE) — the next
      // CAPTURE command will call ltmu.init() via g_pendingInit below.
      initialized = false;
      lastSelected = selected;
    }

    RingBuffer &ring = (selected == 1) ? leftRing : rightRing;
    cv::Mat frame;
    int frameId;
    if (!ring.latest(frame, frameId)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    lastFrameSize = frame.size();

    // Pending-init handshake with the control thread: a CAPTURE command
    // stashes a bbox here; the tracker thread performs the actual
    // (possibly slow, first-frame) init on its own thread so the UDP
    // control thread never blocks on tracker work.
    {
      std::lock_guard<std::mutex> lk(g_pendingInitMtx);
      if (g_pendingInit) {
        ltmu.applyParams(g_params);
        ltmu.init(frame, g_pendingInitBbox);
        initialized = true;
        g_pendingInit = false;
        g_target_confirmed.store(0);
      }
    }

    if (!initialized) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    ltmu.applyParams(g_params);
    ltmu.setConfirmed(g_target_confirmed.load() != 0);
    LtmuResult res = ltmu.update(frame);

    {
      std::lock_guard<std::mutex> lk(g_result.mtx);
      g_result.bbox = res.bbox;
      g_result.frameW = frame.cols;
      g_result.frameH = frame.rows;
      g_result.state = res.state;
      g_result.trackerScore = res.trackerScore;
      g_result.verifierScore = res.verifierScore;
      g_result.frameId = frameId;
      g_result.haveFrame = true;
    }
    g_tracker_mode.store(res.state == LtmuState::TRACKING ? 1 : 2);
    g_frameId.store(frameId);
  }
  printf("[TRACK] thread exiting\n");
}
