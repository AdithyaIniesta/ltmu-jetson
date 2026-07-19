// ============================================================
// pending_init.h — handshake between control threads and the tracker
// thread, one slot per camera (dual-lock: L and R init independently).
//
// CAPTURE / CMD_HANDOFF_MANUAL arrive on a UDP control thread and must
// not block on tracker work (first-frame init can briefly stall on a
// cold ONNX Runtime CUDA context). The control thread stashes the
// requested bbox here; the tracker thread performs the actual init on
// its own cadence, per camera.
// ============================================================
#pragma once

#include <mutex>
#include <opencv2/core.hpp>

struct PendingInit {
  std::mutex mtx;
  bool pending = false;   // a fresh CAPTURE is waiting to be applied
  bool resetRequested = false;  // RESET/HANDOFF: stop tracking this camera
  cv::Rect bbox;
};

extern PendingInit g_pendingInitL;
extern PendingInit g_pendingInitR;

inline PendingInit &pendingInitFor(int cameraId) {
  return cameraId == 2 ? g_pendingInitR : g_pendingInitL;
}
