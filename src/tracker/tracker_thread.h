// ============================================================
// tracker_thread.h — dual-lock: drives an independent LtmuTracker per
// camera off that camera's own ring buffer, every tick. Both cameras
// may be TRACKING simultaneously; g_selected_camera (see globals.h) is
// only the UART "primary" pointer, not an exclusive gate.
//
// One thread serially updates L then R each loop rather than two
// threads sharing the Embedder, which keeps the ONNX Runtime session
// usage single-threaded and simple to reason about — Orin's embedder
// forward is cheap enough (~3ms) that serializing both cameras still
// clears real-time budget at the target frame rate.
// ============================================================
#pragma once

#include <mutex>

#include "../dual/ring_buffer.h"
#include "embedder_onnx.h"
#include "ltmu.h"

// Shared result state, read by telemetry/streaming/control threads.
// One instance per camera — dual-lock means both may be TRACKING at once.
struct TrackerResultState {
  std::mutex mtx;
  cv::Rect bbox;
  int frameW = 0, frameH = 0;
  LtmuState state = LtmuState::LOST;
  float trackerScore = 0.0f;
  float verifierScore = 0.0f;
  int frameId = 0;
  bool haveFrame = false;
  bool initialized = false;
};

extern TrackerResultState g_resultL;
extern TrackerResultState g_resultR;

inline TrackerResultState &resultFor(int cameraId) {
  return cameraId == 2 ? g_resultR : g_resultL;
}

void trackerThread(RingBuffer &leftRing, RingBuffer &rightRing, Embedder &embedder,
                   MotionModel motionModel = MotionModel::CV);
