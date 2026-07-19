// ============================================================
// tracker_thread.h — reads the active camera's ring buffer, drives
// LtmuTracker, publishes results for the output/telemetry/streaming
// threads.
//
// Mirrors jetson-tracking-perception's trackerThread role: always
// running, checks g_selected_camera every frame so a HANDOFF mid-flight
// is picked up without a restart.
// ============================================================
#pragma once

#include "../dual/ring_buffer.h"
#include "embedder_onnx.h"
#include "ltmu.h"

// Shared result state, read by telemetry/streaming/control threads.
// Mirrors ResultSlot in the original repo's globals.h.
struct TrackerResultState {
  std::mutex mtx;
  cv::Rect bbox;
  int frameW = 0, frameH = 0;
  LtmuState state = LtmuState::LOST;
  float trackerScore = 0.0f;
  float verifierScore = 0.0f;
  int frameId = 0;
  bool haveFrame = false;
};

extern TrackerResultState g_result;

void trackerThread(RingBuffer &leftRing, RingBuffer &rightRing, Embedder &embedder);
