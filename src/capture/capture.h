// ============================================================
// capture.h — uav-dataset branch: paced image-sequence capture.
//
// Reads a UAV123-style folder of numbered JPGs (e.g.
// data_seq/UAV123/bike1/000001.jpg ...) and pushes frames into a
// RingBuffer at a fixed rate, so the rest of the pipeline (tracker,
// control, telemetry, streaming) behaves exactly as it would against a
// live camera. Two independent sequence paths simulate left/right
// cameras for dual-camera flow validation without real hardware.
//
// The econ-cameras branch replaces this file with real V4L2/GStreamer
// dual capture; every other module is unchanged between branches.
// ============================================================
#pragma once

#include <string>

#include "../dual/ring_buffer.h"

struct DatasetCaptureConfig {
  std::string sequenceDir;   // folder of 000001.jpg, 000002.jpg, ...
  int fps = 30;
  bool loop = true;
};

// Runs until g_running is false. Pushes frames into `ring` at
// cfg.fps, looping the sequence if cfg.loop is set.
void datasetCaptureThread(const DatasetCaptureConfig &cfg, RingBuffer &ring,
                          const char *label);
