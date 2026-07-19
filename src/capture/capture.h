// ============================================================
// capture.h — econ-cameras branch: real dual V4L2 GStreamer capture.
//
// Builds a v4l2src ! ... ! appsink pipeline per camera and pushes
// decoded BGR frames into a RingBuffer, mirroring
// jetson-tracking-perception's dual_capture.cpp / pipeline.cpp
// CameraConfig pattern. The uav-dataset branch replaces this file with
// a paced image-sequence reader; every other module is unchanged
// between branches.
// ============================================================
#pragma once

#include <string>

#include "../dual/ring_buffer.h"

struct CameraConfig {
  std::string videoDevicePath;   // e.g. "/dev/video0"
  std::string pixelFormat = "UYVY";  // "UYVY", "YUYV", or "MJPG"
  int captureWidth = 1280;
  int captureHeight = 720;
  int fps = 30;
};

// Runs until g_running is false. Builds and drives the GStreamer
// pipeline for one camera, pushing frames into `ring`.
void econCaptureThread(const CameraConfig &cfg, RingBuffer &ring, const char *label);
