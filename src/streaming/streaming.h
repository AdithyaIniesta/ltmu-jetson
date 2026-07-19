// ============================================================
// streaming.h — draws the tracker overlay and pushes frames through a
// GStreamer appsrc -> H.264 -> RTP -> UDP pipeline, matching the ground
// station's expected receive pipeline:
//   udpsrc ! rtph264depay ! h264parse ! avdec_h264 ! ...
//
// uav-dataset branch defaults to x264enc (software) so this runs
// identically on the laptop and the Orin without hardware dependencies.
// The econ-cameras branch overrides ENCODER_ELEMENT to nvv4l2h264enc at
// build time for the hardware encoder path.
// ============================================================
#pragma once

#include <opencv2/core.hpp>

bool streamingInit(const std::string &host, int port, int width, int height, int fps);
void streamingPushFrame(const cv::Mat &bgr);
void streamingShutdown();

void outputThread(class RingBuffer &leftRing, class RingBuffer &rightRing,
                  int width, int height, int fps, const std::string &clientIp,
                  int leftVideoPort, int rightVideoPort);
