#include "streaming.h"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "../common/globals.h"
#include "../dual/ring_buffer.h"
#include "../tracker/tracker_thread.h"

#ifndef LTMU_ENCODER_ELEMENT
#define LTMU_ENCODER_ELEMENT "x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000"
#endif

namespace {
GstElement *g_pipeline = nullptr;
GstElement *g_appsrc = nullptr;
guint64 g_frameCount = 0;
int g_w = 0, g_h = 0, g_streamFps = 30;
}  // namespace

bool streamingInit(const std::string &host, int port, int width, int height, int fps) {
  static bool gstInited = false;
  if (!gstInited) { gst_init(nullptr, nullptr); gstInited = true; }

  g_w = width; g_h = height; g_streamFps = fps;

  char pipelineDesc[1024];
  snprintf(pipelineDesc, sizeof(pipelineDesc),
           "appsrc name=src is-live=true block=true format=time "
           "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
           "videoconvert ! video/x-raw,format=I420 ! "
           "%s ! h264parse config-interval=1 ! rtph264pay pt=96 config-interval=1 ! "
           "udpsink host=%s port=%d sync=false async=false",
           width, height, fps, LTMU_ENCODER_ELEMENT, host.c_str(), port);

  GError *err = nullptr;
  g_pipeline = gst_parse_launch(pipelineDesc, &err);
  if (!g_pipeline) {
    fprintf(stderr, "[STREAM] pipeline failed: %s\n", err ? err->message : "?");
    if (err) g_error_free(err);
    return false;
  }
  g_appsrc = gst_bin_get_by_name(GST_BIN(g_pipeline), "src");
  gst_element_set_state(g_pipeline, GST_STATE_PLAYING);
  printf(LOG_CYAN "[STREAM]" LOG_RESET " RTP H.264 -> %s:%d (%dx%d@%d)\n", host.c_str(), port,
         width, height, fps);
  return true;
}

void streamingPushFrame(const cv::Mat &bgr) {
  if (!g_appsrc) return;
  cv::Mat resized;
  const cv::Mat *src = &bgr;
  if (bgr.cols != g_w || bgr.rows != g_h) {
    cv::resize(bgr, resized, cv::Size(g_w, g_h));
    src = &resized;
  }
  gsize size = src->total() * src->elemSize();
  GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_WRITE);
  memcpy(map.data, src->data, size);
  gst_buffer_unmap(buffer, &map);

  GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(g_frameCount, GST_SECOND, g_streamFps);
  GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, g_streamFps);
  g_frameCount++;

  GstFlowReturn ret;
  g_signal_emit_by_name(g_appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);
}

void streamingShutdown() {
  if (g_pipeline) {
    gst_element_set_state(g_pipeline, GST_STATE_NULL);
    gst_object_unref(g_pipeline);
    g_pipeline = nullptr;
    g_appsrc = nullptr;
  }
}

// Dual-lock: draws whichever camera's own live tracker result is
// current — no gate on primary/selected, since both cameras may be
// TRACKING independently and the operator watches both video panels.
static void drawOverlay(cv::Mat &frame, int cameraId) {
  TrackerResultState &result = resultFor(cameraId);
  cv::Rect bbox; LtmuState state; float vscore;
  {
    std::lock_guard<std::mutex> lk(result.mtx);
    if (!result.haveFrame) return;
    bbox = result.bbox; state = result.state; vscore = result.verifierScore;
  }
  cv::Scalar color = (state == LtmuState::TRACKING) ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 200, 255);
  cv::rectangle(frame, bbox, color, 2);
  char label[64];
  snprintf(label, sizeof(label), "%s %.2f", state == LtmuState::TRACKING ? "TRACK" : "LOST", vscore);
  cv::putText(frame, label, cv::Point(bbox.x, std::max(0, bbox.y - 6)), cv::FONT_HERSHEY_SIMPLEX,
             0.5, color, 2);
  if (g_target_confirmed.load() && g_selected_camera.load() == cameraId)
    cv::putText(frame, "CONFIRMED", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6,
               cv::Scalar(0, 220, 0), 2);

  // g_last_rect_x/y is the CMD_HANDOFF_MANUAL source-pixel fallback —
  // only meaningful for the PRIMARY camera (matches the original repo's
  // single pair of atomics tied to whichever camera is primary).
  if (g_selected_camera.load() == cameraId && state == LtmuState::TRACKING) {
    g_last_rect_x.store(bbox.x + bbox.width / 2, std::memory_order_relaxed);
    g_last_rect_y.store(bbox.y + bbox.height / 2, std::memory_order_relaxed);
  }
}

void outputThread(RingBuffer &leftRing, RingBuffer &rightRing, int width, int height, int fps,
                  const std::string &clientIp, int leftVideoPort, int rightVideoPort) {
  streamingInit(clientIp, leftVideoPort, width, height, fps);
  bool streamingRight = leftVideoPort != rightVideoPort;
  // Second stream (passthrough, no tracker overlay) reuses the same
  // appsrc pattern via a second pipeline instance if the ports differ.
  GstElement *pipeR = nullptr, *appsrcR = nullptr;
  if (streamingRight) {
    gst_init(nullptr, nullptr);
    char desc[512];
    snprintf(desc, sizeof(desc),
             "appsrc name=srcR is-live=true block=true format=time "
             "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
             "videoconvert ! video/x-raw,format=I420 ! %s ! "
             "h264parse config-interval=1 ! rtph264pay pt=96 config-interval=1 ! "
             "udpsink host=%s port=%d sync=false async=false",
             width, height, fps, LTMU_ENCODER_ELEMENT, clientIp.c_str(), rightVideoPort);
    GError *err = nullptr;
    pipeR = gst_parse_launch(desc, &err);
    if (pipeR) {
      appsrcR = gst_bin_get_by_name(GST_BIN(pipeR), "srcR");
      gst_element_set_state(pipeR, GST_STATE_PLAYING);
    } else if (err) {
      fprintf(stderr, "[STREAM] right pipeline failed: %s\n", err->message);
      g_error_free(err);
    }
  }

  const auto period = std::chrono::microseconds(1000000 / std::max(1, fps));
  guint64 frameCountR = 0;

  while (g_running.load()) {
    auto t0 = std::chrono::steady_clock::now();

    cv::Mat left, right;
    int fid;
    bool haveLeft = leftRing.latest(left, fid);
    bool haveRight = rightRing.latest(right, fid);

    // Dual-lock: both cameras' overlays are drawn from their own live
    // tracker result every frame, independent of which is primary.
    if (haveLeft) {
      drawOverlay(left, 1);
      streamingPushFrame(left);
    }
    if (streamingRight && haveRight && appsrcR) {
      drawOverlay(right, 2);
      cv::Mat resized;
      const cv::Mat *src = &right;
      if (right.cols != width || right.rows != height) {
        cv::resize(right, resized, cv::Size(width, height));
        src = &resized;
      }
      gsize size = src->total() * src->elemSize();
      GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
      GstMapInfo map;
      gst_buffer_map(buffer, &map, GST_MAP_WRITE);
      memcpy(map.data, src->data, size);
      gst_buffer_unmap(buffer, &map);
      GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(frameCountR, GST_SECOND, fps);
      GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, fps);
      frameCountR++;
      GstFlowReturn ret;
      g_signal_emit_by_name(appsrcR, "push-buffer", buffer, &ret);
      gst_buffer_unref(buffer);
    }

    auto elapsed = std::chrono::steady_clock::now() - t0;
    auto sleepFor = period - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
    if (sleepFor.count() > 0) std::this_thread::sleep_for(sleepFor);
  }

  streamingShutdown();
  if (pipeR) { gst_element_set_state(pipeR, GST_STATE_NULL); gst_object_unref(pipeR); }
  printf("[STREAM] thread exiting\n");
}
