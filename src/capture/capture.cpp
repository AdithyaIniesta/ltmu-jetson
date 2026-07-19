#include "capture.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>

#include "../common/globals.h"

namespace {

// Builds a v4l2src pipeline for one e-Con camera. UYVY/YUYV come off the
// sensor raw and are converted in software; MJPG is hardware-decoded via
// jpegdec (cheaper on CPU, preferred if the camera supports it at the
// target resolution/fps).
std::string buildPipelineDesc(const CameraConfig &cfg) {
  char buf[1024];
  if (cfg.pixelFormat == "MJPG") {
    snprintf(buf, sizeof(buf),
             "v4l2src device=%s ! image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
             "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
             "appsink name=sink emit-signals=false sync=false max-buffers=2 drop=true",
             cfg.videoDevicePath.c_str(), cfg.captureWidth, cfg.captureHeight, cfg.fps);
  } else {
    snprintf(buf, sizeof(buf),
             "v4l2src device=%s ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
             "videoconvert ! video/x-raw,format=BGR ! "
             "appsink name=sink emit-signals=false sync=false max-buffers=2 drop=true",
             cfg.videoDevicePath.c_str(), cfg.pixelFormat.c_str(), cfg.captureWidth,
             cfg.captureHeight, cfg.fps);
  }
  return buf;
}

}  // namespace

void econCaptureThread(const CameraConfig &cfg, RingBuffer &ring, const char *label) {
  static bool gstInited = false;
  if (!gstInited) { gst_init(nullptr, nullptr); gstInited = true; }

  std::string desc = buildPipelineDesc(cfg);
  printf(LOG_CYAN "[CAP-%s]" LOG_RESET " %s\n", label, desc.c_str());

  GError *err = nullptr;
  GstElement *pipeline = gst_parse_launch(desc.c_str(), &err);
  if (!pipeline) {
    fprintf(stderr, "[CAP-%s] pipeline failed: %s\n", label, err ? err->message : "?");
    if (err) g_error_free(err);
    return;
  }
  GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  int frameId = 0;
  while (g_running.load()) {
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 200 * GST_MSECOND);
    if (!sample) continue;

    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);
    int w = 0, h = 0;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      if (w > 0 && h > 0 && map.size >= static_cast<gsize>(w) * h * 3) {
        cv::Mat frame(h, w, CV_8UC3, map.data);
        ring.push(frame, frameId++);  // push() clones, safe once we unmap below
      }
      gst_buffer_unmap(buffer, &map);
    }
    gst_sample_unref(sample);
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  printf("[CAP-%s] thread exiting\n", label);
}
