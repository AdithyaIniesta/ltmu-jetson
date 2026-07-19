// ============================================================
// recorder.h — optional MP4 sidecar + events.jsonl, disabled by default
// (SD card wear on a UAV). Deliberately simpler than the original
// repo's FlightRecorder (raw per-camera MKV + annotated stream +
// command replay log) — this proves the recording *path* exists
// end-to-end; extend if the manager wants full replay parity.
// ============================================================
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>

class Recorder {
public:
  bool start(const std::string &outDir, int width, int height, int fps);
  void writeFrame(const cv::Mat &bgr);
  void logEvent(const std::string &jsonLine);
  void stop();
  bool active() const { return active_; }

private:
  cv::VideoWriter writer_;
  std::string outDir_;
  bool active_ = false;
  FILE *eventsFile_ = nullptr;
};

void recorderThread(class RingBuffer &leftRing, class RingBuffer &rightRing,
                    const std::string &basePath, bool enabled, int width, int height, int fps);
