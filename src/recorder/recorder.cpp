#include "recorder.h"

#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

#include "../common/globals.h"
#include "../dual/ring_buffer.h"
#include "../tracker/tracker_thread.h"

bool Recorder::start(const std::string &outDir, int width, int height, int fps) {
  outDir_ = outDir;
  mkdir(outDir.c_str(), 0755);
  std::string mp4Path = outDir + "/annotated.mp4";
  writer_.open(mp4Path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(width, height));
  if (!writer_.isOpened()) {
    fprintf(stderr, "[REC] failed to open %s for writing\n", mp4Path.c_str());
    return false;
  }
  std::string eventsPath = outDir + "/events.jsonl";
  eventsFile_ = fopen(eventsPath.c_str(), "w");
  active_ = true;
  printf(LOG_CYAN "[REC]" LOG_RESET " recording to %s\n", outDir.c_str());
  return true;
}

void Recorder::writeFrame(const cv::Mat &bgr) {
  if (active_ && writer_.isOpened()) writer_.write(bgr);
}

void Recorder::logEvent(const std::string &jsonLine) {
  if (eventsFile_) { fprintf(eventsFile_, "%s\n", jsonLine.c_str()); fflush(eventsFile_); }
}

void Recorder::stop() {
  if (writer_.isOpened()) writer_.release();
  if (eventsFile_) { fclose(eventsFile_); eventsFile_ = nullptr; }
  active_ = false;
}

// Dual-lock: both cameras record independently from boot (matches the
// original repo's "dual annotated MKVs" behaviour) rather than only
// whichever camera happens to be primary — a handoff mid-flight must
// not create a gap in either camera's recording.
void recorderThread(RingBuffer &leftRing, RingBuffer &rightRing, const std::string &basePath,
                    bool enabled, int width, int height, int fps) {
  if (!enabled) return;
  Recorder recL, recR;
  bool okL = recL.start(basePath + "/left", width, height, fps);
  bool okR = recR.start(basePath + "/right", width, height, fps);
  if (!okL && !okR) return;

  const auto period = std::chrono::microseconds(1000000 / std::max(1, fps));
  int lastFrameIdL = -1, lastFrameIdR = -1;

  while (g_running.load()) {
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat left, right;
    int fidL, fidR;
    if (okL && leftRing.latest(left, fidL) && fidL != lastFrameIdL) {
      lastFrameIdL = fidL;
      recL.writeFrame(left);
    }
    if (okR && rightRing.latest(right, fidR) && fidR != lastFrameIdR) {
      lastFrameIdR = fidR;
      recR.writeFrame(right);
    }
    auto elapsed = std::chrono::steady_clock::now() - t0;
    auto sleepFor = period - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
    if (sleepFor.count() > 0) std::this_thread::sleep_for(sleepFor);
  }
  recL.stop();
  recR.stop();
  printf("[REC] thread exiting\n");
}
