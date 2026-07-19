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

void recorderThread(RingBuffer &leftRing, RingBuffer &rightRing, const std::string &basePath,
                    bool enabled, int width, int height, int fps) {
  if (!enabled) return;
  Recorder rec;
  if (!rec.start(basePath, width, height, fps)) return;

  const auto period = std::chrono::microseconds(1000000 / std::max(1, fps));
  int lastFrameId = -1;

  while (g_running.load()) {
    auto t0 = std::chrono::steady_clock::now();
    int selected = g_selected_camera.load();
    RingBuffer &ring = (selected == 2) ? rightRing : leftRing;
    cv::Mat frame;
    int fid;
    if (ring.latest(frame, fid) && fid != lastFrameId) {
      lastFrameId = fid;
      rec.writeFrame(frame);
    }
    auto elapsed = std::chrono::steady_clock::now() - t0;
    auto sleepFor = period - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
    if (sleepFor.count() > 0) std::this_thread::sleep_for(sleepFor);
  }
  rec.stop();
  printf("[REC] thread exiting\n");
}
