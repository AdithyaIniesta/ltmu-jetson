#include "capture.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "../common/globals.h"

namespace fs = std::filesystem;

static std::vector<std::string> listFrames(const std::string &dir) {
  std::vector<std::string> files;
  if (!fs::is_directory(dir)) return files;
  for (const auto &entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    auto ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp")
      files.push_back(entry.path().string());
  }
  std::sort(files.begin(), files.end());
  return files;
}

void datasetCaptureThread(const DatasetCaptureConfig &cfg, RingBuffer &ring,
                          const char *label) {
  std::vector<std::string> frames = listFrames(cfg.sequenceDir);
  if (frames.empty()) {
    fprintf(stderr, "[CAP-%s] no frames found under %s\n", label,
            cfg.sequenceDir.c_str());
    return;
  }
  printf(LOG_CYAN "[CAP-%s]" LOG_RESET " %zu frames from %s @ %d fps%s\n", label,
         frames.size(), cfg.sequenceDir.c_str(), cfg.fps, cfg.loop ? " (loop)" : "");

  const auto period = std::chrono::microseconds(1000000 / std::max(1, cfg.fps));
  int frameId = 0;
  size_t idx = 0;

  while (g_running.load()) {
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat img = cv::imread(frames[idx]);
    if (!img.empty()) {
      ring.push(img, frameId++);
    }
    idx++;
    if (idx >= frames.size()) {
      if (!cfg.loop) break;
      idx = 0;
    }
    auto elapsed = std::chrono::steady_clock::now() - t0;
    auto sleepFor = period - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
    if (sleepFor.count() > 0) std::this_thread::sleep_for(sleepFor);
  }
  printf("[CAP-%s] thread exiting\n", label);
}
