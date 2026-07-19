// ============================================================
// redetector.h — motion-gated batched global search.
//
// Port of ltmu-tracker/ltmu/redetector.py. Multi-scale grid over the
// frame, one batched embedder forward pass, score = appearance * motion
// gate. Runner-up candidates that look like the target but sit outside
// the motion gate get harvested as hard negatives (see verifier.h) —
// this is what fixed the "identity swap to another rider" failure mode
// seen on the UAV123 bike1 sequence.
// ============================================================
#pragma once

#include <algorithm>
#include <opencv2/core.hpp>
#include <vector>

#include "embedder_onnx.h"
#include "motion.h"
#include "verifier.h"

class Redetector {
public:
  Redetector(Embedder &embedder, Verifier &verifier, MotionFilter &motion)
      : embedder_(embedder), verifier_(verifier), motion_(motion) {}

  void init(const cv::Rect &bbox) { initWH_ = {bbox.width, bbox.height}; }

  void setAcceptScore(float s) { acceptScore_ = s; }
  void setMotionSigma(float s) { motionSigma_ = s; }
  void setMotionPriorEnabled(bool e) { motionPriorEnabled_ = e; }

  // Returns {found, bbox, score}.
  struct Result {
    bool found = false;
    cv::Rect bbox;
    float score = 0.0f;
  };

  Result search(const cv::Mat &frame) {
    Result out;
    if (initWH_.width <= 0) return out;
    cv::Mat anchor = verifier_.anchorEmbedding();
    if (anchor.empty()) return out;

    std::vector<cv::Rect> boxes = candidates(frame);
    if (boxes.empty()) return out;
    if (static_cast<int>(boxes.size()) > kMaxCandidates) {
      int step = static_cast<int>(boxes.size()) / kMaxCandidates + 1;
      std::vector<cv::Rect> sub;
      for (size_t i = 0; i < boxes.size(); i += step) sub.push_back(boxes[i]);
      boxes = std::move(sub);
    }

    std::vector<cv::Mat> crops;
    crops.reserve(boxes.size());
    for (auto &b : boxes) crops.push_back(frame(b));
    cv::Mat feats = embedder_.embed(crops);
    if (feats.empty()) return out;

    std::vector<float> appScores(boxes.size()), motionScores(boxes.size()), combined(boxes.size());
    int best = -1;
    float bestCombined = -1.0f;
    for (size_t i = 0; i < boxes.size(); i++) {
      cv::Mat e = feats.row(static_cast<int>(i));
      appScores[i] = std::max(0.0f, verifier_.scoreEmbedding(e));
      if (motionPriorEnabled_) {
        float cx = boxes[i].x + boxes[i].width / 2.0f;
        float cy = boxes[i].y + boxes[i].height / 2.0f;
        motionScores[i] = motion_.gateScore(cx, cy, motionSigma_);
      } else {
        motionScores[i] = 1.0f;
      }
      combined[i] = appScores[i] * motionScores[i];
      if (combined[i] > bestCombined) {
        bestCombined = combined[i];
        best = static_cast<int>(i);
      }
    }

    // Harvest hard negatives: strong appearance match, but far outside
    // the motion gate — almost certainly a distractor, not the target.
    for (size_t i = 0; i < boxes.size(); i++) {
      if (static_cast<int>(i) == best) continue;
      if (appScores[i] > kNegAppearanceMin && motionScores[i] < kNegMotionMax) {
        verifier_.addNegative(feats.row(static_cast<int>(i)).clone());
      }
    }

    if (best < 0 || bestCombined < acceptScore_) return out;
    out.found = true;
    out.bbox = boxes[best];
    out.score = bestCombined;
    return out;
  }

private:
  std::vector<cv::Rect> candidates(const cv::Mat &frame) {
    std::vector<cv::Rect> out;
    const int W = frame.cols, H = frame.rows;
    const float scales[] = {0.75f, 1.0f, 1.33f};
    for (float s : scales) {
      int w = std::max(8, static_cast<int>(initWH_.width * s));
      int h = std::max(8, static_cast<int>(initWH_.height * s));
      if (w >= W || h >= H) continue;
      int stepX = std::max(4, static_cast<int>(w * kStrideRatio));
      int stepY = std::max(4, static_cast<int>(h * kStrideRatio));
      for (int y = 0; y + h < H; y += stepY)
        for (int x = 0; x + w < W; x += stepX) out.emplace_back(x, y, w, h);
    }
    return out;
  }

  static constexpr float kStrideRatio = 0.5f;
  static constexpr int kMaxCandidates = 48;
  static constexpr float kNegAppearanceMin = 0.6f;
  static constexpr float kNegMotionMax = 0.4f;

  Embedder &embedder_;
  Verifier &verifier_;
  MotionFilter &motion_;
  cv::Size initWH_{-1, -1};
  float acceptScore_ = 0.55f;
  float motionSigma_ = 60.0f;
  bool motionPriorEnabled_ = true;
};
