// ============================================================
// verifier.h — positive/negative embedding banks, discriminative score.
//
// Direct port of ltmu-tracker/ltmu/verifier.py. Score = best positive
// match minus best negative match, which is what fixed the "confusable
// distractor" case (e.g. two riders in frame) observed on UAV123.
// ============================================================
#pragma once

#include <algorithm>
#include <opencv2/core.hpp>
#include <vector>

#include "embedder_onnx.h"

class Verifier {
public:
  static constexpr int kBankMax = 5;
  static constexpr int kNegBankMaxDefault = 8;

  explicit Verifier(Embedder &embedder) : embedder_(embedder) {}

  void init(const cv::Mat &frame, const cv::Rect &bbox) {
    bank_.clear();
    negBank_.clear();
    confirmed_ = false;
    cv::Mat e = embedOne(frame, bbox);
    if (!e.empty()) bank_.push_back(e);
  }

  // score() for a live bbox on the current frame.
  float score(const cv::Mat &frame, const cv::Rect &bbox) {
    cv::Mat e = embedOne(frame, bbox);
    if (e.empty() || bank_.empty()) return 0.0f;
    return std::max(0.0f, scoreEmbedding(e));
  }

  // scoreEmbedding() for a pre-computed embedding — used by the batched
  // redetector so it doesn't re-embed each candidate individually.
  float scoreEmbedding(const cv::Mat &e) const {
    float pos = -1.0f;
    for (const auto &t : bank_) pos = std::max(pos, static_cast<float>(t.dot(e)));
    float score = pos;
    if (!negBank_.empty()) {
      float neg = -1.0f;
      for (const auto &t : negBank_) neg = std::max(neg, static_cast<float>(t.dot(e)));
      score = pos - negMarginWeight_ * std::max(0.0f, neg);
    }
    return score;
  }

  void maybeUpdate(const cv::Mat &frame, const cv::Rect &bbox, float score) {
    if (score < updateThresh_) return;
    cv::Mat e = embedOne(frame, bbox);
    if (e.empty()) return;
    bank_.push_back(e);
    if (static_cast<int>(bank_.size()) > kBankMax) {
      // Always keep the frame-0 anchor (index 0) — drop the oldest of the
      // rest instead. CONFIRM_TARGET (see ltmu.h) doesn't need extra logic
      // here because the anchor is never evicted regardless of bank churn.
      bank_.erase(bank_.begin() + 1);
    }
  }

  void addNegative(const cv::Mat &e) {
    negBank_.push_back(e);
    if (static_cast<int>(negBank_.size()) > negBankMax_) negBank_.erase(negBank_.begin());
  }

  cv::Mat anchorEmbedding() const { return bank_.empty() ? cv::Mat() : bank_[0]; }

  void setThresholds(float lostThresh, float updateThresh) {
    lostThresh_ = lostThresh;
    updateThresh_ = updateThresh;
  }
  void setNegBankMax(int n) { negBankMax_ = n; }
  void setNegMarginWeight(float w) { negMarginWeight_ = w; }
  void setConfirmed(bool c) { confirmed_ = c; }
  bool confirmed() const { return confirmed_; }

private:
  cv::Mat embedOne(const cv::Mat &frame, const cv::Rect &bbox) {
    cv::Rect r = bbox & cv::Rect(0, 0, frame.cols, frame.rows);
    if (r.width < 4 || r.height < 4) return cv::Mat();
    cv::Mat crop = frame(r);
    cv::Mat feats = embedder_.embed({crop});
    return feats.empty() ? cv::Mat() : feats.row(0).clone();
  }

  Embedder &embedder_;
  std::vector<cv::Mat> bank_;
  std::vector<cv::Mat> negBank_;
  float lostThresh_ = 0.45f;
  float updateThresh_ = 0.75f;
  int negBankMax_ = kNegBankMaxDefault;
  float negMarginWeight_ = 0.6f;
  bool confirmed_ = false;
};
