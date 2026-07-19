// ============================================================
// ltmu.h — LTMU orchestrator: short-term tracker + verifier +
// meta-updater gate + motion-gated redetector.
//
// Port of ltmu-tracker/ltmu/tracker.py, ltmu/meta_updater.py, and the
// redetect-throttle fix (skip N-1 frames between redetection attempts
// while LOST) added after CPU playback stutter was observed on the
// laptop prototype. On Orin's GPU embedder that throttle mostly stops
// mattering (~3ms/forward instead of ~200ms), but it's kept as a bound
// so a slow frame never stalls the tracker thread.
//
// CONFIRM_TARGET (CMD_CONFIRM_TARGET / ground station "CONFIRM TARGET"
// button): freezes the verifier's frame-0 anchor as authoritative and
// keeps the redetector running through LOST — this is the long-mission
// behaviour the manager asked for: once the operator confirms, LTMU
// keeps hunting for THIS target indefinitely rather than accepting a
// plausible-looking substitute.
// ============================================================
#pragma once

#include <algorithm>
#include <deque>
#include <opencv2/core.hpp>

#include "../common/params.h"
#include "embedder_onnx.h"
#include "motion.h"
#include "redetector.h"
#include "shortterm.h"
#include "verifier.h"

enum class LtmuState { TRACKING, LOST };

struct LtmuResult {
  LtmuState state = LtmuState::LOST;
  cv::Rect bbox;
  float trackerScore = 0.0f;
  float verifierScore = 0.0f;
};

class LtmuTracker {
public:
  explicit LtmuTracker(Embedder &embedder)
      : embedder_(embedder), verifier_(embedder), redetector_(embedder, verifier_, motion_) {}

  void applyParams(const ParamStore &p) {
    verifier_.setThresholds(p.get(LtmuParam::VERIFIER_LOST_THRESH),
                            p.get(LtmuParam::VERIFIER_UPDATE_THRESH));
    verifier_.setNegBankMax(static_cast<int>(p.get(LtmuParam::NEG_BANK_MAX)));
    verifier_.setNegMarginWeight(p.get(LtmuParam::NEG_MARGIN_WEIGHT));
    redetector_.setAcceptScore(p.get(LtmuParam::REDETECT_ACCEPT_SCORE));
    redetector_.setMotionSigma(p.get(LtmuParam::MOTION_SIGMA));
    redetector_.setMotionPriorEnabled(p.get(LtmuParam::ENABLE_MOTION_PRIOR) != 0.0f);
    redetectEvery_ = std::max(1, static_cast<int>(p.get(LtmuParam::REDETECT_INTERVAL)));
    lostThresh_ = p.get(LtmuParam::VERIFIER_LOST_THRESH);
    updateThresh_ = p.get(LtmuParam::VERIFIER_UPDATE_THRESH);
    maxFramesLost_ = std::max(1, static_cast<int>(p.get(LtmuParam::MAX_FRAMES_IN_LOST_MODE)));
  }

  void init(const cv::Mat &frame, const cv::Rect &bbox) {
    shortterm_.init(frame, bbox);
    verifier_.init(frame, bbox);
    redetector_.init(bbox);
    motion_.init(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
    verifierHistory_.clear();
    lastGoodBbox_ = bbox;
    initArea_ = static_cast<float>(bbox.width * bbox.height);
    lostTicks_ = 0;
    state_ = LtmuState::TRACKING;
  }

  void setConfirmed(bool confirmed) { verifier_.setConfirmed(confirmed); }

  LtmuResult update(const cv::Mat &frame) {
    if (state_ == LtmuState::TRACKING) return stepTracking(frame);
    return stepLost(frame);
  }

  LtmuState state() const { return state_; }
  cv::Rect lastGoodBbox() const { return lastGoodBbox_; }

private:
  LtmuResult stepTracking(const cv::Mat &frame) {
    motion_.predict();
    cv::Rect bbox;
    bool ok = shortterm_.update(frame, bbox);
    LtmuResult res;
    if (!ok) {
      state_ = LtmuState::LOST;
      motion_.notifyLost();
      res.state = LtmuState::LOST;
      res.bbox = lastGoodBbox_;
      return res;
    }

    float vscore = verifier_.score(frame, bbox);
    float areaRatio = (bbox.width * bbox.height) / std::max(1.0f, initArea_);
    verifierHistory_.push_back(vscore);
    if (verifierHistory_.size() > kHistoryWindow) verifierHistory_.pop_front();

    bool isLost = false;
    bool shouldUpdate = false;
    if (areaRatio < kAreaMin || areaRatio > kAreaMax) {
      isLost = true;
    } else {
      float avg = recentMean(5);
      if (avg < lostThresh_) {
        isLost = true;
      } else {
        shouldUpdate = vscore >= updateThresh_;
      }
    }

    if (isLost) {
      state_ = LtmuState::LOST;
      motion_.notifyLost();
      res.state = LtmuState::LOST;
      res.bbox = lastGoodBbox_;
      res.trackerScore = 1.0f;
      res.verifierScore = vscore;
      return res;
    }

    if (shouldUpdate) {
      verifier_.maybeUpdate(frame, bbox, vscore);
      motion_.update(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
      lastGoodBbox_ = bbox;
    }

    res.state = LtmuState::TRACKING;
    res.bbox = bbox;
    res.trackerScore = 1.0f;
    res.verifierScore = vscore;
    return res;
  }

  LtmuResult stepLost(const cv::Mat &frame) {
    motion_.predict();
    lostTicks_++;
    LtmuResult res;
    res.state = LtmuState::LOST;
    res.bbox = lastGoodBbox_;

    // CONFIRM_TARGET (operator asserted "this IS the correct object")
    // makes redetection persist indefinitely — a long occlusion or an
    // off-frame excursion shouldn't make LTMU give up on a confirmed
    // target. Without confirmation, stop spending cycles searching once
    // MAX_FRAMES_IN_LOST_MODE is exceeded; a fresh CAPTURE/RESET restarts
    // the counter. This is the CONFIRM_TARGET behaviour requested for
    // long-duration missions.
    if (!verifier_.confirmed() && lostTicks_ > maxFramesLost_) {
      motion_.notifyLost();
      return res;
    }

    // Throttle: only run the (expensive) redetector every REDETECT_INTERVAL
    // frames. Bounds worst-case per-frame cost when running on CPU; a
    // no-op on GPU where the embedder forward is cheap either way.
    if (lostTicks_ % redetectEvery_ != 0) {
      motion_.notifyLost();
      return res;
    }

    Redetector::Result r = redetector_.search(frame);
    if (!r.found) {
      motion_.notifyLost();
      res.verifierScore = r.score;
      return res;
    }

    shortterm_.reinit(frame, r.bbox);
    motion_.update(r.bbox.x + r.bbox.width / 2.0f, r.bbox.y + r.bbox.height / 2.0f);
    verifierHistory_.clear();
    lastGoodBbox_ = r.bbox;
    lostTicks_ = 0;
    state_ = LtmuState::TRACKING;

    res.state = LtmuState::TRACKING;
    res.bbox = r.bbox;
    res.trackerScore = 1.0f;
    res.verifierScore = r.score;
    return res;
  }

  float recentMean(size_t k) const {
    if (verifierHistory_.size() < 2) return 1.0f;
    size_t n = std::min(k, verifierHistory_.size());
    float sum = 0.0f;
    for (size_t i = verifierHistory_.size() - n; i < verifierHistory_.size(); i++)
      sum += verifierHistory_[i];
    return sum / static_cast<float>(n);
  }

  static constexpr size_t kHistoryWindow = 20;
  static constexpr float kAreaMin = 0.25f;
  static constexpr float kAreaMax = 4.0f;

  Embedder &embedder_;
  ShortTermTracker shortterm_;
  Verifier verifier_;
  MotionFilter motion_;
  Redetector redetector_;

  LtmuState state_ = LtmuState::LOST;
  cv::Rect lastGoodBbox_;
  float initArea_ = 1.0f;
  std::deque<float> verifierHistory_;
  int lostTicks_ = 0;
  int redetectEvery_ = 3;
  float lostThresh_ = 0.45f;
  float updateThresh_ = 0.75f;
  int maxFramesLost_ = 300;
};
