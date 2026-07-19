// ============================================================
// shortterm.h — thin wrapper around OpenCV's CSRT tracker.
//
// Same role as ltmu-tracker/ltmu/shortterm.py: keeps the short-term
// tracker behind a small interface so it can be swapped (e.g. for a
// TensorRT ODTrack) without touching the LTMU orchestrator.
// ============================================================
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>

class ShortTermTracker {
public:
  void init(const cv::Mat &frame, const cv::Rect &bbox) {
    tracker_ = cv::TrackerCSRT::create();
    tracker_->init(frame, bbox);
    lastBbox_ = bbox;
  }

  bool update(const cv::Mat &frame, cv::Rect &bboxOut) {
    cv::Rect r;
    bool ok = tracker_ && tracker_->update(frame, r);
    if (!ok) {
      bboxOut = lastBbox_;
      return false;
    }
    lastBbox_ = r;
    bboxOut = r;
    return true;
  }

  void reinit(const cv::Mat &frame, const cv::Rect &bbox) { init(frame, bbox); }

private:
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect lastBbox_;
};
