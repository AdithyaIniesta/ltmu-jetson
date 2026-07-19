// ============================================================
// motion.h — constant-velocity Kalman filter on bbox center.
//
// Direct C++ port of the validated Python prototype
// (ltmu-tracker/ltmu/motion.py). Same 4-state model, same gate_score
// formula — ported after the Python version was confirmed on UAV123
// sequences to fix the "target briefly out of frame" redetection case.
// ============================================================
#pragma once

#include <cmath>
#include <opencv2/core.hpp>

class MotionFilter {
public:
  explicit MotionFilter(float processNoise = 4.0f, float measNoise = 2.0f)
      : Q_(cv::Mat::eye(4, 4, CV_32F) * processNoise),
        R_(cv::Mat::eye(2, 2, CV_32F) * measNoise) {}

  void init(float cx, float cy) {
    x_ = (cv::Mat_<float>(4, 1) << cx, cy, 0.f, 0.f);
    P_ = cv::Mat::eye(4, 4, CV_32F) * 25.0f;
    initialized_ = true;
    lostFrames_ = 0;
  }

  cv::Point2f predict(float dt = 1.0f) {
    if (!initialized_) return {-1, -1};
    F_ = cv::Mat::eye(4, 4, CV_32F);
    F_.at<float>(0, 2) = dt;
    F_.at<float>(1, 3) = dt;
    x_ = F_ * x_;
    float scale = 1.0f + 0.5f * lostFrames_;
    P_ = F_ * P_ * F_.t() + Q_ * scale;
    return {x_.at<float>(0), x_.at<float>(1)};
  }

  void update(float cx, float cy) {
    if (!initialized_) {
      init(cx, cy);
      return;
    }
    cv::Mat H = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    cv::Mat z = (cv::Mat_<float>(2, 1) << cx, cy);
    cv::Mat y = z - H * x_;
    cv::Mat S = H * P_ * H.t() + R_;
    cv::Mat K = P_ * H.t() * S.inv();
    x_ = x_ + K * y;
    P_ = (cv::Mat::eye(4, 4, CV_32F) - K * H) * P_;
    lostFrames_ = 0;
  }

  void notifyLost() { lostFrames_++; }

  // Gaussian score in (0,1]; sigma widens the longer we've been lost.
  float gateScore(float cx, float cy, float sigma) const {
    if (!initialized_) return 1.0f;
    float dx = cx - x_.at<float>(0);
    float dy = cy - x_.at<float>(1);
    float d2 = dx * dx + dy * dy;
    float s = sigma * (1.0f + 0.4f * lostFrames_);
    return std::exp(-d2 / (2.0f * s * s));
  }

  bool initialized() const { return initialized_; }

private:
  cv::Mat x_ = cv::Mat::zeros(4, 1, CV_32F);
  cv::Mat P_ = cv::Mat::eye(4, 4, CV_32F) * 100.0f;
  cv::Mat F_ = cv::Mat::eye(4, 4, CV_32F);
  cv::Mat Q_, R_;
  bool initialized_ = false;
  int lostFrames_ = 0;
};
