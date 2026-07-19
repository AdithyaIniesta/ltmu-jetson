// ============================================================
// motion.h — selectable motion model for the redetector's position
// prior: CV, CA, CTRV, or IMM (probability-weighted blend of the three).
//
// Empirically validated in ltmu-tracker/scripts/compare_motion_models.py
// and compare_motion_models_freefall.py before being ported here:
//
//   scenario          CV RMSE(gap)   CA RMSE(gap)   CTRV RMSE(gap)
//   circular orbit        130.0 px       132.7 px        1.4 px
//   free fall                8.0 px        0.0 px       11.1 px
//
// No single fixed model wins both. CA is exact for free fall (gravity IS
// constant acceleration); CTRV is near-exact for turning/loitering
// targets; each is markedly worse than CV on the other's scenario. IMM
// runs all three in parallel and blends by how well each currently
// predicts real measurements, so it adapts instead of committing to one
// assumption at boot.
//
// Model choice is set ONCE at construction from run_jp5.sh / argv — not
// exposed as a live GUI parameter (see docs/protocol.md: ground station
// GUI is intentionally left untouched by this feature).
// ============================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <opencv2/core.hpp>

enum class MotionModel { CV, CA, CTRV, IMM };

inline MotionModel motionModelFromString(const std::string &s) {
  if (s == "ca") return MotionModel::CA;
  if (s == "ctrv") return MotionModel::CTRV;
  if (s == "imm") return MotionModel::IMM;
  return MotionModel::CV;
}

inline const char *motionModelName(MotionModel m) {
  switch (m) {
    case MotionModel::CA: return "CA";
    case MotionModel::CTRV: return "CTRV";
    case MotionModel::IMM: return "IMM";
    default: return "CV";
  }
}

namespace motion_detail {

// ---- CV: 4-state [x, y, vx, vy] ---------------------------------------
struct CVModel {
  static constexpr int kDim = 4;
  cv::Mat x = cv::Mat::zeros(4, 1, CV_32F);
  cv::Mat P = cv::Mat::eye(4, 4, CV_32F) * 100.0f;
  cv::Mat Q, R;

  explicit CVModel(float processNoise, float measNoise)
      : Q(cv::Mat::eye(4, 4, CV_32F) * processNoise), R(cv::Mat::eye(2, 2, CV_32F) * measNoise) {}

  void init(float cx, float cy) {
    x = (cv::Mat_<float>(4, 1) << cx, cy, 0.f, 0.f);
    P = cv::Mat::eye(4, 4, CV_32F) * 25.0f;
  }

  cv::Mat F(float dt) const {
    cv::Mat f = cv::Mat::eye(4, 4, CV_32F);
    f.at<float>(0, 2) = dt;
    f.at<float>(1, 3) = dt;
    return f;
  }

  void predict(float dt, float noiseScale) {
    cv::Mat f = F(dt);
    x = f * x;
    P = f * P * f.t() + Q * noiseScale;
  }

  // Returns the innovation likelihood (for IMM mixing) and applies the update.
  float update(float cx, float cy) {
    cv::Mat H = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    cv::Mat z = (cv::Mat_<float>(2, 1) << cx, cy);
    cv::Mat y = z - H * x;
    cv::Mat S = H * P * H.t() + R;
    cv::Mat K = P * H.t() * S.inv();
    x = x + K * y;
    P = (cv::Mat::eye(4, 4, CV_32F) - K * H) * P;
    return gaussianLikelihood(y, S);
  }

  cv::Point2f pos() const { return {x.at<float>(0), x.at<float>(1)}; }
  cv::Point2f vel() const { return {x.at<float>(2), x.at<float>(3)}; }

  static float gaussianLikelihood(const cv::Mat &y, const cv::Mat &S) {
    cv::Mat Sinv = S.inv();
    cv::Mat maha = y.t() * Sinv * y;
    double detS = cv::determinant(S);
    if (detS <= 0) return 1e-6f;
    double exponent = -0.5 * maha.at<float>(0, 0);
    double norm = 1.0 / (2.0 * CV_PI * std::sqrt(detS));
    return static_cast<float>(std::max(1e-6, norm * std::exp(exponent)));
  }
};

// ---- CA: 6-state [x, y, vx, vy, ax, ay] --------------------------------
struct CAModel {
  static constexpr int kDim = 6;
  cv::Mat x = cv::Mat::zeros(6, 1, CV_32F);
  cv::Mat P = cv::Mat::eye(6, 6, CV_32F) * 100.0f;
  cv::Mat Q, R;

  explicit CAModel(float processNoise, float measNoise)
      : Q(cv::Mat::eye(6, 6, CV_32F) * processNoise), R(cv::Mat::eye(2, 2, CV_32F) * measNoise) {}

  void init(float cx, float cy) {
    x = (cv::Mat_<float>(6, 1) << cx, cy, 0.f, 0.f, 0.f, 0.f);
    P = cv::Mat::eye(6, 6, CV_32F) * 25.0f;
  }

  cv::Mat F(float dt) const {
    cv::Mat f = cv::Mat::eye(6, 6, CV_32F);
    f.at<float>(0, 2) = dt; f.at<float>(0, 4) = 0.5f * dt * dt;
    f.at<float>(1, 3) = dt; f.at<float>(1, 5) = 0.5f * dt * dt;
    f.at<float>(2, 4) = dt;
    f.at<float>(3, 5) = dt;
    return f;
  }

  void predict(float dt, float noiseScale) {
    cv::Mat f = F(dt);
    x = f * x;
    P = f * P * f.t() + Q * noiseScale;
  }

  float update(float cx, float cy) {
    cv::Mat H = cv::Mat::zeros(2, 6, CV_32F);
    H.at<float>(0, 0) = 1; H.at<float>(1, 1) = 1;
    cv::Mat z = (cv::Mat_<float>(2, 1) << cx, cy);
    cv::Mat y = z - H * x;
    cv::Mat S = H * P * H.t() + R;
    cv::Mat K = P * H.t() * S.inv();
    x = x + K * y;
    P = (cv::Mat::eye(6, 6, CV_32F) - K * H) * P;
    return CVModel::gaussianLikelihood(y, S);
  }

  cv::Point2f pos() const { return {x.at<float>(0), x.at<float>(1)}; }
  cv::Point2f vel() const { return {x.at<float>(2), x.at<float>(3)}; }
};

// ---- CTRV: 5-state [x, y, v, heading, yaw_rate], nonlinear -> EKF ------
struct CTRVModel {
  static constexpr int kDim = 5;
  cv::Mat x = cv::Mat::zeros(5, 1, CV_32F);
  cv::Mat P = cv::Mat::eye(5, 5, CV_32F) * 100.0f;
  float qScale;
  cv::Mat R;
  cv::Point2f prevMeas{0, 0};
  bool havePrev = false;

  explicit CTRVModel(float processNoise, float measNoise)
      : qScale(processNoise * 0.005f), R(cv::Mat::eye(2, 2, CV_32F) * measNoise) {}

  void init(float cx, float cy) {
    x = (cv::Mat_<float>(5, 1) << cx, cy, 0.f, 0.f, 0.f);
    P = cv::Mat::eye(5, 5, CV_32F) * 25.0f;
    prevMeas = {cx, cy};
    havePrev = true;
  }

  static cv::Mat f(const cv::Mat &xin, float dt) {
    float px = xin.at<float>(0), py = xin.at<float>(1), v = xin.at<float>(2);
    float psi = xin.at<float>(3), psid = xin.at<float>(4);
    float nx, ny;
    if (std::abs(psid) < 1e-4f) {
      nx = px + v * std::cos(psi) * dt;
      ny = py + v * std::sin(psi) * dt;
    } else {
      nx = px + (v / psid) * (std::sin(psi + psid * dt) - std::sin(psi));
      ny = py + (v / psid) * (-std::cos(psi + psid * dt) + std::cos(psi));
    }
    float npsi = psi + psid * dt;
    return (cv::Mat_<float>(5, 1) << nx, ny, v, npsi, psid);
  }

  static cv::Mat jacobian(const cv::Mat &xin, float dt) {
    cv::Mat F = cv::Mat::eye(5, 5, CV_32F);
    float v = xin.at<float>(2), psi = xin.at<float>(3), psid = xin.at<float>(4);
    if (std::abs(psid) < 1e-4f) {
      F.at<float>(0, 2) = std::cos(psi) * dt;
      F.at<float>(0, 3) = -v * std::sin(psi) * dt;
      F.at<float>(1, 2) = std::sin(psi) * dt;
      F.at<float>(1, 3) = v * std::cos(psi) * dt;
    } else {
      float s0 = std::sin(psi), c0 = std::cos(psi);
      float s1 = std::sin(psi + psid * dt), c1 = std::cos(psi + psid * dt);
      F.at<float>(0, 2) = (s1 - s0) / psid;
      F.at<float>(0, 3) = (v / psid) * (c1 - c0);
      F.at<float>(0, 4) = (v * dt / psid) * c1 - (v / (psid * psid)) * (s1 - s0);
      F.at<float>(1, 2) = (-c1 + c0) / psid;
      F.at<float>(1, 3) = (v / psid) * (s1 - s0);
      F.at<float>(1, 4) = (v * dt / psid) * s1 - (v / (psid * psid)) * (-c1 + c0);
    }
    F.at<float>(3, 4) = dt;
    return F;
  }

  void predict(float dt, float noiseScale) {
    cv::Mat F = jacobian(x, dt);
    x = f(x, dt);
    cv::Mat Q = cv::Mat::eye(5, 5, CV_32F) * qScale * noiseScale;
    P = F * P * F.t() + Q;
  }

  float update(float cx, float cy) {
    // Bootstrap / smooth speed+heading from consecutive raw measurements —
    // CTRV's v/heading aren't directly observed, only inferred.
    if (havePrev) {
      float dx = cx - prevMeas.x, dy = cy - prevMeas.y;
      float v = std::hypot(dx, dy);
      float psi = std::atan2(dy, dx);
      if (x.at<float>(2) == 0.0f) {
        x.at<float>(2) = v;
        x.at<float>(3) = psi;
      } else {
        float dpsi = psi - x.at<float>(3);
        while (dpsi > CV_PI) dpsi -= 2 * CV_PI;
        while (dpsi < -CV_PI) dpsi += 2 * CV_PI;
        x.at<float>(2) = 0.5f * x.at<float>(2) + 0.5f * v;
        x.at<float>(4) = 0.5f * x.at<float>(4) + 0.5f * dpsi;
        x.at<float>(3) = psi;
      }
    }
    prevMeas = {cx, cy};
    havePrev = true;

    cv::Mat H = cv::Mat::zeros(2, 5, CV_32F);
    H.at<float>(0, 0) = 1; H.at<float>(1, 1) = 1;
    cv::Mat z = (cv::Mat_<float>(2, 1) << cx, cy);
    cv::Mat y = z - H * x;
    cv::Mat S = H * P * H.t() + R;
    cv::Mat K = P * H.t() * S.inv();
    x = x + K * y;
    P = (cv::Mat::eye(5, 5, CV_32F) - K * H) * P;
    return CVModel::gaussianLikelihood(y, S);
  }

  cv::Point2f pos() const { return {x.at<float>(0), x.at<float>(1)}; }
};

}  // namespace motion_detail

// ============================================================
// MotionFilter — same public API regardless of model. IMM runs all
// three sub-filters every frame and blends pos()/vel() by mode
// probability; single-model modes just proxy to one filter.
//
// IMM mixing is a lightweight variant: sub-filters do NOT get their
// full-covariance states re-mixed every step (correct IMM does this,
// including cross-model covariance folding — worth doing if this proves
// out and you want the textbook version). Here each sub-filter runs
// independently and only the *mode probabilities* + *blended output
// position* are combined via measurement likelihood. Simpler, cheap on
// Orin, and already captures the main benefit: whichever model is
// currently predicting real measurements best dominates the blend.
// ============================================================
class MotionFilter {
public:
  explicit MotionFilter(MotionModel model = MotionModel::CV, float processNoise = 4.0f,
                        float measNoise = 2.0f)
      : model_(model), cv_(processNoise, measNoise), ca_(processNoise, measNoise),
        ctrv_(processNoise, measNoise) {}

  void init(float cx, float cy) {
    cv_.init(cx, cy);
    ca_.init(cx, cy);
    ctrv_.init(cx, cy);
    muCV_ = muCA_ = muCTRV_ = 1.0f / 3.0f;
    initialized_ = true;
    lostFrames_ = 0;
  }

  cv::Point2f predict(float dt = 1.0f) {
    if (!initialized_) return {-1, -1};
    float noiseScale = 1.0f + 0.5f * lostFrames_;
    switch (model_) {
      case MotionModel::CV: cv_.predict(dt, noiseScale); lastPos_ = cv_.pos(); break;
      case MotionModel::CA: ca_.predict(dt, noiseScale); lastPos_ = ca_.pos(); break;
      case MotionModel::CTRV: ctrv_.predict(dt, noiseScale); lastPos_ = ctrv_.pos(); break;
      case MotionModel::IMM:
        cv_.predict(dt, noiseScale);
        ca_.predict(dt, noiseScale);
        ctrv_.predict(dt, noiseScale);
        lastPos_ = blendedPos();
        break;
    }
    return lastPos_;
  }

  void update(float cx, float cy) {
    if (!initialized_) { init(cx, cy); return; }
    switch (model_) {
      case MotionModel::CV: cv_.update(cx, cy); break;
      case MotionModel::CA: ca_.update(cx, cy); break;
      case MotionModel::CTRV: ctrv_.update(cx, cy); break;
      case MotionModel::IMM: {
        float lCV = cv_.update(cx, cy);
        float lCA = ca_.update(cx, cy);
        float lCTRV = ctrv_.update(cx, cy);
        // Bayesian mode-probability update: mu_i <- mu_i * likelihood_i, renormalized.
        // Markov transition folded in as a mild floor so no mode fully dies
        // (a target can switch from orbiting to falling mid-flight).
        constexpr float kFloor = 0.02f;
        float uCV = muCV_ * lCV, uCA = muCA_ * lCA, uCTRV = muCTRV_ * lCTRV;
        float sum = uCV + uCA + uCTRV;
        if (sum > 1e-9f) {
          muCV_ = std::max(kFloor, uCV / sum);
          muCA_ = std::max(kFloor, uCA / sum);
          muCTRV_ = std::max(kFloor, uCTRV / sum);
          float renorm = muCV_ + muCA_ + muCTRV_;
          muCV_ /= renorm; muCA_ /= renorm; muCTRV_ /= renorm;
        }
        break;
      }
    }
    lostFrames_ = 0;
  }

  void notifyLost() { lostFrames_++; }

  float gateScore(float cx, float cy, float sigma) const {
    if (!initialized_) return 1.0f;
    float dx = cx - lastPos_.x, dy = cy - lastPos_.y;
    float d2 = dx * dx + dy * dy;
    float s = sigma * (1.0f + 0.4f * lostFrames_);
    return std::exp(-d2 / (2.0f * s * s));
  }

  bool initialized() const { return initialized_; }
  MotionModel model() const { return model_; }

  // Diagnostics — IMM mode weights, 0 for single-model modes.
  std::array<float, 3> modeWeights() const { return {muCV_, muCA_, muCTRV_}; }

private:
  cv::Point2f blendedPos() const {
    cv::Point2f p = cv_.pos() * muCV_ + ca_.pos() * muCA_ + ctrv_.pos() * muCTRV_;
    return p;
  }

  MotionModel model_;
  motion_detail::CVModel cv_;
  motion_detail::CAModel ca_;
  motion_detail::CTRVModel ctrv_;
  float muCV_ = 1.0f / 3.0f, muCA_ = 1.0f / 3.0f, muCTRV_ = 1.0f / 3.0f;
  cv::Point2f lastPos_{0, 0};
  bool initialized_ = false;
  int lostFrames_ = 0;
};
