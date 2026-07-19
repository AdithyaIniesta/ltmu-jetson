/**
 * @file handoff.cpp
 * @brief Implementation of cross-view target handoff.
 */

#include "handoff.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace handoff {

// ─────────────────────────────────────────────────────────────────────
// Ultra-minimal JSON reader — enough to parse stereo_calib.json shape
// (K_L, K_R, dist_L, dist_R, R, t_mm, img_size). Avoids dragging a real
// JSON library into this build; the parsers we care about here all
// live on the Jetson side of the wire.
// ─────────────────────────────────────────────────────────────────────
namespace {

// Extract the first number after the given key. Returns true and writes
// values on success; false leaves out unchanged. Handles simple
// "key": [ n, n, n, ... ] arrays that may span multiple lines.
bool extractNumberArray(const std::string& src, const std::string& key,
                        std::vector<double>& out) {
    std::string needle = "\"" + key + "\"";
    size_t p = src.find(needle);
    if (p == std::string::npos) return false;
    p = src.find(':', p);
    if (p == std::string::npos) return false;
    p = src.find('[', p);
    if (p == std::string::npos) return false;
    // Find MATCHING outer ']' — K_L / K_R / R are nested arrays like
    // [[…], […], […]] and stopping at the first ']' would clip us to
    // just 3 numbers instead of 9. Track bracket depth.
    size_t end = std::string::npos;
    int depth = 0;
    for (size_t i = p; i < src.size(); ++i) {
        if (src[i] == '[') ++depth;
        else if (src[i] == ']') {
            if (--depth == 0) { end = i; break; }
        }
    }
    if (end == std::string::npos) return false;
    out.clear();
    std::string body = src.substr(p + 1, end - p - 1);
    // Strip nested brackets — R is a list-of-lists but we treat it flat.
    for (char& c : body) {
        if (c == '[' || c == ']') c = ' ';
    }
    std::stringstream ss(body);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        try {
            double v = std::stod(tok);
            out.push_back(v);
        } catch (...) {
            // ignore malformed entries — better to salvage what we can
        }
    }
    return !out.empty();
}

bool arrayToK(const std::vector<double>& v, std::array<double, 9>& out) {
    if (v.size() != 9) return false;
    for (int i = 0; i < 9; ++i) out[i] = v[i];
    return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// JSON loader
// ─────────────────────────────────────────────────────────────────────
bool loadStereoCalib(const std::string& path, StereoCalib& out) {
    std::ifstream f(path);
    if (!f.is_open()) {
        fprintf(stderr, "[HANDOFF] Cannot open %s\n", path.c_str());
        return false;
    }
    std::stringstream buf;
    buf << f.rdbuf();
    std::string src = buf.str();

    std::vector<double> tmp;

    // K_L
    if (!extractNumberArray(src, "K_L", tmp) || !arrayToK(tmp, out.left.K)) {
        fprintf(stderr, "[HANDOFF] Missing/invalid K_L in %s\n", path.c_str());
        return false;
    }
    // K_R
    if (!extractNumberArray(src, "K_R", tmp) || !arrayToK(tmp, out.right.K)) {
        fprintf(stderr, "[HANDOFF] Missing/invalid K_R in %s\n", path.c_str());
        return false;
    }
    // dist_L
    if (extractNumberArray(src, "dist_L", tmp)) {
        for (int i = 0; i < 5 && i < (int)tmp.size(); ++i) out.left.dist[i] = tmp[i];
    }
    // dist_R
    if (extractNumberArray(src, "dist_R", tmp)) {
        for (int i = 0; i < 5 && i < (int)tmp.size(); ++i) out.right.dist[i] = tmp[i];
    }
    // R (3x3)
    if (!extractNumberArray(src, "R", tmp) || tmp.size() != 9) {
        fprintf(stderr, "[HANDOFF] Missing/invalid R in %s\n", path.c_str());
        return false;
    }
    for (int i = 0; i < 9; ++i) out.ext.R[i] = tmp[i];
    // t_mm
    if (!extractNumberArray(src, "t_mm", tmp) || tmp.size() != 3) {
        fprintf(stderr, "[HANDOFF] Missing/invalid t_mm in %s\n", path.c_str());
        return false;
    }
    for (int i = 0; i < 3; ++i) out.ext.t_mm[i] = tmp[i];
    // img_size (optional)
    if (extractNumberArray(src, "img_size", tmp) && tmp.size() >= 2) {
        out.img_w = (int)tmp[0];
        out.img_h = (int)tmp[1];
    }

    fprintf(stdout,
            "[HANDOFF] Loaded stereo_calib.json  fx_L=%.1f fx_R=%.1f  "
            "|t|=%.1f mm\n",
            out.left.K[0], out.right.K[0],
            std::sqrt(out.ext.t_mm[0]*out.ext.t_mm[0] +
                       out.ext.t_mm[1]*out.ext.t_mm[1] +
                       out.ext.t_mm[2]*out.ext.t_mm[2]));
    return true;
}

// ─────────────────────────────────────────────────────────────────────
// HandoffModel::Impl — hides OpenCV Mat from the header
// ─────────────────────────────────────────────────────────────────────
struct HandoffModel::Impl {
    bool ready = false;
    double depth_mm = 0.0;

    cv::Mat K_L, K_R;               // 3x3 double
    cv::Mat dist_L, dist_R;         // 1x5 double
    cv::Mat R;                      // 3x3 LEFT->RIGHT
    cv::Mat t;                      // 3x1 in mm
    cv::Mat H_L2R;                  // 3x3 homography
    cv::Mat H_R2L;                  // 3x3 homography

    // Recompute both homographies for the current depth. Called from
    // initialise() and setDepth().
    //
    // We model the target as lying on a plane parallel to the LEFT
    // camera's XY plane at distance Z = depth_mm from the LEFT optical
    // centre. Plane normal in LEFT frame is n = [0, 0, 1]^T.
    //
    // Standard plane-induced homography:
    //   H_L2R = K_R (R - (t · n^T) / d) K_L^{-1}
    // Reverse:
    //   H_R2L = H_L2R^{-1}
    //
    // where d is the plane distance from the LEFT optical origin
    // (positive along the LEFT +Z / forward axis).
    void recomputeHomographies() {
        if (depth_mm <= 0.0) return;
        cv::Mat n = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
        // (t · n^T)/d — outer product of 3x1 vectors then divide.
        cv::Mat tn = (t * n.t()) / depth_mm;
        cv::Mat inner = R - tn;                    // 3x3
        H_L2R = K_R * inner * K_L.inv();           // 3x3
        H_R2L = H_L2R.inv();
    }
};

// ─────────────────────────────────────────────────────────────────────
// HandoffModel — public interface
// ─────────────────────────────────────────────────────────────────────
HandoffModel::HandoffModel() : d_(new Impl) {}
HandoffModel::~HandoffModel() { delete d_; }

bool HandoffModel::initialise(const StereoCalib& c, double depth_mm) {
    // Basic sanity — non-zero focal lengths and a reasonable depth.
    if (c.left.K[0] <= 0.0 || c.left.K[4] <= 0.0 ||
        c.right.K[0] <= 0.0 || c.right.K[4] <= 0.0) {
        fprintf(stderr, "[HANDOFF] Zero focal length in calibration\n");
        return false;
    }
    if (depth_mm <= 0.0) {
        fprintf(stderr, "[HANDOFF] Non-positive depth (%.2f)\n", depth_mm);
        return false;
    }

    // The calibration is used verbatim. If the tracker runs at a
    // different resolution than the calibration, the caller must load
    // a calibration captured at the tracker's resolution — main.cpp
    // picks the file by tracker size. No K scaling here.

    d_->K_L = (cv::Mat_<double>(3,3) <<
        c.left.K[0], c.left.K[1], c.left.K[2],
        c.left.K[3], c.left.K[4], c.left.K[5],
        c.left.K[6], c.left.K[7], c.left.K[8]);
    d_->K_R = (cv::Mat_<double>(3,3) <<
        c.right.K[0], c.right.K[1], c.right.K[2],
        c.right.K[3], c.right.K[4], c.right.K[5],
        c.right.K[6], c.right.K[7], c.right.K[8]);
    d_->dist_L = (cv::Mat_<double>(1,5) <<
        c.left.dist[0], c.left.dist[1], c.left.dist[2],
        c.left.dist[3], c.left.dist[4]);
    d_->dist_R = (cv::Mat_<double>(1,5) <<
        c.right.dist[0], c.right.dist[1], c.right.dist[2],
        c.right.dist[3], c.right.dist[4]);
    d_->R = (cv::Mat_<double>(3,3) <<
        c.ext.R[0], c.ext.R[1], c.ext.R[2],
        c.ext.R[3], c.ext.R[4], c.ext.R[5],
        c.ext.R[6], c.ext.R[7], c.ext.R[8]);
    d_->t = (cv::Mat_<double>(3,1) <<
        c.ext.t_mm[0], c.ext.t_mm[1], c.ext.t_mm[2]);
    d_->depth_mm = depth_mm;
    d_->recomputeHomographies();
    d_->ready = true;
    fprintf(stdout, "[HANDOFF] Ready — plane depth = %.1f mm\n", depth_mm);
    return true;
}

bool HandoffModel::ready() const { return d_->ready; }
double HandoffModel::planeDepthMm() const { return d_->depth_mm; }
double HandoffModel::fxL() const { return d_->ready ? d_->K_L.at<double>(0,0) : 0.0; }
double HandoffModel::fyL() const { return d_->ready ? d_->K_L.at<double>(1,1) : 0.0; }
double HandoffModel::fxR() const { return d_->ready ? d_->K_R.at<double>(0,0) : 0.0; }
double HandoffModel::fyR() const { return d_->ready ? d_->K_R.at<double>(1,1) : 0.0; }

namespace {
// Fundamental matrix F such that x_R^T · F · x_L = 0 (homogeneous
// coords). Derived from calibration: F = K_R^{-T} · [t]_x · R · K_L^{-1}.
// This is exact for a rigid stereo pair — no plane-depth assumption
// involved (unlike the plane-induced homography used for point seeds).
cv::Mat fundamentalMatrix(const cv::Mat& K_L, const cv::Mat& K_R,
                          const cv::Mat& R, const cv::Mat& t)
{
    const double tx = t.at<double>(0, 0);
    const double ty = t.at<double>(1, 0);
    const double tz = t.at<double>(2, 0);
    cv::Mat tCross = (cv::Mat_<double>(3, 3) <<
                        0.0, -tz,   ty,
                         tz,  0.0, -tx,
                        -ty,  tx,   0.0);
    return K_R.inv().t() * tCross * R * K_L.inv();
}

// Normalise line coefficients so sqrt(a² + b²) == 1. Makes downstream
// point-line distance a simple |a·u + b·v + c|.
bool normaliseLine(double& a, double& b, double& c)
{
    const double n = std::sqrt(a * a + b * b);
    if (n < 1e-12) return false;
    const double inv = 1.0 / n;
    a *= inv; b *= inv; c *= inv;
    return true;
}
}  // namespace

bool HandoffModel::epipolarLineInR(double u_L, double v_L,
                                   double& a, double& b, double& c) const
{
    if (!d_->ready) return false;
    // F maps a LEFT point to a RIGHT line: (a, b, c)^T = F · (u_L, v_L, 1)^T.
    cv::Mat F = fundamentalMatrix(d_->K_L, d_->K_R, d_->R, d_->t);
    cv::Mat pL = (cv::Mat_<double>(3, 1) << u_L, v_L, 1.0);
    cv::Mat lR = F * pL;
    a = lR.at<double>(0, 0);
    b = lR.at<double>(1, 0);
    c = lR.at<double>(2, 0);
    return normaliseLine(a, b, c);
}

bool HandoffModel::epipolarLineInL(double u_R, double v_R,
                                   double& a, double& b, double& c) const
{
    if (!d_->ready) return false;
    // Reverse: transpose of F takes a RIGHT point to a LEFT line.
    cv::Mat F = fundamentalMatrix(d_->K_L, d_->K_R, d_->R, d_->t);
    cv::Mat pR = (cv::Mat_<double>(3, 1) << u_R, v_R, 1.0);
    cv::Mat lL = F.t() * pR;
    a = lL.at<double>(0, 0);
    b = lL.at<double>(1, 0);
    c = lL.at<double>(2, 0);
    return normaliseLine(a, b, c);
}

void HandoffModel::setDepth(double depth_mm) {
    if (!d_->ready || depth_mm <= 0.0) return;
    d_->depth_mm = depth_mm;
    d_->recomputeHomographies();
}

// Homography apply: [u' v' w'] = H * [u v 1], then divide by w'.
static bool applyH(const cv::Mat& H, double u, double v, double& ou, double& ov) {
    if (H.empty()) return false;
    double h00 = H.at<double>(0,0), h01 = H.at<double>(0,1), h02 = H.at<double>(0,2);
    double h10 = H.at<double>(1,0), h11 = H.at<double>(1,1), h12 = H.at<double>(1,2);
    double h20 = H.at<double>(2,0), h21 = H.at<double>(2,1), h22 = H.at<double>(2,2);
    double x = h00*u + h01*v + h02;
    double y = h10*u + h11*v + h12;
    double w = h20*u + h21*v + h22;
    // Behind-camera guard — sign flip in w means the ray traced through
    // the target plane goes the wrong way.
    if (std::abs(w) < 1e-9 || w < 0.0) return false;
    ou = x / w;
    ov = y / w;
    return true;
}

bool HandoffModel::projectLtoR(double u, double v, double& ou, double& ov) const {
    if (!d_->ready) return false;
    return applyH(d_->H_L2R, u, v, ou, ov);
}

bool HandoffModel::projectRtoL(double u, double v, double& ou, double& ov) const {
    if (!d_->ready) return false;
    return applyH(d_->H_R2L, u, v, ou, ov);
}

}  // namespace handoff
