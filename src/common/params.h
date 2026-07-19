// ============================================================
// params.h — LTMU's VTrackerParam-equivalent enum + a thread-safe store.
//
// Ids 1-15 are the SHARED block every engine in the ground station GUI
// already renders (ENGINE_PROFILES SHARED_ROWS) — kept name-compatible so
// existing GUI rows (Search Win W/H, Rect W/H, Auto Size/Pos, ...) work
// against this engine with no GUI changes. Ids 16-23 are new, LTMU-
// specific knobs; see docs/protocol.md and the GUI ENGINE_PROFILES patch.
// ============================================================
#pragma once

#include <array>
#include <atomic>
#include <mutex>

enum class LtmuParam : int {
  SEARCH_WINDOW_WIDTH = 1,   // px — redetector grid search half-extent
  SEARCH_WINDOW_HEIGHT = 2,
  RECT_WIDTH = 3,            // px — init bbox width (set by GUI drag)
  RECT_HEIGHT = 4,
  LOST_MODE_OPTION = 5,      // unused by LTMU state machine, kept for GUI compat
  FRAME_BUFFER_SIZE = 6,     // ring buffer depth
  MAX_FRAMES_IN_LOST_MODE = 7,  // cap on consecutive LOST frames before giving up
  RECT_AUTO_SIZE = 8,        // 0/1 — let CSRT's own scale filter adjust rect size
  RECT_AUTO_POSITION = 9,    // 0/1 — unused, kept for GUI toggle compat
  MULTIPLE_THREADS = 10,     // unused, kept for GUI compat
  NUM_CHANNELS = 11,         // unused, kept for GUI compat
  TYPE = 12,                 // unused, kept for GUI compat
  CUSTOM_1 = 13,             // = VERIFIER_LOST_THRESH
  CUSTOM_2 = 14,             // = VERIFIER_UPDATE_THRESH
  CUSTOM_3 = 15,             // = REDETECT_ACCEPT_SCORE

  VERIFIER_LOST_THRESH = 16,     // duplicate of CUSTOM_1, real storage id
  VERIFIER_UPDATE_THRESH = 17,   // duplicate of CUSTOM_2
  REDETECT_ACCEPT_SCORE = 18,    // duplicate of CUSTOM_3
  REDETECT_INTERVAL = 19,        // run redetector every N lost frames
  MOTION_SIGMA = 20,             // px — Kalman gate width
  NEG_BANK_MAX = 21,             // hard-negative bank capacity
  NEG_MARGIN_WEIGHT = 22,        // how strongly negatives suppress score
  ENABLE_MOTION_PRIOR = 23,      // 0/1 toggle
};

// Simple lock-guarded float array, indexed 1..MAX_TRACKER_PARAM_ID.
// Mirrors cr::vtracker::CvTracker::getParam/setParam so control.cpp's
// dispatch logic ports over almost verbatim.
class ParamStore {
public:
  ParamStore() { setDefaults(); }

  void setDefaults() {
    std::lock_guard<std::mutex> lk(mtx_);
    vals_.fill(0.0f);
    set(LtmuParam::SEARCH_WINDOW_WIDTH, 240.0f);
    set(LtmuParam::SEARCH_WINDOW_HEIGHT, 240.0f);
    set(LtmuParam::RECT_WIDTH, 64.0f);
    set(LtmuParam::RECT_HEIGHT, 64.0f);
    set(LtmuParam::LOST_MODE_OPTION, 1.0f);
    set(LtmuParam::FRAME_BUFFER_SIZE, 4.0f);
    set(LtmuParam::MAX_FRAMES_IN_LOST_MODE, 300.0f);
    set(LtmuParam::RECT_AUTO_SIZE, 1.0f);
    set(LtmuParam::RECT_AUTO_POSITION, 0.0f);
    set(LtmuParam::MULTIPLE_THREADS, 0.0f);
    set(LtmuParam::NUM_CHANNELS, 1.0f);
    set(LtmuParam::TYPE, 0.0f);
    set(LtmuParam::CUSTOM_1, 0.45f);
    set(LtmuParam::CUSTOM_2, 0.75f);
    set(LtmuParam::CUSTOM_3, 0.55f);
    set(LtmuParam::VERIFIER_LOST_THRESH, 0.45f);
    set(LtmuParam::VERIFIER_UPDATE_THRESH, 0.75f);
    set(LtmuParam::REDETECT_ACCEPT_SCORE, 0.55f);
    set(LtmuParam::REDETECT_INTERVAL, 3.0f);
    set(LtmuParam::MOTION_SIGMA, 60.0f);
    set(LtmuParam::NEG_BANK_MAX, 8.0f);
    set(LtmuParam::NEG_MARGIN_WEIGHT, 0.6f);
    set(LtmuParam::ENABLE_MOTION_PRIOR, 1.0f);
  }

  float get(LtmuParam p) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return getLocked(p);
  }
  float get(int id) const { return get(static_cast<LtmuParam>(id)); }

  bool set(LtmuParam p, float v) {
    std::lock_guard<std::mutex> lk(mtx_);
    int idx = static_cast<int>(p);
    if (idx < 1 || idx > 23) return false;
    vals_[idx - 1] = v;
    // CUSTOM_1/2/3 and VERIFIER_*/REDETECT_* alias the same live threshold
    // — GUI rows 13-15 ("loss thr"/"recapture thr"/"learning rate") and
    // rows 16-18 both drive the same underlying value so either UI works.
    if (p == LtmuParam::CUSTOM_1) vals_[static_cast<int>(LtmuParam::VERIFIER_LOST_THRESH) - 1] = v;
    if (p == LtmuParam::CUSTOM_2) vals_[static_cast<int>(LtmuParam::VERIFIER_UPDATE_THRESH) - 1] = v;
    if (p == LtmuParam::CUSTOM_3) vals_[static_cast<int>(LtmuParam::REDETECT_ACCEPT_SCORE) - 1] = v;
    if (p == LtmuParam::VERIFIER_LOST_THRESH) vals_[static_cast<int>(LtmuParam::CUSTOM_1) - 1] = v;
    if (p == LtmuParam::VERIFIER_UPDATE_THRESH) vals_[static_cast<int>(LtmuParam::CUSTOM_2) - 1] = v;
    if (p == LtmuParam::REDETECT_ACCEPT_SCORE) vals_[static_cast<int>(LtmuParam::CUSTOM_3) - 1] = v;
    return true;
  }
  bool set(int id, float v) { return set(static_cast<LtmuParam>(id), v); }

private:
  float getLocked(LtmuParam p) const {
    int idx = static_cast<int>(p);
    if (idx < 1 || idx > 23) return 0.0f;
    return vals_[idx - 1];
  }

  mutable std::mutex mtx_;
  std::array<float, 23> vals_{};
};
