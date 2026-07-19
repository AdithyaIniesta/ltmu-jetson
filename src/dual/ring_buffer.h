// ============================================================
// ring_buffer.h — per-camera frame ring buffer.
//
// Mirrors jetson-tracking-perception's DualRingBuffer: fixed-size slots,
// pre-allocated, so a slow tracker frame never blocks capture and vice
// versa. Shared by both the uav-dataset and econ-cameras capture
// backends — only the producer differs.
// ============================================================
#pragma once

#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>

static constexpr int RING_SIZE = 4;

struct RingSlot {
  cv::Mat bgr;
  int frameId = 0;
  bool ready = false;
};

class RingBuffer {
public:
  void push(const cv::Mat &frame, int frameId) {
    std::lock_guard<std::mutex> lk(mtx_);
    RingSlot &slot = slots_[writeIdx_ % RING_SIZE];
    slot.bgr = frame.clone();
    slot.frameId = frameId;
    slot.ready = true;
    writeIdx_++;
    cv_.notify_all();
  }

  // Returns the newest ready slot without blocking; frameId=-1 if none yet.
  bool latest(cv::Mat &out, int &frameId) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (writeIdx_ == 0) return false;
    const RingSlot &slot = slots_[(writeIdx_ - 1) % RING_SIZE];
    if (!slot.ready) return false;
    out = slot.bgr;
    frameId = slot.frameId;
    return true;
  }

private:
  std::mutex mtx_;
  std::condition_variable cv_;
  RingSlot slots_[RING_SIZE];
  size_t writeIdx_ = 0;
};
