#include "local_viewer.h"

#include <algorithm>
#include <cstdio>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../common/globals.h"
#include "../tracker/pending_init.h"
#include "../tracker/tracker_thread.h"

namespace {
constexpr char kWindow[] = "LTMU — uav-dataset (local)";

// Draw the left tracker's current result onto a copy of the frame, in the
// same style the streaming overlay uses (green = TRACKING, amber = LOST).
// result.bbox is in native ring-frame pixels — the same space selectROI
// returns — so no scaling is needed.
void drawOverlay(cv::Mat &frame) {
  TrackerResultState &result = resultFor(1);
  cv::Rect bbox;
  LtmuState state;
  float vscore;
  bool have;
  {
    std::lock_guard<std::mutex> lk(result.mtx);
    have = result.haveFrame;
    bbox = result.bbox;
    state = result.state;
    vscore = result.verifierScore;
  }
  if (have) {
    cv::Scalar color =
        (state == LtmuState::TRACKING) ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 200, 255);
    cv::rectangle(frame, bbox, color, 2);
    char label[64];
    snprintf(label, sizeof(label), "%s %.2f", state == LtmuState::TRACKING ? "TRACK" : "LOST",
             vscore);
    cv::putText(frame, label, cv::Point(bbox.x, std::max(0, bbox.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
  }

  const bool paused = g_paused.load();
  char hud[96];
  snprintf(hud, sizeof(hud), "%s  |  SPACE play/pause   c: select ROI   r: reset   q: quit",
           paused ? "PAUSED" : "PLAY");
  cv::putText(frame, hud, cv::Point(10, frame.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// Hand the drawn box to the tracker via the same slot the UDP CAPTURE
// path uses, so init behaviour is identical to the ground-station flow.
void captureRoi(const cv::Rect &roi) {
  if (roi.width < 4 || roi.height < 4) return;  // stray click, not a drag
  g_selected_camera.store(1);
  g_target_confirmed.store(0);
  PendingInit &pend = pendingInitFor(1);
  std::lock_guard<std::mutex> lk(pend.mtx);
  pend.bbox = roi;
  pend.pending = true;
  pend.resetRequested = false;
  printf(LOG_YELLOW "[VIEW]" LOG_RESET " CAPTURE @ (%d,%d,%d,%d)\n", roi.x, roi.y, roi.width,
         roi.height);
}

void requestReset() {
  PendingInit &pend = pendingInitFor(1);
  std::lock_guard<std::mutex> lk(pend.mtx);
  pend.resetRequested = true;
  pend.pending = false;
  g_target_confirmed.store(0);
  printf(LOG_YELLOW "[VIEW]" LOG_RESET " RESET\n");
}
}  // namespace

void localViewerLoop(RingBuffer &leftRing) {
  cv::namedWindow(kWindow, cv::WINDOW_AUTOSIZE);
  printf(LOG_GREEN "[VIEW]" LOG_RESET
                   " local display — SPACE play/pause, c select ROI, r reset, q quit\n");

  cv::Mat frame;
  int fid = 0;
  while (g_running.load()) {
    if (leftRing.latest(frame, fid) && !frame.empty()) {
      cv::Mat shown = frame.clone();
      drawOverlay(shown);
      cv::imshow(kWindow, shown);
    }

    int key = cv::waitKey(15) & 0xFF;
    if (key == ' ') {
      bool now = !g_paused.load();
      g_paused.store(now);
      printf(LOG_CYAN "[VIEW]" LOG_RESET " %s\n", now ? "paused" : "resumed");
    } else if (key == 'c' || key == 'C') {
      // selectROI freezes on the last shown frame and runs its own event
      // loop until the operator confirms (ENTER) or cancels (ESC).
      if (!frame.empty()) {
        cv::Rect roi = cv::selectROI(kWindow, frame, /*showCrosshair=*/false,
                                     /*fromCenter=*/false);
        captureRoi(roi);
      }
    } else if (key == 'r' || key == 'R') {
      requestReset();
    } else if (key == 'q' || key == 'Q' || key == 27) {  // 27 = ESC
      g_running.store(false);
      break;
    }
  }

  cv::destroyWindow(kWindow);
}
