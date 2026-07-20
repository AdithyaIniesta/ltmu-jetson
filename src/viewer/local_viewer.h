// ============================================================
// local_viewer.h — standalone on-Jetson display for the uav-dataset
// branch. Restores the Python prototype's interactive workflow that the
// C++ port dropped: open an OpenCV window on the Jetson itself, watch a
// dataset sequence, pause it, draw the target box with the mouse, and
// see the tracker overlay live — no ground-station GUI, no RTP stream.
//
// This does not replace the stream-to-GUI path (that stays the default
// and is what econ-cameras uses). It's an opt-in demo/benchmark mode,
// enabled by LTMU_LOCAL_DISPLAY=1 (set by run_jp5.sh's "Local display?"
// prompt). It reuses the exact same handshake the UDP control thread
// uses — g_pendingInitL / g_resultL — so the tracker itself is unchanged.
//
// Keys (window must be focused):
//   SPACE  play / pause the sequence
//   c      draw ROI (drag a box) → CAPTURE the drawn target
//   r      RESET (drop the current lock)
//   q/ESC  quit
//
// Must run on the main thread: OpenCV highgui window/event handling is
// not thread-safe and expects the process's GUI thread.
// ============================================================
#pragma once

#include "../dual/ring_buffer.h"

// Blocks until the operator quits (q/ESC) or g_running is cleared, then
// returns so main() can join the worker threads. Left camera only — the
// dataset local demo is single-view by design.
void localViewerLoop(RingBuffer &leftRing);
