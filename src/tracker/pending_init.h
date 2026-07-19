// ============================================================
// pending_init.h — handshake between control thread and tracker thread.
//
// CAPTURE / SET_RECT_POS arrive on the UDP control thread and must not
// block on tracker work (first-frame init can briefly stall on a cold
// ONNX Runtime CUDA context). The control thread stashes the requested
// bbox here; the tracker thread performs the actual init on its own
// cadence.
// ============================================================
#pragma once

#include <mutex>
#include <opencv2/core.hpp>

extern std::mutex g_pendingInitMtx;
extern bool g_pendingInit;
extern cv::Rect g_pendingInitBbox;
