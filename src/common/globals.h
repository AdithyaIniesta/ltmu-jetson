// ============================================================
// globals.h — process-wide shared state.
//
// Mirrors jetson-tracking-perception's src/common/globals.h shape: a
// small set of atomics that every thread (capture, tracker, control,
// telemetry, streaming, recorder) reads or writes without a mutex on
// the hot path. See that repo for the pattern this is deliberately
// copying — this is a fresh implementation, not shared code.
// ============================================================
#pragma once

#include <arpa/inet.h>

#include <atomic>
#include <mutex>
#include <string>

#include "params.h"
#include "protocol.h"
#include "../handoff/handoff.h"

// ── ANSI log helpers ─────────────────────────────────────────
#define LOG_RESET "\033[0m"
#define LOG_DIM "\033[2m"
#define LOG_BOLD "\033[1m"
#define LOG_RED "\033[0;31m"
#define LOG_GREEN "\033[0;32m"
#define LOG_YELLOW "\033[1;33m"
#define LOG_BLUE "\033[0;34m"
#define LOG_MAGENTA "\033[0;35m"
#define LOG_CYAN "\033[0;36m"
#define LOG_GRAY "\033[0;37m"

// ── Process lifetime ─────────────────────────────────────────
extern std::atomic<bool> g_running;

// ── Camera selection / handoff ──────────────────────────────────
// Dual-lock: BOTH cameras may track simultaneously. g_selected_camera is
// the "primary" — which camera's result drives UART (single physical
// gimbal link) — not an exclusive gate on who may CAPTURE/track.
// 0 = NONE, 1 = LEFT, 2 = RIGHT
extern std::atomic<int> g_selected_camera;
extern std::atomic<bool> g_handoff_requested;
extern std::atomic<int> g_target_confirmed;
extern std::atomic<int> g_tracker_mode;  // 0 FREE / 1 TRACKING / 2 LOST (primary camera)
extern std::atomic<int> g_frameId;

// ── Geometric handoff (see src/handoff/handoff.h) ──────────────
// Plane-induced homography seeded from stereo_calib_{W}x{H}.json (or
// stereo_calib.json) + a constant target-plane depth. CMD_HANDOFF_MANUAL
// uses this to compute the destination-camera pixel automatically
// instead of the operator hunting for the target by eye on the peer
// camera. g_handoff.ready() is false (and CMD_HANDOFF_MANUAL rejected)
// until a calibration loads successfully at boot.
extern handoff::HandoffModel g_handoff;
extern float g_target_depth_mm;
// Primary camera's current tracker rect centre, updated every output
// frame by streaming.cpp. Informational only — CMD_HANDOFF_MANUAL's
// pixel fallback reads each camera's own live TrackerResultState
// directly (see control.cpp), which works for either source camera,
// not just the primary; kept here for parity with the original repo's
// naming and as a cheap "where is the primary target" query point.
extern std::atomic<int32_t> g_last_rect_x;
extern std::atomic<int32_t> g_last_rect_y;

// ── Parameter store (ids 1-23, see params.h) ───────────────────
extern ParamStore g_params;

// ── Networking (set by main.cpp from argv, used by control/telemetry) ─
extern int g_ctrlSockL, g_ctrlSockR;
extern int g_telemSockL, g_telemSockR;
extern sockaddr_in g_gsAddrL, g_gsAddrR;
extern bool g_gsAddrValidL, g_gsAddrValidR;
extern std::mutex g_gsMtx;
extern std::string g_clientIp;
extern int g_leftVideoPort, g_leftCtrlPort, g_rightVideoPort, g_rightCtrlPort;
extern int g_tracker_W, g_tracker_H, g_fps;
extern std::string g_uartDev;
extern int g_uartFd;

// ── Config persistence ─────────────────────────────────────────
extern std::string CONFIG_FILE;
bool saveParams();
bool loadParams();

static inline const char *modeStr(int m) {
  switch (m) {
    case 0: return "FREE";
    case 1: return "TRACKING";
    case 2: return "LOST";
    default: return "?";
  }
}
