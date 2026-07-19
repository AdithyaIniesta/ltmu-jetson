// ============================================================
// telemetry.h — UDP telemetry sender (per camera, dual-lock: each
// camera's own live state goes to its own GUI port) + UART angle
// sender (single physical link, primary camera only).
//
// TelemetryPacket layout and field order match the ground station's
// TELEM_FMT exactly (see docs/protocol.md). reserved is stamped with
// ENGINE_TAG|ENGINE_LTMU so the GUI auto-selects the LTMU parameter
// profile without operator action.
// ============================================================
#pragma once

#include <opencv2/core.hpp>

#include "../tracker/ltmu.h"  // LtmuState

int uartOpen(const char *device, int baudrate);
void uartSendAngle(int fd, int mode, float angleX, float angleY, float detProb,
                   int pixelX, int pixelY, int confirmed);

// Plain-data snapshot of one camera's tracking result plus derived
// pixel-offset/angle fields, computed once per tick in telemetry.cpp and
// reused for both the UDP packet and (for the primary camera) UART.
struct AngleReportPod {
  cv::Rect bbox;
  LtmuState state;
  float vscore;
  int frameId, fw, fh;
  float pixOffX, pixOffY, angleX, angleY;
};

void sendTelemetry(int cameraId, const AngleReportPod &report);

void telemetryThread(int fps);
