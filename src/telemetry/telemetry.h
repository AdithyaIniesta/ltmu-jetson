// ============================================================
// telemetry.h — UDP telemetry sender (per camera) + UART angle sender.
//
// TelemetryPacket layout and field order match the ground station's
// TELEM_FMT exactly (see docs/protocol.md). reserved is stamped with
// ENGINE_TAG|ENGINE_LTMU so the GUI auto-selects the LTMU parameter
// profile without operator action.
// ============================================================
#pragma once

int uartOpen(const char *device, int baudrate);
void uartSendAngle(int fd, int mode, float angleX, float angleY, float detProb,
                   int pixelX, int pixelY, int confirmed);

// Sends one TelemetryPacket to whichever ground-station port owns the
// currently-selected camera (mirrors the original repo's per-packet
// routing so S1/S2 panels never show the wrong stream's numbers).
void sendTelemetry(int frameId, int frameW, int frameH, float angleX, float angleY,
                   int pixOffX, int pixOffY);

void telemetryThread(int fps);
