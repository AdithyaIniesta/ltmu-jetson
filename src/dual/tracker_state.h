// ============================================================
// tracker_state.h — active-camera selection + handoff.
//
// Mirrors jetson-tracking-perception's src/dual/tracker_state.h: a
// single atomic drives which ring buffer the tracker thread reads,
// which stream gets the overlay, and which UART/telemetry port is fed.
// ============================================================
#pragma once

enum class ActiveCamera { NONE = 0, LEFT = 1, RIGHT = 2 };

void activate_left_camera();
void activate_right_camera();
void deactivate_all_cameras();
bool is_camera_active(ActiveCamera cam);
