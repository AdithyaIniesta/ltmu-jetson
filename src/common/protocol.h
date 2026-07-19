// ============================================================
// protocol.h — wire-format structs shared with the ground-station GUI.
//
// Copied field-for-field from jetson-tracking-perception's
// src/common/globals.h so the GUI (CvTracker Ground Station UI) talks to
// this tracker with zero changes to its packet parsing. Do not reorder,
// resize, or repad any field here without updating the GUI's TELEM_FMT /
// CmdPacket struct definitions in lockstep.
// ============================================================
#pragma once

#include <cstdint>

#pragma pack(push, 1)

struct CmdPacket {
  uint32_t magic;
  uint32_t type;
  float arg1;
  float arg2;
  float arg3;
};

struct AckPacket {
  uint32_t magic;
  uint32_t ack_type;
  uint32_t param_id;
  float value;
  uint32_t success;
  uint32_t reserved;
};

struct UartAnglePacket {
  uint8_t header[2];
  uint8_t mode;
  float angle_x_deg;
  float angle_y_deg;
  float det_prob;
  int16_t pixel_x;
  int16_t pixel_y;
  uint8_t confirmed;
  uint8_t checksum;
};

struct TelemetryPacket {
  uint32_t magic;
  uint32_t frame_id;
  uint32_t mode;
  float det_prob;
  uint32_t proc_time_us;

  int32_t rect_x;
  int32_t rect_y;
  int32_t rect_width;
  int32_t rect_height;

  int32_t object_x;
  int32_t object_y;
  int32_t object_width;
  int32_t object_height;

  float angle_x_deg;
  float angle_y_deg;
  int32_t pixel_offset_x;
  int32_t pixel_offset_y;

  int32_t search_win_width;
  int32_t search_win_height;

  float lost_mode_option;

  int32_t frame_buffer_size;
  int32_t max_frames_lost;

  float rect_auto_size;
  float rect_auto_position;

  int32_t multiple_threads;
  int32_t num_channels;
  int32_t tracker_type;

  float custom_1;
  float custom_2;
  float custom_3;

  float ekf_sigma_a;
  float ekf_sigma_alpha;
  float ekf_r_base;

  int32_t dnn_verify_interval;
  float dnn_veto_threshold;
  float dnn_accept_threshold;
  float dnn_similarity;

  int32_t frame_width;
  int32_t frame_height;

  uint32_t reserved;
};

#pragma pack(pop)

static constexpr uint32_t CMD_MAGIC = 0x54524B43;
static constexpr uint32_t ACK_MAGIC = 0x41434B00;
static constexpr uint32_t TELEMETRY_MAGIC = 0x544C4D54;

enum CmdType : uint32_t {
  CMD_CAPTURE = 1,
  CMD_RESET = 2,
  CMD_CHANGE_SIZE = 3,
  CMD_SET_RECT_POS = 4,
  CMD_SET_PARAM = 5,
  CMD_SAVE_PARAMS = 6,
  CMD_HANDOFF = 7,
  CMD_SET_CAMERA_PARAM = 8,
  CMD_GET_PARAMS = 9,
  CMD_CONFIRM_TARGET = 10,
  // arg1 = source camera id (1=L, 2=R), arg2/arg3 = source pixel (u, v);
  // negative u/v means "use that camera's current tracker rect centre".
  // Computes the destination-camera pixel via the plane-induced
  // homography (see src/handoff/handoff.h) and CAPTUREs the destination
  // tracker there — an operator-triggered but geometry-computed handoff,
  // as opposed to CMD_HANDOFF's blind unlock+re-click.
  CMD_HANDOFF_MANUAL = 11,
};

// Confirmation flag echoed in TelemetryPacket::reserved bit 8.
static constexpr uint32_t CONFIRM_TELEM_BIT = 0x00000100u;

// Engine tag stamped into TelemetryPacket::reserved high byte so the GUI's
// engine-aware parameter remap picks the LTMU profile automatically.
// See ground station's ENGINE_TAG / ENGINE_* constants — LTMU claims id 3,
// the next free slot after 0=constant_robotics, 1=cuda_library, 2=lockon.
static constexpr uint32_t ENGINE_TAG = 0xE0000000u;
static constexpr uint32_t ENGINE_LTMU = 3u;

// Highest VTrackerParam-equivalent id this engine accepts over UDP.
// Ids 1-15 are the shared block (rect/search-window/auto/etc, same as
// every other engine so the GUI's SHARED_ROWS keep working unmodified).
// Ids 16-23 are LTMU-specific — see docs/protocol.md and the
// ENGINE_PROFILES[ENGINE_LTMU] patch for the ground station.
static constexpr int MAX_TRACKER_PARAM_ID = 23;
