#include "tracker_state.h"

#include "../common/globals.h"

void activate_left_camera() {
  g_selected_camera.store(1);
  g_target_confirmed.store(0);
}

void activate_right_camera() {
  g_selected_camera.store(2);
  g_target_confirmed.store(0);
}

void deactivate_all_cameras() {
  g_selected_camera.store(0);
  g_handoff_requested.store(true);
  g_target_confirmed.store(0);
}

bool is_camera_active(ActiveCamera cam) {
  return g_selected_camera.load() == static_cast<int>(cam);
}
