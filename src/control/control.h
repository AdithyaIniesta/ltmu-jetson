// ============================================================
// control.h — UDP command listener (one instance per camera, ports
// leftCtrlPort / rightCtrlPort, matching the ground station GUI's
// Control class exactly: same CmdPacket/AckPacket wire format, same
// CMD_* dispatch).
// ============================================================
#pragma once

// device_path: V4L2 device for SET_CAMERA_PARAM ioctls. Empty string on
// the uav-dataset branch (no real camera to tune) — the command is
// accepted and ACKed false rather than crashing.
void controlThread(int port, int camera_id, const char *device_path);

bool setCameraControl(const char *device, unsigned int ctrl_id, int value);
int getCameraControl(const char *device, unsigned int ctrl_id);
void sendAck(int camera_id, unsigned int cmdType, unsigned int paramId, float value, bool success);
