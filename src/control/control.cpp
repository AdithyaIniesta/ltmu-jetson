#include "control.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <opencv2/core.hpp>

#include "../common/globals.h"
#include "../tracker/pending_init.h"
#include "../tracker/tracker_thread.h"

#ifdef LTMU_ECON_CAMERAS
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#endif

bool saveParams() {
  std::string tmp = CONFIG_FILE + ".tmp";
  std::ofstream f(tmp);
  if (!f.is_open()) return false;
  for (int id = 1; id <= MAX_TRACKER_PARAM_ID; id++) f << id << "=" << g_params.get(id) << "\n";
  f.close();
  if (!f) {
    unlink(tmp.c_str());
    return false;
  }
  return rename(tmp.c_str(), CONFIG_FILE.c_str()) == 0;
}

bool loadParams() {
  std::ifstream f(CONFIG_FILE);
  if (!f.is_open()) {
    printf("[CFG] no config at %s, using defaults\n", CONFIG_FILE.c_str());
    saveParams();
    return false;
  }
  std::string line;
  int loaded = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    try {
      int id = std::stoi(line.substr(0, eq));
      float val = std::stof(line.substr(eq + 1));
      if (id >= 1 && id <= MAX_TRACKER_PARAM_ID) {
        g_params.set(id, val);
        loaded++;
      }
    } catch (...) {
    }
  }
  printf("[CFG] loaded %d params from %s\n", loaded, CONFIG_FILE.c_str());
  return loaded > 0;
}

#ifdef LTMU_ECON_CAMERAS
struct CameraParamEntry { int simple_id; uint32_t v4l2_id; const char *name; };
static const CameraParamEntry kCameraParams[] = {
    {1, V4L2_CID_BRIGHTNESS, "brightness"}, {2, V4L2_CID_CONTRAST, "contrast"},
    {3, V4L2_CID_SATURATION, "saturation"}, {4, V4L2_CID_GAMMA, "gamma"},
    {5, V4L2_CID_GAIN, "gain"},             {6, V4L2_CID_SHARPNESS, "sharpness"},
    {7, V4L2_CID_EXPOSURE_ABSOLUTE, "exposure"},
};
static const int kNumCameraParams = sizeof(kCameraParams) / sizeof(kCameraParams[0]);

bool setCameraControl(const char *device, unsigned int ctrl_id, int value) {
  int fd = open(device, O_RDWR);
  if (fd < 0) return false;
  if (ctrl_id == V4L2_CID_EXPOSURE_ABSOLUTE) {
    struct v4l2_control autoCtrl {};
    autoCtrl.id = V4L2_CID_EXPOSURE_AUTO;
    autoCtrl.value = V4L2_EXPOSURE_MANUAL;
    (void)ioctl(fd, VIDIOC_S_CTRL, &autoCtrl);
  }
  struct v4l2_control ctrl {};
  ctrl.id = ctrl_id;
  ctrl.value = value;
  bool ok = ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0;
  close(fd);
  return ok;
}
int getCameraControl(const char *device, unsigned int ctrl_id) {
  int fd = open(device, O_RDWR);
  if (fd < 0) return -1;
  struct v4l2_control ctrl {};
  ctrl.id = ctrl_id;
  int r = ioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0 ? ctrl.value : -1;
  close(fd);
  return r;
}
#else
bool setCameraControl(const char *, unsigned int, int) { return false; }
int getCameraControl(const char *, unsigned int) { return -1; }
static const struct { int simple_id; unsigned v4l2_id; const char *name; } kCameraParams[] = {
    {1, 0, "brightness"}, {2, 0, "contrast"}, {3, 0, "saturation"}, {4, 0, "gamma"},
    {5, 0, "gain"},       {6, 0, "sharpness"}, {7, 0, "exposure"},
};
static const int kNumCameraParams = 7;
#endif

// Per-camera ground station address, so ACKs and telemetry go back to
// whichever port sent the command (mirrors g_gsAddrL / g_gsAddrR).
static void rememberGsAddr(int camera_id, const sockaddr_in &addr) {
  std::lock_guard<std::mutex> lk(g_gsMtx);
  if (camera_id == 1) { g_gsAddrL = addr; g_gsAddrValidL = true; }
  else { g_gsAddrR = addr; g_gsAddrValidR = true; }
}

void sendAck(int camera_id, unsigned int cmdType, unsigned int paramId, float value, bool success) {
  std::lock_guard<std::mutex> lk(g_gsMtx);
  int sock = (camera_id == 1) ? g_ctrlSockL : g_ctrlSockR;
  bool valid = (camera_id == 1) ? g_gsAddrValidL : g_gsAddrValidR;
  const sockaddr_in &addr = (camera_id == 1) ? g_gsAddrL : g_gsAddrR;
  if (sock < 0 || !valid) return;
  AckPacket ack{};
  ack.magic = ACK_MAGIC;
  ack.ack_type = cmdType;
  ack.param_id = paramId;
  ack.value = value;
  ack.success = success ? 1 : 0;
  sendto(sock, &ack, sizeof(ack), 0, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));
}

void controlThread(int port, int camera_id, const char *device_path) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { perror("[CTRL] socket"); return; }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("[CTRL] bind");
    close(sock);
    return;
  }

  {
    std::lock_guard<std::mutex> lk(g_gsMtx);
    if (camera_id == 1) g_ctrlSockL = sock; else g_ctrlSockR = sock;
  }

  timeval tv{}; tv.tv_usec = 200000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  printf(LOG_CYAN "[CTRL]" LOG_RESET " listening on UDP :%d (camera_id=%d)\n", port, camera_id);

  CmdPacket pkt{};
  sockaddr_in sender{};
  socklen_t senderLen = sizeof(sender);

  while (g_running.load()) {
    senderLen = sizeof(sender);
    ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0, reinterpret_cast<sockaddr *>(&sender), &senderLen);
    if (n != sizeof(pkt) || pkt.magic != CMD_MAGIC) continue;
    rememberGsAddr(camera_id, sender);

    switch (pkt.type) {
      case CMD_CAPTURE: {
        // Dual-lock: CAPTURE always dispatches to the sending camera's
        // own tracker — no exclusive gate. g_selected_camera tracks the
        // "primary" (UART routing) only: first-ever CAPTURE claims it,
        // a secondary-camera CAPTURE switches primary to that camera
        // without disturbing the other camera's independent track.
        int locked = g_selected_camera.load();
        if (locked != 0 && locked != camera_id) {
          printf(LOG_CYAN "[CTRL-%s]" LOG_RESET
                 " secondary-camera CAPTURE — switching primary %s -> %s\n",
                 camera_id == 1 ? "L" : "R", locked == 1 ? "LEFT" : "RIGHT",
                 camera_id == 1 ? "LEFT" : "RIGHT");
        }
        g_selected_camera.store(camera_id);

        float w = g_params.get(LtmuParam::RECT_WIDTH);
        float h = g_params.get(LtmuParam::RECT_HEIGHT);
        cv::Rect bbox(static_cast<int>(pkt.arg1 - w / 2), static_cast<int>(pkt.arg2 - h / 2),
                      static_cast<int>(w), static_cast<int>(h));
        PendingInit &pend = pendingInitFor(camera_id);
        {
          std::lock_guard<std::mutex> lk(pend.mtx);
          pend.bbox = bbox;
          pend.pending = true;
        }
        g_target_confirmed.store(0);
        printf(LOG_YELLOW "[CTRL-%s]" LOG_RESET " CAPTURE @ (%.0f, %.0f)\n",
               camera_id == 1 ? "L" : "R", pkt.arg1, pkt.arg2);
        break;
      }
      case CMD_RESET: {
        // Dual-lock: RESET only the sender's own tracker; the peer keeps
        // tracking if it has an independent lock.
        PendingInit &pend = pendingInitFor(camera_id);
        {
          std::lock_guard<std::mutex> lk(pend.mtx);
          pend.resetRequested = true;
          pend.pending = false;
        }
        if (g_selected_camera.load() == camera_id) g_selected_camera.store(0);
        g_target_confirmed.store(0);
        printf(LOG_YELLOW "[CTRL-%s]" LOG_RESET " RESET\n", camera_id == 1 ? "L" : "R");
        break;
      }
      case CMD_CHANGE_SIZE: {
        float w = g_params.get(LtmuParam::RECT_WIDTH) + pkt.arg1;
        float h = g_params.get(LtmuParam::RECT_HEIGHT) + pkt.arg2;
        g_params.set(LtmuParam::RECT_WIDTH, std::max(8.0f, w));
        g_params.set(LtmuParam::RECT_HEIGHT, std::max(8.0f, h));
        break;
      }
      case CMD_SET_RECT_POS:
        // LTMU has no free-standing rect-position concept outside an
        // active track; accepted and ACKed so the GUI doesn't stall.
        sendAck(camera_id, pkt.type, 0, 0, true);
        break;
      case CMD_SET_PARAM: {
        int paramId = static_cast<int>(pkt.arg1);
        float value = pkt.arg2;
        bool isRect = paramId == static_cast<int>(LtmuParam::RECT_WIDTH) ||
                      paramId == static_cast<int>(LtmuParam::RECT_HEIGHT);
        int locked = g_selected_camera.load();
        bool allowed = !isRect || locked == 0 || locked == camera_id;
        bool ok = false;
        if (paramId >= 1 && paramId <= MAX_TRACKER_PARAM_ID && allowed) ok = g_params.set(paramId, value);
        printf(LOG_YELLOW "[CTRL-%s]" LOG_RESET " SET_PARAM id=%d value=%.4f -> %s\n",
               camera_id == 1 ? "L" : "R", paramId, value, ok ? "OK" : "FAIL");
        sendAck(camera_id, CMD_SET_PARAM, paramId, value, ok);
        break;
      }
      case CMD_GET_PARAMS: {
        for (int id = 1; id <= MAX_TRACKER_PARAM_ID; id++) {
          sendAck(camera_id, CMD_GET_PARAMS, id, g_params.get(id), true);
          usleep(1000);
        }
        break;
      }
      case CMD_SAVE_PARAMS: {
        bool ok = saveParams();
        sendAck(camera_id, CMD_SAVE_PARAMS, 0, 0.0f, ok);
        break;
      }
      case CMD_SET_CAMERA_PARAM: {
        int simpleId = static_cast<int>(pkt.arg1);
        int value = static_cast<int>(pkt.arg2);
        bool ok = false;
        for (int i = 0; i < kNumCameraParams; i++) {
          if (kCameraParams[i].simple_id == simpleId) {
            ok = setCameraControl(device_path, kCameraParams[i].v4l2_id, value);
            break;
          }
        }
        sendAck(camera_id, pkt.type, simpleId, static_cast<float>(value), ok);
        break;
      }
      case CMD_HANDOFF: {
        // Blind handoff: unlock primary, reset only the SENDER's own
        // tracker. The peer camera (if independently locked) keeps
        // tracking — dual-lock, no forced reset of both.
        printf(LOG_YELLOW "[CTRL-%s]" LOG_RESET " HANDOFF\n", camera_id == 1 ? "L" : "R");
        g_selected_camera.store(0);
        g_handoff_requested.store(true);
        PendingInit &pend = pendingInitFor(camera_id);
        {
          std::lock_guard<std::mutex> lk(pend.mtx);
          pend.resetRequested = true;
          pend.pending = false;
        }
        g_target_confirmed.store(0);
        sendAck(camera_id, pkt.type, 0, 0, true);
        break;
      }
      case CMD_HANDOFF_MANUAL: {
        // Geometric handoff: compute the destination-camera pixel via
        // the plane-induced homography and CAPTURE the destination
        // tracker there directly — no blind re-click needed. Source
        // tracker is left running (dual-lock).
        const int sourceCam = static_cast<int>(pkt.arg1);
        if (!g_handoff.ready()) {
          printf(LOG_RED "[HANDOFF]" LOG_RESET
                 " model not initialised — rejected. Check stereo_calib.json + target depth.\n");
          sendAck(camera_id, pkt.type, 0, 0, false);
          break;
        }
        if (sourceCam != 1 && sourceCam != 2) {
          printf(LOG_RED "[HANDOFF]" LOG_RESET " invalid source camera id %d\n", sourceCam);
          sendAck(camera_id, pkt.type, 0, 0, false);
          break;
        }

        double srcU = pkt.arg2, srcV = pkt.arg3;
        if (srcU < 0.0 || srcV < 0.0) {
          // Fall back to the source camera's own current tracker rect
          // centre (dual-lock: each camera has its own live result now,
          // not just the primary's — see tracker_thread.h resultFor()).
          TrackerResultState &srcResult = resultFor(sourceCam);
          std::lock_guard<std::mutex> lk(srcResult.mtx);
          if (!srcResult.haveFrame || srcResult.state != LtmuState::TRACKING) {
            printf(LOG_RED "[HANDOFF]" LOG_RESET
                   " source cam %d has no live track and no explicit pixel supplied\n", sourceCam);
            sendAck(camera_id, pkt.type, 0, 0, false);
            break;
          }
          srcU = srcResult.bbox.x + srcResult.bbox.width / 2.0;
          srcV = srcResult.bbox.y + srcResult.bbox.height / 2.0;
        }

        double dstU = 0.0, dstV = 0.0;
        bool ok = (sourceCam == 1) ? g_handoff.projectLtoR(srcU, srcV, dstU, dstV)
                                    : g_handoff.projectRtoL(srcU, srcV, dstU, dstV);
        if (!ok) {
          printf(LOG_RED "[HANDOFF]" LOG_RESET " projection failed (behind camera / bad geometry)\n");
          sendAck(camera_id, pkt.type, 0, 0, false);
          break;
        }

        const int dstCam = (sourceCam == 1) ? 2 : 1;
        printf(LOG_CYAN "[HANDOFF]" LOG_RESET " %s (%.1f, %.1f) -> %s (%.1f, %.1f)  depth=%.1fmm\n",
               sourceCam == 1 ? "LEFT" : "RIGHT", srcU, srcV, dstCam == 1 ? "LEFT" : "RIGHT",
               dstU, dstV, g_handoff.planeDepthMm());

        float w = g_params.get(LtmuParam::RECT_WIDTH);
        float h = g_params.get(LtmuParam::RECT_HEIGHT);
        cv::Rect dstBbox(static_cast<int>(dstU - w / 2), static_cast<int>(dstV - h / 2),
                         static_cast<int>(w), static_cast<int>(h));
        PendingInit &dstPend = pendingInitFor(dstCam);
        {
          std::lock_guard<std::mutex> lk(dstPend.mtx);
          dstPend.bbox = dstBbox;
          dstPend.pending = true;
        }
        g_selected_camera.store(dstCam);  // primary follows operator intent
        g_target_confirmed.store(0);
        sendAck(camera_id, pkt.type, static_cast<unsigned int>(dstCam), static_cast<float>(dstU), true);
        break;
      }
      case CMD_CONFIRM_TARGET: {
        int confirmed = pkt.arg1 != 0.0f ? 1 : 0;
        g_target_confirmed.store(confirmed);
        printf(LOG_YELLOW "[CTRL-%s]" LOG_RESET " CONFIRM_TARGET -> %s\n",
               camera_id == 1 ? "L" : "R", confirmed ? "CONFIRMED" : "unconfirmed");
        sendAck(camera_id, CMD_CONFIRM_TARGET, 0, static_cast<float>(confirmed), true);
        break;
      }
      default:
        printf("[CTRL] unknown type %u\n", pkt.type);
        break;
    }
  }
  close(sock);
  printf("[CTRL] thread exiting (camera_id=%d)\n", camera_id);
}
