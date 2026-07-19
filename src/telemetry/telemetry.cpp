#include "telemetry.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <opencv2/core.hpp>
#include <thread>

#include "../common/globals.h"
#include "../tracker/tracker_thread.h"

int uartOpen(const char *device, int baudrate) {
  int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  struct termios tty {};
  if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
  speed_t baud;
  switch (baudrate) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    default: baud = B115200; break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  tcflush(fd, TCIFLUSH);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
  int fl = fcntl(fd, F_GETFL, 0);
  if (fl != -1) fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
  return fd;
}

void uartSendAngle(int fd, int mode, float angleX, float angleY, float detProb,
                   int pixelX, int pixelY, int confirmed) {
  if (fd < 0) return;
  UartAnglePacket pkt{};
  pkt.header[0] = 0xAA; pkt.header[1] = 0x55;
  pkt.mode = static_cast<uint8_t>(mode);
  pkt.angle_x_deg = angleX;
  pkt.angle_y_deg = angleY;
  pkt.det_prob = detProb;
  pkt.pixel_x = static_cast<int16_t>(pixelX);
  pkt.pixel_y = static_cast<int16_t>(pixelY);
  pkt.confirmed = confirmed ? 1 : 0;
  uint8_t cs = 0;
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&pkt);
  for (size_t i = 0; i < offsetof(UartAnglePacket, checksum); i++) cs ^= raw[i];
  pkt.checksum = cs;

  const uint8_t *out = raw;
  size_t remaining = sizeof(pkt);
  while (remaining > 0) {
    ssize_t n = write(fd, out, remaining);
    if (n > 0) { out += n; remaining -= static_cast<size_t>(n); }
    else if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    else break;
  }
}

namespace {

// Computes pixel-offset-from-center and a placeholder-FOV angle for one
// camera's current result. Real deployments should replace kHfovDeg/
// kVfovDeg with the lens's actual FOV (see original repo's HFOV_ECON/
// VFOV_ECON constants).
AngleReportPod computeReport(TrackerResultState &result) {
  AngleReportPod r{};
  {
    std::lock_guard<std::mutex> lk(result.mtx);
    r.bbox = result.bbox;
    r.state = result.state;
    r.vscore = result.verifierScore;
    r.frameId = result.frameId;
    r.fw = result.frameW;
    r.fh = result.frameH;
  }
  if (r.fw > 0 && r.fh > 0) {
    float cx = r.bbox.x + r.bbox.width / 2.0f;
    float cy = r.bbox.y + r.bbox.height / 2.0f;
    r.pixOffX = cx - r.fw / 2.0f;
    r.pixOffY = cy - r.fh / 2.0f;
    constexpr float kHfovDeg = 90.0f, kVfovDeg = 60.0f;
    r.angleX = (r.pixOffX / r.fw) * kHfovDeg;
    r.angleY = (r.pixOffY / r.fh) * kVfovDeg;
  }
  return r;
}

}  // namespace

// Dual-lock: each camera sends its OWN live telemetry to its OWN GUI
// port — S1/S2 panels both show real tracking state simultaneously,
// not just whichever camera is "primary".
void sendTelemetry(int cameraId, const AngleReportPod &r) {
  int sock = (cameraId == 2) ? g_telemSockR : g_telemSockL;
  if (sock < 0) return;
  bool valid = (cameraId == 2) ? g_gsAddrValidR : g_gsAddrValidL;
  if (!valid) return;
  const sockaddr_in &dest = (cameraId == 2) ? g_gsAddrR : g_gsAddrL;

  TelemetryPacket t{};
  t.magic = TELEMETRY_MAGIC;
  t.frame_id = static_cast<uint32_t>(r.frameId);
  t.mode = static_cast<uint32_t>(r.state == LtmuState::TRACKING ? 1 : 2);
  t.det_prob = r.vscore;
  t.proc_time_us = 0;
  t.rect_x = r.bbox.x; t.rect_y = r.bbox.y;
  t.rect_width = r.bbox.width; t.rect_height = r.bbox.height;
  t.object_x = r.bbox.x; t.object_y = r.bbox.y;
  t.object_width = r.bbox.width; t.object_height = r.bbox.height;
  t.angle_x_deg = r.angleX; t.angle_y_deg = r.angleY;
  t.pixel_offset_x = static_cast<int32_t>(r.pixOffX);
  t.pixel_offset_y = static_cast<int32_t>(r.pixOffY);
  t.search_win_width = static_cast<int32_t>(g_params.get(LtmuParam::SEARCH_WINDOW_WIDTH));
  t.search_win_height = static_cast<int32_t>(g_params.get(LtmuParam::SEARCH_WINDOW_HEIGHT));
  t.lost_mode_option = g_params.get(LtmuParam::LOST_MODE_OPTION);
  t.frame_buffer_size = static_cast<int32_t>(g_params.get(LtmuParam::FRAME_BUFFER_SIZE));
  t.max_frames_lost = static_cast<int32_t>(g_params.get(LtmuParam::MAX_FRAMES_IN_LOST_MODE));
  t.rect_auto_size = g_params.get(LtmuParam::RECT_AUTO_SIZE);
  t.rect_auto_position = g_params.get(LtmuParam::RECT_AUTO_POSITION);
  t.multiple_threads = static_cast<int32_t>(g_params.get(LtmuParam::MULTIPLE_THREADS));
  t.num_channels = static_cast<int32_t>(g_params.get(LtmuParam::NUM_CHANNELS));
  t.tracker_type = static_cast<int32_t>(g_params.get(LtmuParam::TYPE));
  t.custom_1 = g_params.get(LtmuParam::CUSTOM_1);
  t.custom_2 = g_params.get(LtmuParam::CUSTOM_2);
  t.custom_3 = g_params.get(LtmuParam::CUSTOM_3);
  // LTMU has no EKF sigma knobs — those fields are cuda_library-specific.
  t.ekf_sigma_a = 0.0f; t.ekf_sigma_alpha = 0.0f; t.ekf_r_base = 0.0f;
  t.dnn_verify_interval = static_cast<int32_t>(g_params.get(LtmuParam::REDETECT_INTERVAL));
  t.dnn_veto_threshold = g_params.get(LtmuParam::VERIFIER_LOST_THRESH);
  t.dnn_accept_threshold = g_params.get(LtmuParam::VERIFIER_UPDATE_THRESH);
  t.dnn_similarity = r.vscore;
  t.frame_width = r.fw;
  t.frame_height = r.fh;
  t.reserved = ENGINE_TAG | ENGINE_LTMU;
  if (g_target_confirmed.load()) t.reserved |= CONFIRM_TELEM_BIT;

  sendto(sock, &t, sizeof(t), 0, reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
}

void telemetryThread(int fps) {
  const auto period = std::chrono::milliseconds(1000 / std::max(1, fps));
  int uartFd = g_uartDev.empty() ? -1 : uartOpen(g_uartDev.c_str(), 115200);
  if (!g_uartDev.empty())
    printf(LOG_CYAN "[TELEM]" LOG_RESET " UART %s %s\n", g_uartDev.c_str(),
           uartFd >= 0 ? "opened" : "FAILED to open");

  while (g_running.load()) {
    AngleReportPod rL = computeReport(g_resultL);
    AngleReportPod rR = computeReport(g_resultR);

    sendTelemetry(1, rL);
    sendTelemetry(2, rR);

    // UART is a single physical link — only the primary camera's angle
    // is sent, matching the gimbal's single active setpoint.
    int primary = g_selected_camera.load();
    const AngleReportPod &pr = (primary == 2) ? rR : rL;
    int mode = pr.state == LtmuState::TRACKING ? 1 : 2;
    if (primary != 0)
      uartSendAngle(uartFd, mode, pr.angleX, pr.angleY, pr.vscore,
                   static_cast<int>(pr.pixOffX), static_cast<int>(pr.pixOffY),
                   g_target_confirmed.load());

    std::this_thread::sleep_for(period);
  }
  if (uartFd >= 0) close(uartFd);
  printf("[TELEM] thread exiting\n");
}
