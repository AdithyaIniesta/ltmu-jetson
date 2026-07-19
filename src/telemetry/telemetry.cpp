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

void sendTelemetry(int frameId, int frameW, int frameH, float angleX, float angleY,
                   int pixOffX, int pixOffY) {
  int cam = g_selected_camera.load();
  int sock = (cam == 2) ? g_telemSockR : g_telemSockL;
  if (sock < 0) return;

  cv::Rect bbox; float vscore = 0.0f; LtmuState state = LtmuState::LOST;
  {
    std::lock_guard<std::mutex> lk(g_result.mtx);
    bbox = g_result.bbox;
    vscore = g_result.verifierScore;
    state = g_result.state;
  }

  TelemetryPacket t{};
  t.magic = TELEMETRY_MAGIC;
  t.frame_id = static_cast<uint32_t>(frameId);
  t.mode = static_cast<uint32_t>(state == LtmuState::TRACKING ? 1 : 2);
  t.det_prob = vscore;
  t.proc_time_us = 0;
  t.rect_x = bbox.x; t.rect_y = bbox.y;
  t.rect_width = bbox.width; t.rect_height = bbox.height;
  t.object_x = bbox.x; t.object_y = bbox.y;
  t.object_width = bbox.width; t.object_height = bbox.height;
  t.angle_x_deg = angleX; t.angle_y_deg = angleY;
  t.pixel_offset_x = pixOffX; t.pixel_offset_y = pixOffY;
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
  t.dnn_similarity = vscore;
  t.frame_width = frameW;
  t.frame_height = frameH;
  t.reserved = ENGINE_TAG | ENGINE_LTMU;
  if (g_target_confirmed.load()) t.reserved |= CONFIRM_TELEM_BIT;

  const sockaddr_in &dest = (cam == 2) ? g_gsAddrR : g_gsAddrL;
  bool valid = (cam == 2) ? g_gsAddrValidR : g_gsAddrValidL;
  if (!valid) return;
  sendto(sock, &t, sizeof(t), 0, reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
}

void telemetryThread(int fps) {
  const auto period = std::chrono::milliseconds(1000 / std::max(1, fps));
  int uartFd = g_uartDev.empty() ? -1 : uartOpen(g_uartDev.c_str(), 115200);
  if (!g_uartDev.empty())
    printf(LOG_CYAN "[TELEM]" LOG_RESET " UART %s %s\n", g_uartDev.c_str(),
           uartFd >= 0 ? "opened" : "FAILED to open");

  while (g_running.load()) {
    cv::Rect bbox; float vscore = 0.0f; LtmuState state = LtmuState::LOST;
    int frameId = 0, fw = 0, fh = 0;
    {
      std::lock_guard<std::mutex> lk(g_result.mtx);
      bbox = g_result.bbox; vscore = g_result.verifierScore; state = g_result.state;
      frameId = g_result.frameId; fw = g_result.frameW; fh = g_result.frameH;
    }

    float pixOffX = 0, pixOffY = 0, angleX = 0, angleY = 0;
    if (fw > 0 && fh > 0) {
      float cx = bbox.x + bbox.width / 2.0f;
      float cy = bbox.y + bbox.height / 2.0f;
      pixOffX = cx - fw / 2.0f;
      pixOffY = cy - fh / 2.0f;
      // Simple linear pixel->degree mapping using a placeholder HFOV/VFOV;
      // replace with the real lens FOV constants when known (see the
      // original repo's HFOV_ECON/VFOV_ECON pattern in common/globals.h).
      constexpr float kHfovDeg = 90.0f, kVfovDeg = 60.0f;
      angleX = (pixOffX / fw) * kHfovDeg;
      angleY = (pixOffY / fh) * kVfovDeg;
    }

    sendTelemetry(frameId, fw, fh, angleX, angleY, static_cast<int>(pixOffX),
                 static_cast<int>(pixOffY));

    int mode = state == LtmuState::TRACKING ? 1 : 2;
    uartSendAngle(uartFd, mode, angleX, angleY, vscore, static_cast<int>(pixOffX),
                 static_cast<int>(pixOffY), g_target_confirmed.load());

    std::this_thread::sleep_for(period);
  }
  if (uartFd >= 0) close(uartFd);
  printf("[TELEM] thread exiting\n");
}
