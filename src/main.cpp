// ============================================================
// main.cpp — LTMU tracker entry point (econ-cameras branch).
//
// Thread topology mirrors jetson-tracking-perception/src/main.cpp:
//
//   captureThread(L) ─┐            ┌─→ outputThread   → RTP H.264 → UDP (both cams)
//   captureThread(R) ─┤ ring bufs  ├─→ recorderThread → optional MP4 (both cams)
//   trackerThread    ─┤            │   (dual-lock: L and R tracked independently)
//   controlThread(L) ─┤ UDP CAPTURE/RESET/PARAM/HANDOFF/HANDOFF_MANUAL/CONFIRM
//   controlThread(R) ─┤
//   telemetryThread  ─┘ UDP telemetry (both cams) + UART angle (primary only)
//
// Dual-lock: both cameras may CAPTURE and track simultaneously; there is
// no exclusive gate. g_selected_camera is the UART "primary" pointer
// only. CMD_HANDOFF_MANUAL adds geometric handoff on top — a stereo
// calibration + assumed target-plane depth (src/handoff/) lets the
// destination-camera pixel be computed automatically instead of the
// operator hunting for the target by eye. See docs/protocol.md.
//
// econ-cameras branch: capture reads real dual e-Con V4L2 cameras via
// GStreamer (src/capture/capture.cpp), and CMD_SET_CAMERA_PARAM ioctls
// go to the actual device. Every other module (control, telemetry,
// streaming, recorder, tracker, handoff) is byte-for-byte the same code
// as the uav-dataset branch — that's the point: the pipeline was proven
// there first, only the capture layer changes for real hardware.
// ============================================================
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sys/stat.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "capture/capture.h"
#include "common/globals.h"
#include "control/control.h"
#include "dual/ring_buffer.h"
#include "handoff/handoff.h"
#include "recorder/recorder.h"
#include "streaming/streaming.h"
#include "telemetry/telemetry.h"
#include "tracker/embedder_onnx.h"
#include "tracker/tracker_thread.h"

// ── Global definitions (declared extern in globals.h) ──────────
std::atomic<bool> g_running{true};
std::atomic<int> g_selected_camera{0};
std::atomic<bool> g_handoff_requested{false};
std::atomic<int> g_target_confirmed{0};
std::atomic<int> g_tracker_mode{0};
std::atomic<int> g_frameId{0};
ParamStore g_params;
int g_ctrlSockL = -1, g_ctrlSockR = -1;
int g_telemSockL = -1, g_telemSockR = -1;
sockaddr_in g_gsAddrL{}, g_gsAddrR{};
bool g_gsAddrValidL = false, g_gsAddrValidR = false;
std::mutex g_gsMtx;
std::string g_clientIp = "192.168.0.20";
int g_leftVideoPort = 5000, g_leftCtrlPort = 5001, g_rightVideoPort = 5002,
    g_rightCtrlPort = 5003;
int g_tracker_W = 1280, g_tracker_H = 720, g_fps = 30;
std::string g_uartDev;
int g_uartFd = -1;
std::string CONFIG_FILE = "ltmu_params.cfg";
handoff::HandoffModel g_handoff;
float g_target_depth_mm = 0.0f;
std::atomic<int32_t> g_last_rect_x{0};
std::atomic<int32_t> g_last_rect_y{0};

static void sigHandler(int) {
  static std::atomic<int> hits{0};
  g_running = false;
  if (hits.fetch_add(1) >= 1) {
    fprintf(stderr, "\n[MAIN] second SIGINT — forcing exit\n");
    _exit(2);
  }
}

static int openTelemSocket(int &sockOut) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockOut = sock;
  return sock;
}

int main(int argc, char *argv[]) {
  // argv layout (econ-cameras branch):
  //  [1] clientIp
  //  [2] leftVideoPort  [3] leftCtrlPort  [4] rightVideoPort [5] rightCtrlPort
  //  [6] W [7] H [8] fps
  //  [9] leftDev  [10] rightDev   (e.g. /dev/video0 /dev/video1)
  //  [11] onnxModelPath
  //  [12] uartDev (optional, "" to disable)
  //  [13] recBasePath (optional, "" to disable recording)
  //  [14] pixelFormat (optional, default UYVY — "UYVY"|"YUYV"|"MJPG")
  //  [15] targetDepthMm (optional, 0/omitted disables geometric handoff —
  //       distance in mm from the boresight camera's optical origin to
  //       the target plane; indoor: measured, airframe: barometer AGL)
  if (argc < 12) {
    fprintf(stderr,
            "usage: %s clientIp leftVideoPort leftCtrlPort rightVideoPort "
            "rightCtrlPort W H fps leftDev rightDev onnxModelPath "
            "[uartDev] [recBasePath] [pixelFormat] [targetDepthMm]\n",
            argv[0]);
    return 1;
  }
  g_clientIp = argv[1];
  g_leftVideoPort = atoi(argv[2]);
  g_leftCtrlPort = atoi(argv[3]);
  g_rightVideoPort = atoi(argv[4]);
  g_rightCtrlPort = atoi(argv[5]);
  g_tracker_W = atoi(argv[6]);
  g_tracker_H = atoi(argv[7]);
  g_fps = atoi(argv[8]);
  std::string leftDev = argv[9];
  std::string rightDev = argv[10];
  std::string onnxPath = argv[11];
  g_uartDev = (argc > 12) ? argv[12] : "";
  std::string recBasePath = (argc > 13) ? argv[13] : "";
  bool recEnabled = !recBasePath.empty();
  std::string pixelFormat = (argc > 14) ? argv[14] : "UYVY";
  g_target_depth_mm = (argc > 15) ? static_cast<float>(atof(argv[15])) : 0.0f;

  CONFIG_FILE = "ltmu_params_econ_cameras.cfg";
  loadParams();

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  printf(LOG_GREEN "[MAIN]" LOG_RESET " LTMU tracker (econ-cameras branch)\n");
  printf("  client        : %s\n", g_clientIp.c_str());
  printf("  video ports   : L=%d R=%d\n", g_leftVideoPort, g_rightVideoPort);
  printf("  ctrl ports    : L=%d R=%d\n", g_leftCtrlPort, g_rightCtrlPort);
  printf("  resolution    : %dx%d @ %d fps (%s)\n", g_tracker_W, g_tracker_H, g_fps,
         pixelFormat.c_str());
  printf("  left device   : %s\n", leftDev.c_str());
  printf("  right device  : %s\n", rightDev.c_str());
  printf("  onnx model    : %s\n", onnxPath.c_str());
  printf("  uart          : %s\n", g_uartDev.empty() ? "(disabled)" : g_uartDev.c_str());
  printf("  recording     : %s\n", recEnabled ? recBasePath.c_str() : "(disabled)");
  printf("  target depth  : %s\n",
         g_target_depth_mm > 0.0f ? (std::to_string(g_target_depth_mm) + " mm").c_str()
                                  : "(geometric handoff disabled)");

  Embedder embedder(onnxPath, /*preferCuda=*/true);

  // ── Geometric handoff ────────────────────────────────────────
  // Resolution-specific calibration preferred (stereo_calib_WxH.json),
  // falls back to stereo_calib.json — same lookup as the original repo.
  // No K-scaling: a mismatched-resolution calibration would produce
  // wrong seeds, so recalibrate at the tracker's operating resolution
  // instead of scaling on the fly.
  if (g_target_depth_mm > 0.0f) {
    char sizedPath[64];
    snprintf(sizedPath, sizeof(sizedPath), "stereo_calib_%dx%d.json", g_tracker_W, g_tracker_H);
    struct stat st;
    bool sizedExists = stat(sizedPath, &st) == 0;
    const char *calibPath = sizedExists ? sizedPath : "stereo_calib.json";
    printf("[HANDOFF] %s calibration: %s\n", sizedExists ? "using resolution-specific" : "using",
           calibPath);

    handoff::StereoCalib calib{};
    if (handoff::loadStereoCalib(calibPath, calib)) {
      if (!g_handoff.initialise(calib, g_target_depth_mm))
        fprintf(stderr, "[HANDOFF] initialise() failed — CMD_HANDOFF_MANUAL disabled\n");
    } else {
      fprintf(stderr, "[HANDOFF] %s not loaded — CMD_HANDOFF_MANUAL disabled\n", calibPath);
    }
  } else {
    printf("[HANDOFF] target depth not provided — CMD_HANDOFF_MANUAL disabled "
           "(CMD_HANDOFF still works)\n");
  }

  // Ground-station-facing sockets for telemetry (control.cpp opens its own
  // per-port sockets for ACKs; these are separate send-only sockets).
  openTelemSocket(g_telemSockL);
  openTelemSocket(g_telemSockR);

  RingBuffer leftRing, rightRing;

  CameraConfig leftCfg{leftDev, pixelFormat, g_tracker_W, g_tracker_H, g_fps};
  CameraConfig rightCfg{rightDev, pixelFormat, g_tracker_W, g_tracker_H, g_fps};

  std::thread capL(econCaptureThread, leftCfg, std::ref(leftRing), "L");
  std::thread capR(econCaptureThread, rightCfg, std::ref(rightRing), "R");
  std::thread trk(trackerThread, std::ref(leftRing), std::ref(rightRing), std::ref(embedder));
  std::thread ctrlL(controlThread, g_leftCtrlPort, 1, leftDev.c_str());
  std::thread ctrlR(controlThread, g_rightCtrlPort, 2, rightDev.c_str());
  std::thread telem(telemetryThread, g_fps);
  std::thread out(outputThread, std::ref(leftRing), std::ref(rightRing), g_tracker_W,
                  g_tracker_H, g_fps, g_clientIp, g_leftVideoPort, g_rightVideoPort);
  std::thread rec(recorderThread, std::ref(leftRing), std::ref(rightRing), recBasePath,
                  recEnabled, g_tracker_W, g_tracker_H, g_fps);

  printf(LOG_GREEN "[MAIN]" LOG_RESET " all threads started — waiting for CAPTURE\n");

  capL.join();
  capR.join();
  trk.join();
  ctrlL.join();
  ctrlR.join();
  telem.join();
  out.join();
  rec.join();

  if (g_uartFd >= 0) close(g_uartFd);
  printf(LOG_GREEN "[MAIN]" LOG_RESET " clean shutdown\n");
  return 0;
}
