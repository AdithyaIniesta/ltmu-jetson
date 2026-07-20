// ============================================================
// main.cpp — LTMU tracker entry point (uav-dataset branch).
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
// uav-dataset branch: capture reads paced image sequences instead of
// live V4L2 cameras — see src/capture/capture.cpp. Every other module
// (control, telemetry, streaming, recorder, tracker, handoff) is
// identical to the econ-cameras branch, which is the point: this
// branch proves the full pipeline against ground truth before hardware
// is involved.
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
#include "viewer/local_viewer.h"

// ── Global definitions (declared extern in globals.h) ──────────
std::atomic<bool> g_running{true};
std::atomic<bool> g_paused{false};
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
  // argv layout (uav-dataset branch):
  //  [1] clientIp
  //  [2] leftVideoPort  [3] leftCtrlPort  [4] rightVideoPort [5] rightCtrlPort
  //  [6] W [7] H [8] fps
  //  [9] leftSequenceDir  [10] rightSequenceDir (may repeat [9] to simulate dual)
  //  [11] onnxModelPath
  //  [12] uartDev (optional, "" to disable)
  //  [13] recBasePath (optional, "" to disable recording)
  //  [14] targetDepthMm (optional, 0/omitted disables geometric handoff —
  //       distance in mm from the boresight camera's optical origin to
  //       the target plane; indoor: measured, airframe: barometer AGL)
  //  [15] motionModel (optional, default "cv" — "cv"|"ca"|"ctrv"|"imm";
  //       see src/tracker/motion.h for why no single fixed model is
  //       right for every target — set once at boot, not GUI-switchable)
  if (argc < 12) {
    fprintf(stderr,
            "usage: %s clientIp leftVideoPort leftCtrlPort rightVideoPort "
            "rightCtrlPort W H fps leftSeqDir rightSeqDir onnxModelPath "
            "[uartDev] [recBasePath] [targetDepthMm] [motionModel]\n",
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
  std::string leftSeqDir = argv[9];
  std::string rightSeqDir = argv[10];
  std::string onnxPath = argv[11];
  g_uartDev = (argc > 12) ? argv[12] : "";
  std::string recBasePath = (argc > 13) ? argv[13] : "";
  bool recEnabled = !recBasePath.empty();
  g_target_depth_mm = (argc > 14) ? static_cast<float>(atof(argv[14])) : 0.0f;
  MotionModel motionModel = motionModelFromString((argc > 15) ? argv[15] : "cv");

  CONFIG_FILE = "ltmu_params_uav_dataset.cfg";
  loadParams();

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  printf(LOG_GREEN "[MAIN]" LOG_RESET " LTMU tracker (uav-dataset branch)\n");
  printf("  client        : %s\n", g_clientIp.c_str());
  printf("  video ports   : L=%d R=%d\n", g_leftVideoPort, g_rightVideoPort);
  printf("  ctrl ports    : L=%d R=%d\n", g_leftCtrlPort, g_rightCtrlPort);
  printf("  resolution    : %dx%d @ %d fps\n", g_tracker_W, g_tracker_H, g_fps);
  printf("  left seq      : %s\n", leftSeqDir.c_str());
  printf("  right seq     : %s\n", rightSeqDir.c_str());
  printf("  onnx model    : %s\n", onnxPath.c_str());
  printf("  uart          : %s\n", g_uartDev.empty() ? "(disabled)" : g_uartDev.c_str());
  printf("  recording     : %s\n", recEnabled ? recBasePath.c_str() : "(disabled)");
  printf("  target depth  : %s\n",
         g_target_depth_mm > 0.0f ? (std::to_string(g_target_depth_mm) + " mm").c_str()
                                  : "(geometric handoff disabled)");
  printf("  motion model  : %s\n", motionModelName(motionModel));

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

  DatasetCaptureConfig leftCfg{leftSeqDir, g_fps, /*loop=*/true};
  DatasetCaptureConfig rightCfg{rightSeqDir, g_fps, /*loop=*/true};

  std::thread capL(datasetCaptureThread, leftCfg, std::ref(leftRing), "L");
  std::thread capR(datasetCaptureThread, rightCfg, std::ref(rightRing), "R");
  std::thread trk(trackerThread, std::ref(leftRing), std::ref(rightRing), std::ref(embedder),
                  motionModel);
  std::thread ctrlL(controlThread, g_leftCtrlPort, 1, "");
  std::thread ctrlR(controlThread, g_rightCtrlPort, 2, "");
  std::thread telem(telemetryThread, g_fps);
  std::thread out(outputThread, std::ref(leftRing), std::ref(rightRing), g_tracker_W,
                  g_tracker_H, g_fps, g_clientIp, g_leftVideoPort, g_rightVideoPort);
  std::thread rec(recorderThread, std::ref(leftRing), std::ref(rightRing), recBasePath,
                  recEnabled, g_tracker_W, g_tracker_H, g_fps);

  printf(LOG_GREEN "[MAIN]" LOG_RESET " all threads started — waiting for CAPTURE\n");

  // Local-display mode (LTMU_LOCAL_DISPLAY=1, set by run_jp5.sh): drive an
  // on-Jetson OpenCV window on the main thread — SPACE play/pause, draw the
  // ROI with the mouse, watch the overlay — no ground-station GUI needed.
  // The stream/control/telemetry threads keep running harmlessly; the
  // viewer just feeds CAPTURE through the same g_pendingInitL slot. When
  // it returns (operator pressed q) g_running is already cleared, so the
  // worker threads below wind down exactly as they do on SIGINT.
  const char *localDisplay = getenv("LTMU_LOCAL_DISPLAY");
  if (localDisplay && localDisplay[0] == '1') {
    localViewerLoop(leftRing);
  }

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
