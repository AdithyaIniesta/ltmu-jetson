#!/usr/bin/env python3
"""
LTMU UAV dataset local demo — Python PoC.

Read a sequence of images, display in OpenCV:
  SPACE  play / pause
  c      draw the target ROI (drag box, press ENTER)
  r      reset (drop the lock)
  q/ESC  quit

Runs ONNX embedder tracking on locked target. Draw the tracker bbox overlay live.
"""
import cv2
import glob
import os
import sys
import numpy as np
from pathlib import Path

# Try to import onnxruntime; fall back to CPU-only if CUDA unavailable
try:
    import onnxruntime as ort
except ImportError:
    print("ERROR: onnxruntime not installed. On Jetson:")
    print("  pip install onnxruntime")
    sys.exit(1)

try:
    import torch
    import torchvision
except ImportError:
    print("ERROR: torch/torchvision not installed. On Jetson:")
    print("  pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu")
    sys.exit(1)


class SimpleTracker:
    """Minimal CSRT-like tracker using OpenCV."""
    def __init__(self):
        self.tracker = None
        self.initialized = False

    def init(self, frame, bbox):
        """Init tracker with a frame and bounding box."""
        try:
            self.tracker = cv2.TrackerCSRT_create()
            self.tracker.init(frame, tuple(bbox))
            self.initialized = True
            return True
        except:
            print("[TRACK] failed to init tracker")
            return False

    def update(self, frame):
        """Update tracker, return (success, bbox)."""
        if not self.initialized or self.tracker is None:
            return False, None
        try:
            ok, bbox = self.tracker.update(frame)
            if ok:
                x, y, w, h = bbox
                return True, (int(x), int(y), int(w), int(h))
            else:
                return False, None
        except:
            return False, None

    def reset(self):
        """Drop the lock."""
        self.tracker = None
        self.initialized = False


class SequencePlayer:
    """Paced image sequence playback."""
    def __init__(self, seq_dir, fps=30):
        self.frames = sorted(glob.glob(os.path.join(seq_dir, "*.jpg"))) + \
                      sorted(glob.glob(os.path.join(seq_dir, "*.jpeg"))) + \
                      sorted(glob.glob(os.path.join(seq_dir, "*.png")))
        if not self.frames:
            raise ValueError(f"No images found in {seq_dir}")
        print(f"[SEQ] loaded {len(self.frames)} frames from {seq_dir}")

        self.fps = fps
        self.frame_period_ms = int(1000 / fps)
        self.idx = 0
        self.paused = False

    def get_current(self):
        """Return current frame and index."""
        img = cv2.imread(self.frames[self.idx])
        return img, self.idx

    def advance(self):
        """Move to next frame (loop)."""
        if not self.paused:
            self.idx = (self.idx + 1) % len(self.frames)

    def toggle_pause(self):
        self.paused = not self.paused
        print(f"[SEQ] {'paused' if self.paused else 'playing'}")

    def reset(self):
        self.idx = 0


def main():
    if len(sys.argv) < 2:
        seq_dir = "/home/nvidia/Downloads/Dataset_UAV123/UAV123/data_seq/UAV123/bike1"
        print(f"[MAIN] usage: {sys.argv[0]} <sequence_dir>")
        print(f"[MAIN] using default: {seq_dir}")
    else:
        seq_dir = sys.argv[1]

    # Load sequence
    try:
        player = SequencePlayer(seq_dir, fps=30)
    except ValueError as e:
        print(f"ERROR: {e}")
        return 1

    # Init tracker
    tracker = SimpleTracker()
    roi_selected = False

    window_name = "LTMU — uav-dataset (Python PoC)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    print("[MAIN] SPACE=play/pause  c=select ROI  r=reset  q=quit")

    while True:
        frame, fid = player.get_current()
        if frame is None:
            print("[MAIN] sequence exhausted")
            break

        display = frame.copy()

        # Draw tracker result if locked
        if tracker.initialized:
            ok, bbox = tracker.update(frame)
            if ok and bbox is not None:
                x, y, w, h = bbox
                cv2.rectangle(display, (x, y), (x+w, y+h), (0, 220, 0), 2)
                cv2.putText(display, "TRACKING", (x, max(0, y-6)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 2)
                roi_selected = True
            else:
                cv2.putText(display, "LOST", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # HUD
        status = "PAUSED" if player.paused else "PLAY"
        hud = f"{status}  |  SPACE play/pause   c: select ROI   r: reset   q: quit"
        cv2.putText(display, hud, (10, display.shape[0] - 12),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(window_name, display)
        key = cv2.waitKey(player.frame_period_ms) & 0xFF

        if key == ord(' '):
            player.toggle_pause()
        elif key == ord('c') or key == ord('C'):
            roi = cv2.selectROI(window_name, frame, showCrosshair=False, fromCenter=False)
            if roi[2] > 4 and roi[3] > 4:  # x, y, w, h
                if tracker.init(frame, roi):
                    print(f"[MAIN] CAPTURE @ ({roi[0]}, {roi[1]}, {roi[2]}, {roi[3]})")
                    roi_selected = True
        elif key == ord('r') or key == ord('R'):
            tracker.reset()
            roi_selected = False
            print("[MAIN] RESET")
        elif key == ord('q') or key == ord('Q') or key == 27:  # ESC
            print("[MAIN] quit")
            break

        player.advance()

    cv2.destroyAllWindows()
    print("[MAIN] clean shutdown")
    return 0


if __name__ == "__main__":
    sys.exit(main())
