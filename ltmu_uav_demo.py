#!/usr/bin/env python3
"""LTMU UAV dataset demo — click-drag to select target, SPACE play/pause, q quit."""
import cv2
import glob
import os
import sys

try:
    import onnxruntime as ort
except ImportError:
    print("ERROR: pip install onnxruntime")
    sys.exit(1)

try:
    import torch, torchvision
except ImportError:
    print("ERROR: pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu")
    sys.exit(1)


class SimpleTracker:
    def __init__(self):
        self.tracker = None
        self.initialized = False

    def init(self, frame, bbox):
        try:
            self.tracker = cv2.TrackerCSRT_create()
            self.tracker.init(frame, tuple(bbox))
            self.initialized = True
            return True
        except:
            print("[TRACK] failed to init")
            return False

    def update(self, frame):
        if not self.initialized or self.tracker is None:
            return False, None
        try:
            ok, bbox = self.tracker.update(frame)
            return (True, (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))) if ok else (False, None)
        except:
            return False, None

    def reset(self):
        self.tracker = None
        self.initialized = False


class ROISelector:
    def __init__(self, frame):
        self.frame = frame.copy()
        self.drawing = False
        self.start = None
        self.current = None
        self.roi = None

    def on_mouse(self, event, x, y, flags, param):
        self.current = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drawing and self.start:
            self.drawing = False
            x1, y1 = self.start
            x2, y2 = x, y
            if x2 < x1: x1, x2 = x2, x1
            if y2 < y1: y1, y2 = y2, y1
            w, h = x2 - x1, y2 - y1
            if w > 4 and h > 4:
                self.roi = (x1, y1, w, h)

    def show(self):
        """Show window and wait for ROI selection."""
        win = "Select ROI: click-drag, SPACE to confirm, q to cancel"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self.on_mouse)

        while True:
            display = self.frame.copy()
            if self.drawing and self.start and self.current:
                cv2.rectangle(display, self.start, self.current, (0, 255, 0), 2)
            if self.roi:
                x, y, w, h = self.roi
                cv2.rectangle(display, (x, y), (x+w, y+h), (0, 220, 0), 3)
            cv2.putText(display, "Click-drag, SPACE confirm, q cancel", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(win, display)

            key = cv2.waitKey(30) & 0xFF
            if key == ord(' ') and self.roi:
                break
            if key == ord('q') or key == 27:
                self.roi = None
                break

        cv2.destroyWindow(win)
        return self.roi


class SequencePlayer:
    def __init__(self, seq_dir, fps=30):
        self.frames = sorted(glob.glob(os.path.join(seq_dir, "*.jpg"))) + \
                      sorted(glob.glob(os.path.join(seq_dir, "*.jpeg"))) + \
                      sorted(glob.glob(os.path.join(seq_dir, "*.png")))
        if not self.frames:
            raise ValueError(f"No images in {seq_dir}")
        print(f"[SEQ] {len(self.frames)} frames from {seq_dir}")
        self.fps = fps
        self.frame_period_ms = int(1000 / fps)
        self.idx = 0
        self.paused = False

    def get_current(self):
        img = cv2.imread(self.frames[self.idx])
        return img, self.idx

    def advance(self):
        if not self.paused:
            self.idx = (self.idx + 1) % len(self.frames)

    def toggle_pause(self):
        self.paused = not self.paused
        print(f"[SEQ] {'paused' if self.paused else 'playing'}")


def select_sequence():
    """Show menu of available sequences."""
    base = "/home/nvidia/Downloads/Dataset_UAV123/UAV123/data_seq/UAV123"
    if not os.path.isdir(base):
        print(f"ERROR: {base} not found")
        return None

    seqs = [d for d in sorted(os.listdir(base)) if os.path.isdir(os.path.join(base, d))]
    if not seqs:
        print(f"ERROR: no sequences in {base}")
        return None

    print("\nAvailable sequences:")
    for i, seq in enumerate(seqs, 1):
        print(f"  [{i}] {seq}")

    while True:
        try:
            choice = int(input("\nSelect [1-{}]: ".format(len(seqs))))
            if 1 <= choice <= len(seqs):
                return os.path.join(base, seqs[choice-1])
        except:
            pass
        print(f"Invalid choice, try again")


def main():
    seq_dir = sys.argv[1] if len(sys.argv) > 1 else select_sequence()
    if not seq_dir:
        return 1

    try:
        player = SequencePlayer(seq_dir, fps=30)
    except ValueError as e:
        print(f"ERROR: {e}")
        return 1

    tracker = SimpleTracker()
    window = "LTMU — uav-dataset"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    print("[MAIN] SPACE play/pause   c: select ROI   r: reset   q: quit")

    while True:
        frame, fid = player.get_current()
        if frame is None:
            break

        display = frame.copy()

        # Draw tracker
        if tracker.initialized:
            ok, bbox = tracker.update(frame)
            if ok and bbox:
                x, y, w, h = bbox
                cv2.rectangle(display, (x, y), (x+w, y+h), (0, 220, 0), 2)
                cv2.putText(display, "TRACKING", (x, max(0, y-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 2)
            else:
                cv2.putText(display, "LOST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        status = "PAUSED" if player.paused else "PLAY"
        cv2.putText(display, f"{status} | SPACE play/pause  c: select ROI  r: reset  q: quit",
                   (10, display.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow(window, display)
        key = cv2.waitKey(player.frame_period_ms) & 0xFF

        if key == ord(' '):
            player.toggle_pause()
        elif key == ord('c') or key == ord('C'):
            selector = ROISelector(frame)
            roi = selector.show()
            if roi and tracker.init(frame, roi):
                print(f"[MAIN] CAPTURE @ {roi}")
        elif key == ord('r') or key == ord('R'):
            tracker.reset()
            print("[MAIN] RESET")
        elif key == ord('q') or key == ord('Q') or key == 27:
            print("[MAIN] quit")
            break

        player.advance()

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
