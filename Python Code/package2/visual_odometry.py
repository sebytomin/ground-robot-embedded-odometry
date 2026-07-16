"""
visual_odometry.py  –  Package 2
-----------------------------------
Standalone Monocular Visual Odometry pipeline (camera only, no IMU).
run live

Usage
-----
    # Live camera
    python visual_odometry.py 

Output
------
    vo_log.csv          – per-frame log
    vo_trajectory.npy   – (N, 4) [ts, x, y, yaw]
"""

import argparse
import csv
import time
import sys
import numpy as np
import cv2
from pathlib import Path

from camera_calibration import load_params
from feature_tracker    import FeatureTracker
from mono_vo            import MonocularVO


CAMERA_FPS = 30


# ── Simple scale estimator ────────────────────────────────────────────────────

class MedianScaleEstimator:
    """
    Estimates VO scale using the median distance between
    consecutive camera positions (works best when robot speed is known).
    """

    def __init__(self, nominal_speed_m_per_frame: float = 0.03):
        """
        nominal_speed_m_per_frame : expected distance per frame (m).
        For a robot moving at ~0.9 m/s at 30 fps → 0.03 m/frame.
        """
        self._nominal = nominal_speed_m_per_frame
        self._history = []
        self._scale   = nominal_speed_m_per_frame

    def update(self, t_norm: float) -> float:
        """Update scale estimate with latest VO translation norm."""
        if t_norm > 1e-4:
            est = self._nominal / t_norm
            self._history.append(est)
            if len(self._history) > 30:
                self._history.pop(0)
            self._scale = float(np.median(self._history))
        return self._scale

    @property
    def scale(self):
        return self._scale


# ── Logger ────────────────────────────────────────────────────────────────────

class VOLogger:
    HEADER = ["t_s", "px", "py", "pz", "yaw_deg",
              "n_tracks", "n_inliers", "scale"]

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)

    def write(self, t, pos, yaw, n_tracks, n_inliers, scale):
        self._w.writerow([
            f"{t:.4f}",
            f"{pos[0]:.5f}", f"{pos[1]:.5f}", f"{pos[2]:.5f}",
            f"{np.degrees(yaw):.3f}",
            n_tracks, n_inliers,
            f"{scale:.5f}",
        ])

    def flush(self): self._f.flush()
    def close(self): self._f.close()


# ── Visualiser ────────────────────────────────────────────────────────────────

class VOVisualizer:
    def __init__(self):
        try:
            import matplotlib.pyplot as plt
            self._plt = plt
            self._fig, axes = plt.subplots(1, 2, figsize=(13, 6))
            self._fig.suptitle("Visual Odometry (camera-only)",
                               fontsize=13, fontweight="bold")
            self._ax_traj, self._ax_feat = axes
            self._ax_traj.set_title("Top-down trajectory")
            self._ax_traj.set_xlabel("X [m]"); self._ax_traj.set_ylabel("Y [m]")
            self._ax_traj.set_aspect("equal")
            self._ax_traj.grid(True, linestyle="--", alpha=0.5)
            self._line, = self._ax_traj.plot([], [], "g-", lw=2, label="VO")
            self._scat, = self._ax_traj.plot([], [], "ko", ms=8)
            self._ax_traj.legend()
            self._ax_feat.set_title("Camera + features")
            self._ax_feat.axis("off")
            self._im = None
            plt.tight_layout(); plt.ion(); plt.show()
            self._xs = []; self._ys = []
            self._ok = True
        except ImportError:
            self._ok = False

    def update(self, pos, feat_frame):
        if not self._ok: return
        self._xs.append(pos[0]); self._ys.append(pos[1])
        self._line.set_data(self._xs, self._ys)
        self._scat.set_data([pos[0]], [pos[1]])
        self._ax_traj.relim(); self._ax_traj.autoscale_view()
        if feat_frame is not None:
            rgb = cv2.cvtColor(feat_frame, cv2.COLOR_BGR2RGB)
            if self._im is None:
                self._im = self._ax_feat.imshow(rgb)
            else:
                self._im.set_data(rgb)
        try:
            self._fig.canvas.flush_events(); self._plt.pause(0.001)
        except Exception:
            pass

    def stop(self):
        if self._ok:
            self._plt.ioff(); self._plt.close("all")


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Standalone Visual Odometry")
    p.add_argument("--params",  default="camera_params.npz")
    p.add_argument("--camera",  default=0, type=int)
    p.add_argument("--video",   default=None, help="Replay a video file")
    p.add_argument("--log",     default="vo_log.csv")
    p.add_argument("--out-npy", default="vo_trajectory.npy")
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--nominal-speed", default=0.03, type=float,
                   help="Expected metres per frame for scale init (default 0.03)")
    return p.parse_args()


def main():
    args = parse_args()

    if not Path(args.params).exists():
        print(f"[VO] Camera params not found: {args.params}")
        print("     Run: python generate_default_params.py")
        sys.exit(1)

    K, dist, img_shape = load_params(args.params)
    print(f"[VO] Camera K loaded: fx={K[0,0]:.1f} fy={K[1,1]:.1f}")

    tracker = FeatureTracker(K, dist, max_features=300, min_features=50)
    mono_vo = MonocularVO(K, ransac_thr=1.0, min_inliers=20)
    scaler  = MedianScaleEstimator(args.nominal_speed)
    logger  = VOLogger(args.log)
    vis     = VOVisualizer() if not args.no_plot else None

    # State
    position = np.zeros(3)
    R_world  = np.eye(3)
    yaw      = 0.0
    traj     = []

    # Open video source — mirrors the fallback chain in vio_main.py
    cap = None

    if args.video:
        # ── Video file: try several backends in order ──────────────────────────
        backends = [
            (cv2.CAP_FFMPEG,   "FFMPEG"),
            (cv2.CAP_GSTREAMER,"GStreamer"),
            (cv2.CAP_ANY,      "auto"),
        ]
        for backend, name in backends:
            try:
                c = cv2.VideoCapture(args.video, backend)
                if c.isOpened():
                    cap = c
                    print(f"[VO] Replaying: {args.video}  (backend: {name})")
                    break
                c.release()
            except Exception:
                pass

        # Last-resort: plain path, no backend flag
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(args.video)
            if cap.isOpened():
                print(f"[VO] Replaying: {args.video}  (backend: default)")
            else:
                print(f"[VO] ERROR: Cannot open video file: {args.video}")
                print("     Check the path exists and the file is a valid AVI/MP4/MKV.")
                sys.exit(1)

    else:
        # ── Live camera: GStreamer (Jetson CSI) → plain index → V4L2 ──────────
        gst = (
            f"nvarguscamerasrc sensor-id={args.camera} ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)640, height=(int)480, "
            "format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv ! video/x-raw, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
        )
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print(f"[VO] Live camera {args.camera} via GStreamer (Jetson)")
        else:
            cap.release()
            # Try V4L2 by device path
            v4l2_path = f"/dev/video{args.camera}"
            cap = cv2.VideoCapture(v4l2_path, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)
                print(f"[VO] Live camera via V4L2: {v4l2_path}")
            else:
                cap.release()
                # Plain index fallback
                cap = cv2.VideoCapture(args.camera)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)
                if cap.isOpened():
                    print(f"[VO] Live camera {args.camera} (default backend)")
                else:
                    print(f"[VO] ERROR: Cannot open camera index {args.camera}")
                    print("     Try:  --camera 1   or check  ls /dev/video*")
                    sys.exit(1)

    frame_idx = 0
    t_start   = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            t = time.time() - t_start
            pts_prev, pts_curr, n_tracks = tracker.process(frame)

            n_inliers = 0
            if pts_prev is not None and n_tracks >= 20:
                t_norm = float(np.linalg.norm(
                    (pts_curr - pts_prev), axis=1).mean()) if n_tracks > 0 else 0
                scale = scaler.update(t_norm / max(t_norm, 1e-4))

                success, R_rel, t_rel, n_inliers = mono_vo.process(
                    pts_prev, pts_curr, scale=scale
                )

                if success:
                    t_world  = R_world @ t_rel
                    t_world[2] = 0.0
                    position   += t_world

                    vo_yaw   = float(np.arctan2(R_rel[1, 0], R_rel[0, 0]))
                    yaw      = float(np.arctan2(
                        np.sin(yaw + vo_yaw), np.cos(yaw + vo_yaw)
                    ))
                    cy, sy   = np.cos(yaw), np.sin(yaw)
                    R_world  = np.array([
                        [cy, -sy, 0], [sy, cy, 0], [0, 0, 1]
                    ])

            traj.append([t, position[0], position[1], yaw])
            logger.write(t, position, yaw, n_tracks, n_inliers, scaler.scale)

            if vis:
                feat_dbg = tracker.draw_tracks(frame)
                vis.update(position, feat_dbg)

            frame_idx += 1
            if frame_idx % 90 == 0:
                print(f"[VO] frame={frame_idx:5d}  "
                      f"pos=({position[0]:.2f},{position[1]:.2f}) m  "
                      f"yaw={np.degrees(yaw):.1f}°  "
                      f"tracks={n_tracks}  inliers={n_inliers}")
                logger.flush()

    except KeyboardInterrupt:
        print("\n[VO] Stopped by user.")
    finally:
        cap.release()
        logger.close()
        if vis:
            vis.stop()

    traj_np = np.array(traj, dtype=np.float64)
    np.save(args.out_npy, traj_np)
    print(f"[VO] Log → {args.log}")
    print(f"[VO] Traj → {args.out_npy}  ({len(traj_np)} frames)")


if __name__ == "__main__":
    main()
