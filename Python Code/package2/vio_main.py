"""
vio_main.py  –  Package 2
---------------------------
Visual-Inertial Odometry entry point.


Usage
-----
    # Full run (auto-detect port + camera 0):
    python3 vio_main.py

"""

import argparse, csv, threading, queue, time, sys, os
import numpy as np
import cv2
from pathlib import Path

from imu_receiver       import IMUReceiver
from camera_calibration import load_params
from feature_tracker    import FeatureTracker
from mono_vo            import MonocularVO
from vio_fusion         import VIOFusion

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'package1'))
from ahrs import MahonyAHRS

IMU_HZ    = 100
IMU_DT    = 1.0 / IMU_HZ
CAM_FPS   = 30


# ── Camera capture thread ──────────────────────────────────────────────────────

def camera_thread(cam_idx, cam_q, stop_ev, width=640, height=480):
    """Try GStreamer (Jetson), fall back to V4L2."""
    gst = (
        f"nvarguscamerasrc sensor-id={cam_idx} ! "
        "video/x-raw(memory:NVMM),"
        f"width=(int){width},height=(int){height},"
        "format=(string)NV12,framerate=(fraction)30/1 ! "
        "nvvidconv ! video/x-raw,format=(string)BGRx ! "
        "videoconvert ! video/x-raw,format=(string)BGR ! appsink drop=1"
    )
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[Camera] GStreamer failed, trying V4L2…")
        cap = cv2.VideoCapture(cam_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)

    if not cap.isOpened():
        print("[Camera] ERROR: cannot open camera — check connection")
        stop_ev.set()
        return

    print(f"[Camera] Opened cam {cam_idx}  {width}×{height} @ {CAM_FPS} fps")

    while not stop_ev.is_set():
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.005)
            continue
        ts = time.time()
        try:
            cam_q.put_nowait((ts, frame))
        except queue.Full:
            try:
                cam_q.get_nowait()
            except queue.Empty:
                pass
            cam_q.put_nowait((ts, frame))

    cap.release()
    print("[Camera] Thread exited.")


# ── CSV logger ─────────────────────────────────────────────────────────────────

class VIOLogger:

    # t_s, px, py  (same as dr_log.csv so both can be loaded identically)
    HDR = ["t_s", "px", "py", "vx", "vy",
           "yaw_deg", "scale", "n_tracks", "n_inliers", "mode"]

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.HDR)

    def write(self, t, pos, vel, yaw, scale, n_tracks, n_inliers, mode):
        self._w.writerow([
            f"{t:.4f}",
            f"{pos[0]:.5f}", f"{pos[1]:.5f}",
            f"{vel[0]:.5f}", f"{vel[1]:.5f}",
            f"{np.degrees(yaw):.3f}",
            f"{scale:.5f}",
            n_tracks, n_inliers, mode,
        ])

    def flush(self): self._f.flush()
    def close(self): self._f.close()


# ── Visualiser ─────────────────────────────────────────────────────────────────

class VIOVis:
    def __init__(self):
        try:
            import matplotlib.pyplot as plt
            self._plt = plt
            self._fig, (self._at, self._af) = plt.subplots(1, 2, figsize=(13, 6))
            self._fig.suptitle("Package 2 – Visual-Inertial Odometry",
                                fontsize=13, fontweight="bold")
            self._at.set_title("Top-down trajectory")
            self._at.set_xlabel("X [m] (forward)")
            self._at.set_ylabel("Y [m] (left)")
            self._at.set_aspect("equal")
            self._at.grid(True, linestyle="--", alpha=0.5)
            self._ln_vio, = self._at.plot([], [], "b-", lw=2, label="VIO")
            self._ln_cur, = self._at.plot([], [], "ko", ms=8)
            self._at.legend()
            self._af.set_title("Camera + features")
            self._af.axis("off")
            self._im = None
            plt.tight_layout()
            plt.ion()
            plt.show()
            self._xs, self._ys = [], []
            self._ok = True
        except ImportError:
            self._ok = False

    def update(self, pos, feat_frame):
        if not self._ok:
            return
        self._xs.append(float(pos[0]))
        self._ys.append(float(pos[1]))
        self._ln_vio.set_data(self._xs, self._ys)
        self._ln_cur.set_data([pos[0]], [pos[1]])
        self._at.relim()
        self._at.autoscale_view()
        if feat_frame is not None:
            rgb = cv2.cvtColor(feat_frame, cv2.COLOR_BGR2RGB)
            if self._im is None:
                self._im = self._af.imshow(rgb)
            else:
                self._im.set_data(rgb)
        try:
            self._fig.canvas.flush_events()
            self._plt.pause(0.001)
        except Exception:
            pass

    def stop(self):
        if self._ok:
            self._plt.ioff()
            self._plt.close("all")


# ── Args ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="VIO Package 2")
    p.add_argument("--port",       default=None)
    p.add_argument("--baud",       default=115200, type=int)
    p.add_argument("--camera",     default=0,      type=int)
    p.add_argument("--params",     default="camera_params.npz")
    p.add_argument("--log",        default="vio_log.csv")
    p.add_argument("--out-npy",    default="vio_trajectory.npy",
                   help="Save trajectory as .npy for evaluation (t,x,y,yaw)")
    p.add_argument("--no-plot",    action="store_true")
    p.add_argument("--cam-width",  default=640, type=int)
    p.add_argument("--cam-height", default=480, type=int)
    p.add_argument("--kill-port",  action="store_true",
                   help="Free the serial port before opening")
    return p.parse_args()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    # Camera params must exist
    if not Path(args.params).exists():
        print(f"[vio_main] Camera params not found: {args.params}")
        print("           Run: python3 generate_default_params.py")
        sys.exit(1)

    K, dist, img_shape = load_params(args.params)
    print(f"[vio_main] Camera K: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    # Free port if requested
    if args.kill_port:
        import subprocess, shutil
        port = args.port or "/dev/ttyACM0"
        if shutil.which("fuser"):
            print(f"[vio_main] Freeing {port}…")
            subprocess.run(["sudo", "fuser", "-k", port], capture_output=True)
            time.sleep(1.0)

    # ── Subsystems ────────────────────────────────────────────────────────────
    ahrs    = MahonyAHRS(kp=2.0, ki=0.001)   # shared with VIOFusion
    tracker = FeatureTracker(K, dist, max_features=200, min_features=50)
    vo      = MonocularVO(K, ransac_thr=1.0, min_inliers=25)
    fusion  = VIOFusion(ahrs=ahrs)
    logger  = VIOLogger(args.log)
    vis     = VIOVis() if not args.no_plot else None

    # ── IMU receiver ──────────────────────────────────────────────────────────
    rx = IMUReceiver(port=args.port, baud=args.baud)
    rx.start()

    imu_q  = queue.Queue(maxsize=500)
    stop_ev = threading.Event()

    def imu_forwarder():
        for s in rx:
            if stop_ev.is_set():
                break
            try:
                imu_q.put_nowait(s)
            except queue.Full:
                try:
                    imu_q.get_nowait()
                except queue.Empty:
                    pass
                imu_q.put_nowait(s)

    threading.Thread(target=imu_forwarder, daemon=True).start()

    # ── Camera thread ─────────────────────────────────────────────────────────
    cam_q = queue.Queue(maxsize=5)
    threading.Thread(
        target=camera_thread,
        args=(args.camera, cam_q, stop_ev, args.cam_width, args.cam_height),
        daemon=True,
    ).start()

    # ── Warm-up ───────────────────────────────────────────────────────────────
    print("[vio_main] Keep robot still for ~1 s (warm-up)…")
    drained = 0
    t_start = time.time()
    while time.time() - t_start < 1.0:
        try:
            imu_q.get(timeout=0.05)
            drained += 1
        except queue.Empty:
            pass
    print(f"[vio_main] Drained {drained} warm-up IMU samples. Starting VIO.")

    # ── VIO loop ──────────────────────────────────────────────────────────────
    frame_count  = 0
    prev_imu_t   = None
    wall_start   = time.time()
    traj         = []   # accumulate [t, x, y, yaw] for .npy export

    def save_npy():
        """Save trajectory .npy — called periodically and on exit."""
        if traj:
            traj_np = np.array(traj, dtype=np.float64)
            np.save(args.out_npy, traj_np)
            print(f"[vio_main] Traj → {args.out_npy}  ({len(traj_np)} frames)")

    # Handle SIGTERM (kill command) so npy is saved even if killed
    import signal
    def _sigterm(sig, frame):
        print("\n[vio_main] SIGTERM received — saving and exiting.")
        save_npy()
        stop_ev.set()
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        while not stop_ev.is_set():

            # Wait for next camera frame
            try:
                cam_t, frame = cam_q.get(timeout=0.5)
            except queue.Empty:
                continue

            # Drain ALL IMU samples accumulated since last camera frame
            while True:
                try:
                    s = imu_q.get_nowait()
                    # Use hardware timestamp for accurate dt
                    hw_dt = (float(np.clip(s.t_s - prev_imu_t, 0.001, 0.05))
                             if prev_imu_t is not None else IMU_DT)
                    prev_imu_t = s.t_s
                    fusion.feed_imu(s.gyro, s.accel, hw_dt,
                                    stationary=s.hw_stationary)
                except queue.Empty:
                    break

            # Visual odometry
            pts_prev, pts_curr, n_tracks = tracker.process(frame)

            n_inliers = 0
            mode      = "IMU"

            if pts_prev is not None and n_tracks >= 25:
                ok, R_rel, t_rel, n_inliers = vo.process(
                    pts_prev, pts_curr,
                    scale=fusion.current_scale if fusion.current_scale > 0 else None,
                )
                if ok:
                    pos, vel, mode = fusion.update_fused(R_rel, t_rel, n_inliers)
                else:
                    pos, vel = fusion.update_imu_only()
            else:
                pos, vel = fusion.update_imu_only()

            # Log + display
            traj.append([cam_t, float(pos[0]), float(pos[1]), fusion.yaw])
            logger.write(cam_t, pos, vel, fusion.yaw,
                         fusion.current_scale, n_tracks, n_inliers, mode)

            if vis is not None:
                vis.update(pos, tracker.draw_tracks(frame))

            frame_count += 1
            if frame_count % 90 == 0:
                elapsed = time.time() - wall_start
                print(f"[vio_main] frame={frame_count:5d}  "
                      f"pos=({pos[0]:.2f},{pos[1]:.2f}) m  "
                      f"yaw={fusion.heading_deg:.1f}°  "
                      f"tracks={n_tracks}  inliers={n_inliers}  "
                      f"scale={fusion.current_scale:.3f}  "
                      f"mode={mode}  "
                      f"{frame_count/elapsed:.1f} fps")
                logger.flush()
            # Periodic npy save every 300 frames (~10 s at 30 fps)
            if frame_count % 300 == 0:
                save_npy()

    except KeyboardInterrupt:
        print("\n[vio_main] Stopped.")
    finally:
        stop_ev.set()
        rx.stop()
        logger.close()
        if vis:
            vis.stop()
        save_npy()   # always save on exit
        print(f"[vio_main] {frame_count} frames processed. Log → {args.log}")


if __name__ == "__main__":
    main()
