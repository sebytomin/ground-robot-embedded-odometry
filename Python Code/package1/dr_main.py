"""
dr_main.py  –  Package 1 Main Entry Point
------------------------------------------
IMU Dead Reckoning for a ground robot using the STM32F3Discovery.

The STM32 firmware (Project2scratch3) streams:
    timestamp_ms, ax, ay, az, gx, gy, gz   at 100 Hz over USB CDC

This script:
    1. Reads IMU samples from the STM32
    2. Runs Mahony AHRS + ZUPT dead reckoning
    3. Logs trajectory to CSV + NPY
    4. Shows real-time visualisation (visualizer.py)

Usage
-----
    # Auto-detect STM32 port
    python dr_main.py

    # Specify port
    python dr_main.py --port /dev/ttyACM0

    # Headless (no plot)
    python dr_main.py --no-plot

    # Replay a recorded dataset
    python dr_main.py --replay dataset.csv

Output
------
    dr_log.csv          – full timestamped log
"""

import argparse
import csv
import time
import sys
import numpy as np
from pathlib import Path

from imu_receiver   import IMUReceiver
from dead_reckoning import DeadReckoning
from visualizer     import Visualizer


IMU_DT = 0.01   # 100 Hz


def parse_args():
    p = argparse.ArgumentParser(description="Package 1 – IMU Dead Reckoning")
    p.add_argument("--port",    default=None, help="STM32 serial port (auto if omitted)")
    p.add_argument("--baud",    default=115200, type=int)
    p.add_argument("--log",     default="dr_log.csv")
    p.add_argument("--out-npy", default="dr_trajectory.npy")
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--replay",  default=None,
                   help="Replay a CSV dataset (offline mode, no hardware needed)")
    p.add_argument("--kill-port", action="store_true",
                   help="Free the serial port before opening (kills other processes using it)")
    return p.parse_args()


# ── CSV logger ────────────────────────────────────────────────────────────────

class DRLogger:
    HEADER = ["t_s", "px", "py", "pz",
              "vx", "vy", "vz",
              "roll_deg", "pitch_deg", "yaw_deg",
              "ax", "ay", "az", "gx", "gy", "gz",
              "bias_x", "bias_y", "bias_z",
              "stationary"]

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)

    def write(self, t, pos, vel, euler, accel, gyro, bias, stationary):
        self._w.writerow([
            f"{t:.4f}",
            f"{pos[0]:.5f}", f"{pos[1]:.5f}", f"{pos[2]:.5f}",
            f"{vel[0]:.5f}", f"{vel[1]:.5f}", f"{vel[2]:.5f}",
            f"{np.degrees(euler[0]):.3f}",
            f"{np.degrees(euler[1]):.3f}",
            f"{np.degrees(euler[2]):.3f}",
            f"{accel[0]:.5f}", f"{accel[1]:.5f}", f"{accel[2]:.5f}",
            f"{gyro[0]:.6f}", f"{gyro[1]:.6f}", f"{gyro[2]:.6f}",
            f"{bias[0]:.6f}", f"{bias[1]:.6f}", f"{bias[2]:.6f}",
            int(stationary),
        ])

    def flush(self):
        self._f.flush()

    def close(self):
        self._f.close()


# ── Replay mode ───────────────────────────────────────────────────────────────

def replay_dataset(csv_path: str):
    """
    Yield (t_s, accel, gyro) tuples from a recorded dataset CSV.
    Expected columns: t_s, ax, ay, az, gx, gy, gz   (or full dr_log.csv)
    """
    import csv as _csv
    with open(csv_path, newline="") as f:
        reader = _csv.DictReader(f)
        for row in reader:
            try:
                t  = float(row["t_s"])
                ax = float(row.get("ax", row.get("accel_x", 0)))
                ay = float(row.get("ay", row.get("accel_y", 0)))
                az = float(row.get("az", row.get("accel_z", 0)))
                gx = float(row.get("gx", row.get("gyro_x", 0)))
                gy = float(row.get("gy", row.get("gyro_y", 0)))
                gz = float(row.get("gz", row.get("gyro_z", 0)))
                yield t, np.array([ax, ay, az]), np.array([gx, gy, gz])
            except (KeyError, ValueError):
                continue


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    dr     = DeadReckoning(dt=IMU_DT)
    logger = DRLogger(args.log)
    traj   = []

    # ── Free port if requested ─────────────────────────────────────────────────
    if args.kill_port and not args.replay:
        import subprocess, shutil
        port = args.port or "[auto]"
        if shutil.which("fuser"):
            p = args.port or "/dev/ttyACM0"
            print(f"[dr_main] Freeing port {p} …")
            subprocess.run(["sudo", "fuser", "-k", p],
                           capture_output=True)
            time.sleep(1.0)
        else:
            print("[dr_main] --kill-port: 'fuser' not found, skipping.")          # list of [ts, x, y, yaw]

    vis = None
    if not args.no_plot:
        vis = Visualizer()

    print("[dr_main] Starting Package 1 – IMU Dead Reckoning")

    # ── REPLAY MODE ───────────────────────────────────────────────────────────
    if args.replay:
        print(f"[dr_main] Replay mode: {args.replay}")
        samples = list(replay_dataset(args.replay))
        print(f"[dr_main] Loaded {len(samples)} samples. Running…")

        prev_t = None
        for i, (t, accel, gyro) in enumerate(samples):
            dt = (t - prev_t) if prev_t is not None else IMU_DT
            dt = np.clip(dt, 0.001, 0.1)
            prev_t = t

            dr.update(gyro, accel, dt, hw_stationary=False)  # replay: no hw flag

            euler = dr.euler
            traj.append([t, dr.position[0], dr.position[1], euler[2]])

            logger.write(t, dr.position, dr.velocity, euler,
                         accel, gyro, dr.gyro_bias, dr.is_stationary)

            if vis is not None:
                vis.update(dr.position, dr.velocity, euler,
                           is_stationary=dr.is_stationary, t=t)

            if (i+1) % 500 == 0:
                print(f"  [{i+1:5d}/{len(samples)}]  "
                      f"pos=({dr.position[0]:.2f},{dr.position[1]:.2f}) m  "
                      f"yaw={np.degrees(euler[2]):.1f}°  "
                      f"stat={dr.is_stationary}")

        if vis is not None:
            print("[dr_main] Close the plot window to exit.")
            vis.start()   # blocks

    # ── LIVE MODE ─────────────────────────────────────────────────────────────
    else:
        rx = IMUReceiver(port=args.port, baud=args.baud)
        rx.start()

        print("[dr_main] Keep robot STILL for 2 s (warm-up / bias collection)…")
        time.sleep(2.0)
        # Drain warm-up samples
        count = 0
        while True:
            s = rx.get_nowait()
            if s is None:
                break
            count += 1
        print(f"[dr_main] Drained {count} warm-up samples. Running.")

        frame_count = 0
        prev_t      = None
        prev_hw_t   = None   # STM32 hardware timestamp for dt calculation

        try:
            for sample in rx:
                # ── Use hardware timestamp for dt ──────────────────────────────
                # sample.t_s is derived from STM32

                hw_t = sample.t_s
                if prev_hw_t is None:
                    dt = IMU_DT
                else:
                    dt = hw_t - prev_hw_t
                    dt = float(np.clip(dt, 0.001, 0.05))  # sanity bounds
                prev_hw_t = hw_t

                dr.update(sample.gyro, sample.accel, dt,
                          hw_stationary=sample.hw_stationary)

                euler = dr.euler
                traj.append([hw_t, dr.position[0], dr.position[1], euler[2]])

                logger.write(hw_t, dr.position, dr.velocity, euler,
                             sample.accel, sample.gyro, dr.gyro_bias, dr.is_stationary)

                if vis is not None:
                    vis.update(dr.position, dr.velocity, euler,
                               is_stationary=dr.is_stationary, t=hw_t)

                frame_count += 1
                if frame_count % 500 == 0:
                    logger.flush()
                    print(f"[dr_main] {frame_count:6d} samples  "
                          f"pos=({dr.position[0]:.2f},{dr.position[1]:.2f}) m  "
                          f"yaw={np.degrees(euler[2]):.1f}°  "
                          f"stat={dr.is_stationary}  "
                          f"bias=({dr.gyro_bias[0]*1000:.2f},{dr.gyro_bias[1]*1000:.2f},{dr.gyro_bias[2]*1000:.2f}) mrad/s")

                # If visualiser is used, run animation from main thread
                if vis is not None and frame_count == 1:

                    import threading
                    vis_thread = threading.Thread(target=vis.start, daemon=True)
                    vis_thread.start()

        except KeyboardInterrupt:
            print("\n[dr_main] Stopped by user.")
        finally:
            rx.stop()

    # ── Save trajectory ───────────────────────────────────────────────────────
    logger.close()
    traj_np = np.array(traj, dtype=np.float64)
    np.save(args.out_npy, traj_np)
    print(f"\n[dr_main] Log      → {args.log}")
    print(f"[dr_main] Traj NPY → {args.out_npy}  ({len(traj_np)} samples)")


if __name__ == "__main__":
    main()
