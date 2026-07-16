"""
visualizer.py  (Package 1)
--------------------------
Real-time matplotlib visualisation of the dead-reckoning state.

Two panels
    Left   2-D top-down trajectory  (X forward, Y left)
    Right  time-series of roll / pitch / yaw

Usage
-----
    vis = Visualizer()
    vis.start()          # opens the window in the main thread
    ...
    vis.update(pos, vel, euler)
    vis.stop()

"""

import numpy as np
import threading
import time

try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    from matplotlib.patches import FancyArrow
    _MATPLOTLIB_OK = True
except ImportError:
    _MATPLOTLIB_OK = False
    print("[Visualizer] matplotlib not available – plotting disabled")


class Visualizer:
    def __init__(self, max_trail=2000, update_hz=20):
        self._max_trail  = max_trail
        self._update_hz  = update_hz
        self._lock       = threading.Lock()
        self._running    = False

        # Buffers
        self._xs   = []
        self._ys   = []
        self._ts   = []
        self._roll = []
        self._pitch= []
        self._yaw  = []
        self._stationary_flags = []

        self._current_pos   = np.zeros(3)
        self._current_euler = np.zeros(3)
        self._is_stationary = False

    # ── public ────────────────────────────────────────────────────────────────

    def update(self, position, velocity, euler, is_stationary=False, t=None):
        """Thread-safe state update."""
        with self._lock:
            self._current_pos   = np.array(position)
            self._current_euler = np.array(euler)
            self._is_stationary = is_stationary

            if t is None:
                t = time.time()

            self._xs.append(position[0])
            self._ys.append(position[1])
            self._ts.append(t)
            self._roll.append(np.degrees(euler[0]))
            self._pitch.append(np.degrees(euler[1]))
            self._yaw.append(np.degrees(euler[2]))
            self._stationary_flags.append(is_stationary)

            # Trim to max trail length
            if len(self._xs) > self._max_trail:
                self._xs   = self._xs[-self._max_trail:]
                self._ys   = self._ys[-self._max_trail:]
                self._ts   = self._ts[-self._max_trail:]
                self._roll = self._roll[-self._max_trail:]
                self._pitch= self._pitch[-self._max_trail:]
                self._yaw  = self._yaw[-self._max_trail:]
                self._stationary_flags = self._stationary_flags[-self._max_trail:]

    def start(self):
        """Start the plot (must be called from the main thread on most systems)."""
        if not _MATPLOTLIB_OK:
            return
        self._running = True
        self._build_figure()
        self._animate()

    def stop(self):
        self._running = False

    # ── private ───────────────────────────────────────────────────────────────

    def _build_figure(self):
        self._fig = plt.figure(figsize=(13, 6))
        self._fig.suptitle("Package 1 – IMU Dead Reckoning", fontsize=13, fontweight="bold")
        gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1.3])

        # Left: 2D trajectory
        self._ax_traj = self._fig.add_subplot(gs[0])
        self._ax_traj.set_xlabel("X  [m]  (forward)")
        self._ax_traj.set_ylabel("Y  [m]  (left)")
        self._ax_traj.set_title("Top-down trajectory")
        self._ax_traj.set_aspect("equal")
        self._ax_traj.grid(True, linestyle="--", alpha=0.5)
        self._line_traj,   = self._ax_traj.plot([], [], "b-", lw=1.5, label="trajectory")
        self._scat_still,  = self._ax_traj.plot([], [], "rs", ms=4,   label="ZUPT")
        self._scat_curr,   = self._ax_traj.plot([], [], "ko", ms=8,   label="current")
        self._ax_traj.legend(loc="upper left", fontsize=8)

        # Right: attitude time series
        self._ax_att = self._fig.add_subplot(gs[1])
        self._ax_att.set_xlabel("sample index")
        self._ax_att.set_ylabel("angle  [°]")
        self._ax_att.set_title("Orientation (Mahony AHRS)")
        self._ax_att.grid(True, linestyle="--", alpha=0.5)
        self._line_roll,  = self._ax_att.plot([], [], "r-",  lw=1, label="roll")
        self._line_pitch, = self._ax_att.plot([], [], "g-",  lw=1, label="pitch")
        self._line_yaw,   = self._ax_att.plot([], [], "b-",  lw=1, label="yaw")
        self._ax_att.legend(loc="upper left", fontsize=8)

        plt.tight_layout()
        plt.ion()
        plt.show()

    def _animate(self):
        interval = 1.0 / self._update_hz
        while self._running:
            with self._lock:
                xs    = list(self._xs)
                ys    = list(self._ys)
                stfl  = list(self._stationary_flags)
                rolls = list(self._roll)
                pitches=list(self._pitch)
                yaws  = list(self._yaw)
                pos   = self._current_pos.copy()

            if xs:
                # Trajectory
                self._line_traj.set_data(xs, ys)
                sx = [x for x, f in zip(xs, stfl) if f]
                sy = [y for y, f in zip(ys, stfl) if f]
                self._scat_still.set_data(sx, sy)
                self._scat_curr.set_data([pos[0]], [pos[1]])
                self._ax_traj.relim()
                self._ax_traj.autoscale_view()

                # Attitude
                idx = list(range(len(rolls)))
                self._line_roll.set_data(idx,  rolls)
                self._line_pitch.set_data(idx, pitches)
                self._line_yaw.set_data(idx,   yaws)
                self._ax_att.relim()
                self._ax_att.autoscale_view()

            try:
                self._fig.canvas.flush_events()
                plt.pause(interval)
            except Exception:
                break

        plt.ioff()
        plt.close(self._fig)
