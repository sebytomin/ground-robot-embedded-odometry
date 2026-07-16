"""
vio_fusion.py  –  Package 2
------------------------------
Loosely-coupled Visual-Inertial Odometry fusion.


USAGE
-----
    ahrs   = MahonyAHRS(kp=2.0, ki=0.001)
    fusion = VIOFusion(ahrs=ahrs)

    # 100 Hz IMU loop:
    fusion.feed_imu(gyro, accel, dt, stationary=hw_stat)

    # 30 Hz camera loop:
    if vo_ok:
        pos, vel, mode = fusion.update_fused(R_rel, t_rel, n_inliers)
    else:
        pos, vel = fusion.update_imu_only()
"""

import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'package1'))

try:
    from ahrs import MahonyAHRS
    _AHRS_OK = True
except ImportError:
    _AHRS_OK = False


class VIOFusion:
    G                = 9.81
    MIN_VO_INLIERS   = 20
    SCALE_LPF        = 0.04
    SCALE_MIN_FRAMES = 10
    VO_POS_ALPHA     = 0.55

    # Software ZUPT thresholds — mirrors dead_reckoning.py exactly
    ZUPT_ACC_VAR  = 0.5    # (m/s²)² variance of |accel| - G
    ZUPT_GYRO_MAX = 0.15   # rad/s  max raw gyro magnitude
    ZUPT_WIN      = 30     # samples (300 ms at 100 Hz)

    # Camera-to-body extrinsic rotation.
    # recoverPose returns t in CAMERA frame (Z=forward, X=right, Y=down).
    # Robot body frame: X=forward, Y=left, Z=up.
    # Assumes camera mounted horizontally, facing forward, not tilted.
    R_CAM_TO_BODY = np.array([
        [ 0,  0,  1],   # body-X = cam-Z (forward)
        [-1,  0,  0],   # body-Y = -cam-X (left = -right)
        [ 0, -1,  0],   # body-Z = -cam-Y (up = -down)
    ], dtype=np.float64)

    def __init__(self, ahrs=None):
        if ahrs is not None:
            self._ahrs = ahrs
        elif _AHRS_OK:
            self._ahrs = MahonyAHRS(kp=2.0, ki=0.001)
        else:
            self._ahrs = None

        self.position       = np.zeros(3)
        self.velocity       = np.zeros(3)
        self.current_scale  = 0.0
        self._scale_frames  = 0
        self._is_stationary = False
        self._last_gz       = 0.0   # for pure-rotation gate
        self._R_world       = np.eye(3)

        # Software ZUPT sliding windows — same algorithm as Package 1
        self._acc_win  = []
        self._gyro_win = []

        # Per-frame IMU accumulators — reset each camera frame
        self._dv   = np.zeros(3)   # velocity delta
        self._dp   = np.zeros(3)   # position delta
        self._dt   = 0.0
        self._n    = 0

    # ── properties ────────────────────────────────────────────────────────────

    @property
    def yaw(self):
        if self._ahrs is not None:
            return float(self._ahrs.euler[2])
        return float(np.arctan2(self._R_world[1, 0], self._R_world[0, 0]))

    @property
    def heading_deg(self):
        return float(np.degrees(self.yaw))

    @property
    def euler(self):
        if self._ahrs is not None:
            return self._ahrs.euler
        return np.zeros(3)

    # ── IMU feed — 100 Hz ─────────────────────────────────────────────────────

    def feed_imu(self, gyro: np.ndarray, accel: np.ndarray,
                 dt: float, stationary: bool = False):
        """
        Update AHRS and accumulate IMU pre-integration delta.
        Call every IMU sample (100 Hz).
        """
        self._is_stationary = stationary
        self._last_gz       = float(gyro[2])

        # ── Software ZUPT (same as Package 1) ────────────────────────────────
        # Hardware flag alone is not reliable — supplement with sliding-window
        # accel variance + raw gyro magnitude check, exactly as dead_reckoning.py.
        a_norm = float(np.linalg.norm(accel))
        g_norm = float(np.linalg.norm(gyro))   # raw gyro
        self._acc_win.append(a_norm - self.G)
        self._gyro_win.append(g_norm)
        if len(self._acc_win) > self.ZUPT_WIN:
            self._acc_win.pop(0)
            self._gyro_win.pop(0)
        sw_stat = self._sw_zupt()
        self._is_stationary = stationary or sw_stat

        # Update shared AHRS
        if self._ahrs is not None:
            self._ahrs.update(gyro, accel, dt, stationary=stationary)

        # World rotation from current AHRS yaw
        yaw = self.yaw
        cy, sy = np.cos(yaw), np.sin(yaw)
        self._R_world = np.array([
            [ cy, -sy, 0.0],
            [ sy,  cy, 0.0],
            [0.0, 0.0, 1.0],
        ])

        # Rotate accel to world frame, subtract gravity, zero vertical
        R = self._ahrs.R if self._ahrs is not None else self._R_world
        a_world   = R @ accel
        a_linear  = a_world - np.array([0.0, 0.0, self.G])
        a_linear[2] = 0.0

        # Accumulate (zero during ZUPT)
        if stationary:
            self._dv[:] = 0.0
            self._dp[:] = 0.0
        else:
            self._dv   += a_linear * dt
            self._dv[2] = 0.0
            self._dp   += self._dv * dt + 0.5 * a_linear * dt**2
            self._dp[2] = 0.0

        self._dt += dt
        self._n  += 1

    # ── Camera frame update — 30 Hz ───────────────────────────────────────────

    def update_fused(self, R_rel: np.ndarray, t_rel: np.ndarray,
                     n_inliers: int = 0):
        """
        Fuse VO pose with accumulated IMU delta for this frame interval.

        Returns
        -------
        position, velocity, mode_str
        """
        dt = self._dt if self._dt > 1e-6 else 1.0 / 30.0

        # ── IMU prediction ────────────────────────────────────────────────────
        if self._is_stationary:
            self.velocity[:] = 0.0
            imu_pos = self.position.copy()
        else:
            imu_vel = self.velocity + self._dv
            imu_vel[2] = 0.0
            spd = float(np.linalg.norm(imu_vel[:2]))
            if spd > 2.0:
                imu_vel[:2] *= 2.0 / spd
            imu_pos = self.position + self._dp
            imu_pos[2] = 0.0
            self.velocity = imu_vel

        # ── Scale estimation ──────────────────────────────────────────────────
        imu_speed = float(np.linalg.norm(self.velocity[:2]))
        # Convert t_rel from camera frame to body frame, then take XY magnitude
        t_body    = self.R_CAM_TO_BODY @ t_rel
        vo_t_norm = float(np.linalg.norm(t_body[:2]))

        # Gate: reject VO during pure rotation (IMU spinning, not translating)
        pure_rotation = (imu_speed < 0.05 and abs(self._last_gz) > 0.15)

        vo_trusted = (n_inliers >= self.MIN_VO_INLIERS
                      and vo_t_norm > 1e-4
                      and not self._is_stationary
                      and not pure_rotation)

        # Use IMU position delta magnitude for scale — more accurate than
        # velocity * dt because it directly measures how far the robot moved.
        imu_dist = float(np.linalg.norm(self._dp[:2]))

        if vo_trusted and imu_dist > 0.005:   # robot moved at least 5mm
            scale_est = imu_dist / vo_t_norm
            if 0.001 < scale_est < 10.0:
                if self.current_scale < 0.001:
                    self.current_scale = scale_est
                    self._scale_frames += 1
                else:
                    ratio = scale_est / self.current_scale
                    if 0.5 < ratio < 2.0:   # ±50% clamp
                        self.current_scale = ((1 - self.SCALE_LPF) * self.current_scale
                                              + self.SCALE_LPF * scale_est)
                        self._scale_frames += 1

        # ── VO position correction ────────────────────────────────────────────
        scale_ok = (self.current_scale > 0.001
                    and self._scale_frames >= self.SCALE_MIN_FRAMES)

        if vo_trusted and scale_ok:
            # t_body is already in robot body frame — rotate to world frame
            # Use full AHRS rotation matrix (not just yaw) for correct projection
            R_body_to_world = self._ahrs.R if self._ahrs is not None else self._R_world
            t_world    = R_body_to_world @ (t_body * self.current_scale)
            t_world[2] = 0.0   # ground robot
            vo_pos     = self.position + t_world
            self.position = (self.VO_POS_ALPHA * vo_pos
                             + (1 - self.VO_POS_ALPHA) * imu_pos)
            mode = "VIO"
        else:
            self.position = imu_pos
            mode = "IMU"

        self.position[2] = 0.0
        self._reset_accum()
        return self.position.copy(), self.velocity.copy(), mode

    def update_imu_only(self):
        """IMU-only fallback when VO fails. Heading still from AHRS."""
        if self._is_stationary:
            self.velocity[:] = 0.0
        else:
            imu_vel = self.velocity + self._dv
            imu_vel[2] = 0.0
            spd = float(np.linalg.norm(imu_vel[:2]))
            if spd > 2.0:
                imu_vel[:2] *= 2.0 / spd
            self.velocity  = imu_vel
            self.position += self._dp
            self.position[2] = 0.0

        self._reset_accum()
        return self.position.copy(), self.velocity.copy()

    def reset(self):
        self.position[:]    = 0.0
        self.velocity[:]    = 0.0
        self.current_scale  = 0.0
        self._scale_frames  = 0
        self._acc_win       = []
        self._gyro_win      = []
        if self._ahrs is not None:
            self._ahrs.reset()
        self._reset_accum()

    def _sw_zupt(self) -> bool:
        """Software ZUPT — same thresholds as Package 1 DeadReckoning."""
        if len(self._acc_win) < self.ZUPT_WIN:
            return False
        acc_var  = float(np.var(self._acc_win))
        gyro_max = float(np.max(self._gyro_win))
        return acc_var < self.ZUPT_ACC_VAR and gyro_max < self.ZUPT_GYRO_MAX

    def _reset_accum(self):
        self._dv[:] = 0.0
        self._dp[:] = 0.0
        self._dt    = 0.0
        self._n     = 0
