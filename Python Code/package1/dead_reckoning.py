"""
dead_reckoning.py  –  Package 1
---------------------------------
IMU Dead Reckoning with Zero-Velocity Update (ZUPT) + runtime gyro bias
estimation for a ground robot.

Pipeline
--------
1. Subtract runtime gyro bias estimate
2. Mahony AHRS  – fuse corrected gyro + accel → orientation (R)
3. Gravity subtraction  – remove gravity from accelerometer
4. ZUPT detection  – sliding window on accel variance + gyro magnitude
5. Velocity integration – zeroed during ZUPT
6. Position integration

"""

import numpy as np
from ahrs import MahonyAHRS


class ZUPTBiasEstimator:
    """

    alpha : low-pass smoothing factor
            0.01 = very slow adaptation (stable, lags behind fast drift)
            0.10 = faster adaptation   (follows drift better)
    """

    # L3GD20 at ±250 dps: max bias is physically bounded.
    # Any bias estimate beyond ±1 rad/s (~57°/s) is wrong — clamp it.
    MAX_BIAS = 1.0   # rad/s per axis

    def __init__(self, alpha: float = 0.02):
        self.alpha        = alpha
        self.bias         = np.zeros(3)
        self._buf         = []
        self._min_samples = 20
        self._commit_every = 100
        self._sample_count = 0

    def accumulate(self, gyro: np.ndarray):
        """Call every sample while ZUPT is active."""
        self._buf.append(gyro.copy())
        self._sample_count += 1
        if self._sample_count >= self._commit_every:
            self._periodic_commit()
            self._sample_count = 0

    def _periodic_commit(self):
        if len(self._buf) < self._min_samples:
            return
        window_mean = np.mean(self._buf, axis=0)
        # Sanity check: if window_mean is absurdly large, the buffer is corrupt
        if np.any(np.abs(window_mean) > self.MAX_BIAS):
            self._buf = []   # discard corrupt window
            return
        alpha = min(self.alpha * 3.0, 0.15)
        new_bias = (1.0 - alpha) * self.bias + alpha * window_mean
        # Hard clamp: bias cannot exceed physical sensor limits
        self.bias = np.clip(new_bias, -self.MAX_BIAS, self.MAX_BIAS)

    def commit(self):
        """Called at stationary→moving transition."""
        if len(self._buf) < self._min_samples:
            self._buf = []; self._sample_count = 0
            return
        window_mean = np.mean(self._buf, axis=0)
        if np.any(np.abs(window_mean) > self.MAX_BIAS):
            self._buf = []; self._sample_count = 0
            return
        new_bias = (1.0 - self.alpha) * self.bias + self.alpha * window_mean
        self.bias = np.clip(new_bias, -self.MAX_BIAS, self.MAX_BIAS)
        self._buf = []; self._sample_count = 0

    def reset_buf(self):
        self._buf = []; self._sample_count = 0

    def correct(self, gyro: np.ndarray) -> np.ndarray:
        return gyro - self.bias


class DeadReckoning:
    """
    Strapdown inertial navigator with:
      - Mahony AHRS
      - Runtime gyro bias estimation via ZUPT windows
      - Yaw freeze during stationary phases
      - ZUPT velocity zeroing
    """

    # ── ZUPT detection thresholds ─────────────────────────────────────────────
    # Loosened to tolerate Jetson Nano cooling fan + motor chassis vibrations.
    # If ZUPT never triggers, the arcing drift when stopped will not be corrected.
    # If ZUPT is too sensitive, it fires while moving and loses position.
    #
    # Accel variance: spread of (|accel_m/s²| - 9.81) over window
    ZUPT_ACC_VAR_THR  = 0.5    # (m/s²)²
    # Gyro magnitude: max |gyro| in window (rad/s).
    # Raised to 0.08 (~4.6°/s) — at 30° tilt, board vibrations on chassis
    # project more strongly onto the gyro axes than when level.
    ZUPT_GYRO_THR     = 0.12   # rad/s raw — covers bias + noise floor
    # Window length: 30 samples @ 100 Hz = 300 ms
    ZUPT_WINDOW       = 30

    def __init__(self, dt=0.01, ahrs_kp=2.0, ahrs_ki=0.001,
                 bias_alpha=0.02):
        self.dt            = dt
        self._ahrs         = MahonyAHRS(kp=ahrs_kp, ki=ahrs_ki, dt=dt)
        self._bias_est     = ZUPTBiasEstimator(alpha=bias_alpha)

        # State
        self.position      = np.zeros(3)
        self.velocity      = np.zeros(3)

        # ZUPT detector history
        self._acc_buf      = []
        self._gyro_buf     = []

        # Track ZUPT transitions to commit bias on motion start
        self._prev_stationary = False

        # Public flags / diagnostics
        self.is_stationary = False
        self.gyro_bias     = np.zeros(3)   # exposed for logging/display

        # Gravity
        self.G = 9.81

    # ── public ────────────────────────────────────────────────────────────────

    @property
    def euler(self):
        return self._ahrs.euler

    @property
    def R(self):
        return self._ahrs.R

    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float = None,
               hw_stationary: bool = False):
        """
        Process one IMU sample.

        Parameters
        ----------
        gyro           : (3,) rad/s   (raw, from STM32)
        accel          : (3,) m/s²    (body frame, already in m/s² from firmware)
        dt             : override time step [s]
        hw_stationary  : hardware ZUPT flag from STM32 (8th CSV field).
                         When True, overrides the software ZUPT detector.
        """
        dt = dt if dt is not None else self.dt

        # ── 0. Subtract runtime bias estimate ─────────────────────────────────
        gyro_corrected = self._bias_est.correct(gyro)
        self.gyro_bias = self._bias_est.bias.copy()

        # ── 1. ZUPT detection ──────────────────────────────────────────────────

        self._acc_buf.append(np.linalg.norm(accel) - self.G)
        self._gyro_buf.append(np.linalg.norm(gyro))   # RAW gyro, not corrected
        if len(self._acc_buf) > self.ZUPT_WINDOW:
            self._acc_buf.pop(0)
            self._gyro_buf.pop(0)

        sw_stationary = self._detect_zupt()
        self.is_stationary = hw_stationary or sw_stationary

        # ── 2. Bias accumulation  ────────────────────────────────
        if self.is_stationary:
            # Accumulate raw gyro (the bias is whatever the gyro reads at rest)
            self._bias_est.accumulate(gyro)
        else:
            if self._prev_stationary:
                # Transition: stationary → moving → commit the bias estimate
                self._bias_est.commit()
            else:
                self._bias_est.reset_buf()

        self._prev_stationary = self.is_stationary

        # ── 3. AHRS update ─────────────────────────────────────────────────────
        self._ahrs.update(gyro_corrected, accel, dt,
                          stationary=self.is_stationary)
        R = self._ahrs.R                   # body → world

        # ── 4. Gravity removal ─────────────────────────────────────────────────

        a_world  = R @ accel
        gravity  = np.array([0.0, 0.0, self.G])
        a_lin    = a_world - gravity

        # Ground-robot planar constraint: zero the TRUE vertical component.

        a_lin[2] = 0.0

        # ── 5. Velocity integration ────────────────────────────────────────────
        if self.is_stationary:
            self.velocity = np.zeros(3)    # hard zero – no creep
        else:
            self.velocity += a_lin * dt
            self.velocity[2] = 0.0

            # Speed cap: ground robot should not exceed 3 m/s
            speed = np.linalg.norm(self.velocity[:2])
            if speed > 3.0:
                self.velocity[:2] *= 3.0 / speed

        # ── 6. Position integration ────────────────────────────────────────────
        self.position += self.velocity * dt

    def reset(self):
        self._ahrs.reset()
        self.position          = np.zeros(3)
        self.velocity          = np.zeros(3)
        self._acc_buf          = []
        self._gyro_buf         = []
        self._prev_stationary  = False


    def _detect_zupt(self) -> bool:
        if len(self._acc_buf) < self.ZUPT_WINDOW:
            return False
        acc_var  = float(np.var(self._acc_buf))
        gyro_max = float(np.max(self._gyro_buf))
        # Simple AND: both sensors must agree the robot is still.
        # Using raw gyro so threshold must cover the bias (~0.01–0.05 rad/s).
        return acc_var < self.ZUPT_ACC_VAR_THR and gyro_max < self.ZUPT_GYRO_THR
