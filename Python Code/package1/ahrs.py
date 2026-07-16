"""
ahrs.py  –  Package 1
---------------------
"""

import numpy as np


class MahonyAHRS:
    def __init__(self, kp=2.0, ki=0.001, dt=0.01):
        self.kp   = kp
        self.ki   = ki
        self.dt   = dt
        self._q   = np.array([1.0, 0.0, 0.0, 0.0])
        self._int = np.zeros(3)

    @property
    def euler(self):
        return _tilt_euler(self._q)

    @property
    def R(self):
        return _quat_R(self._q)

    @property
    def q(self):
        return self._q.copy()

    def update(self, gyro, accel, dt=None, stationary=False):
        dt = dt if dt is not None else self.dt
        q  = self._q.copy()
        gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
        ax, ay, az = float(accel[0]), float(accel[1]), float(accel[2])

        # ── Stationary: freeze world-yaw component of gyro ────────────────────
        if stationary:
            R = _quat_R(q)
            gw = R @ np.array([gx, gy, gz])
            gw[2] = 0.0                # zero vertical (yaw) component
            gb = R.T @ gw              # back to body frame
            gx, gy, gz = float(gb[0]), float(gb[1]), float(gb[2])
            self._int[2] = 0.0         # prevent Z integral buildup

        # ── Accel correction — roll/pitch only ────────────────────────────────
        norm = float(np.sqrt(ax*ax + ay*ay + az*az))
        # Gate: only use accel if magnitude is close to 1g (9.81 m/s²).
        # Far from 1g means the sensor is in motion or the reading is corrupt.
        # Acceptable range: 0.5g–1.5g (4.9–14.7 m/s²)
        accel_ok = (norm > 4.9 and norm < 14.7)
        if accel_ok:
            ax /= norm; ay /= norm; az /= norm
            # Estimated gravity in body frame from quaternion
            vx = 2.0*(q[1]*q[3] - q[0]*q[2])
            vy = 2.0*(q[0]*q[1] + q[2]*q[3])
            vz = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
            # Cross-product error
            ex = ay*vz - az*vy
            ey = az*vx - ax*vz
            ez = ax*vy - ay*vx
            # Integral
            self._int += np.array([ex, ey, ez]) * self.ki * dt
            self._int  = np.clip(self._int, -0.1, 0.1)
            if stationary:
                self._int[2] = 0.0
            gx += self.kp*ex + self._int[0]
            gy += self.kp*ey + self._int[1]
            gz += self.kp*ez + self._int[2]

        # ── Clamp gyro to physical limit before integrating ───────────────────
        MAX_G = 2.0   # rad/s — matches imu_receiver GYRO_MAX
        gx = float(np.clip(gx, -MAX_G, MAX_G))
        gy = float(np.clip(gy, -MAX_G, MAX_G))
        gz = float(np.clip(gz, -MAX_G, MAX_G))

        # ── Quaternion integration ────────────────────────────────────────────
        gx *= 0.5*dt; gy *= 0.5*dt; gz *= 0.5*dt
        qa, qb, qc, qd = q[0], q[1], q[2], q[3]
        q[0] += -qb*gx - qc*gy - qd*gz
        q[1] +=  qa*gx + qc*gz - qd*gy
        q[2] +=  qa*gy - qb*gz + qd*gx
        q[3] +=  qa*gz + qb*gy - qc*gx

        n = float(np.linalg.norm(q))
        if n < 1e-10:
            return   # degenerate, keep old
        q_new = q / n

        # ── Reject physically impossible orientation jumps ────────────────────
        # Check all three angles — a spike can corrupt roll/pitch too.
        # At 100 Hz, even the fastest real robot turn is <5°/sample.
        # 10° per sample = 1000°/s — impossible for a ground robot.
        euler_prev = _tilt_euler(self._q)
        euler_new  = _tilt_euler(q_new)
        for ep, en in zip(euler_prev, euler_new):
            dang = abs(float(np.arctan2(np.sin(en - ep), np.cos(en - ep))))
            if dang > np.radians(10):   # >10° per 10ms step → spike
                return                   # discard, keep old quaternion

        # Antipodal fix
        if np.dot(q_new, self._q) < 0.0:
            q_new = -q_new
        self._q = q_new

    def reset(self):
        self._q   = np.array([1.0, 0.0, 0.0, 0.0])
        self._int = np.zeros(3)


def _tilt_euler(q):
    """Tilt-compensated Euler angles — yaw correct at any board tilt."""
    w, x, y, z = q
    # Project body-X axis to world frame, take horizontal angle = yaw
    hx  = 1.0 - 2.0*(y*y + z*z)
    hy  = 2.0*(x*y + w*z)
    yaw = np.arctan2(hy, hx)
    # Roll and pitch from standard ZYX decomposition
    roll  = np.arctan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    pitch = np.arcsin(np.clip(2.0*(w*y - z*x), -1.0, 1.0))
    return np.array([roll, pitch, yaw])


def _quat_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


# Backward-compat aliases
def quat_to_euler(q): return _tilt_euler(q)
def quat_to_R(q):     return _quat_R(q)
