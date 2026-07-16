"""
imu_preintegration.py  –  Package 2
--------------------------------------
Simple IMU pre-integration between two camera frames.


Usage
-----
    pint = IMUPreintegrator()
    pint.integrate(gyro, accel, dt)    # call for each IMU sample
    delta = pint.get_delta()           # read result
    pint.reset()                       # reset for next frame interval
"""

import numpy as np


def _skew(v):
    """3×3 skew-symmetric matrix."""
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ])


def _rodrigues(omega, dt):
    """Integrate angular velocity to rotation matrix via Rodrigues formula."""
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-10:
        return np.eye(3)
    axis = omega / np.linalg.norm(omega)
    K    = _skew(axis)
    return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K @ K)


class IMUPreintegrator:
    """
    Accumulates IMU measurements between two camera frames.
    Produces delta rotation and delta velocity for VIO fusion.
    """

    G_VEC = np.array([0, 0, 9.81])   # gravity vector (world frame, Z-up)

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset accumulators for a new frame interval."""
        self._R_delta   = np.eye(3)      # accumulated rotation
        self._v_delta   = np.zeros(3)    # accumulated velocity change
        self._p_delta   = np.zeros(3)    # accumulated position change
        self._dt_total  = 0.0
        self._n         = 0              # number of IMU samples integrated

    def integrate(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """
        Integrate one IMU sample.

        Parameters
        ----------
        gyro  : (3,) rad/s   (body frame, bias-corrected by firmware)
        accel : (3,) m/s²    (body frame)
        dt    : time step [s]
        """
        # Rotate accel to world frame using current accumulated rotation
        a_world = self._R_delta @ accel

        # Increment rotation
        dR = _rodrigues(gyro, dt)
        self._R_delta = self._R_delta @ dR

        # Increment velocity (gravity NOT subtracted here – handled in fusion)
        self._v_delta += a_world * dt

        # Increment position
        self._p_delta += self._v_delta * dt + 0.5 * a_world * dt * dt

        self._dt_total += dt
        self._n        += 1

    def get_delta(self) -> dict:
        """
        Return the accumulated pre-integrated delta.

        Keys
        ----
        R_delta   : (3, 3) rotation increment body_k → body_k+1
        v_delta   : (3,)   velocity increment (world frame, gravity not removed)
        p_delta   : (3,)   position increment (world frame, gravity not removed)
        dt        : total time elapsed
        n         : number of integrated samples
        """
        return {
            "R_delta":  self._R_delta.copy(),
            "v_delta":  self._v_delta.copy(),
            "p_delta":  self._p_delta.copy(),
            "dt":       self._dt_total,
            "n":        self._n,
        }
