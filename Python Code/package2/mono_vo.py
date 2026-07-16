"""
mono_vo.py  –  Package 2
--------------------------
Monocular Visual Odometry using the 5-point Essential matrix algorithm.

Usage
-----
    vo = MonocularVO(K)
    ok, R, t, n = vo.process(pts_prev, pts_curr, scale=0.5)
"""

import numpy as np
import cv2


class MonocularVO:

    # Post-check: if |t| / rot_angle < this, reject as pure rotation
    PURE_ROT_THR = 0.02

    def __init__(self, K: np.ndarray,
                 ransac_thr: float = 1.0,
                 min_inliers: int  = 25,
                 prob: float       = 0.999):
        self.K           = K
        self.ransac_thr  = ransac_thr
        self.min_inliers = min_inliers
        self.prob        = prob

    def process(self, pts_prev: np.ndarray, pts_curr: np.ndarray,
                scale: float = None):
        """
        Parameters
        ----------
        pts_prev   : (N,2) undistorted points from previous frame
        pts_curr   : (N,2) undistorted points from current frame
        scale      : metric scale from IMU (None = return unit translation)

        Returns
        -------
        success    : bool
        R_rel      : (3,3) relative rotation  — None on failure
        t_rel      : (3,)  relative translation (metric if scale given)
        n_inliers  : int
        """
        if pts_prev is None or pts_curr is None:
            return False, None, None, 0
        if len(pts_prev) < self.min_inliers:
            return False, None, None, 0

        # ── Pre-check: pure rotation degeneracy ──────────────────────────────
        flow = pts_curr - pts_prev                         # (N,2)
        rms  = float(np.sqrt(np.mean(np.sum(flow**2, axis=1))))
        mean_mag = float(np.linalg.norm(np.mean(flow, axis=0)))
        # If features all moved but the mean is near zero → symmetric rotation
        if rms > 1.0 and mean_mag < 0.15 * rms:
            return False, None, None, 0

        # ── Essential matrix ─────────────────────────────────────────────────
        E, mask = cv2.findEssentialMat(
            pts_prev.astype(np.float32),
            pts_curr.astype(np.float32),
            self.K,
            method    = cv2.RANSAC,
            prob      = self.prob,
            threshold = self.ransac_thr,
        )
        if E is None or mask is None:
            return False, None, None, 0

        n_in, R, t, mask2 = cv2.recoverPose(
            E,
            pts_prev.astype(np.float32),
            pts_curr.astype(np.float32),
            self.K,
            mask = mask,
        )
        if n_in < self.min_inliers:
            return False, None, None, int(n_in)

        # ── Post-check: translation tiny vs rotation ──────────────────────────
        rot_angle = float(np.arccos(
            np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
        t_norm = float(np.linalg.norm(t.ravel()))
        if rot_angle > 0.05 and t_norm < self.PURE_ROT_THR * rot_angle:
            return False, None, None, int(n_in)

        # ── Apply scale ───────────────────────────────────────────────────────
        t_out = t.ravel().copy()
        if scale is not None and scale > 0.001:
            t_out = t_out * scale

        return True, R, t_out, int(n_in)
