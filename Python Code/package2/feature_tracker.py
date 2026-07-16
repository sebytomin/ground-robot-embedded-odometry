"""
feature_tracker.py  –  Package 2
----------------------------------
KLT optical flow feature tracker for monocular VO.

Detects Shi-Tomasi corners, tracks them with Lucas-Kanade optical flow,
validates with forward-backward consistency check, and undistorts points.

Usage
-----
tracker = FeatureTracker(K, dist)
pts_prev, pts_curr, n = tracker.process(frame)

"""

import numpy as np
import cv2


class FeatureTracker:

    SHI_PARAMS = dict(
        maxCorners   = 200,    # fewer = faster, still enough for Essential mat
        qualityLevel = 0.01,
        minDistance  = 20,     # px — avoid clustering
        blockSize    = 7,
    )

    LK_PARAMS = dict(
        winSize  = (21, 21),
        maxLevel = 3,
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )

    FB_THRESH = 1.0    # forward-backward pixel error threshold

    def __init__(self, K: np.ndarray, dist: np.ndarray,
                 max_features: int = 200, min_features: int = 50):
        self.K            = K
        self.dist         = dist
        self.max_features = max_features
        self.min_features = min_features
        self.SHI_PARAMS["maxCorners"] = max_features

        self._prev_gray = None
        self._prev_pts  = None    # tracked points in previous frame
        self._cur_pts   = None    # tracked points in current frame (for draw)
        self._prev_draw = None    # copy of prev points for draw arrows

    def process(self, frame: np.ndarray):
        """
        Track features from last frame to this frame.

        Returns
        -------
        pts_prev  : (N,2) undistorted points from previous frame, or None
        pts_curr  : (N,2) undistorted points from current frame,  or None
        n_tracks  : int
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # First frame — detect only, no tracking yet
        if self._prev_gray is None:
            self._prev_gray = gray
            self._prev_pts  = self._detect(gray)
            self._cur_pts   = self._prev_pts
            return None, None, 0

        # Re-detect if too few points
        if self._prev_pts is None or len(self._prev_pts) < self.min_features:
            self._prev_pts = self._detect(self._prev_gray)
            if len(self._prev_pts) < self.min_features:
                self._prev_gray = gray
                return None, None, 0

        # Forward optical flow
        p0 = self._prev_pts.reshape(-1, 1, 2).astype(np.float32)
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, p0, None, **self.LK_PARAMS)

        if p1 is None:
            self._prev_gray = gray
            self._prev_pts  = self._detect(gray)
            return None, None, 0

        # Backward check
        p0b, st2, _ = cv2.calcOpticalFlowPyrLK(
            gray, self._prev_gray, p1, None, **self.LK_PARAMS)

        # Good points: both tracked AND forward-backward consistent
        fb_err = np.linalg.norm(
            p0.reshape(-1, 2) - p0b.reshape(-1, 2), axis=1)
        good = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb_err < self.FB_THRESH)

        prev_good = p0.reshape(-1, 2)[good]
        curr_good = p1.reshape(-1, 2)[good]

        # Store raw (distorted) points for visualisation
        self._prev_draw = prev_good.copy()
        self._cur_pts   = curr_good.copy()

        # Update for next iteration
        self._prev_gray = gray
        self._prev_pts  = curr_good    # continue tracking from current positions

        if len(curr_good) < self.min_features:
            return None, None, len(curr_good)

        # Undistort for Essential matrix computation
        prev_ud = self._undistort(prev_good)
        curr_ud = self._undistort(curr_good)

        return prev_ud, curr_ud, len(curr_good)

    def draw_tracks(self, frame: np.ndarray) -> np.ndarray:
        """Draw tracked features and motion arrows on a copy of frame."""
        vis = frame.copy()
        if self._cur_pts is not None and len(self._cur_pts) > 0:
            for pt in self._cur_pts:
                cv2.circle(vis, tuple(pt.astype(int)), 3, (0, 255, 0), -1)
            if (self._prev_draw is not None and
                    len(self._prev_draw) == len(self._cur_pts)):
                for p, c in zip(self._prev_draw, self._cur_pts):
                    cv2.arrowedLine(vis, tuple(p.astype(int)),
                                    tuple(c.astype(int)),
                                    (0, 165, 255), 1, tipLength=0.4)
        n = len(self._cur_pts) if self._cur_pts is not None else 0
        cv2.putText(vis, f"Tracks: {n}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        return vis

    def reset(self):
        self._prev_gray = None
        self._prev_pts  = None
        self._cur_pts   = None
        self._prev_draw = None

    def _detect(self, gray: np.ndarray) -> np.ndarray:
        pts = cv2.goodFeaturesToTrack(gray, **self.SHI_PARAMS)
        if pts is None:
            return np.zeros((0, 2), dtype=np.float32)
        return pts.reshape(-1, 2).astype(np.float32)

    def _undistort(self, pts: np.ndarray) -> np.ndarray:
        if len(pts) == 0:
            return pts
        ud = cv2.undistortPoints(
            pts.reshape(-1, 1, 2).astype(np.float32),
            self.K, self.dist, P=self.K)
        return ud.reshape(-1, 2)
