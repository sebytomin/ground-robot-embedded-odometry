"""
generate_default_params.py  –  Package 2
-----------------------------------------
Generates camera_params.npz using the known factory specifications of the
Raspberry Pi Camera Module v2 (Sony IMX219 sensor).

No checkerboard or live camera required.
"""

import numpy as np
import argparse


# ── IMX219 full-sensor constants ──────────────────────────────────────────────
SENSOR_W_PX   = 3280          # full sensor width  [pixels]
SENSOR_H_PX   = 2464          # full sensor height [pixels]
PIXEL_SIZE_MM = 0.00112       # 1.12 µm pixel pitch [mm]
FOCAL_LEN_MM  = 3.04          # fixed lens focal length [mm]

# Focal length in pixels at full sensor resolution
FX_FULL = FOCAL_LEN_MM / PIXEL_SIZE_MM   # ≈ 2714 px
FY_FULL = FX_FULL                         # square pixels


def compute_K(width, height):
    """
    Scale the full-sensor intrinsics to the capture resolution.

    The Raspberry Pi Camera uses centre-cropping + binning when capturing
    at sub-full resolutions.  We model this as a simple uniform scale.

    Returns K (3×3) and distortion (1×5).
    """
    sx = width  / SENSOR_W_PX
    sy = height / SENSOR_H_PX

    fx = FX_FULL * sx
    fy = FY_FULL * sy
    cx = width  / 2.0
    cy = height / 2.0

    K = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1],
    ], dtype=np.float64)

    # Distortion: RPi Camera v2 has very low distortion (wide-angle but not fisheye)
    # k1, k2, p1, p2, k3  –  use small representative values from community measurements
    dist = np.array([[-0.35, 0.12, 0.0, 0.0, -0.02]], dtype=np.float64)

    return K, dist


def main():
    p = argparse.ArgumentParser(description="Generate default RPi Camera v2 params")
    p.add_argument("--width",  type=int, default=640,                help="Capture width  (default 640)")
    p.add_argument("--height", type=int, default=480,                help="Capture height (default 480)")
    p.add_argument("--output", default="camera_params.npz",          help="Output file")
    args = p.parse_args()

    K, dist = compute_K(args.width, args.height)

    print(f"\nRaspberry Pi Camera v2 — estimated intrinsics at {args.width}×{args.height}")
    print(f"\n  fx = {K[0,0]:.2f} px    fy = {K[1,1]:.2f} px")
    print(f"  cx = {K[0,2]:.2f} px    cy = {K[1,2]:.2f} px")
    print(f"\n  Distortion  k1={dist[0,0]:.3f}  k2={dist[0,1]:.3f}  "
          f"p1={dist[0,2]:.3f}  p2={dist[0,3]:.3f}  k3={dist[0,4]:.3f}")

    np.savez(
        args.output,
        K          = K,
        dist       = dist,
        img_shape  = np.array([args.height, args.width]),
    )
    print(f"\n  Saved → {args.output}")
    print("\n  You can now run:  python3 vio_main.py")
    print("  (replace camera_params.npz with a real calibration later for best accuracy)\n")


if __name__ == "__main__":
    main()
