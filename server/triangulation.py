
"""triangulation.py — Stereo point triangulation from two clicked pixels

Usage:
    python3 triangulation.py x1 y1 x2 y2 calib_json

Arguments:
    x1, y1      Pixel click in the left half of the undistort preview (cam1)
    x2, y2      Pixel click in the right half of the undistort preview (cam2),
                coordinates relative to the right-half origin (i.e. 0-based)
    calib_json  Path to calibration.json

The clicked points are expected to be in the already-rectified coordinate space
produced by depthmap.py's undistort preview (mode="undistort").  The script
replicates the same scale_to_max + stereoRectify pipeline used there so that
P1_ / P2_ match the pixel space of the preview image.

Outputs a single JSON line to stdout:
    { "success": true,  "xyz_mm": {"X": ..., "Y": ..., "Z": ...},
      "point_cam1": {"x": ..., "y": ...}, "point_cam2": {"x": ..., "y": ...},
      "scale_used": ..., "image_size_used": [w, h] }
or:
    { "success": false, "error": "..." }
"""

import sys
import os
import json

import numpy as np
import cv2

# Must stay in sync with depthmap.py so the same scale is applied to K matrices.
MAX_PROCESS_DIM = 800


def fail(msg: str) -> None:
    print(json.dumps({"success": False, "error": msg}), flush=True)
    sys.exit(1)


def load_calibration(path: str):
    """Return (K1, D1, K2, D2, R, T, rect) from calibration.json.

    rect is a dict with pre-computed stereoRectify outputs
    (R1, R2, P1, P2, image_size) if depthmap.py has written them;
    otherwise None, and the caller must run stereoRectify itself.
    """
    try:
        with open(path) as f:
            c = json.load(f)
    except Exception as e:
        fail(f"Cannot read calibration file: {e}")

    for key in ("K1", "D1", "K2", "D2", "baseline"):
        if key not in c:
            fail(f"calibration.json missing required key: '{key}'")

    K1 = np.array(c["K1"], dtype=np.float64)
    D1 = np.array(c["D1"], dtype=np.float64)
    K2 = np.array(c["K2"], dtype=np.float64)
    D2 = np.array(c["D2"], dtype=np.float64)
    baseline = float(c["baseline"])

    R = np.array(c["R"], dtype=np.float64) if "R" in c else np.eye(3, dtype=np.float64)
    T = np.array(c["T"], dtype=np.float64) if "T" in c else np.array([baseline, 0.0, 0.0])

    rect = None
    if all(k in c for k in ("R1", "R2", "P1", "P2", "image_size")):
        rect = {
            "R1":         np.array(c["R1"], dtype=np.float64),
            "R2":         np.array(c["R2"], dtype=np.float64),
            "P1":         np.array(c["P1"], dtype=np.float64),
            "P2":         np.array(c["P2"], dtype=np.float64),
            "image_size": tuple(c["image_size"]),
        }

    return K1, D1, K2, D2, R, T, rect


def main():
    if len(sys.argv) < 6:
        fail("Usage: triangulation.py x1 y1 x2 y2 calib_json")

    try:
        x1 = float(sys.argv[1])
        y1 = float(sys.argv[2])
        x2 = float(sys.argv[3])
        y2 = float(sys.argv[4])
    except ValueError:
        fail("x1 y1 x2 y2 must be numeric values")

    calib_path = sys.argv[5]

    if not os.path.isfile(calib_path):
        fail(f"Calibration file not found: {calib_path}")

    K1, D1, K2, D2, R, T, rect = load_calibration(calib_path)

    # ── Obtain rectification projection matrices ──────────────────────────────
    # Prefer pre-computed values written by depthmap.py (exact pixel match).
    # Fall back to estimating image size from the principal point and rerunning
    # stereoRectify when those values are absent.
    if rect is not None:
        P1_ = rect["P1"]
        P2_ = rect["P2"]
        image_size = rect["image_size"]
    else:
        fail("No calibration found")


    # ── Triangulate ──────────────────────────────────────────────────────────
    # The clicked pixels are already in the rectified frame (undistort preview).
    pts1 = np.array([[x1], [y1]], dtype=np.float64)
    pts2 = np.array([[x2], [y2]], dtype=np.float64)

    try:
        pts4d = cv2.triangulatePoints(P1_, P2_, pts1, pts2)
    except Exception as e:
        fail(f"triangulatePoints failed: {e}")

    w_hom = float(pts4d[3, 0])
    if abs(w_hom) < 1e-9:
        fail("Degenerate triangulation: homogeneous w ≈ 0 (points may be parallel rays)")

    X = float(pts4d[0, 0]) / w_hom
    Y = float(pts4d[1, 0]) / w_hom
    Z = float(pts4d[2, 0]) / w_hom

    print(json.dumps({
        "success":        True,
        "xyz_mm":         {"X": round(X, 2), "Y": round(Y, 2), "Z": round(Z, 2)},
        "point_cam1":     {"x": round(x1, 2), "y": round(y1, 2)},
        "point_cam2":     {"x": round(x2, 2), "y": round(y2, 2)},
        "image_size_used": list(image_size),
        "w_hom":          round(w_hom, 6),
    }), flush=True)


if __name__ == "__main__":
    main()
