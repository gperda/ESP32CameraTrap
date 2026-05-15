"""
depthmap.py — Stereo disparity map computation using OpenCV

Usage:
    python3 depthmap.py <img1> <img2> <outpath> [<calib_json>] [<mode>]

Arguments:
    img1        Path to left camera JPEG (cam1)
    img2        Path to right camera JPEG (cam2)
    outpath     Path for output PNG (colorized disparity)
    calib_json  (optional) Path to calibration.json
    mode        (optional) one of:
                - "depth" (default)
                - "undistort" (side-by-side preview)
                - "undistort_cam1" (left image only)
                - "undistort_cam2" (right image only)

calibration.json format:
    {
      "K1": [[fx,0,cx],[0,fy,cy],[0,0,1]],
      "D1": [k1,k2,p1,p2,k3],
      "K2": [[fx,0,cx],[0,fy,cy],[0,0,1]],
      "D2": [k1,k2,p1,p2,k3],
      "baseline": 65.0,          // mm between lens centers
      "R": [[...],[...],[...]],   // optional: rotation matrix (defaults to identity)
      "T": [tx, ty, tz]          // optional: translation vector (defaults to [baseline,0,0])
    }

Exits with code 0 on success, 1 on failure.
Prints a single JSON line to stdout: {"success": true} or {"success": false, "error": "..."}
"""

import sys
import json
import os
import numpy as np
import cv2

# ── SGBM parameters — tune these for your baseline / resolution ──────────────
MIN_DISPARITY   = 0
NUM_DISPARITIES = 64     # must be divisible by 16
BLOCK_SIZE      = 111      # odd, 3–11 recommended
P1              = 8  * 3 * BLOCK_SIZE ** 2
P2              = 32 * 3 * BLOCK_SIZE ** 2
DISP12_MAX_DIFF = -1
UNIQUENESS_RATIO     = 10
SPECKLE_WINDOW_SIZE  = 0
SPECKLE_RANGE        = 10

# Maximum long-edge size to process (resize if larger, for speed)
MAX_PROCESS_DIM = 800


def fail(msg: str) -> None:
    print(json.dumps({"success": False, "error": msg}), flush=True)
    sys.exit(1)


def load_calibration(path: str):
    """
    Returns (K1, D1, K2, D2, R, T) as numpy arrays, or None for each if absent.
    R defaults to identity, T defaults to [baseline, 0, 0].
    """
    with open(path) as f:
        c = json.load(f)

    required = ("K1", "D1", "K2", "D2", "baseline")
    for key in required:
        if key not in c:
            fail(f"calibration.json missing required key: '{key}'")

    K1 = np.array(c["K1"], dtype=np.float64)
    D1 = np.array(c["D1"], dtype=np.float64)
    K2 = np.array(c["K2"], dtype=np.float64)
    D2 = np.array(c["D2"], dtype=np.float64)
    baseline = float(c["baseline"])

    R = np.array(c["R"], dtype=np.float64) if "R" in c else np.eye(3, dtype=np.float64)
    T = np.array(c["T"], dtype=np.float64) if "T" in c else np.array([baseline, 0.0, 0.0])

    return K1, D1, K2, D2, R, T


def scale_to_max(img, max_dim: int):
    """Uniformly scale img so its longest edge ≤ max_dim. Returns (img, scale_factor)."""
    h, w = img.shape[:2]
    long_edge = max(h, w)
    if long_edge <= max_dim:
        return img, 1.0
    scale = max_dim / long_edge
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA), scale


def compute_disparity(left_gray, right_gray):
    sgbm = cv2.StereoSGBM_create(
        minDisparity       = MIN_DISPARITY,
        numDisparities     = NUM_DISPARITIES,
        blockSize          = BLOCK_SIZE,
        P1                 = P1,
        P2                 = P2,
        disp12MaxDiff      = DISP12_MAX_DIFF,
        uniquenessRatio    = UNIQUENESS_RATIO,
        speckleWindowSize  = SPECKLE_WINDOW_SIZE,
        speckleRange       = SPECKLE_RANGE,
        mode               = cv2.STEREO_SGBM_MODE_HH,
    )
    return sgbm.compute(left_gray, right_gray)


def colorize_disparity(disp16):
    """
    Normalize a fixed-point disparity map (16-bit, scale factor 16) to [0,255]
    and apply COLORMAP_TURBO. Invalid pixels (< 0) are rendered black.
    """
    valid_mask = disp16 > 0
    disp_float = disp16.astype(np.float32) / 16.0

    if valid_mask.any():
        vmin = disp_float[valid_mask].min()
        vmax = disp_float[valid_mask].max()
    else:
        vmin, vmax = 0.0, 1.0

    norm = np.zeros_like(disp_float, dtype=np.uint8)
    if vmax > vmin:
        norm[valid_mask] = np.clip(
            (disp_float[valid_mask] - vmin) / (vmax - vmin) * 255, 0, 255
        ).astype(np.uint8)

    colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    colored[~valid_mask] = (10, 10, 10)   # near-black for invalid zones
    return colored


def make_undistort_preview(left_bgr, right_bgr):
    """Create side-by-side undistorted preview image for browser display."""
    h = max(left_bgr.shape[0], right_bgr.shape[0])
    w = left_bgr.shape[1] + right_bgr.shape[1]
    out = np.zeros((h, w, 3), dtype=np.uint8)

    out[:left_bgr.shape[0], :left_bgr.shape[1]] = left_bgr
    out[:right_bgr.shape[0], left_bgr.shape[1]:left_bgr.shape[1] + right_bgr.shape[1]] = right_bgr

    split_x = left_bgr.shape[1]
    cv2.line(out, (split_x, 0), (split_x, h - 1), (255, 255, 255), 1)
    cv2.putText(out, "cam1 undistorted", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(out, "cam2 undistorted", (split_x + 8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    return out


def main():
    if len(sys.argv) < 4:
        fail("Usage: depthmap.py <img1> <img2> <outpath> [<calib_json>]")

    img1_path   = sys.argv[1]
    img2_path   = sys.argv[2]
    out_path    = sys.argv[3]
    calib_path  = sys.argv[4] if len(sys.argv) >= 5 else None
    mode        = (sys.argv[5] if len(sys.argv) >= 6 else "depth").strip().lower()
    if mode not in ("depth", "undistort", "undistort_cam1", "undistort_cam2"):
        fail(
            f"Invalid mode: '{mode}'. Expected one of "
            "'depth', 'undistort', 'undistort_cam1', 'undistort_cam2'"
        )

    # ── Load images ───────────────────────────────────────────────────────────
    left_bgr  = cv2.imread(img1_path)
    right_bgr = cv2.imread(img2_path)
    if left_bgr is None:
        fail(f"Could not read cam1 image: {img1_path}")
    if right_bgr is None:
        fail(f"Could not read cam2 image: {img2_path}")

    # Resize both to the same (smaller) resolution for speed
    left_bgr,  scale = scale_to_max(left_bgr,  MAX_PROCESS_DIM)
    right_bgr, _     = scale_to_max(right_bgr, MAX_PROCESS_DIM)

    # Ensure same size (crop/pad if cameras have slightly different aspect)
    h = min(left_bgr.shape[0], right_bgr.shape[0])
    w = min(left_bgr.shape[1], right_bgr.shape[1])
    left_bgr  = left_bgr[:h, :w]
    right_bgr = right_bgr[:h, :w]

    image_size = (w, h)

    # ── Calibrated path ───────────────────────────────────────────────────────
    if calib_path and os.path.isfile(calib_path):
        K1, D1, K2, D2, R, T = load_calibration(calib_path)

        # Scale camera matrices to match the (possibly) downsampled resolution
        K1 = K1.copy(); K1[0] *= scale; K1[1] *= scale
        K2 = K2.copy(); K2[0] *= scale; K2[1] *= scale

        R1, R2, P1_, P2_, Q, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, image_size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1_, image_size, cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2_, image_size, cv2.CV_32FC1)

        left_rect  = cv2.remap(left_bgr,  map1x, map1y, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_bgr, map2x, map2y, cv2.INTER_LINEAR)

        left_gray  = cv2.cvtColor(left_rect,  cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        calibrated = True

    # ── Uncalibrated path ─────────────────────────────────────────────────────
    else:
        left_gray  = cv2.cvtColor(left_bgr,  cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        calibrated = False

    if mode == "depth":
        # ── Compute disparity ─────────────────────────────────────────────────
        disp16 = compute_disparity(left_gray, right_gray)

        # ── Colorize and save ─────────────────────────────────────────────────
        out_img = colorize_disparity(disp16)
        mode_label = "depth"
    elif mode == "undistort":
        # Keep the preview in color so image quality / alignment are easy to inspect.
        preview_left = left_rect if calibrated else left_bgr
        preview_right = right_rect if calibrated else right_bgr
        out_img = make_undistort_preview(preview_left, preview_right)
        mode_label = "undistort"
    elif mode == "undistort_cam1":
        out_img = left_rect if calibrated else left_bgr
        mode_label = "undistort_cam1"
    else:
        out_img = right_rect if calibrated else right_bgr
        mode_label = "undistort_cam2"

    # Annotate processing mode and calibration state
    state_label = "calibrated" if calibrated else "uncalibrated"
    cv2.putText(
        out_img, f"{mode_label} · {state_label}", (8, out_img.shape[0] - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA,
    )

    cv2.imwrite(out_path, out_img)
    print(json.dumps({"success": True, "calibrated": calibrated, "mode": mode_label}), flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
