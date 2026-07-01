"""
undistort.py — Stereo undistortion/rectification outputs using OpenCV

Usage:
    python3 undistort.py <img1> <img2> <outpath> [<calib_json>] [<mode>]

Arguments:
    img1        Path to left camera JPEG (cam1)
    img2        Path to right camera JPEG (cam2)
    outpath     Path for output PNG
    calib_json  (optional) Path to calibration.json
    mode        (optional) one of:
                - "undistort" (side-by-side preview)
                - "undistort_cam1" (cam1 image only)
                - "undistort_cam2" (cam2 image only)

Exits with code 0 on success, 1 on failure.
Prints a single JSON line to stdout.
"""

import sys
import json
import os
import numpy as np
import cv2


def fail(msg: str) -> None:
    print(json.dumps({"success": False, "error": msg}), flush=True)
    sys.exit(1)


def load_calibration(path: str):
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


def make_undistort_preview(left_bgr, right_bgr):
    """Create side-by-side undistorted preview image for browser display."""
    h = max(left_bgr.shape[0], right_bgr.shape[0])
    w = left_bgr.shape[1] + right_bgr.shape[1]
    out = np.zeros((h, w, 3), dtype=np.uint8)

    out[:left_bgr.shape[0], :left_bgr.shape[1]] = left_bgr
    out[:right_bgr.shape[0], left_bgr.shape[1]:left_bgr.shape[1] + right_bgr.shape[1]] = right_bgr
    split_x = left_bgr.shape[1]
    cv2.line(out, (split_x, 0), (split_x, h - 1), (255, 255, 255), 1)
    return out


def main():
    if len(sys.argv) < 4:
        fail("Usage: undistort.py <img1> <img2> <out1> <out2> <outpath> [<calib_json>]")

    left_path = sys.argv[1]
    right_path = sys.argv[2]
    out1_path = sys.argv[3]
    out2_path = sys.argv[4]
    out_path = sys.argv[5]
    calib_path = sys.argv[6] if len(sys.argv) >= 5 else None

    left_bgr = cv2.imread(left_path)
    right_bgr = cv2.imread(right_path)
    if left_bgr is None:
        fail(f"Could not read cam1 image: {left_path}")
    if right_bgr is None:
        fail(f"Could not read cam2 image: {right_path}")

    h = min(left_bgr.shape[0], right_bgr.shape[0])
    w = min(left_bgr.shape[1], right_bgr.shape[1])
    left_bgr = left_bgr[:h, :w]
    right_bgr = right_bgr[:h, :w]

    image_size = (w, h)
    calibrated = False

    left_out = left_bgr
    right_out = right_bgr

    if calib_path and os.path.isfile(calib_path):
        K1, D1, K2, D2, R, T = load_calibration(calib_path)

        R1, R2, P1_, P2_, _, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, image_size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )

        # Persist rectification matrices so triangulation can reuse them.
        try:
            with open(calib_path) as _f:
                _calib_data = json.load(_f)
            _calib_data["R1"] = R1.tolist()
            _calib_data["R2"] = R2.tolist()
            _calib_data["P1"] = P1_.tolist()
            _calib_data["P2"] = P2_.tolist()
            _calib_data["image_size"] = list(image_size)
            with open(calib_path, "w") as _f:
                json.dump(_calib_data, _f, indent=2)
        except Exception:
            pass

        map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1_, image_size, cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2_, image_size, cv2.CV_32FC1)

        left_out = cv2.remap(left_bgr, map2x, map2y, cv2.INTER_LINEAR)
        right_out = cv2.remap(right_bgr, map1x, map1y, cv2.INTER_LINEAR)
        calibrated = True

        out_img = make_undistort_preview(right_out, left_out)

    cv2.imwrite(out_path, out_img)
    cv2.imwrite(out1_path, right_out)
    cv2.imwrite(out2_path, left_out)
    print(json.dumps({"success": True, "calibrated": calibrated, "mode": "undistort", "size": image_size}), flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
