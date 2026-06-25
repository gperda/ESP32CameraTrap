"""detection.py — Run MegaDetector on a single image and save annotated output

Usage:
    python3 detection.py <img_path> <out_path>

Arguments:
    img_path    Path to input JPEG image
    out_path    Path for annotated output PNG

Prints a single JSON line to stdout:
    { "success": true,  "detections": N }
or:
    { "success": false, "error": "..." }

Exits with code 0 on success, 1 on failure.
"""

import sys
import json

CONFIDENCE_THRESHOLD = 0.2


def fail(msg: str) -> None:
    print(json.dumps({"success": False, "error": msg}), flush=True)
    sys.exit(1)


def main():
    if len(sys.argv) < 3:
        fail("Usage: detection.py <img_path> <out_path>")

    img_path = sys.argv[1]
    out_path = sys.argv[2]

    try:
        from megadetector.visualization import visualization_utils as vis_utils
        from megadetector.detection import run_detector
    except ImportError as e:
        fail(f"Failed to import MegaDetector: {e}")

    try:
        image = vis_utils.open_image(img_path)
    except Exception as e:
        fail(f"Could not read image: {e}")

    try:
        model = run_detector.load_detector('MDV5A')
    except Exception as e:
        fail(f"Failed to load MegaDetector model: {e}")

    try:
        result = model.generate_detections_one_image(image)
    except Exception as e:
        fail(f"Detection failed: {e}")

    detections_above_threshold = [d for d in result['detections'] if d['conf'] > CONFIDENCE_THRESHOLD]

    if detections_above_threshold:
        import numpy as np
        boxes = np.array([
            [ymin, xmin, ymin + height, xmin + width]
            for xmin, ymin, width, height in [d['bbox'] for d in detections_above_threshold]
        ])
        confidences = [f"{d['conf']:.2f}" for d in detections_above_threshold]
        colors = np.random.randint(0, 256, size=len(detections_above_threshold)).tolist()
        vis_utils.draw_bounding_boxes_on_image(image, boxes, colors, display_strs=confidences)

    try:
        image.save(out_path)
    except Exception as e:
        fail(f"Failed to save output image: {e}")

    print(json.dumps({"success": True, "detections": len(detections_above_threshold)}), flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()