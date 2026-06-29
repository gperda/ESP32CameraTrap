"""detection.py — Run MegaDetector on a single image and save annotated output

Usage:
    python3 detection.py <img_path> <out_path>

Arguments:
    img_path    Path to input JPEG image
    out_path    Path for annotated output

Prints a single JSON line to stdout:
    { "success": true,  "detections": N }
or:
    { "success": false, "error": "..." }

Exits with code 0 on success, 1 on failure.
"""

import sys
import json
import numpy as np
import torch
from PIL import Image
from transformers import Sam3Model, Sam3Processor

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
        detector_model = run_detector.load_detector('MDV5A')
    except Exception as e:
        fail(f"Failed to load MegaDetector model: {e}")

    try:
        result = detector_model.generate_detections_one_image(image)
    except Exception as e:
        fail(f"Detection failed: {e}")

    detections_above_threshold = [d for d in result['detections'] if d['conf'] > CONFIDENCE_THRESHOLD]

    if detections_above_threshold:
        sam_model = Sam3Model.from_pretrained("facebook/sam3").to("cuda" if torch.cuda.is_available() else "cpu")
        processor = Sam3Processor.from_pretrained("facebook/sam3")

        image = image.convert("RGB")
        boxes = np.array([
            [ymin, xmin, ymin + height, xmin + width]
            for xmin, ymin, width, height in [d['bbox'] for d in detections_above_threshold]
        ])
        sam_boxes = [[
            [xmin, ymin, xmax, ymax] for ymin, xmin, ymax, xmax in boxes
        ]]
        confidences = [f"{d['conf']:.2f}" for d in detections_above_threshold]
        box_colors = np.random.randint(0, 256, size=len(detections_above_threshold)).tolist()
        mask_colors = np.random.randint(0, 256, size=(len(detections_above_threshold), 3), dtype=np.uint8)

        inputs = processor(
            images=image,
            text="animal",
            input_boxes=sam_boxes,
            input_boxes_labels=[np.array([1] * len(sam_boxes[0]))],
            return_tensors="pt"
        ).to(sam_model.device)

        with torch.no_grad():
            outputs = sam_model(**inputs)

        results = processor.post_process_instance_segmentation(
            outputs,
            threshold=0.5,
            mask_threshold=0.5,
            target_sizes=inputs.get("original_sizes").tolist()
        )[0]

        # Blend predicted masks onto the original image, then draw boxes on top.
        masks = results.get("masks", [])
        image_np = np.array(image, dtype=np.float32)
        alpha = 0.35

        for i, mask in enumerate(masks):
            mask_np = np.asarray(mask.cpu())
            if mask_np.ndim > 2:
                mask_np = np.squeeze(mask_np)
            mask_bool = mask_np > 0

            binary_mask = (mask_bool.astype(np.uint8) * 255)
            Image.fromarray(binary_mask, mode="L").save(out_path.replace(".png", f"_mask_{i}.png"))

            if np.any(mask_bool):
                color = mask_colors[i % len(mask_colors)].astype(np.float32)
                image_np[mask_bool] = (1.0 - alpha) * image_np[mask_bool] + alpha * color

            

        image = Image.fromarray(image_np.astype(np.uint8), mode="RGB")
        #vis_utils.draw_bounding_boxes_on_image(image, boxes, box_colors, display_strs=confidences)

    try:
        image.save(out_path)
    except Exception as e:
        fail(f"Failed to save output image: {e}")

    print(json.dumps({"success": True, "detections": len(detections_above_threshold)}), flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()