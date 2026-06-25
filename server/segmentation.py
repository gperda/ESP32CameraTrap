"""detection.py — Run SAM3 on a single image and save annotated output

Usage:
    python3 segmentation.py <img_path> <out_path>

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
        image = Image.open(img_path).convert(RGB)
    except Exception as e:
        fail(f"Could not read image: {e}")

        inputs = processor(
        images=image,
        text="animal",
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
      

    mask_colors = np.random.randint(0, 256, size=(len(results["boxes"]), 3), dtype=np.uint8)

    # Blend predicted masks onto the original image, then draw boxes on top.
    masks = results.get("masks", [])
    image_np = np.array(image, dtype=np.float32)
    alpha = 0.35

    for i, mask in enumerate(masks):
        mask_np = np.asarray(mask.cpu())
        if mask_np.ndim > 2:
            mask_np = np.squeeze(mask_np)
        mask_bool = mask_np > 0
        if np.any(mask_bool):
            color = mask_colors[i % len(mask_colors)].astype(np.float32)
            image_np[mask_bool] = (1.0 - alpha) * image_np[mask_bool] + alpha * color

    image = Image.fromarray(image_np.astype(np.uint8), mode="RGB")

    try:
        image.save(out_path)
    except Exception as e:
        fail(f"Failed to save output image: {e}")

    print(json.dumps({"success": True, "detections": len(results["boxes"])}), flush=True)
    sys.exit(0)

if __name__ == "__main__":
    main()