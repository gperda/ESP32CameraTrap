"""
depthmap.py — Stereo disparity map computation using OpenCV

Usage:
    python3 depthmap.py <img1> <img2> <outpath> [<calib_json>]

Arguments:
    img1        Path to left camera JPEG (cam1)
    img2        Path to right camera JPEG (cam2)
    outpath     Path for output PNG (colorized disparity)
    calib_json  (optional) Path to calibration.json

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
import torch
import open3d as o3d
from core.foundation_stereo import FoundationStereo
from omegaconf import OmegaConf
from core.utils.utils import InputPadder


Z_FAR = 3.0 #discard points beyond this distance
DENOISE_NB_POINTS = 30 #number of points to consider for denoising
DENOISE_RADIUS = 0.03 #radius for denoising
VALID_ITERS = 32
SCALE = 1
HIERARCHICAL = 1

def vis_disparity(disp, min_val=None, max_val=None, invalid_thres=np.inf, color_map=cv2.COLORMAP_TURBO, cmap=None, other_output={}):
    """
    @disp: np array (H,W)
    @invalid_thres: > thres is invalid
    """
    disp = disp.copy()
    H,W = disp.shape[:2]
    invalid_mask = disp>=invalid_thres
    if (invalid_mask==0).sum()==0:
        other_output['min_val'] = None
        other_output['max_val'] = None
        return np.zeros((H,W,3))
    if min_val is None:
        min_val = disp[invalid_mask==0].min()
    if max_val is None:
        max_val = disp[invalid_mask==0].max()
    other_output['min_val'] = min_val
    other_output['max_val'] = max_val
    vis = ((disp-min_val)/(max_val-min_val)).clip(0,1) * 255
    if cmap is None:
        vis = cv2.applyColorMap(vis.clip(0, 255).astype(np.uint8), color_map)[...,::-1]
    else:
        vis = cmap(vis.astype(np.uint8))[...,:3]*255
    if invalid_mask.any():
        vis[invalid_mask] = 0
    return vis.astype(np.uint8)

def toOpen3dCloud(points,colors=None,normals=None):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if colors is not None:
        if colors.max()>1:
            colors = colors/255.0
        cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    if normals is not None:
        cloud.normals = o3d.utility.Vector3dVector(normals.astype(np.float64))
    return cloud


def depth2xyzmap(depth:np.ndarray, K, uvs:np.ndarray=None, zmin=0.1, mask = None):
    #invalid_mask = (depth<zmin)
    invalid_mask = (mask==0)
    H,W = depth.shape[:2]
    if uvs is None:
        vs,us = np.meshgrid(np.arange(0,H),np.arange(0,W), sparse=False, indexing='ij')
        vs = vs.reshape(-1)
        us = us.reshape(-1)
    else:
        uvs = uvs.round().astype(int)
        us = uvs[:,0]
        vs = uvs[:,1]
    zs = depth[vs,us]
    xs = (us-(-K[0,3]))*zs/K[2,3]
    ys = (vs-(-K[1,3]))*zs/K[2,3]
    pts = np.stack((xs.reshape(-1),ys.reshape(-1),zs.reshape(-1)), 1)  #(N,3)
    xyz_map = np.zeros((H,W,3), dtype=np.float32)
    xyz_map[vs,us] = pts
    if invalid_mask.any():
        xyz_map[invalid_mask] = 0
    return xyz_map

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

    required = ("Q", "baseline")
    for key in required:
        if key not in c:
            fail(f"calibration.json missing required key: '{key}'")

    Q = np.array(c["Q"], dtype=np.float64)
    baseline = float(c["baseline"])/1000.0

    return Q, baseline


def scale_to_max(img, max_dim: int):
    """Uniformly scale img so its longest edge ≤ max_dim. Returns (img, scale_factor)."""
    h, w = img.shape[:2]
    long_edge = max(h, w)
    if long_edge <= max_dim:
        return img, 1.0
    scale = max_dim / long_edge
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA), scale


def load_detection_masks(raw: str):
    """Parse detection masks JSON argument into a list of mask paths."""
    if raw is None:
        return []

    text = str(raw).strip()
    if not text:
        return []

    try:
        parsed = json.loads(text)
    except Exception as e:
        fail(f"Failed to parse detection masks JSON: {e}")

    if not isinstance(parsed, list):
        fail("detection_masks must be a JSON array")

    masks = []
    for entry in parsed:
        if entry is None:
            continue
        path_str = str(entry).strip()
        if not path_str:
            continue
        masks.append(path_str)
    return masks


def main():
    if len(sys.argv) < 5:
        fail("Usage: depthmap.py <left_img> <right_img> <detection_masks_json> <out_path> [<calib_json>]")

    foundation_path = os.environ.get('PYTHONPATH')
    torch.autograd.set_grad_enabled(False)

    ckpt_dir = os.path.join(foundation_path, 'pretrained_models/23-51-11/model_best_bp2.pth')
    cfg = OmegaConf.load(f'{os.path.dirname(ckpt_dir)}/cfg.yaml')
    if 'vit_size' not in cfg:
        cfg['vit_size'] = 'vitl'
    for k in range(0, len(sys.argv)):
        cfg[k] = sys.argv[k]
    args = OmegaConf.create(cfg)

    model = FoundationStereo(args)
    ckpt = torch.load(ckpt_dir, weights_only=False)
    model.load_state_dict(ckpt['model'])
    model.cuda()
    model.eval()

    left_path   = sys.argv[1]
    right_path   = sys.argv[2]
    detection_masks = load_detection_masks(sys.argv[3])
    print(json.dumps({"success": True, "detection_masks": detection_masks}), flush=True)
    out_path    = sys.argv[4]
    calib_path  = sys.argv[5] if len(sys.argv) > 5 else None

    # ── Load images ───────────────────────────────────────────────────────────
    left_bgr  = cv2.imread(left_path)
    right_bgr = cv2.imread(right_path)
    if SCALE <1:
        left_bgr = cv2.resize(left_bgr, fx=SCALE, fy=SCALE, dsize=None)
        right_bgr = cv2.resize(right_bgr, fx=SCALE, fy=SCALE, dsize=None)
    left_ori = left_bgr.copy()

    if left_bgr is None:
        fail(f"Could not read cam1 image: {left_path}")
    if right_bgr is None:
        fail(f"Could not read cam2 image: {right_path}")

    H,W = left_bgr.shape[:2]
    left = torch.as_tensor(left_bgr).cuda().float()[None].permute(0,3,1,2)
    right = torch.as_tensor(right_bgr).cuda().float()[None].permute(0,3,1,2)
    padder = InputPadder(left.shape, divis_by=32, force_square=False)
    left, right = padder.pad(left, right)

    if calib_path is None:
        fail("calibration.json path is required")
    Q, baseline = load_calibration(calib_path)

    with torch.cuda.amp.autocast(True):
        if not HIERARCHICAL:
            disp = model.forward(left, right, iters=VALID_ITERS, test_mode=True)
        else:
            disp = model.run_hierachical(left, right, iters=VALID_ITERS, test_mode=True, small_ratio=0.5)
        disp = padder.unpad(disp.float())
        disp = disp.data.cpu().numpy().reshape(H,W)
        vis = vis_disparity(disp)
        vis = np.concatenate([left_ori, vis], axis=1)

    # remove invisible pixels
    yy,xx = np.meshgrid(np.arange(disp.shape[0]), np.arange(disp.shape[1]), indexing='ij')
    us_right = xx-disp
    invalid = us_right<0
    disp[invalid] = np.inf

    Q[:3,3] *= SCALE
    depth = Q[2,3]*baseline/disp

    if detection_masks:
        for i, mask_path in enumerate(detection_masks):
            mask = cv2.imread(mask_path)
            if SCALE <1:
                mask = cv2.resize(mask, fx=SCALE, fy=SCALE, dsize=None)
            xyz_map = depth2xyzmap(depth, Q, mask = mask)
            pcd = toOpen3dCloud(xyz_map.reshape(-1,3), left_ori.reshape(-1,3))
            keep_mask = (np.asarray(pcd.points)[:,2]>0) & (np.asarray(pcd.points)[:,2]<=Z_FAR)
            keep_ids = np.arange(len(np.asarray(pcd.points)))[keep_mask]
            pcd = pcd.select_by_index(keep_ids)
            print(json.dumps({"success": True, "mask_index": i, "point_count": len(np.asarray(pcd.points))}), flush=True)
    
            # # denoise point cloud TODO: denoising sends timeout
            # cl, ind = pcd.remove_radius_outlier(nb_points=30, radius=1.0)
            # inlier_cloud = pcd.select_by_index(ind)
            o3d.io.write_point_cloud(f'{out_path}_cloud_{i}_.ply', pcd)
            # pcd = inlier_cloud

    cv2.imwrite(f'{out_path}_depthmap.png', disp)
    print(json.dumps({"success": True, }), flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
