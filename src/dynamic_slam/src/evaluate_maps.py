#!/usr/bin/env python3

import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim
import os
from ament_index_python.packages import get_package_share_directory
from scipy.spatial import cKDTree

def load_map(map_path):
    image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not load map: {map_path}")
    return image


def normalized_nearest_cell_error(gt_map, test_map):
    gt_occ = np.where(gt_map < 50) 
    test_occ = np.where(test_map < 50)  

    if len(gt_occ[0]) == 0 or len(test_occ[0]) == 0:
        return float('inf')

    gt_points = np.array(list(zip(gt_occ[1], gt_occ[0])))
    test_points = np.array(list(zip(test_occ[1], test_occ[0])))

    tree = cKDTree(test_points)
    distances, _ = tree.query(gt_points)
    return np.mean(distances)

def main():
    pkg_dir = get_package_share_directory("dynamic_slam")
    maps_dir = os.path.join(pkg_dir, "maps")

    gt_map_path = os.path.join(maps_dir, "Ground_Truth_save.pgm")
    test_map_path = os.path.join(maps_dir, "test_median.pgm")

    gt = load_map(gt_map_path)
    test = load_map(test_map_path)

    if gt.shape != test.shape:
        test = cv2.resize(test, (gt.shape[1], gt.shape[0]))

    sim_score, _ = ssim(gt, test, full=True)
    ne_score = normalized_nearest_cell_error(gt, test)

    print(f"SSIM: {sim_score:.4f}")
    print(f"Normalized NE: {ne_score:.2f} px")

if __name__ == "__main__":
    main()
