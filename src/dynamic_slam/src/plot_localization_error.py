#!/usr/bin/env python3
import os
import csv
import matplotlib.pyplot as plt

def plot_paths(csv_path):
    gt_xs = []
    gt_ys = []
    est_xs = []
    est_ys = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            gt_xs.append(float(row[1]))
            gt_ys.append(float(row[2]))
            est_xs.append(float(row[4]))
            est_ys.append(float(row[5]))
    plt.figure(figsize=(8, 8))
    plt.plot(gt_xs, gt_ys, label='Ground Truth Path')
    plt.plot(est_xs, est_ys, label='Estimated Path')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    csv_file = '/home/jaafar/slam_ws/src/dynamic_slam/data/localization_error_20250802_074413.csv'
    if not os.path.exists(csv_file):
        print(f"File not found: {csv_file}")
    else:
        plot_paths(csv_file)
