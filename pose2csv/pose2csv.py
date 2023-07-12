#!/usr/bin/python3
import argparse
import os

import numpy as np
import bagpy
from bagpy import bagreader

from sklearn import metrics
from sklearn.cluster import DBSCAN
import open3d as o3d


def parse_args():
    parser = argparse.ArgumentParser(
        prog="pose2csv.py",
        description="Finds object poses using unsupervised clustering.",
    )

    parser.add_argument('bagfile', help="Path to bagfile.")
    
    parser.add_argument('-O', '--output-path',
        help="Output CSV file path.",
        default="~/obj_pose.csv")
    
    parser.add_argument('-V', '--visualizer', 
        action='store_true',
        default=False)
    
    args = parser.parse_args()
    return args

# input bagfile. Need /object_detector/detection_info topic and /tf or /tf_static world cordinates
# output is clusters, cluster centers with class label

def cluster_points(bagfile: str):
    b = bagreader(bagfile)
    print()


# input pcd file, cluster_points output
# ouput visualization with colors for each class, cluster centers
def visualize():
    pass

# input cluster centers, class labels
# output csv file: object_number, x, y, z
def save_csv():
    pass

if __name__ == "__main__":
    args = parse_args()
    print(args.bagfile)
    print(args.output_path)
    print(args.visualizer)

    cluster_points(args.bagfile)

    if args.visualizer:
        visualize()
