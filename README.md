# Iterative-Closest-Point
This repo presents an algorithm to find the closest point in the another point cloud iteratively, therefore also 
known as Iterative Closest Point (ICP) algorithm.

This repo is a part of the course project and the data is provided by the instructor. Data incorporates
two point clouds for the Drill object, one source point cloud and the other target point cloud.

Program return the translation and orientation in between two point clouds and it also plots the overlapping
3D source and target point clouds to visulalize the convergance of the results.

# Dependencies
import os
import scipy.io as sio
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
