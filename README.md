# Iterative-Closest-Point
This repo presents an algorithm to find the closest point in another point cloud iteratively, therefore also 
known as the Iterative Closest Point (ICP) algorithm.

This repo is a part of the course project, and the instructor provides the data. Data incorporates
two point clouds for the Drill object, one source point cloud and the other target point cloud.

The program returns the translation and orientation between two point clouds, and it also plots the overlapping
3D source and target point clouds to visualise the convergence of the results.

# Dependencies
Mentioned under first part of the code under [import] section. 
import os
import scipy.io as sio
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
