import os
import scipy.io as sio
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree

def read_canonical_model(model_name):
    '''
    Read canonical model from .mat file
    model_name: str, 'drill' or 'liq_container'
    return: numpy array, (N, 3)
    '''
    model_fname = os.path.join('.\data', model_name, 'model.mat')
    model = sio.loadmat(model_fname)

    cano_pc = model['Mdata'].T / 1000.0 # convert to meter

    return cano_pc


def load_pc(model_name, id):
    '''
    Load point cloud from .npy file
    model_name: str, 'drill' or 'liq_container'
    id: int, point cloud id
    return: numpy array, (N, 3)
    '''
    pc_fname = os.path.join('./data', model_name, '%d.npy' % id)
    pc = np.load(pc_fname)

    return pc

def visualize_icp_result(source_pc, target_pc, pose):
    '''
    Visualize the result of ICP
    source_pc: numpy array, (N, 3)
    target_pc: numpy array, (N, 3)
    pose: SE(4) numpy array, (4, 4)
    '''
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_pc.reshape(-1, 3))
    source_pcd.paint_uniform_color([0, 0, 1])

    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_pc.reshape(-1, 3))
    target_pcd.paint_uniform_color([1, 0, 0])
    
    source_pcd.transform(pose)

    o3d.visualization.draw_geometries([source_pcd, target_pcd])

def transformation_matrix(theta, translation):

    # Create a rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Create a pose matrix with zero translation and rotation about the z-axis
    pose = np.eye(4)

    pose[:3, :3] = rotation_matrix[:3, :3]
    pose[:3, 3] = translation


    return pose

def kabsch_algorithm(Z, M):
    z_centroid = np.mean(Z, axis = 0)
    m_centroid = np.mean(M, axis = 0)

    Q = np.zeros((3,3))

    for i in range(M.shape[0]):
        Q += np.outer((M[i, :] - m_centroid), (Z[i, :] - z_centroid))

    # Singular Value Decomposition
    U, S, Vt = np.linalg.svd(Q)

    # Compute det(UV^T)
    det_UVt = np.linalg.det(U @ Vt)

    # Create the scaling matrix with ones and det(UV^T)
    scaling_matrix = np.eye(len(S))
    scaling_matrix[-1, -1] = det_UVt

    # Construct R using U and Vt
    R = U @ scaling_matrix @ Vt
    P = m_centroid - R @ z_centroid
    
    return R, P

def icp_algorithm(T0, Z, M):
    max_iteration = 100
    R = T0[:3, :3]
    P = T0[:3, 3]

    tree = KDTree(M)

    for i in range(max_iteration):
        Z_tilda = (R @ Z.T + P[:, np.newaxis]).T
        
        # D2 = np.sum(M_tilda[...,None,:] - Z[None, ...]**2, axis = 2)
        # jj = np.argmin(D2, axis =1)

        # Use KDTree query with specified number of neighbors (k=1)
        dd, jj = tree.query(Z_tilda)
        M_tilda = M[jj, :]
        
        R, P = kabsch_algorithm(Z, M_tilda)


    
    transformed_matrix = np.eye(4)
    transformed_matrix[:3,:3] = R
    transformed_matrix[:3,3] = P

    return transformed_matrix


obj_name = 'drill' # drill or liq_container
num_pc = 4 # number of point clouds

source_pc = read_canonical_model(obj_name)

for i in range(num_pc):
    target_pc = load_pc(obj_name, i)

    # estimated_pose, you need to estimate the pose with ICP
    # initial transform matriax
    yaw = np.pi/3 # Adjust this angle as needed
    z_bar = np.mean(source_pc, axis = 0)
    m_bar = np.mean(target_pc, axis = 0)
    
    translation = m_bar - z_bar

    T0 = transformation_matrix(yaw, translation)

    # call the icp algorithm
    pose = icp_algorithm(T0, source_pc, target_pc)

    # visualize the estimated result
    visualize_icp_result(source_pc, target_pc, pose)
