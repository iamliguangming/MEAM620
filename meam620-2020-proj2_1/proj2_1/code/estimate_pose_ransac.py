# Imports

import numpy as np
from scipy.spatial.transform import Rotation


# %%

def estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold):
    """
    Estimate Pose by repeatedly calling ransac

    :param uvd1:
    :param uvd2:
    :param pose_iterations:
    :param ransac_iterations:
    :param ransac_threshold:
    :return: Rotation, R; Translation, T; inliers, array of n booleans
    """

    R = Rotation.identity()

    for i in range(0, pose_iterations):
        w, t, inliers = ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold)
        R = Rotation.from_rotvec(w.ravel()) * R

    return R, t, inliers


def ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold):
    # find total number of correspondences
    n = uvd1.shape[1]

    # initialize inliers all false
    best_inliers = np.zeros(n, dtype=bool)

    for i in range(0, ransac_iterations):
        # Select 3  correspondences
        selection = np.random.choice(n, 3, replace=False)

        # Solve for w and  t
        w, t = solve_w_t(uvd1[:, selection], uvd2[:, selection], R)

        # find inliers
        inliers = find_inliers(w, t, uvd1, uvd2, R, ransac_threshold)

        # Update best inliers
        if inliers.sum() > best_inliers.sum():
            best_inliers = inliers.copy()

    # Solve for w and t using best inliers
    w, t = solve_w_t(uvd1[:, best_inliers], uvd2[:, best_inliers], R)

    return w, t, find_inliers(w, t, uvd1, uvd2, R, ransac_threshold)



def find_inliers(w, t, uvd1, uvd2, R0, threshold):
    """

    find_inliers core routine used to detect which correspondences are inliers

    :param w: ndarray with 3 entries angular velocity vector in radians/sec
    :param t: ndarray with 3 entries, translation vector
    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2:  3xn ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param threshold: Threshold to use
    :return: ndarray with n boolean entries : Only True for correspondences that pass the test
    """


    n = uvd1.shape[1]
    A = np.zeros((2*n,6))
    b = np.zeros((2*n))
    x = np.concatenate((w,t),axis=0)
    for i in range(n):
        u1_prime = uvd1[0,i]
        u2_prime = uvd2[0,i]
        v1_prime = uvd1[1,i]
        v2_prime = uvd2[1,i]
        d1_prime = uvd1[2,i]
        d2_prime = uvd2[2,i]
        scale_ratio = d2_prime/d1_prime
        # u1_prime = u1_prime * scale_ratio
        # v1_prime = v1_prime * scale_ratio
        y = R0.as_matrix() @ np.array([u2_prime,v2_prime,1])
        b[2*i:2*i+2] = -np.array([[1,0,-u1_prime],[0,1,-v1_prime]]) @ y
        A[2*i:2*i+2,:] = np.array([[1,0,-u1_prime],[0,1,-v1_prime]])@np.array([[0,y[2],-y[1],d2_prime,0,0],
                                                                               [-y[2],0,y[0],0,d2_prime,0],
                                                                               [y[1],-y[0],0,0,0,d2_prime]])
    # bool_array = np.abs(A@x.flatten()-b)<threshold
    difference_array = A@x.flatten() - b
    return_array = np.zeros(n,dtype='bool')
    for i in range(n):
        # if bool_array[2*i] and bool_array[2*i+1]:
        if np.sqrt(difference_array[2*i]**2 + difference_array[2*i+1]**2) < threshold:
            return_array[i] = True
        else:
            return_array[i] = False
    return return_array
    # TODO Your code here replace the dummy return value with a value you compute
    # return np.zeros(n, dtype='bool')

def solve_w_t(uvd1, uvd2, R0):
    """
    solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    """

    # TODO Your code here replace the dummy return value with a value you compute
    w = t = np.zeros((3,1))
    n = uvd1.shape[1]
    A = np.zeros((2*n,6))
    b = np.zeros((2*n))
    for i in range(n):
        u1_prime = uvd1[0,i]
        u2_prime = uvd2[0,i]
        v1_prime = uvd1[1,i]
        v2_prime = uvd2[1,i]
        d1_prime = uvd1[2,i]
        d2_prime = uvd2[2,i]
        # scale_ratio = d2_prime/d1_prime
        # u1_prime = u1_prime * scale_ratio
        # v1_prime = v1_prime * scale_ratio
        y = R0.as_matrix() @ np.array([u2_prime,v2_prime,1])
        b[2*i:2*i+2] = -np.array([[1,0,-u1_prime],[0,1,-v1_prime]]) @ y
        A[2*i:2*i+2,:] = np.array([[1,0,-u1_prime],[0,1,-v1_prime]])@np.array([[0,y[2],-y[1],d2_prime,0,0],
                                                                               [-y[2],0,y[0],0,d2_prime,0],
                                                                               [y[1],-y[0],0,0,0,d2_prime]])
    x = np.linalg.lstsq(A,b,rcond=None)[0].reshape(6,1)
    w = x[0:3]
    t = x[3:6]
        

    
    
    return w, t
