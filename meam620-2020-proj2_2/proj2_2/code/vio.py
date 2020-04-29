#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state
    R = q.as_matrix()
    new_p = p + v * dt + 1/2 * (R @ (a_m - a_b) + g) * dt ** 2
    new_v = v + (R @ (a_m - a_b) + g) * dt 
    r_2= Rotation.from_rotvec(((w_m - w_b) * dt).flatten())
    new_q = q * r_2

    # YOUR CODE HERE

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state
    Fx = np.zeros((18,18))
    Fx[0:3,0:3] = np.identity(3)
    Fx[0:3,3:6] = np.identity(3) * dt
    Fx[3:6,3:6] = np.identity(3)
    delta_a = a_m - a_b
    Fx[3:6,6:9] = -q.as_matrix() @ np.array([[0,-delta_a[2],delta_a[1]],
                                           [delta_a[2],0,-delta_a[0]],
                                           [-delta_a[1],delta_a[0],0]]) * dt
    Fx[6:9,6:9] = q.as_matrix().T @ Rotation.from_rotvec(((w_m -w_b)*dt).flatten()).as_matrix()
    Fx[3:6,9:12] = -q.as_matrix() * dt
    Fx[3:6,15:18] = np.identity(3) * dt
    Fx[6:9,12:15] = - np.identity(3)*dt
    Fx[9:18,9:18] = np.identity(9)
    
    Fi = np.zeros((18,12))
    Fi[3:15,0:12] = np.identity(12)
    Qi = np.zeros((12,12))
    Qi[0:3,0:3] = accelerometer_noise_density ** 2 * dt**2 *np.identity(3)
    Qi[3:6,3:6] = gyroscope_noise_density ** 2 * dt ** 2 * np.identity(3)
    Qi[6:9,6:9] = accelerometer_random_walk ** 2 * dt * np.identity(3)
    Qi[9:12,9:12] = gyroscope_random_walk ** 2 * dt * np.identity(3)
    
    error_state_covariance = Fx @ error_state_covariance @ Fx.T + Fi @ Qi @ Fi.T
    
    return error_state_covariance
    
    
    

    # YOUR CODE HERE

    # return an 18x18 covariance matrix


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state
    (Xc, Yc, Zc) = (q.as_matrix().T @ (Pw - p)).flatten()
    innovation = uv - 1/Zc * np.array([[Xc],[Yc]])
    if np.linalg.norm(innovation) >= error_threshold:
            return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
    Pc_0 = q.as_matrix().T @ (Pw - p)
    del_zt_del_Pc = np.array([[1/Zc,0,-Xc/Zc**2],
                              [0,1/Zc,-Yc/Zc**2]])
    Pc_0_mat = np.array([[0,-Pc_0[2],Pc_0[1]],
                         [Pc_0[2],0,-Pc_0[0]],
                         [-Pc_0[1],Pc_0[0],0]])
    H = np.zeros((2,18))
    H[0:2,0:3] = del_zt_del_Pc @ (-q.as_matrix().T)
    H[0:2,6:9] = del_zt_del_Pc @ Pc_0_mat
    K_t = error_state_covariance @ H.T @ np.linalg.inv(H 
                                          @ error_state_covariance 
                                          @ H.T + Q)
    
    delta_x = K_t @ innovation
    p = p + delta_x[0:3]
    v = v + delta_x[3:6]
    q = Rotation.from_euler('XYZ',q.as_euler('XYZ')+ delta_x[6:9].flatten())
    a_b = a_b + delta_x[9:12]
    w_b = w_b + delta_x[12:15]
    g = g + delta_x[15:18]
    I = np.identity(18)
    error_state_covariance = (I - K_t @ H) @ error_state_covariance @ (
        I - K_t @ H).T + K_t @ Q @ K_t.T
    
    

    # YOUR CODE HERE - compute the innovation next state, next error_state covariance
    
    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
