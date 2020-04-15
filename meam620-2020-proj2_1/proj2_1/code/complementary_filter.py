# %% Imports

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


# %%

def complementary_filter_update(initial_rotation, angular_velocity, linear_acceleration, dt):
    """
    Implements a complementary filter update

    :param initial_rotation: rotation_estimate at start of update
    :param angular_velocity: angular velocity vector at start of interval in radians per second
    :param linear_acceleration: linear acceleration vector at end of interval in meters per second squared
    :param dt: duration of interval in seconds
    :return: final_rotation - rotation estimate after update
    """

    #TODO Your code here - replace the return value with one you compute
    # initial_rotation = initial_rotation.as_matrix()
    r = Rotation.from_rotvec(angular_velocity*dt)

    updated_rotation = initial_rotation * r
    g_prime  = updated_rotation.as_matrix() @ (linear_acceleration/9.81)
    g_prime = g_prime/norm(g_prime)
    r_acc = Rotation.from_quat([0,g_prime[2]/np.sqrt((1+g_prime[0])*2),-g_prime[1]/np.sqrt((1+g_prime[0])*2),np.sqrt((1+g_prime[0])/2)])

    e_m= np.abs((norm(linear_acceleration/9.81)-1))

    if e_m < 0.1:
        alpha = 1 
    elif e_m >= 0.1 and e_m < 0.2:
        alpha = 2 - 10 * e_m
    elif e_m >= 0.2:
        alpha = 0

    r_acc_prime = (1-alpha)*np.array([0,0,0,1]) + alpha*r_acc.as_quat()
    r_acc_prime = r_acc_prime/norm(r_acc_prime)
    R_acc = Rotation.from_quat(r_acc_prime)
    # print(R_acc.as_euler('XYZ'))
    
    return R_acc *updated_rotation
    
    
    
    # return Rotation.identity()
