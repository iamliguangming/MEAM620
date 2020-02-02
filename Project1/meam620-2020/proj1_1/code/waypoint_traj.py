import numpy as np

class WaypointTraj(object):
    max_Velocity = 0.2
    max_Acceleration = 0.5
    """

    """
    def __init__(self, points):
        self.point_List = [np.array([0,0,0])]
        self.trajectory_List = []
        self.direction_List = []
        self.time_List = [0]
        for i in range(points.shape[0]):
            self.point_List.append(points[i,:])
        for i in range(len(self.point_List)-1):
            self.trajectory_List.append(self.point_List[i+1]-self.point_List[i])
            self.direction_List.append(self.trajectory_List[i]/np.linalg.norm(self.trajectory_List[i]))
            self.time_List.append(np.linalg.norm(self.trajectory_List[i])/self.max_Velocity + self.time_List[i])

            
        
        
        
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is a+n array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0
        
        for i in range(len(self.time_List)):
            if t == self.time_List[i] or t > self.time_List[-1]:
                x = self.point_List[i]
                x_dot = np.zeros((3,))
            elif t > self.time_List[i] and t< self.time_List[i+1]:
                x = (t - self.time_List[i])/(self.time_List[i+1] - self.time_List[i])*(self.point_List[i+1]-self.point_List[i]) + self.point_List[i]
                x_dot = self.max_Velocity * self.direction_List[i]
            else:
                pass
        
            

        # STUDENT CODE HERE

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
