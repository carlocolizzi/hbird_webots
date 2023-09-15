import numpy as np
from math import sin, cos
from utils import *

class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, hbparams, pid_gains, dt=0):
        """
        Inputs:
        - hbparams (HBarams dataclass):             model parameter class for the drone
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params = hbparams


        ### Control Gains ###
        # translational 
        self.kp_x = pid_gains.kp_x
        self.kp_y = pid_gains.kp_y
        self.kp_z = pid_gains.kp_z
        #-----
        self.ki_x = pid_gains.ki_x
        self.ki_y = pid_gains.ki_y
        self.ki_z = pid_gains.ki_z
        #-----
        self.kd_x = pid_gains.kd_x
        self.kd_y = pid_gains.kd_y
        self.kd_z = pid_gains.kd_z

        # rotational 
        self.kp_phi = pid_gains.kp_phi
        self.kp_theta = pid_gains.kp_theta
        self.kp_psi = pid_gains.kp_psi
        #-----
        self.ki_phi = pid_gains.ki_phi
        self.ki_theta = pid_gains.ki_theta
        self.ki_psi = pid_gains.ki_psi
        #-----
        self.kd_p = pid_gains.kd_p
        self.kd_q = pid_gains.kd_q
        self.kd_r = pid_gains.kd_r


        self.error = State()



    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        """

        x_dd = (self.error.x)

        self.error.z
        Ux = m * (x_dd)
        Uy = m * y_dd
        Uz = m * (z_dd + g)
        Uyaw = 

        U = np.array([0.,0.,0.,0.])

        # your code here

        

        return U