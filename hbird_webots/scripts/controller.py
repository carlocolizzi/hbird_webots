import numpy as np
from math import sin, cos

from hbird_msgs.msg import Waypoint, State  #need to figure this out, file is in a different directory

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
        # translational                    ---P---
        self.kp_x = pid_gains.kp_x
        self.kp_y = pid_gains.kp_y
        self.kp_z = pid_gains.kp_z
        #                                  ---I---
        self.ki_x = pid_gains.ki_x
        self.ki_y = pid_gains.ki_y
        self.ki_z = pid_gains.ki_z
        #                                  ---D---
        self.kd_x = pid_gains.kd_x
        self.kd_y = pid_gains.kd_y
        self.kd_z = pid_gains.kd_z
        
        # rotational                       ---P---
        self.kp_phi = pid_gains.kp_phi     
        self.kp_theta = pid_gains.kp_theta
        self.kp_psi = pid_gains.kp_psi
        #                                  ---I---
        self.ki_phi = pid_gains.ki_phi
        self.ki_theta = pid_gains.ki_theta
        self.ki_psi = pid_gains.ki_psi
        #                                  ---D---
        self.kd_p = pid_gains.kd_p
        self.kd_q = pid_gains.kd_q
        self.kd_r = pid_gains.kd_r

        ### STATE MEMBERS

        self.integral = State()
        self.integral.x = 0.0
        self.integral.y = 0.0
        self.integral.z = 0.0
        self.integral.w = 0.0

        self.prev_error = State()
        self.prev_error.x = 0.0
        self.prev_error.y = 0.0
        self.prev_error.z = 0.0
        self.prev_error.w = 0.0

        self.error = State()
        self.error.x = 0.0
        self.error.y = 0.0
        self.error.z = 0.0
        self.error.w = 0.0

        ### PHYSICAL PARAMETERS

        self.m = 1.0    # mass of the drone
        self.I = 1.0    # moment of rotational inertia about z axis 
        self.g = 9.81   # gravity

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (Waypoint): the desired control setpoint
        - state (State):       the current state of the system
        Returns:
        - U (np.array):        array of control outputs {u1-u4} - Drone's resultant Thrust vector

        """
        # compute and store current positional/rotational error
        self.error = setpoint - state
        self.integral += self.error   # update integral terms

        # compute derivative (velocities)
        derivative = State()
        derivative = self.error - self.prev_error
        """
        x_dd_c = x_dd_d + (self.kp_x * derivative.x) + (self.kp_x * self.error.x)
        y_dd_c = y_dd_d + (self.kp_y * derivative.y) + (self.kp_y * self.error.y)

        phi_d = (1/ self.g)*((x_dd_c * sin(self.psi_r))- (y_dd_c * cos(self.psi_r)))
        theta_d = (1/ self.g)*((x_dd_c * cos(self.psi_r))- (y_dd_c * sin(self.psi_r)))
        psi_d = self.psi_r

        u2 = p_d_d + (self.kd_p * derivative.p) + (self.kp_p * self.error.phi)
        u3 = p_d_d + (self.kd_p * derivative.p) + (self.kp_p * self.error.phi)
        u4 = p_d_d + (self.kd_p * derivative.p) + (self.kp_p * self.error.phi)

        p_d = u2/Ixx
        q_d = u3/Iyy
        r_d = u4/Izz
        """




        # compute accelerations via PID control ( "_dd" means "doule dot" )
        x_dd = (self.kp_x * self.error.x) + (self.ki_x * self.integral.x) + (self.kd_x * derivative.x)
        y_dd = (self.kp_y * self.error.y) + (self.ki_y * self.integral.y) + (self.kd_y * derivative.y)
        z_dd = (self.kp_z * self.error.z) + (self.ki_z * self.integral.z) + (self.kd_z * derivative.z)
        w_dd = (self.kp_psi * self.error.w) + (self.ki_psi * self.integral.w) + (self.kd_r * derivative.w)

        # update controler's error (why here and now in the code?)
        self.prev_error = self.error

        # Use controller's physical parameters to compute drone's resultant control thrust vector
        Ux = self.m * x_dd
        Uy = self.m * y_dd
        Uz = self.m * (z_dd + self.g)
        Uw = self.I * w_dd

        U = np.array([Ux, Uy, Uz, Uw])

        return U

        
        