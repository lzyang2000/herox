import casadi as ca
import numpy as np
import math


class TurtleModel:

    def __init__(self):

        self.state_bound_lb = [-10, -10, -2*np.pi, -1, -1, np.radians(-45.0)]  
        self.state_bound_ub = [10, 10, 2*np.pi, 1, 1, np.radians(45.0)]  
        self.input_bound_lb = [-10, -10, np.radians(-90.0)]  
        self.input_bound_ub = [10, 10, np.radians(90.0)]  

    def get_model(self):
        x = ca.SX.sym("x", 6)
        u = ca.SX.sym("u", 3)
        
        x_dot = x[3]
        y_dot = x[4]
        yaw_dot = x[5]
        x_ddot = u[0]
        y_ddot = u[1]
        yaw_ddot = u[2] 

        dx = ca.vertcat(x_dot, y_dot, yaw_dot, x_ddot, y_ddot, yaw_ddot)

        # vx = x[3]
        # vy = x[4]
        # vyaw = x[5]

        # x_p = x[0] + vx * np.cos(x[2]) * dt - vy * np.sin(x[2]) * dt
        # y_p = x[1] + vx * np.sin(x[2]) * dt + vy * np.cos(x[2]) * dt
        # theta_p = x[2] + vyaw * dt
        # x_pp = u[0]
        # y_pp = u[1]
        # yaw_pp = u[2]
        
        # dx = ca.vertcat(x_p, y_p, theta_p, x_pp, y_pp, yaw_pp)

        return ca.Function("double_integrator", [x, u], [dx])


