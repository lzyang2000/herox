#!/usr/bin/env python
import rospy
from casadi import Opti, sin, cos, tan, vertcat, mtimes
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import math
class turtle_opti:
    def __init__(self):
        
        
        self.n = 100
        self.dt = 0.2
        self.L = 0.3
        self.xy_low = [-100, -100]
        self.xy_high = [100, 100]
        self.u1_max = 0.4
        self.u2_max = 0.8
        self.obs_list = [[5, 5, 1], [-3, 4, 1], [4, 2, 2]]
        self.Q = np.diag([1, 1, 2])
        self.R = 2 * np.diag([1, 0.5])
        self.P = self.n * self.Q
        ###### SETUP PROBLEM ######
        
        self.q_lb = self.xy_low + [-1000]
        self.q_ub = self.xy_high + [1000]

        self.u_lb = [-self.u1_max, -self.u2_max]
        self.u_ub = [self.u1_max, self.u2_max]
        self.global_path = None
        self.q_goal = None
        self.q_start = None
        
        self.opts_setting = {"ipopt.max_iter": 1000, "verbose": False, "ipopt.print_level": 0, "print_time": 0}
        
        self.replan_period = rospy.get_param('/local_planner/replan_period', 1.0/20.0)
        self.timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.replan_cb)
        self.timer_localplanner_target = rospy.Timer(rospy.Duration(1/10.0), self.update_next_waypoint)
        self.sub_curr_state = rospy.Subscriber('/correct_odom', Odometry, self.curr_pose_cb, queue_size=1)
		# self.sub_target = rospy.Subscriber("/local_planner/local_planner_target", Float32MultiArray, self.target_cb, queue_size=1)
        self.range_next_waypoint = rospy.get_param('/local_planner/range_next_waypoint',0.3)
        self.sub_goal_state = rospy.Subscriber('/nav_path', Path, self.global_path_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=1)
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians     
    
    def curr_pose_cb(self,data):
        pose = data.pose.pose
        position = pose.position
        orientation = pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation.x,orientation.y,orientation.z,orientation.w)
        self.q_start = np.array([position.x, position.y, yaw]) 
    
    def global_path_callback(self,data):
        self.global_path = data
    
    def get_next_waypoint(self):
        range_selective = self.range_next_waypoint
        curr_position = self.q_start[:2]
        num_pick = 10
        start_pose = self.global_path.poses[0].pose
        if ((start_pose.position.x - curr_position[0])**2 + (start_pose.position.y - curr_position[1])**2) - range_selective**2 >= 0:
            tg_x, tg_y = start_pose.position.x, start_pose.position.y
            theta_target = start_pose.orientation.w
            return np.asarray([tg_x, tg_y, theta_target])
        for ind in range(0, len(self.global_path.poses)-1):
            ind = min(ind,len(self.global_path.poses)-2)
            pose = self.global_path.poses[ind].pose
            pose_next = self.global_path.poses[min(ind+1,len(self.global_path.poses)-1)].pose
            point = np.asarray([pose.position.x, pose.position.y])
            point_next = np.asarray([pose_next.position.x, pose_next.position.y])
            for i in range(0, num_pick):
                point1_mid = (point_next - point)/num_pick*(i) + point
                point2_mid = (point_next - point)/num_pick*(i+1) + point
                theta1_mid = (pose_next.orientation.w-pose.orientation.w)/num_pick*(i)+pose.orientation.w
                dist1 = sum((point1_mid - curr_position) ** 2) - range_selective ** 2
                dist2 = sum((point2_mid - curr_position) ** 2) - range_selective ** 2
                if (dist1 * dist2 <= 0):
                    tg_x, tg_y = point1_mid[0], point1_mid[1]
                    theta_target = theta1_mid
                    return np.asarray([tg_x, tg_y, theta_target])
        last_pose = self.global_path.poses[-1].pose
        theta_target = last_pose.orientation.w
        tg_x,tg_y=last_pose.position.x, last_pose.position.y
        return np.asarray([last_pose.position.x, last_pose.position.y, theta_target])

    def update_next_waypoint(self,event):
        if  self.q_start is not None and self.global_path is not None:
            self.q_goal =  self.get_next_waypoint()        

    def turtlebot_model(self, q, u, dt):
        """
        Implements the discrete time dynamics of your robot.
        i.e. this function implements F in

        q_{t+1} = F(q_{t}, u_{t})

        dt is the discretization timestep.
        L is the axel-to-axel length of the car.

        q = array of casadi MX.sym symbolic variables [x, y, theta, phi].
        u = array of casadi MX.sym symbolic variables [u1, u2] (velocity and steering inputs).

        Use the casadi sin, cos, tan functions.

        The casadi vertcat or vcat functions may also be useful. Note that to turn a list of 
        casadi expressions into a column vector of those expressions, you should use vertcat or
        vcat. vertcat takes as input a variable number of arguments, and vcat takes as input a list.

        Example:
            x = MX.sym('x')
            y = MX.sym('y')
            z = MX.sym('z')

            q = vertcat(x + y, y, z) # makes a 3x1 matrix with entries x + y, y, and z.
            # OR
            q = vcat([x + y, y, z]) # makes a 3x1 matrix with entries x + y, y, and z.
        """
        q_dot =  vertcat(cos(q[2]), sin(q[2]), 0) * u[0] + vertcat(0, 0, 1)*u[1]

        return q + q_dot*dt

    def initial_cond(self, q_start, q_goal, n):
        """
        Construct an initial guess for a solution to "warm start" the optimization.

        An easy way to initialize our optimization is to say that our robot will move 
        in a straight line in configuration space. Of course, this isn't always possible 
        since our dynamics are nonholonomic, but we don't necessarily need our initial 
        condition to be feasible. We just want it to be closer to the final trajectory 
        than any of the other simple initialization strategies (random, all zero, etc).

        We'll set our initial guess for the inputs to zeros to hopefully bias our solver into
        picking low control inputs

        n is the number of timesteps.

        This function will return two arrays: 
            q0 is an array of shape (4, n+1) representing the initial guess for the state
            optimization variables.

            u0 is an array of shape (2, n) representing the initial guess for the state
            optimization variables.
        
        """
        q0 = np.zeros((3, n + 1))
        u0 = np.zeros((2, n))

        diff = self.q_goal - self.q_start
        step_linear = np.linspace(0, 1, n+1)
        for i in range(n+1):
            q0[:, i] = step_linear[i]*diff + self.q_start
        
        return q0, u0

    def objective_func(self, q, u, q_goal, Q, R, P):
        """
        Implements the objective function. q is an array of states and u is an array of inputs. Together,
        these two arrays contain all the optimization variables in our problem.

        In particular, 

        q has shape (4, N+1), so that each column of q is an array q[:, i] = [q0, q1, q2, q3]
        (i.e. [x, y, theta, phi]), the state at time-step i. 

        u has shape (2, N), so that each column of u is an array u[:, i] = [u1, u2], the two control inputs 
        (velocity and steering) of the bicycle model.

        This function should create an expression of the form

        sum_{i = 1, ..., N} ((q(i) - q_goal)^T * Q * (q(i) - q_goal) + (u(i)^T * R * u(i)))
        + (q(N+1) - q_goal)^T * P * (q(N+1) - q_goal)

        Note: When dealing with casadi symbolic variables, you can use @ for matrix multiplication,
        and * for standard, numpy-style element-wise (or broadcasted) multiplication.
        
        """

        n = q.shape[1] - 1
        obj = 0
        for i in range(n):
            qi = q[:, i]
            ui = u[:, i]

            # Define one term of the summation here: ((q(i) - q_goal)^T * Q * (q(i) - q_goal) + (u(i)^T * R * u(i)))
            term = mtimes((qi - q_goal).T ,mtimes(Q ,(qi - q_goal))) + mtimes(ui.T, mtimes(R, ui))
            obj += term

        q_last = q[:, n]
        # Define the last term here: (q(N+1) - q_goal)^T * P * (q(N+1) - q_goal)
        term_last = mtimes((q_last - q_goal).T , mtimes(P, (q_last - q_goal)))
        obj += term_last
        return obj

    def constraints(self, q, u, q_lb, q_ub, u_lb, u_ub, obs_list, q_start, q_goal, L=0.3, dt=0.01):
        """
        Constructs a list where each entry is a casadi.MX symbolic expression representing
        a constraint of our optimization problem.

        q has shape (4, N+1), so that each column of q is an array q[:, i] = [q0, q1, q2, q3]
        (i.e. [x, y, theta, phi]), the state at time-step i. 

        u has shape (2, N), so that each column of u is an array u[:, i] = [u1, u2], the two control inputs 
        (velocity and steering) of the bicycle model.

        q_lb is a size (4,) array [x_lb, y_lb, theta_lb, phi_lb] containing lower bounds for each state variable.

        q_ub is a size (4,) array [x_ub, y_ub, theta_ub, phi_ub] containing upper bounds for each state variable.

        u_lb is a size (2,) array [u1_lb, u2_lb] containing lower bounds for each input.

        u_ub is a size (2,) array [u1_ub, u2_ub] containing upper bounds for each input.

        obs_list is a list of obstacles, where each obstacle is represented by  3-tuple (x, y, r)
                representing the (x, y) center of the obstacle and its radius r. All obstacles are modelled as
                circles.

        q_start is a size (4,) array representing the starting state of the plan.

        q_goal is a size (4,) array representing the goal state of the plan.

        L is the axel-to-axel length of the car.

        dt is the discretization timestep.

        """
        constraints = []

        # State constraints
        constraints.extend([q_lb[0] <= q[0, :], q[0, :] <= q_ub[0]])
        constraints.extend([q_lb[1] <= q[1, :], q[1, :] <= q_ub[1]]) # TODO
        constraints.extend([q_lb[2] <= q[2, :], q[2, :] <= q_ub[2]]) # TODO
        
        # Input constraints
        constraints.extend([u_lb[0] <= u[0, :], u[0, :] <= u_ub[0]])
        constraints.extend([u_lb[1] <= u[1, :], u[1, :] <= u_ub[1]])

        # Dynamics constraints
        for t in range(q.shape[1] - 1):
            q_t   = q[:, t]
            q_tp1 = q[:, t + 1]
            u_t   = u[:, t]
            constraints.append(q_tp1 == self.turtlebot_model(q_t, u_t,self.dt)) # You should use the turtlebot_model function here somehow.

        # # Obstacle constraints
        # for obj in obs_list:
        #     obj_x, obj_y, obj_r = obj
        #     for t in range(q.shape[1]):
        #         constraints.append((q[0, t]-obj_x)**2 + (q[1, t]-obj_y)**2 >= obj_r**2) # Define the obstacle constraints.

        # Initial and final state constraints
        constraints.append(q_start==q[:, 0]) # Constraint on start state.
        constraints.append(q_goal==q[:, -1]) # Constraint on final state.

        return constraints

    def plan_to_pose(self, q_start, q_goal, q_lb, q_ub, u_lb, u_ub, obs_list, L=0.3, n=1000, dt=0.01):
        """
        Plans a path from q_start to q_goal.

        q_lb, q_ub are the state lower and upper bounds repspectively.
        u_lb, u_ub are the input lower and upper bounds repspectively.
        obs_list is the list of obstacles, each given as a tuple (x, y, r).
        L is the length of the car.
        n is the number of timesteps.
        dt is the discretization timestep.

        Returns a plan (shape (4, n+1)) of waypoints and a sequence of inputs
        (shape (2, n)) of inputs at each timestep.
        """
    
        opti = Opti()

        q = opti.variable(3, n + 1)
        u = opti.variable(2, n)

        

        q0, u0 = self.initial_cond(self.q_start, self.q_goal, self.n)

        obj = self.objective_func(q, u, self.q_goal, self.Q, self.R, self.P)

        opti.minimize(obj)

        opti.subject_to(self.constraints(q, u, self.q_lb, self.q_ub, self.u_lb, self.u_ub, self.obs_list, self.q_start, self.q_goal, dt=self.dt))

        opti.set_initial(q, q0)
        opti.set_initial(u, u0)

        ###### CONSTRUCT SOLVER AND SOLVE ######

        opti.solver('ipopt', self.opts_setting)
        sol = opti.solve()

        plan = sol.value(q)
        inputs = sol.value(u)
        return plan, inputs

    def plot(self, plan, inputs, times, q_lb, q_ub, obs_list):

        # Trajectory plot
        ax = plt.subplot(1, 1, 1)
        ax.set_aspect(1)
        ax.set_xlim(q_lb[0], q_ub[0])
        ax.set_ylim(q_lb[1], q_ub[1])

        for obs in obs_list:
            xc, yc, r = obs
            circle = plt.Circle((xc, yc), r, color='black')
            ax.add_artist(circle)

        plan_x = plan[0, :]
        plan_y = plan[1, :]
        ax.plot(plan_x, plan_y, color='green')

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")

        plt.show()

        # States plot
        plt.plot(times, plan[0, :], label='x')
        plt.plot(times, plan[1, :], label='y')
        plt.plot(times, plan[2, :], label='theta')
        plt.plot(times, plan[3, :], label='phi')

        plt.xlabel('Time (s)')
        plt.legend()
        plt.show()

        # Inputs plot
        plt.plot(times[:-1], inputs[0, :], label='u1')
        plt.plot(times[:-1], inputs[1, :], label='u2')
        
        plt.xlabel('Time (s)')
        plt.legend()
        plt.show()

    def replan_cb(self, event):
        if self.q_start is not None and self.q_goal is not None and self.global_path is not None:
            try:
                plan, inputs = self.plan_to_pose(self.q_start, self.q_goal, self.q_lb, self.q_ub, self.u_lb, self.u_ub, self.obs_list, L=self.L, n=self.n, dt=self.dt)
                rate = rospy.Rate(10)
                for i in range(min(len(inputs[0]),10)):
                    print(inputs[:,i])
                    self.cmd_publish(inputs[:,i])
                    rate.sleep()
            except:
                print("Error")
                print(self.q_start, self.q_goal)
    
    def cmd_publish(self,inputt):
        cmd = Twist()
        cmd.linear.x=inputt[0]
        cmd.linear.y=0
        cmd.linear.z=0
        cmd.angular.x=0
        cmd.angular.y=0
        cmd.angular.z=inputt[1]
        self.cmd_pub.publish(cmd)
        
# def main():
    
#     ###### PROBLEM PARAMS ######

    

#     ###### CONSTRUCT SOLVER AND SOLVE ######

    

#     ###### PLOT ######

#     times = np.arange(0.0, (n + 1) * dt, dt)
#     print("Final Position:", plan[:4, -1])
#     plot(plan, inputs, times, q_lb, q_ub, obs_list)

if __name__ == '__main__':
    try:
        print 'start'
        rospy.init_node("local_planner")
        a=turtle_opti()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
