import casadi as ca
from turtle_model import TurtleModel
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


class TurtleOpti:
    def __init__(self):
        self.turtle = TurtleModel()
        self.turtle_model = self.turtle.get_model()
        
        # state
        # [x, y, yaw, dx, dy, dyaw]
        # input
        # [ddx ddy ddyaw]

        self.state_num = 6
        self.input_num = 3
        self.grid_num = 30 + 1
        # self.time_bounds = [2, 3]
        self.time = 1.0
        self.time = float(self.time)

        # self.w_final_state = [400, 400, 1]
        self.w_state = np.array([0, 0, 0, 30, 20, 20]).reshape((1, 6))  # x dx
        self.w_inputs = np.array([2, 2, 2]).reshape((1, 3))
        self.w_slack_final_state = np.array([10000, 10000, 10000]).reshape((1, 3))
        self.w_slack_final_statedot = np.array([100000, 100000, 100000]).reshape((1, 3))        
 
        self.opts_setting = {"ipopt.max_iter": 500, "verbose": False, "ipopt.print_level": 0, "print_time": 0}
        self.reset()

    def reset(self):
        self.opti = ca.Opti()
        self.opti.solver("ipopt", self.opts_setting)
        self.states = self.opti.variable(self.state_num, self.grid_num)
        self.inputs = self.opti.variable(self.input_num, self.grid_num - 1)
        
        self.init_state = np.zeros((self.state_num, 1))
        self.goal_state = np.zeros((self.state_num, 1))
        self.slack_final = self.opti.variable(6)
        self.total_cost = 0

    def __set_init_state(self, init_state):
        self.init_state = init_state

    def set_obstacle(self, obstacle):
        self.obstacle = obstacle

    def __set_goal_states(self, goal_state):
        self.goal_state = goal_state

    def get_state_num(self):
        return self.state_num

    def __add_final_node_cost(self, final_cost):
        self.total_cost = self.total_cost + final_cost

    def __add_final_slack_cost(self, slack_final):
        self.node_slack_final_cost = ca.mtimes(self.w_slack_final_state, slack_final[[0,1,2]] ** 2)
        self.node_slack_final_cost += ca.mtimes(self.w_slack_final_statedot, slack_final[[3,4,5]] ** 2)
        self.total_cost = self.total_cost + self.node_slack_final_cost

    def __add_node_input_cost(self, u):
        self.input_cost = ca.mtimes(self.w_inputs, u**2)
        self.total_cost = self.total_cost + self.input_cost

    def __add_node_state_cost(self, x):
        self.node_state_cost = ca.mtimes(self.w_state, x**2)
        self.total_cost = self.total_cost + self.node_state_cost

    def solve(self,curr_state,goal_state):
        self.reset()
        self.__set_init_state(curr_state)
        self.__set_goal_states(goal_state)
        # print 'init: ',self.init_state
        # print 'goal: ', self.goal_state
        dt = self.time / (self.grid_num - 1)

        # constrain the init state
        self.opti.subject_to(self.states[:, 0] == self.init_state)        
        last_statedot = [self.init_state[3], self.init_state[4], self.init_state[5], 0.0, 0.0, 0.0]

        for k in range(self.grid_num - 1):
            Xk = self.states[:, k]
            Uk = self.inputs[:, k]
   
            self.opti.subject_to(self.opti.bounded(self.turtle.input_bound_lb, Uk, self.turtle.input_bound_ub))
            Xk_next = self.states[:, k + 1]
            self.opti.subject_to(self.opti.bounded(self.turtle.state_bound_lb, Xk_next, self.turtle.state_bound_ub))
            state_dot = self.turtle_model(Xk, Uk)
            state_p = Xk + (state_dot+last_statedot)*dt/2.0

            self.opti.subject_to(state_p == Xk_next)
            self.__add_node_input_cost(Uk)
            self.__add_node_state_cost(Xk)
            last_statedot = state_dot
            
        self.opti.subject_to((self.states[[0,1], -1] - self.goal_state[[0,1]])**2 == self.slack_final[[0,1]])
        self.opti.subject_to(ca.cos(self.states[2, -1] - self.goal_state[2]) == 1-self.slack_final[2])
        self.opti.subject_to((self.states[[3,4,5], -1] - self.goal_state[[3,4,5]])**2 == self.slack_final[[3,4,5]])
        self.opti.subject_to(self.slack_final>=0)
        self.__add_final_slack_cost(self.slack_final)
        
        X_ini = self.__get_initial(self.init_state, self.goal_state)
        self.opti.set_initial(self.states, X_ini)
        self.opti.minimize(self.total_cost)

        sol = self.opti.solve()
        states_sol = sol.value(self.states)
        input_sol = sol.value(self.inputs)
        # print(sol.value(self.slack_final))
        # try:
        #     sol = self.opti.solve()
        #     states_sol = sol.value(self.states)
        #     input_sol = sol.value(self.inputs)
        #     print(sol.value(state_dot_list))
        # except:
        #     states_sol = self.opti.debug.value(self.states)
        #     input_sol = self.opti.debug.value(self.inputs)
        return states_sol, input_sol, self.time

    def __get_initial(self, initState, finalState):
        # total_time_guess = 0.5*(self.time_bounds[0] + self.time_bounds[1])
        grid = np.linspace(0, 1, self.grid_num)
        x0 = initState[0] + grid * (finalState[0] - initState[0])
        y0 = initState[1] + grid * (finalState[1] - initState[1])
        theta0 = initState[2] + grid * (finalState[2] - initState[2])
        vx0 = np.zeros(x0.shape)
        vy0 = np.zeros(y0.shape)
        vtheta0 = np.zeros(theta0.shape)
        X0 = np.vstack([x0, y0, theta0, vx0, vy0, vtheta0])
        return X0


if __name__ == "__main__":
    co = TurtleOpti()
    import time

    start_time = time.time()
    co.set_init_state([0,0,0,0,0,0])
    co.set_goal_states([2,2,np.radians(50),0,0,0])
    co.solve()
    print("solved in:",time.time()-start_time)