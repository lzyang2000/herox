#!/usr/bin/env python
import numpy as np 
import sys
import rospy
import tf
import math
import matplotlib.pyplot as plt
import lcm
import sys, select, os
from local_planner_hop.msg import LocalPlan
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
from cmd_pack import cmd_lcm
from cmd_lcm2 import cmd_lcm2
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int16, Float64, Bool
from scipy.interpolate import interp1d
import time

control_mode = 1 # 0 for manual 1 for auto 

class Controller():
    def __init__(self):
        # rospy.init_node('control')
        self.hz = 100.0
        self.rate = rospy.Rate(self.hz)
        self.lc = lcm.LCM()
        # rospy.Subscriber('gazebo/model_states',ModelStates,loc_callback)
        self.curr_state = np.zeros(5)
        self.time_span = 0.0
        self.local_path_func = None
        self.__sub_local_path = rospy.Subscriber("/local_planner/local_plan", LocalPlan, self.__local_plan_cb, queue_size=1)    
        self.sub2 = rospy.Subscriber('/is_hop', Bool, self.is_hop_cb, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.listener = tf.TransformListener()
        self.curr_time = 0.0
        self.time_sol= 0
        self.local_plan = np.zeros([20,3])
        self.hop_fin_pub = rospy.Publisher('/hop_fin',Bool,queue_size=1)
        self.control_cmd = Twist()
        self.hop_lock = False
        if( control_mode == 1):
            self.auto()
        else:
            self.manual()

    def __global2local(self, xg, yg, yaw):
        # rotate yaw
        xl = xg*np.cos(yaw) + yg*np.sin(yaw)
        yl = -xg*np.sin(yaw) + yg*np.cos(yaw)
        return xl, yl

    def quart_to_rpy(self,x,y,z,w):
        r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        return y

    def cmd(self,data):
        self.control_cmd.linear.x = data[0]
        self.control_cmd.linear.y = data[1]
        self.control_cmd.angular.z = data[2]
        self.pub.publish(self.control_cmd)
        # print "current cmd: ", data
        msg = cmd_lcm2()
        msg.phase = data[3]
        msg.linear[0] = data[0]
        msg.linear[1] = data[1]
        msg.angular = data[2]
        self.lc.publish("cmd",msg.encode())
        
    def auto(self):
        while not rospy.is_shutdown():
            if self.local_path_func is not None and not self.hop_lock:
                self.curr_time = np.clip(self.curr_time+1.0/self.hz, 0.0, self.time_span-0.001)
                # print "cmd times:" , self.curr_time, self.time_span
                ref_states = self.local_path_func(self.curr_time)
                # print "ref_states: ", ref_states
                vx_local, vy_local = self.__global2local(ref_states[3],ref_states[4],ref_states[2])
                self.cmd([vx_local, vy_local, ref_states[5],1])
            self.rate.sleep()

    def __local_plan_cb(self, msg):
        self.time_span = msg.time_opti
        self.local_path_func = None
        state_dim = msg.states_opti.layout.dim[0].size
        grid_dim = msg.states_opti.layout.dim[1].size
        self.states_opti = np.zeros((state_dim, grid_dim))
        for i in range(grid_dim): 
            for j in range(state_dim): 
                self.states_opti[j,i] = msg.states_opti.data[j*grid_dim+i]
        time_seq = np.linspace(0.0, self.time_span, grid_dim, endpoint=True)
        self.curr_time = 0.0
        self.local_path_func = interp1d(time_seq, self.states_opti, axis=-1, kind='linear')
        
    def is_hop_cb(self,msg):
        if msg.data:
            self.hop_lock = True
            print "hopping!!!!!!!!!!!!"
            msg = cmd_lcm2()
            start = time.time()
            print 'walk'
            while (True): 
                if time.time() - start >= 4.3:
                    break
                
                self.cmd([0.1, 0, 0,1])
                self.rate.sleep()    
            print 'jump'
            start = time.time()
            msg.phase = 0
            msg.linear[0] = 0
            msg.linear[1] = 0
            msg.angular = 0
            self.lc.publish("cmd", msg.encode())
            print 'published 1'
            # wait two seconds
            while (True):                
                if time.time() - start >= 2:
                    break
            # jump
            msg.phase = 2
            msg.linear[0] = 0
            msg.linear[1] = 0
            msg.angular = 0
            self.lc.publish("cmd", msg.encode())
            print 'published 2'
            start = time.time()
            while (True):                
                if time.time() - start >= 5:
                    break

            start = time.time()
            while (True):                
                if time.time() - start >= 2:
                    break
                self.cmd([0.15, 0, 0,1])
                self.rate.sleep() 
            while (True):                
                if time.time() - start >= 5:
                    break
                self.cmd([0.05, 0, 0,1])
                self.rate.sleep()  
            hop_fin = Bool()
            hop_fin.data = True
            self.hop_fin_pub.publish(hop_fin)
            self.hop_lock = False
            print "hopping fin!!!!!!!!!!!!"
    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def manual(self):
        
        data = np.array([ 0.0, 0.0, 0.0, 1.0])
        while not rospy.is_shutdown():
            # print("333")
            key = self.getKey()
            if key == 'w':
                if(data[0]< 0.35):
                    data[0] = data[0] + 0.05
                else:
                    data = data
            elif key == 'x':
                if(data[0]> -0.35):
                    data[0] = data[0] - 0.05
                else: 
                    data = data
            elif key == 'a':
                if(data[2]< 0.6):
                    data[2] += 0.2
                else:
                    data = data
            elif key == 'd':
                if(data[2]> -0.6):
                    data[2] -= 0.2
                else:
                    data = data
            elif key == 's':
                data = np.array([ 0.0, 0.0, 0.0, 1.0])
            elif key == 'j':
                if(data[1]< 0.2):
                    data[1] += 0.05
                else:
                    data = data
            elif key == 'k':
                if(data[1]> -0.2):
                    data[1] -= 0.05
                else:
                    data = data

            # stand -> jump
            elif key == 'm':
                data[3] = 0
                start = time.time()                            
                self.cmd(data)
                self.rate.sleep()
                while (True):
                    if time.time() - start >= 2:
                        break

                data[3] = 2

            elif key == 'n':
                data[3] = 2
            
            elif (key == '\x03'):
                break
            else:
                data = data
            # print("111")
            self.cmd(data)
            # print("222")
            self.rate.sleep()
        
if __name__=='__main__':
    print 'start'
    rospy.init_node('control')
    controller = Controller()

    # rospy.spin()
    
