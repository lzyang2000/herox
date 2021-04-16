#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from gazebo_msgs.msg import ModelStates
import numpy as np
from turtle_opti import TurtleOpti
from scipy.interpolate import interp1d
from visualization_msgs.msg import Marker, MarkerArray
import math
import tf
class DescisionMaker:
    def __init__(self):
        self.turtle_opti=TurtleOpti()
        self.side = rospy.get_param('/map/map_size', 15.)
        self.head_x_offset = rospy.get_param('/head_x_offset', 0.1)
        self.base_link_x_offset = rospy.get_param('/base_link_x_offset', -0.1)
        self.hop_range = rospy.get_param('/hop_range', 0.07)
        self.range_next_waypoint = rospy.get_param('/planner/range_next_waypoint',0.3)
        self.__qt_reso = rospy.get_param('/planner/low_reso',0.05)
        self.max_admissible_height = rospy.get_param('/planner/max_admissible_height', 0.3)
        self.__qtmap_width = int(self.side/self.__qt_reso)
        self.__occupied_qt_height_map = Float32MultiArray()
        self.height_map = np.zeros((self.__qtmap_width, self.__qtmap_height))
        self.search_direction = [(0, 1),
                                (1, 0),
                                (0, -1),
                                (-1, 0)]
        self.global_plan = None
        self.curr_state = np.zeros(self.turtle_opti.get_state_num())
        self.desired_global_path = ([ np.zeros([200,4]) , 0])

        self.__timer_localplanner_target = rospy.Timer(rospy.Duration(1/10.0), self.publish_local_plan_target)
        self.__sub_map = rospy.Subscriber('/global_planner/occupied_qtmap_height', Float32MultiArray, self.__map_cb, queue_size=1)
        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=1)
        self.__sub_goal_state = rospy.Subscriber('/global_path', Float32MultiArray, self._global_path_callback, queue_size=1)
        self.__pub_next_waypoint_marker = rospy.Publisher("/local_planner/next_waypoint", MarkerArray, queue_size=1)
        self.__pub_localplanner_target = rospy.Publisher("/local_planner/local_planner_target", Float32MultiArray, queue_size=1)
        self.__pub_is_hop = rospy.Publisher("/is_hop", Bool, queue_size=1)
        self.__sub_hop_fin = rospy.Subscriber('/hop_fin', Bool, self.__hop_fin_cb, queue_size=1)
        self.hop_lock = False
        self.hop_lock_point = None
        self.perform_jump = False

    def __map_cb(self, msg):
        for i in range(self.__qtmap_width):
            for j in range(self.__qtmap_height):
                self.height_map[j, i] = msg.data[j*(self.__qtmap_width)+i]      
        self.map_set = True

    def __hop_fin_cb(self, msg):
        if msg.data:
            self.hop_lock = False
            self.hop_lock_point = None
            self.perform_jump = False

    def __get_height_by_coord(self, x, y):
        idx, idy = self.__get_idx_by_coord(x, y)
        height = self.height_map[idx][idy]
        if height == -1.0:
            height = 0.0
        return height
    
    def __get_height_by_idx(self, idx, idy):
        height = self.height_map[idx][idy]
        if height == -1.0:
            height = 0.0
        return height

    def __get_coord_by_idx(self, i, j):
        x = (j - self.__qtmap_height//2)*self.__qt_reso
        y = (i - self.__qtmap_width//2)*self.__qt_reso
        return (x-self.__qt_reso, y-self.__qt_reso)

    def __get_idx_by_coord(self, x, y):
        off_x = y//self.__qt_reso+self.__qtmap_width//2
        off_y = x//self.__qt_reso+self.__qtmap_height//2
        off_x = np.clip(off_x, 0, self.__qtmap_width-1)
        off_y = np.clip(off_y, 0, self.__qtmap_height-1)
        return (int(int(off_x*2+1)/2)+1, int(int(off_y*2+1)/2)+1)

    def __close_hop(self,curr,tg):
        print (curr[0]-tg[0])**2+(curr[1]-tg[1])**2
        if ((curr[0]-tg[0])**2+(curr[1]-tg[1])**2<=self.hop_range):
            return True
        else:
            return False

    def get_next_waypoint(self):
        range_selective = self.range_next_waypoint
        curr_position = np.asarray(self.curr_coor2d)
        num_pick = 10
        start_pose = self.global_plan.poses[0]
        if ((start_pose.position.x - curr_position[0])**2 + (start_pose.position.y - curr_position[1])**2) - range_selective**2 >= 0:
            tg_x, tg_y = start_pose.position.x, start_pose.position.y
            height = self.__get_height_by_coord(tg_x, tg_y)
            theta_target = np.arctan2(tg_y-self.curr_coor2d[1], tg_x-self.curr_coor2d[0])
            if self.__check_hop(height):
                theta_target_hop = self.__find_hop_jump_direction(tg_x, tg_y)
                if theta_target_hop is not None:
                    self.hop_lock = True
                    theta_target = theta_target_hop
                    if self.__close_hop([self.curr_coor2d[0]-self.base_link_x_offset+self.head_x_offset,self.curr_coor2d[1]],[tg_x, tg_y]) and not self.perform_jump:
                        print 'jump'
                        hop_flag = Bool()
                        hop_flag.data = True
                        self.perform_jump = True
                        self.__pub_is_hop.publish(hop_flag)
                    else:
                        hop_flag = Bool()
                        hop_flag.data = False
                        self.__pub_is_hop.publish(hop_flag)
                self.hop_lock_point = np.asarray([tg_x, tg_y, height, theta_target])
            else:
                hop_flag = Bool()
                hop_flag.data = False
                self.__pub_is_hop.publish(hop_flag)
            if self.hop_lock and self.hop_lock_point is not None:
                print 'locked'
                tg_x = self.hop_lock_point[0]
                tg_y = self.hop_lock_point[1]
                if self.__close_hop([self.curr_coor2d[0]-self.base_link_x_offset+self.head_x_offset,self.curr_coor2d[1]],[tg_x, tg_y]) and not self.perform_jump:
                        print 'jump'
                        hop_flag = Bool()
                        hop_flag.data = True
                        self.perform_jump = True
                        self.__pub_is_hop.publish(hop_flag)
                else:
                    hop_flag = Bool()
                    hop_flag.data = False
                    self.__pub_is_hop.publish(hop_flag)
                return self.hop_lock_point

            return np.asarray([tg_x, tg_y, height, theta_target])
        for ind in range(0, len(self.global_plan.poses)-1):
            ind = min(ind,len(self.global_plan.poses)-1)
            pose = self.global_plan.poses[ind]
            pose_next = self.global_plan.poses[min(ind+1,len(self.global_plan.poses)-1)]
            point = np.asarray([pose.position.x, pose.position.y])
            point_next = np.asarray([pose_next.position.x, pose_next.position.y])
            for i in range(0, num_pick):
                point1_mid = (point_next - point)/num_pick*(i) + point
                point2_mid = (point_next - point)/num_pick*(i+1) + point
                dist1 = sum((point1_mid - curr_position) ** 2) - range_selective ** 2
                dist2 = sum((point2_mid - curr_position) ** 2) - range_selective ** 2
                if (dist1 * dist2 <= 0):
                    tg_x, tg_y = point1_mid[0], point1_mid[1]
                    height = self.__get_height_by_coord(tg_x, tg_y)
                    theta_target = np.arctan2(tg_y-self.curr_coor2d[1], tg_x-self.curr_coor2d[0])
                    if self.__check_hop(height):
                        theta_target_hop = self.__find_hop_jump_direction(tg_x, tg_y)
                        if theta_target_hop is not None:
                            self.hop_lock = True
                            theta_target = theta_target_hop
                            if self.__close_hop([self.curr_coor2d[0]-self.base_link_x_offset+self.head_x_offset,self.curr_coor2d[1]],[tg_x, tg_y]) and not self.perform_jump:
                                print 'jump'
                                hop_flag = Bool()
                                hop_flag.data = True
                                self.perform_jump = True
                                self.__pub_is_hop.publish(hop_flag)
                            else:
                                hop_flag = Bool()
                                hop_flag.data = False
                                self.__pub_is_hop.publish(hop_flag)
                        self.hop_lock_point = np.asarray([tg_x, tg_y, height, theta_target])
                    else:
                        hop_flag = Bool()
                        hop_flag.data = False
                        self.__pub_is_hop.publish(hop_flag)
                    if self.hop_lock and self.hop_lock_point is not None:
                        print 'locked'
                        if self.__close_hop([self.curr_coor2d[0]+self.head_x_offset,self.curr_coor2d[1]],[tg_x, tg_y]) and not self.perform_jump:
                            print 'jump'
                            hop_flag = Bool()
                            hop_flag.data = True
                            self.perform_jump  = True
                            self.__pub_is_hop.publish(hop_flag)
                        else:
                            hop_flag = Bool()
                            hop_flag.data = False
                            self.__pub_is_hop.publish(hop_flag)
                        return self.hop_lock_point
                    return np.asarray([tg_x, tg_y, height, theta_target])
        last_pose = self.global_plan.poses[-1]
        height = self.__get_height_by_coord(last_pose.position.x, last_pose.position.y)
        theta_target = last_pose.position.z
        tg_x,tg_y=last_pose.position.x, last_pose.position.y
        if self.__check_hop(height):
                theta_target_hop = self.__find_hop_jump_direction(tg_x, tg_y)
                if theta_target_hop is not None:
                    self.hop_lock = True
                    theta_target = theta_target_hop
                    if self.__close_hop([self.curr_coor2d[0]+self.head_x_offset,self.curr_coor2d[1]],[tg_x, tg_y]) and not self.perform_jump:
                        print 'jump'
                        hop_flag = Bool()
                        hop_flag.data = True
                        self.perform_jump = True
                        self.__pub_is_hop.publish(hop_flag)
                    else:
                        hop_flag = Bool()
                        hop_flag.data = False
                        self.__pub_is_hop.publish(hop_flag)
                self.hop_lock_point = np.asarray([tg_x, tg_y, height, theta_target])
        else:
            hop_flag = Bool()
            hop_flag.data = False
            self.__pub_is_hop.publish(hop_flag)
        if self.hop_lock and self.hop_lock_point is not None:
            print 'locked'
            if self.__close_hop([self.curr_coor2d[0]+self.head_x_offset,self.curr_coor2d[1]],[tg_x, tg_y]) and not self.perform_jump:
                print 'jump'
                hop_flag = Bool()
                hop_flag.data = True
                self.perform_jump = True
                self.__pub_is_hop.publish(hop_flag)
            else:
                hop_flag = Bool()
                hop_flag.data = False
                self.__pub_is_hop.publish(hop_flag)
            return self.hop_lock_point
        return np.asarray([last_pose.position.x, last_pose.position.y, theta_target])

    def __check_hop(self, height):
        if height > 0.1:
            is_hop = True 
        else:
            is_hop = False
        return is_hop

    def __check_obs(self, height):
        if height > self.max_admissible_height:
            return True 
        else:
            return False

    def __find_neighbor(self, x, y):
        idx, idy = self.__get_idx_by_coord(x, y)
        for direction in self.search_direction:
            next_idx, next_idy = idx + direction[0], idy + direction[1]
            height = self.__get_height_by_idx(next_idx, next_idy)
            if self.__check_hop(height):
                return [next_idx, next_idy]
        return None 
        
    def __get_jump_direction(self, pr, p1, p2):
        # get obstacle direction)
        direct_x = -(p2[1]-p1[1])
        direct_y = p2[0]-p1[0]
        if ((pr[0] - p1[0])*direct_x + (pr[1] - p1[1]) * direct_y > 0):
            pass
        else:
            direct_x, direct_y = -direct_x, -direct_y
        direct_norm = np.sqrt(direct_x ** 2 + direct_y ** 2)
        # return 0
        return np.arctan2(direct_y/direct_norm, direct_x/direct_norm)
    
    def __find_hop_jump_direction(self, x, y):
        neighbor = self.__find_neighbor(x, y)
        if neighbor:
            theta_target = self.__get_jump_direction(pr=self.curr_coor2d, p1=[x,y], p2=neighbor)
        else:
            rospy.logwarn('no neighbor for jumping')
            theta_target = None 
        return theta_target
    
    def __curr_pose_cb(self, data):
        self.init_pose_set = True
        for i in range(3):
            self.curr_state[i] = data.data[i]
        self.curr_coor2d = [self.curr_state[0], self.curr_state[1]]
        # print('curr coor2d', self.curr_coor2d)

    def update_next_waypoint(self):
        if self.global_plan is not None:
            self.target_point_next =  self.get_next_waypoint()
            self.publish_next_waypoint_marker(self.target_point_next)
            # print("global plan:", self.global_plan)

    def _global_path_callback(self, data):
        self.ref_path_set = True
        size = len(data.data)/4
        self.desired_global_path[1]=size
        self.global_plan = Path()
        for i in range(size):
            self.desired_global_path[0][i,0]=data.data[0+4*i]
            self.desired_global_path[0][i,1]=data.data[1+4*i]
            self.desired_global_path[0][i,2]=data.data[2+4*i]
            self.desired_global_path[0][i,3]=data.data[3+4*i]
            p = Pose()
            p.position.x = data.data[0+4*i]
            p.position.y = data.data[1+4*i]
            p.position.z = data.data[2+4*i] #HACK: use z to store yaw
            self.global_plan.poses.append(p)   

    def publish_local_plan_target(self, event):
        if self.global_plan is not None:
            self.update_next_waypoint()
            fa = Float32MultiArray()
            fa.data.append(self.target_point_next[0])
            fa.data.append(self.target_point_next[1])
            fa.data.append(self.target_point_next[2])
            self.__pub_localplanner_target.publish(fa)

    def publish_next_waypoint_marker(self, target_point_next):
        m = Marker()
        m.header.frame_id = "/cassie/map"
        m.header.stamp = rospy.get_rostime()
        m.ns = "planner"
        m.id = 0
        m.type = m.SPHERE
        m.action = m.ADD
        m.pose.position.x = target_point_next[0]
        m.pose.position.y = target_point_next[1]
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 157.0/255
        m.color.g = 173.0/255
        m.color.b = 51.0/255
        m.color.a = 1.0

        arr = Marker()
        arr.header.frame_id = '/cassie/map'
        arr.header.stamp = rospy.get_rostime()
        arr.ns = "planner"
        arr.id = 1
        arr.action = m.ADD
        arr.type = arr.ARROW
        q = tf.transformations.quaternion_from_euler(0, 0, target_point_next[2])
        arr.pose.orientation.x = q[0]
        arr.pose.orientation.y = q[1]
        arr.pose.orientation.z = q[2]
        arr.pose.orientation.w = q[3]
        arr.pose.position.x = target_point_next[0]
        arr.pose.position.y = target_point_next[1]
        arr.pose.position.z = 0
        arr.scale.x = 0.2*3
        arr.scale.y = 0.015*3
        arr.scale.z = 0.015*3
        arr.color.r = 157.0/255
        arr.color.g = 173.0/255
        arr.color.b = 51.0/255
        arr.color.a = 1.0

        array = MarkerArray()
        array.markers.append(m)
        array.markers.append(arr)
        self.__pub_next_waypoint_marker.publish(array)


if __name__ == '__main__':
    print 'start'
    rospy.init_node("descision_maker")
    descision_maker = DescisionMaker()
    rospy.spin()