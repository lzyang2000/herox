#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray,MultiArrayDimension
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
import numpy as np
import tf
from cheetah_opti import CheetahOpti
from visualization_msgs.msg import Marker, MarkerArray
import math
from local_planner_hop.msg import LocalPlan
class CheetahPlanner():
	def __init__(self):
		self.replan_period = rospy.get_param('/local_planner/replan_period', 1.0/20.0)
		self.cheetah_opti = CheetahOpti()
		self.curr_state = np.zeros(self.cheetah_opti.get_state_num())
		self.goal_state = np.zeros(self.cheetah_opti.get_state_num())
		self.target_state_set = False
		self.init_pose_set = False
		self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
		self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=1)
		self.__sub_target = rospy.Subscriber("/local_planner/local_planner_target", Float32MultiArray, self.__target_cb, queue_size=1)
		self.__pub_local_path_marker = rospy.Publisher("/local_planner/local_plan_viz", Marker, queue_size=1)
		self.__pub_local_yaw_marker = rospy.Publisher("/local_planner/local_plan_yaw_viz", MarkerArray, queue_size=1)
		self.__pub_local_plan = rospy.Publisher("/local_planner/local_plan", LocalPlan, queue_size=1)
		
	def __replan_cb(self, event):
		if self.init_pose_set and self.target_state_set:
			self.curr_state_global_locked = np.copy(self.curr_state) 
			self.target_state_global_locked = np.copy(self.goal_state)
			self.target_state_local = np.copy(self.target_state_global_locked)
			self.target_state_local[[0,1]] = self.target_state_global_locked[[0,1]] - self.curr_state_global_locked[[0,1]]			
			self.curr_state_local = np.copy(self.curr_state_global_locked) 
			self.curr_state_local[[0,1]] = np.array([0.0, 0.0])
			start_time = rospy.Time.now()
			self.states_profile, _, self.time_opti = self.cheetah_opti.solve(self.curr_state_local,self.target_state_local)
			# print  "states_profile: ",self.states_profile
			dt = rospy.Time.now()-start_time
			# rospy.loginfo("[local Planner] solved in {} sec".format(dt.to_sec()))

			self.states_profile[[0,1],:] = self.states_profile[[0,1],:] + np.tile(self.curr_state_global_locked[[0,1]].reshape(2,1), (1, self.states_profile.shape[-1]))
			self.publish_local_plan()
			self.publish_local_path_marker()

	def __curr_pose_cb(self, data):
		self.init_pose_set = True
		for i in range(3):
			self.curr_state[i] = data.data[i]
		# print("curr state is: ", self.curr_state)

	def __target_cb(self, msg):
		self.target_state_set = True
		self.goal_state[0] = msg.data[0]
		self.goal_state[1] = msg.data[1]
		self.goal_state[2] = msg.data[2]
		self.goal_state[3] = 0.0
		self.goal_state[4] = 0.0
		self.goal_state[5] = 0.0
		# print "target state is: ", self.goal_state

	def publish_local_plan(self):
		# print self.states_profile
		lp = LocalPlan()
		lp.time_opti = self.time_opti
		ma = Float32MultiArray()
		md = MultiArrayDimension()
		row, col = self.states_profile.shape
		md.label = "state_dim"
		md.size = row
		ma.layout.dim.append(md)
		md = MultiArrayDimension()
		md.label = "grid_dim"
		md.size = col
		ma.layout.dim.append(md)
		for i in range(row*col):
			ma.data.append(self.states_profile.flat[i])
		lp.states_opti = ma
		self.__pub_local_plan.publish(lp)

	def publish_local_path_marker(self):
		m = Marker()
		mr_arr = MarkerArray()

		m.header.frame_id = '/cassie/map'
		m.header.stamp = rospy.Time.now()
		m.id = 0
		m.action = m.ADD
		m.type = m.SPHERE_LIST
		m.pose.orientation.x = 0.0
		m.pose.orientation.y = 0.0
		m.pose.orientation.z = 0.0
		m.pose.orientation.w = 1.0
		m.scale.x = 0.1
		m.scale.y = 0.1
		m.scale.z = 0.1		
		m.color.r = 253.0/255
		m.color.g = 181.0/255
		m.color.b = 21.0/255
		m.color.a = 1.0
		for i in range(self.states_profile.shape[-1]):
			p = Point()
			p.x = self.states_profile[0,i]
			p.y = self.states_profile[1,i]
			p.z = 0.0
			m.points.append(p)

			arr = Marker()
			arr.header.frame_id = '/map'
			arr.header.stamp = rospy.Time.now()
			arr.id = i 
			arr.action = m.ADD
			arr.type = arr.ARROW
			q = tf.transformations.quaternion_from_euler(0, 0, self.states_profile[2,i])
			arr.pose.orientation.x = q[0]
			arr.pose.orientation.y = q[1]
			arr.pose.orientation.z = q[2]
			arr.pose.orientation.w = q[3]
			arr.pose.position.x = self.states_profile[0,i]
			arr.pose.position.y = self.states_profile[1,i]
			arr.pose.position.z = 0.0
			# print(self.states_profile[2,i])
			arr.scale.x = 0.2
			arr.scale.y = 0.015
			arr.scale.z = 0.015
			arr.color.r = 237.0/255
			arr.color.g = 78.0/255
			arr.color.b = 51.0/255
			arr.color.a = 1.0
			mr_arr.markers.append(arr)
		self.__pub_local_path_marker.publish(m)
		self.__pub_local_yaw_marker.publish(mr_arr)


if __name__ == '__main__':
	try:
		print 'start'
		rospy.init_node("cheetah_local_planner")
		cheetah_planner = CheetahPlanner()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
