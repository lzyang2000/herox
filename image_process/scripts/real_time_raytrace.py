#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import time
import sys
import os
import skimage.io as skio
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from skimage import feature
from skimage import filters
import math


class tracer():
    def __init__(self):
        rospy.init_node('tracer', anonymous=True)
        self.odom_sub = rospy.Subscriber("image_process/sync_odom",Odometry,callback=self.callback_odom)
        self.map_sub = rospy.Subscriber("map",OccupancyGrid,callback=self.callback_map)

        self.raytrace_pub = rospy.Publisher("raytrace",OccupancyGrid,queue_size=1,latch=True)

        self.tf_listener = tf.TransformListener()
        self.odomList = []
        self.odomThresh = 0.01

        self.last_odom = None
        self.latest_map = None
        self.mapSize = 2021

        self.TwoD_index = None
        self.orientation = None
        self.wall_hits = None
        self.wall = None


    def callback_odom(self,data):
        pose = PoseStamped()
        pose.header=data.header
        pose.pose=data.pose.pose
        try:
            odom = self.tf_listener.transformPose('map',pose)
        except:
            print('TF error in tracer')
            return

        if self.last_odom is None:
            self.last_odom = odom
            self.odomList.append(odom)
        else:
            if (np.sqrt((self.last_odom.pose.position.x - odom.pose.position.x) ** 2 +
                        (self.last_odom.pose.position.y - odom.pose.position.y) ** 2) > self.odomThresh):
                self.last_odom = odom
                self.odomList.append(odom)

    def callback_map(self, data):
        map = np.array(data.data)
        unknown = map == -1
        wall = map == 100
        free = map == 0
        canvas = np.zeros(map.shape)
        canvas[unknown] = 205
        canvas[wall] = 0
        canvas[free] = 254
        map = canvas.astype(np.uint8)
        size = int(np.sqrt(map.shape[0]))
        self.latest_map = map.reshape((size, size))[::-1]
        # skio.imsave('test.png', self.latest_map)

    def raytrace_on_matplot(self, x, y, theta, step=0.01):
        print("raytracing starting at", x, y, theta)
        path_list = []
        hitpoint = None
        xmin = min(self.wall[:, 0])
        xmax = max(self.wall[:, 0])
        ymin = min(self.wall[:, 1])
        ymax = max(self.wall[:, 1])
        while (x <= xmax and x >= xmin and y <= ymax and y >= ymin):
            if np.logical_and(np.isclose(x, self.wall[:, 0], atol=0.05),
                              np.isclose(y, self.wall[:, 1], atol=0.05)).any():
                hitpoint = [x, y]
                break
            path_list.append([x, y])
            x += step * np.cos(theta)
            y += step * np.sin(theta)

        path_list = np.array(path_list)
        return hitpoint, path_list

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

        return roll_x, pitch_y, yaw_z  # in radians

    def get_wall(self):
        self.wall = []
        m = self.latest_map
        fill = m.copy()
        fill = np.transpose(fill, (1, 0))
        fill = fill[:, ::-1]
        mask = fill == 254
        maskup = np.roll(mask, 1, axis=0)
        maskright = np.roll(mask, 1, axis=1)
        fill[mask] = 0
        fill[maskup] = 0
        fill[maskright] = 0
        fill = filters.gaussian(fill)
        e1 = np.array(feature.canny(fill), dtype=int)

        wall = np.transpose(m, (1, 0))[:, ::-1] == 0
        e1[wall] = 1

        mask = np.ones(e1.shape)
        for i in range(len(e1)):
            for j in range(len(e1[0])):
                if e1[i][j] and np.sum(e1[i - 2:i + 3, j - 2:j + 3]) >= 4:
                    mask[i][j] = 0

        xs, ys = np.where(mask == 0)
        scale = 0.05
        for i in range(len(xs)):
            # self.wall.append([(xs[i] - len(fill) / 2) * scale, (ys[i] - len(fill) / 2) * scale])
            self.wall.append([(xs[i]) * scale, (ys[i]) * scale]) #image coord
        self.wall = np.array(self.wall)



    def process(self):
        self.TwoD_index = []
        self.orientation = []
        self.wall_hits = []
        for i in self.odomList:
            x, y = i.pose.position.x + self.mapSize/2, i.pose.position.y + self.mapSize/2
            self.TwoD_index.append([x, y])
            theta = self.euler_from_quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)[2]
            self.orientation.append(theta)
            hit, _ = self.raytrace_on_matplot(x, y, theta)
            self.wall_hits.append(hit)


    def raytrace(self):
        self.get_wall()
        self.process()
        im = self.latest_map.copy()
        print('map', im)
        for c in self.wall_hits:
            im[int(c[0]), int(c[1])] = 100
        skio.imsave('hit.png', im)


if __name__ == '__main__':
    T = tracer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if T.latest_map is not None and len(T.odomList):
            T.raytrace()
        else:
            print('empty info, sleep')
            rate.sleep()

