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
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
import sys
import os
import skimage.io as skio
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from skimage import feature
from skimage import filters
import math
import pickle
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class tracer():
    def __init__(self):
        rospy.init_node('tracer', anonymous=True)
        self.odom_sub = rospy.Subscriber("image_process/sync_odom",Odometry,callback=self.callback_odom)
        self.map_sub = rospy.Subscriber("map",OccupancyGrid,callback=self.callback_map)

        self.newMap_pub = rospy.Publisher("wallhit_gridmap",OccupancyGrid,queue_size=1,latch=True)
        
        self.wall_hit_publisher = rospy.Publisher('wall_hit', MarkerArray,queue_size=1,latch=True)
        self.ray_publisher = rospy.Publisher('rays', MarkerArray,queue_size=1,latch=True)

        self.tf_listener = tf.TransformListener()
        self.newOdomList = []
        self.odomThresh = 0.1
        
        self.oldOdomList = []
        self.oldWallHits = []
        self.oldRays = []
        
        self.newOdomListLength = 15

        self.last_odom = None
        self.last_theta = None
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
            #print('TF error in tracer')
            return
        
        i = odom
        theta = self.euler_from_quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)[2]
        #print('orin', theta)

        if self.last_odom is None:
            self.last_odom = odom
            self.last_theta = theta
            self.newOdomList.append(odom)
        else:
            if (np.sqrt((self.last_odom.pose.position.x - odom.pose.position.x) ** 2 +
                        (self.last_odom.pose.position.y - odom.pose.position.y) ** 2) > self.odomThresh):
                #print('enough distance', np.sqrt((self.last_odom.pose.position.x - odom.pose.position.x) ** 2 +
                        #(self.last_odom.pose.position.y - odom.pose.position.y) ** 2))
                self.last_odom = odom
                self.last_theta = theta
                self.newOdomList.append(odom)
            
            elif abs(theta - self.last_theta) >= np.pi/36:
                self.last_odom = odom
                self.last_theta = theta
                self.newOdomList.append(odom)

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
        self.latest_map = map.reshape((size, size))
        
        self.latest_map_origin = data.data
        # skio.imsave('test.png', self.latest_map)

    def raytrace_on_matplot(self, x, y, theta, step=0.05):
        #print("raytracing starting at", x, y, theta)
        initial_x, initial_y = x, y
        path_list = []
        hitpoint = None
        xmin = min(self.wall[:, 0])
        xmax = max(self.wall[:, 0])
        ymin = min(self.wall[:, 1])
        ymax = max(self.wall[:, 1])
        
        x_step = step * np.cos(theta)
        y_step = step * np.sin(theta)
        
        length_limit = 2
        x_limit = x + length_limit*np.cos(theta)
        
        while (x <= xmax and x >= xmin and y <= ymax and y >= ymin):
            if (x_limit-x)*(x-initial_x) < 0:
                break
            
            if np.logical_and(np.isclose(x, self.wall[:, 0], atol=0.05),
                              np.isclose(y, self.wall[:, 1], atol=0.05)).any():
                hitpoint = [x, y]
                break
            path_list.append([x, y])
            x += x_step
            y += y_step
        #if hitpoint is None:
            #print('fail to hit wall')
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
        
        sub_m = m[750:1250, 750:1250]
        m = sub_m
        
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
        
        canvas = np.zeros(e1.shape)
        
        for i in range(-2, 3):
            for j in range(-2, 3):
                tmp = np.roll(e1, i, axis=0)
                tmp = np.roll(tmp, j, axis=1)
                canvas += tmp
                
                
        nearwall = canvas >= 6
        realwall = e1 > 0
        mask[np.logical_and(nearwall, realwall)] = 0
        

        mask = mask[:, ::-1]
        
        wholemask = np.ones(self.latest_map.shape)
        wholemask[750:1250, 750:1250] = mask
        mask = wholemask
        
        xs, ys = np.where(mask == 0)
        
        
        realX, realY = [], []
        l_m = self.latest_map
        for x, y in zip(xs, ys):
            local = l_m[y-2:y+3, x-2:x+3]
            #print(x, y)
            if (local == 0).any():
                realX.append(x)
                realY.append(y)
        
        xs, ys = realX, realY
            
        
        scale = 0.05
        for i in range(len(xs)):
            # self.wall.append([(xs[i] - len(fill) / 2) * scale, (ys[i] - len(fill) / 2) * scale])
            self.wall.append([(xs[i]) * scale, (ys[i]) * scale]) #image coord
        self.wall = np.array(self.wall)

    def process(self):
        self.wall_hits = []
        self.rays = []
        odom = self.newOdomList[:]
        hit_total = 0
        hit_total += len(self.oldWallHits)
        scale = 0.05
        
        
        if len(odom) > self.newOdomListLength:
            extraNum = len(odom) - self.newOdomListLength
            pop_odom = self.newOdomList[:extraNum]
            odom = self.newOdomList[extraNum:]
            self.newOdomList = odom
            
            for od in pop_odom:
                self.oldOdomList.append(od)
            
            for i in pop_odom:
                x, y = i.pose.position.x + (self.mapSize/2)*scale, i.pose.position.y + (self.mapSize/2)*scale
                theta = self.euler_from_quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)[2]
                
                for t in np.linspace(-np.pi/6, np.pi/6, 30):
                    hit, ray = self.raytrace_on_matplot(x, y, theta+t)
                    if hit is not None:
                        hit_total += 1
                        self.oldWallHits.append(hit)
                        self.oldRays.append(ray)
      
        for i in odom:
            x, y = i.pose.position.x + (self.mapSize/2)*scale, i.pose.position.y + (self.mapSize/2)*scale
            theta = self.euler_from_quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)[2]
            
            for t in np.linspace(-np.pi/6, np.pi/6, 30):
                hit, ray = self.raytrace_on_matplot(x, y, theta+t)
                if hit is not None:
                    hit_total += 1
                self.wall_hits.append(hit)
                self.rays.append(ray)
        self.num_hits = hit_total


    def filter_repeat(self):
        resolution_set = set()
        distinct_old_wall = []
        distinct_old_ray = []
        
        for w, r in zip(self.oldWallHits, self.oldRays):
            pixelX, pixelY = int(w[0]/0.05), int(w[1]/0.05)
            if (pixelX, pixelY) not in resolution_set:
                for i in range(-2, 3):
                    for j in range(-2, 3):
                        resolution_set.add((pixelX + i, pixelY + j))
                        distinct_old_wall.append(w)
                        
                        newr = []
                        for d in r:
                            pixelX, pixelY = int(d[0]/0.05), int(d[1]/0.05)
                            if (pixelX, pixelY) not in resolution_set:
                                #for i in range(-2, 3):
                                    #for j in range(-2, 3):
                                resolution_set.add((pixelX + i, pixelY + j))
                                newr.append(d)
                        distinct_old_ray.append(newr)
                        
        self.oldWallHits = distinct_old_wall
        self.oldRays = distinct_old_ray
        
        distinct_new_wall = []
        distinct_new_ray = []
        
        for w, r in zip(self.wall_hits, self.rays):
            if w is not None:
                pixelX, pixelY = int(w[0]/0.05), int(w[1]/0.05)
                if (pixelX, pixelY) not in resolution_set:
                    for i in range(-2, 3):
                        for j in range(-2, 3):
                            resolution_set.add((pixelX + i, pixelY + j))
                            distinct_new_wall.append(w)
                            
                            newr = []
                            for d in r:
                                pixelX, pixelY = int(d[0]/0.05), int(d[1]/0.05)
                                if (pixelX, pixelY) not in resolution_set:
                                    #for i in range(-2, 3):
                                        #for j in range(-2, 3):
                                    resolution_set.add((pixelX + i, pixelY + j))
                                    newr.append(d)
                            distinct_new_ray.append(newr)
                        
        self.wall_hits = distinct_new_wall
        self.rays = distinct_new_ray
        
        
        
        

    def raytrace(self):
        t1 = time.time()
        self.get_wall()
        t2 = time.time()
        #print('wall time', t2-t1)
        self.process()
        t3 = time.time()
        #print('process time', t3-t2)
        self.filter_repeat()
        #print('filter time', time.time()-t3)
        mr_arr = MarkerArray()
        #print('num odom', len(self.odomList))
        
        _id = 0
        for i in self.wall_hits + self.oldWallHits:
            if i is None:
                continue
            m = Marker()
            m.header.frame_id = '/map'
            m.header.stamp = rospy.Time.now()
            m.id = _id
            _id += 1
            m.action = m.ADD
            m.type = m.POINTS
            #m.pose.position = []
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1		
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0
            m.color.a = 1.0
            
            p = Point()
            p.x = i[0]-(2021*0.05/2)
            p.y = i[1]-(2021*0.05/2)
            p.z = 0.0
            m.points.append(p)
            
            mr_arr.markers.append(m)
            
            
        
        self.wall_hit_publisher.publish(mr_arr)
        #print('middle publish time', time.time()-t3)
            
        
        wallhit_grid = OccupancyGrid()
        wallhit_grid.header.frame_id = 'map'
        wallhit_grid.header.stamp = rospy.Time.now()
        wallhit_grid.info.width = 2021
        wallhit_grid.info.height = 2021
        wallhit_grid.info.resolution = 0.05
        origin = Pose()
        origin.orientation.w = 1
        origin.orientation.x = 0
        origin.orientation.y = 0
        origin.orientation.z = 0
        
        origin.position.x = -2021*0.05/2
        origin.position.y = -2021*0.05/2
        origin.position.z = 0
    
        wallhit_grid.info.origin = origin
        #print('num hit', self.num_hits)
        assignment_num = 0
        if self.num_hits < 300:
            wallhit_grid.data = self.latest_map_origin
            
        else:
            originmap = np.array(self.latest_map_origin).reshape((2021, 2021))
            
            center = 1010  
            grid = -np.ones((2021, 2021))
            
            for i in range(len(self.wall_hits)):
                hit = self.wall_hits[i]
                ray = self.rays[i]
                if hit is None:
                    continue
                for p in ray:
                    y, x = p[0]-(2021*0.05/2), p[1]-(2021*0.05/2)
                    
                    assignment_num += 1
                    grid[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                         center + int(y/0.05)-5:center + int(y/0.05)+5] = originmap[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                                                                                    center + int(y/0.05)-5:center + int(y/0.05)+5]
                    
                #y, x = hit[0]-(2021*0.05/2), hit[1]-(2021*0.05/2)
                #grid[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                    #center + int(y/0.05)-5:center + int(y/0.05)+5] = originmap[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                                                                            #center + int(y/0.05)-5:center + int(y/0.05)+5]
                
                
            for i in range(len(self.oldWallHits)):
                hit = self.oldWallHits[i]
                ray = self.oldRays[i]
                if hit is None:
                    continue
                for p in ray:
                    y, x = p[0]-(2021*0.05/2), p[1]-(2021*0.05/2)
                    
                    assignment_num += 1
                    grid[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                         center + int(y/0.05)-5:center + int(y/0.05)+5] = originmap[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                                                                                    center + int(y/0.05)-5:center + int(y/0.05)+5]
                    
                #y, x = hit[0]-(2021*0.05/2), hit[1]-(2021*0.05/2)
                #grid[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                    #center + int(y/0.05)-5:center + int(y/0.05)+5] = originmap[center + int(x/0.05)-5:center + int(x/0.05)+5, 
                                                                            #center + int(y/0.05)-5:center + int(y/0.05)+5]
                
            grid = grid.reshape((2021*2021))        

            wallhit_grid.data = list(grid)
            
            
        self.newMap_pub.publish(wallhit_grid)
        
        wallhit_map = np.zeros((2021, 2021))
        
        t4 = time.time()
        #print('publish time', t4-t3)
        #print('assignment num', assignment_num)
            
        
        
        
        #im = self.latest_map.copy()
        #print('map', im)
        
        
        #np.save('debug/wall.npy', self.wall)
        #np.save('debug/pos.npy', self.TwoD_index)
        #np.save('debug/orien.npy', self.orientation)
        #np.save('debug/hit.npy', self.wall_hits)
        #np.save('debug/ray.npy', self.rays)
            
        #print('done saving')
        



if __name__ == '__main__':
    T = tracer()
    while not rospy.is_shutdown():
        if T.latest_map is not None and len(T.newOdomList):
            T.raytrace()

