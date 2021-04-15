#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
import time
import sys
import os
import numpy as np
import pickle
import glob
import os
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import skimage.io as skio
import skimage as sk
from skimage import feature
from skimage import filters
import re
import math

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

class visualizer:
    def __init__(self,path):
        self.count = 0.0
        self.start = True
        self.start_time = 0.0
        self.coords_path = glob.glob(path+'/'+'*.pkl')
        self.images_path = glob.glob(path+'/'+'*.jpg')
        self.coords_path.sort( key=natural_keys)
        self.images_path.sort( key=natural_keys)
        self.coords = []
        self.images = []
        last_seen = None
        for i,j in zip(self.coords_path,self.images_path):
            with open(i,'rb') as f:
                coord = pickle.load(f)
                if not last_seen:
                    self.coords.append(coord)
                    self.images.append(cv2.imread(j))
                    last_seen = coord
                elif last_seen != coord:
                    self.coords.append(coord)
                    self.images.append(cv2.imread(j))
    
    def draw_wall(self):
        self.wall = []
        a = np.array(skio.imread("map3.pgm"))
        #a = a[::-1]
        b = a
        b = sk.transform.rotate(a, -90, preserve_range=True)
        b[b>220] = 0
        e1 = np.array(feature.canny(b,sigma=5), dtype=int)
        for i in range(len(e1)):
            for j in range(len(e1[0])):
                if e1[i][j]:
                    e1[i][j] = 0
                else:
                    e1[i][j] = 255
        xs, ys = np.where(e1 == 0)
        #print ys
        scale = 0.05
        for i in range(len(xs)):
            self.wall.append([(xs[i] - len(a)/2) * scale, (ys[i] - len(a)/2) * scale])
        self.wall = np.array(self.wall)   
    
    def draw_wall_jason(self):
        self.wall = []
        m = np.array(skio.imread("map3.pgm"))
        #self.occupancy = np.zeros(m.shape)
        #print(self.occupancy.shape)
        fill= m.copy()
        fill = np.transpose(fill, (1, 0))
        fill = fill[:, ::-1]
        #fill = sk.transform.rotate(m, -90, preserve_range=True)
        mask = fill == 254
        maskup = np.roll(mask, 1, axis = 0)
        maskright = np.roll(mask, 1, axis = 1)
        #maskdown = np.roll(mask, -1, axis = 0)
        #maskleft = np.roll(mask, -1, axis = 1)
        fill[mask] = 0
        fill[maskup] = 0
        fill[maskright] = 0
        #fill[maskdown] = 0
        #fill[maskleft] = 0
        fill = filters.gaussian(fill)
        e1 = np.array(feature.canny(fill), dtype=int)
        for i in range(len(e1)):
            for j in range(len(e1[0])):
                if e1[i][j]:
                    e1[i][j] = 0
                else:
                    e1[i][j] = 255
        xs, ys = np.where(e1 == 0)
        scale = 0.05
        for i in range(len(xs)):
            self.wall.append([(xs[i] - len(fill)/2) * scale, (ys[i] - len(fill)/2) * scale])
        self.wall = np.array(self.wall)
    
    def raytrace(self, img, x, y, theta):
        path = []
        while(y<len(img) and x<len(img)):
            y += 0.5
            x += np.tan(theta)*0.5
            if y>=len(img) or x>=len(img):
                break

            area = img[int(x)-1:int(x)+2][int(y)-1:int(y)+2]
            if (area==0).any():
                print(int(x), int(y))
                break
            img[int(x)][int(y)] = 125
    
    def raytrace_on_matplot(self, x, y, theta):
        print("raytracing starting at", x, y, theta)
        self.path = []
        xmin = min(self.wall[:, 0])
        xmax = max(self.wall[:, 0])
        ymin = min(self.wall[:, 1])
        ymax = max(self.wall[:, 1])
        while (x <= xmax and x >= xmin and y <=ymax and y >= ymin):
            if np.logical_and(np.isclose(x, self.wall[:,0], atol=0.05), np.isclose(y, self.wall[:,1], atol=0.05)).any():
                break
            self.path.append([x, y])
            x += 0.01 * np.cos(theta)
            y += 0.01 * np.sin(theta)
            
        self.path = np.array(self.path)

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

    def process(self):
        self.TwoD_index = []
        self.orientation = []
        prev = [float('inf'), float('inf')]
        for i in self.coords:
            if (np.sqrt((prev[0] - i.pose.position.x) ** 2 + (prev[1] - i.pose.position.y) ** 2) > 0.1):
                self.TwoD_index.append([i.pose.position.x,i.pose.position.y])
                self.orientation.append(self.euler_from_quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)[2])
            prev = [i.pose.position.x,i.pose.position.y]
        
        for o in self.orientation:
            print(o)
        
        self.TwoD_index = np.array(self.TwoD_index)
        self.fig, self.ax1 = plt.subplots()
        self.ax1.set_title('click on points', picker=True)
        self.ax1.set_ylabel('X(m)', picker=True)
        self.ax1.set_xlabel('Y(m)', picker=True)
        line, = self.ax1.plot(self.TwoD_index[:,1],self.TwoD_index[:,0], '-o')
        lin2, = self.ax1.plot(self.wall[:,1],self.wall[:,0], '.', markersize=4)
        self.ax1.scatter(self.TwoD_index[:,1],self.TwoD_index[:,0], picker=True)
        # q = self.ax1.quiver(self.TwoD_index[:,1],self.TwoD_index[:,0],u,v)
        self.pick_simple()
        #plt.xlim([-0.5,0.5])
        plt.show()
        
    def pick_simple(self):
        # simple picking, lines, rectangles and text
        def onpick(event):
            ind = event.ind
            ind = ind[0]
            print('onpick3 scatter:', ind, self.TwoD_index[:,1][ind], self.TwoD_index[:,0][ind])
            self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind])
            print(self.path)
            lin3, = self.ax1.plot(self.path[:,1],self.path[:,0], '.', markersize=2)
            cv2.namedWindow(str(ind), cv2.WINDOW_NORMAL)
            cv2.resizeWindow(str(ind), 800, 400)
            cv2.imshow(str(ind),self.images[ind])
            self.fig.canvas.draw()
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            lin3.remove()
            self.fig.canvas.draw()

        self.fig.canvas.mpl_connect('pick_event', onpick)
        


def main(args):
    #visualizer('/home/locobot/slam_ws/image_data').process()
    v = visualizer('/home/daly/Downloads/1st_floor')
    v.draw_wall_jason()
    v.process()

if __name__ == '__main__':
    main(sys.argv)
