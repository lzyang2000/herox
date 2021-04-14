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

class visualizer:
    def __init__(self,path):
        self.count = 0.0
        self.start = True
        self.start_time = 0.0
        self.coords_path = glob.glob(path+'/'+'*.pkl')
        self.images_path = glob.glob(path+'/'+'*.jpg')
        self.coords_path.sort( key=os.path.getmtime)
        self.images_path.sort( key=os.path.getmtime)
        #self.coords_path.sort()
        #self.images_path.sort()
        print(self.coords_path)
        print(self.images_path)
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
        #b = sk.transform.rotate(a, 90, preserve_range=True)
        b = a
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
        scale = len(a) / 20
        for i in range(len(xs)):
            self.wall.append([xs[i] - len(a)/2, ys[i] - len(a)/2])
        self.wall = np.array(self.wall)
    
    def alternative_wall(self):
        self.wall = []
        m = np.array(skio.imread("map3.pgm"))
        fill = m.copy()
        plt.figure(figsize=(20,10))
        mask = fill == 254
        maskup = np.roll(mask, 1, axis = 0)
        maskright = np.roll(mask, 1, axis = 1)
        fill[mask] = 0
        fill[maskup] = 0
        fill[maskright] = 0
        fill = filters.gaussian(fill)
    
    def raytrace(self, img, x, y, theta):
        while(y<len(img) and x<len(img)):
            y += 0.5
            x += np.tan(theta)*0.5
            if y>=len(img) or x>=len(img):
                break
            if img[int(x)][int(y)] == 0:
                print(int(x), int(y))
                break
            img[int(x)][int(y)] = 125
    
    
    def process(self):
        self.TwoD_index = []
        for i in self.coords:
            self.TwoD_index.append([i.pose.position.x,i.pose.position.y])
        
        self.TwoD_index = np.array(self.TwoD_index)
        self.fig, self.ax1 = plt.subplots()
        self.ax1.set_title('click on points', picker=True)
        self.ax1.set_ylabel('X(m)', picker=True)
        self.ax1.set_xlabel('Y(m)', picker=True)
        line, = self.ax1.plot(self.TwoD_index[:,1],self.TwoD_index[:,0], '-o')
        #lin2, = self.ax1.plot(self.wall[:,1],self.wall[:,0], '.')
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
            cv2.namedWindow(str(ind), cv2.WINDOW_NORMAL)
            cv2.imshow(str(ind),self.images[ind])
            cv2.waitKey(0)
            cv2.destroyAllWindows()


        self.fig.canvas.mpl_connect('pick_event', onpick)
        


def main(args):
    #visualizer('/home/locobot/slam_ws/image_data').process()
    v = visualizer('/home/daly/Downloads/1st_floor')
    v.draw_wall()
    v.process()


if __name__ == '__main__':
    main(sys.argv)
