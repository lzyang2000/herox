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

class visualizer:
    def __init__(self,path):
        self.count = 0.0
        self.start = True
        self.start_time = 0.0
        self.coords_path = glob.glob(path+'/'+'*.pkl')
        self.images_path = glob.glob(path+'/'+'*.jpg')
        self.coords_path.sort( key=os.path.getmtime)
        self.images_path.sort( key=os.path.getmtime)
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
    
    
    def process(self):
        self.TwoD_index = []
        for i in self.coords:
            self.TwoD_index.append([i.position.x,i.position.y])
        
        self.TwoD_index = np.array(self.TwoD_index)
        self.fig, self.ax1 = plt.subplots()
        self.ax1.set_title('click on points', picker=True)
        self.ax1.set_ylabel('X(m)', picker=True)
        self.ax1.set_xlabel('Y(m)', picker=True)
        line, = self.ax1.plot(self.TwoD_index[:,1],self.TwoD_index[:,0], '-o')
        self.ax1.scatter(self.TwoD_index[:,1],self.TwoD_index[:,0], picker=True)
        # q = self.ax1.quiver(self.TwoD_index[:,1],self.TwoD_index[:,0],u,v)
        self.pick_simple()
        plt.xlim([-0.5,0.5])
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
    visualizer('/home/locobot/slam_ws/image_data').process()


if __name__ == '__main__':
    main(sys.argv)
