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
from matplotlib.widgets import Button
import threading

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
        self.images_path = glob.glob(path+'/'+'*_RGB.jpg')
        self.coords_path.sort( key=natural_keys)
        self.images_path.sort( key=natural_keys)
        print(self.coords_path)
        self.coords = []
        self.images = []
        self.path = []
        self.wall_hits = []
        last_seen = None
        self.current_displayed = None
        
        #for i in self.coords_path:
            #with open(i,'rb') as f:
                #coord = pickle.load(f)
            #self.coords.append(coord)
        
        for i,j in zip(self.coords_path,self.images_path):
            with open(i,'rb') as f:
                coord = pickle.load(f)
                if not last_seen:
                    self.coords.append(coord)
                    im = cv2.imread(j)[:, :, ::-1]
                    ir = cv2.imread(j.replace('RGB', 'IR'))
                    ir = cv2.resize(ir, (im.shape[1], im.shape[0]))
                    #ir = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
                    self.images.append(cv2.hconcat([im, ir]))
                    last_seen = coord
                elif last_seen != coord:
                    self.coords.append(coord)
                    im = cv2.imread(j)[:, :, ::-1]
                    ir = cv2.imread(j.replace('RGB', 'IR'))
                    ir = cv2.resize(ir, (im.shape[1], im.shape[0]))
                    #ir = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
                    self.images.append(cv2.hconcat([im, ir]))

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
        m = np.array(skio.imread("../../../307-1data/map.pgm"))
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
        
        wall = np.transpose(m, (1, 0))[:, ::-1] == 0
        e1[wall] = 1
        
        mask = np.ones(e1.shape)
        # local = np.zeros(e1.shape)
        #
        # for i in range(-2, 3):
        #     for j in range(-2, 3):
        #         roll = np.roll(e1, i, axis=0)
        #         roll = np.roll(roll, j, axis=1)
        #

        for i in range(len(e1)):
            for j in range(len(e1[0])):
                if e1[i][j] and np.sum(e1[i-2:i+3, j-2:j+3]) >= 4:
                    mask[i][j] = 0
        
        xs, ys = np.where(mask == 0)
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
    
    def raytrace_on_matplot(self, x, y, theta, step=0.01):
        print("raytracing starting at", x, y, theta)
        path_list = []
        hitpoint = None
        xmin = min(self.wall[:, 0])
        xmax = max(self.wall[:, 0])
        ymin = min(self.wall[:, 1])
        ymax = max(self.wall[:, 1])
        while (x <= xmax and x >= xmin and y <=ymax and y >= ymin):
            if np.logical_and(np.isclose(x, self.wall[:,0], atol=0.05), np.isclose(y, self.wall[:,1], atol=0.05)).any():
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
     
        return roll_x, pitch_y, yaw_z # in radians     

    def process(self):
        self.TwoD_index = []
        self.orientation = []
        prev = [float('inf'), float('inf')]
        filterimage = []
        for i, j in zip(self.coords, self.images):
            if (np.sqrt((prev[0] - i.pose.position.x) ** 2 + (prev[1] - i.pose.position.y) ** 2) > -0.01):
                self.TwoD_index.append([i.pose.position.x,i.pose.position.y])
                theta = self.euler_from_quaternion(i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)[2]
                self.orientation.append(theta)
                hit, _ = self.raytrace_on_matplot(i.pose.position.x, i.pose.position.y, theta)
                self.wall_hits.append(hit)
                filterimage.append(j)
            prev = [i.pose.position.x,i.pose.position.y]
        self.images = filterimage
        print('total data num', len(self.images))
        

        self.TwoD_index = np.array(self.TwoD_index)
        self.wall_hits = np.array(self.wall_hits)
        self.fig, self.ax1 = plt.subplots(figsize=(7, 10))
        self.ax1.set_title('click on points', picker=True)
        self.ax1.set_ylabel('X(m)', picker=True)
        self.ax1.set_xlabel('Y(m)', picker=True)
        print(self.TwoD_index.shape)
        line, = self.ax1.plot(self.TwoD_index[:,1],self.TwoD_index[:,0], '-o')
        lin2, = self.ax1.plot(self.wall[:,1],self.wall[:,0], '.', markersize=4)
        self.ax1.scatter(self.TwoD_index[:,1],self.TwoD_index[:,0], picker=True)

        axprev = plt.axes([0.69, 0.04, 0.1, 0.03])
        axnext = plt.axes([0.80, 0.04, 0.1, 0.03])
        self.bnext = Button(axnext, 'Up', color='1.0')
        self.bprev = Button(axprev, 'Down', color='1.0')
        
        
        print(self.wall_hits.shape)
        self.ax1.scatter(self.wall_hits[:,1],self.wall_hits[:,0], marker='8', color='purple', picker=True)
        # q = self.ax1.quiver(self.TwoD_index[:,1],self.TwoD_index[:,0],u,v)
        self.click_btn()
        self.pick_simple()
        # plt.xlim([-0.5,0.5])
        plt.show()
    
    def display_img(self, ind):
        def onclose(event):
            if self.current_displayed:
                self.current_displayed[3].remove()
                self.fig.canvas.draw()
                self.current_displayed = None

        if self.current_displayed:
            fig, ax, cur_ind, cur_line = self.current_displayed
            print('reusing onpick fig')
            self.current_displayed[3].remove()
            #fig.clf()

            _, path = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind])
            _, left_limit = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind] - np.pi/4, step=0.1)
            _, right_limit = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind] + np.pi/4, step=0.1)
            combined = np.concatenate((path, left_limit, right_limit), axis=0)
            line, = self.ax1.plot(combined[:,1],combined[:,0], '.', markersize=2)
            self.fig.canvas.draw() # update the plot with the ray
            
            img_with_heatmap = self.images[ind].copy()
            width_cutoff = img_with_heatmap.shape[1] // 2
            img_with_heatmap[:, width_cutoff:] = cv2.applyColorMap(img_with_heatmap[:, width_cutoff:], cv2.COLORMAP_JET)   
            plt.xticks([]), plt.yticks([])
            ax.imshow(img_with_heatmap)
            self.current_displayed = (fig, ax, ind, line) # keep track of current fig, idx and ray
            plt.show()
            
        else:
            print('generating onpick fig', ind, self.TwoD_index[:,1][ind], self.TwoD_index[:,0][ind])
            _, path = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind])
            _, left_limit = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind] - np.pi/4, step=0.1)
            _, right_limit = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind] + np.pi/4, step=0.1)
            combined = np.concatenate((path, left_limit, right_limit), axis=0)
            line, = self.ax1.plot(combined[:,1],combined[:,0], '.', markersize=2)
            self.fig.canvas.draw() # update the plot with the ray
            
            imgfig, imgax = plt.subplots(figsize=(10, 5))
            imgfig.canvas.mpl_connect('close_event', onclose)
            img_with_heatmap = self.images[ind].copy()
            width_cutoff = img_with_heatmap.shape[1] // 2
            img_with_heatmap[:, width_cutoff:] = cv2.applyColorMap(img_with_heatmap[:, width_cutoff:], cv2.COLORMAP_JET)   
            plt.xticks([]), plt.yticks([])
            imgax.imshow(img_with_heatmap)
            self.current_displayed = (imgfig, imgax, ind, line) # keep track of current fig, ax, idx and ray
            plt.show()

    def pick_simple(self):
        def onclose(event):
            if self.current_displayed:
                self.current_displayed[3].remove()
                self.fig.canvas.draw()        

        # simple picking, lines, rectangles and text
        def onpick(event):
            ind = event.ind
            ind = ind[0]
            print('onpick3 scatter:', ind, self.TwoD_index[:,1][ind], self.TwoD_index[:,0][ind])
            self.display_img(ind)
            #_, path = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind])
            #_, left_limit = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind] - np.pi/4, step=0.1)
            #_, right_limit = self.raytrace_on_matplot(self.TwoD_index[:,0][ind], self.TwoD_index[:,1][ind], self.orientation[ind] + np.pi/4, step=0.1)
            #combined = np.concatenate((path, left_limit, right_limit), axis=0)
            #line, = self.ax1.plot(combined[:,1],combined[:,0], '.', markersize=2)
            #self.current_displayed = (ind, line) # keep track of current idx and ray
            #self.fig.canvas.draw() # update the plot with the ray
            #cv2.namedWindow(str(ind), cv2.WINDOW_NORMAL)
            #cv2.startWindowThread()
            #def back(*args):
            #    cv2.destroyAllWindows()
            #cv2.createButton("Back",back,None,cv2.QT_PUSH_BUTTON,1)

            #cv2.resizeWindow(str(ind), 800, 400)
            #img_with_heatmap = self.images[ind].copy()
            #width_cutoff = img_with_heatmap.shape[1] // 2
            #img_with_heatmap[:, width_cutoff:] = cv2.applyColorMap(img_with_heatmap[:, width_cutoff:], cv2.COLORMAP_JET)
            #tempfig, _ = plt.subplots()
            #tempfig.canvas.mpl_connect('close_event', onclose)
            #plt.xticks([]), plt.yticks([])
            #plt.imshow(img_with_heatmap)
            #plt.show()
            #cv2.imshow(str(ind), img_with_heatmap)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            #line.remove() # remove the ray
            #self.fig.canvas.draw()

        self.fig.canvas.mpl_connect('pick_event', onpick)

    def click_btn(self):
        def onprev(event):
            if not self.current_displayed:
                return
            fig, ax, idx, ray = self.current_displayed
            lastx, lasty = self.wall_hits[:, 0][idx], self.wall_hits[:, 1][idx]
            for ind in range(len(self.wall_hits)):
                if 0 < lastx - self.wall_hits[:, 0][ind] < 0.2 and abs(self.wall_hits[:, 1][ind] - lasty) < 0.5:
                    self.display_img(ind)
                    break
            print("cant find prev")
        
        def onnext(event):
            if not self.current_displayed:
                return
            fig, ax, idx, ray = self.current_displayed
            lastx, lasty = self.wall_hits[:, 0][idx], self.wall_hits[:, 1][idx]
            for ind in range(len(self.wall_hits)):
                if 0 < self.wall_hits[:, 0][ind] - lastx < 0.2 and abs(lasty - self.wall_hits[:, 1][ind]) < 0.5:
                    self.display_img(ind)
                    break
            print("cant find next")

        self.bprev.on_clicked(onprev)
        self.bnext.on_clicked(onnext)

def main(args):
    #visualizer('/home/locobot/slam_ws/image_data').process()
    v = visualizer('../../../307-1data')
    v.draw_wall_jason()
    v.process()

if __name__ == '__main__':
    main(sys.argv)
