//
// Created by anxing.
//
#ifndef Point_3D_H
#define Point_3D_H

#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

class Point_3D{
public:
    double x=0;
    double y=0;
    double theta=0;
    bool data_collect = false;
    Point_3D(){
        x=0;
        y=0;
        theta=0;
    }
    Point_3D(const Point_3D &point){
        x=point.x;
        y=point.y;
        theta=point.theta;
 
    }
    Point_3D(double x_,double y_,double theta_,bool data_collect);
    Point_3D(double x_,double y_,double theta_);
    Point_3D(double x_,double y_);
};

#endif