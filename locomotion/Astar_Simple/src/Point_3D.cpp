//
// Created by anxing.
//
#include "Point_3D.h"

Point_3D::Point_3D(double x_,double y_,double theta_,bool data_collect_){
    x=x_;
    y=y_;
    theta=theta_;
    data_collect = data_collect_;
}

Point_3D::Point_3D(double x_,double y_,double theta_){
    x=x_;
    y=y_;
    theta=theta_;

}



Point_3D::Point_3D(double x_,double y_){
    x=x_;
    y=y_;
    theta=0;
}