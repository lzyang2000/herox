//
// Created by anxing.
//

#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "Point_3D.h"
using namespace std;
using namespace cv;


namespace pathplanning{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};



struct Node{
    Point_3D point;  // node coordinate
    int F, G, H;  // cost
    Node* parent; // parent node

    Node(Point_3D _point = Point_3D(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
        
    }
};

struct AstarConfig{
    bool Euclidean;         // true/false
    int OccupyThresh;       // 0~255
    int InflateRadius;      // integer

    AstarConfig(bool _Euclidean = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Euclidean(_Euclidean), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }
};

class Astar{

public:
    // Interface function
    void InitAstar(Mat& _Map, AstarConfig _config = AstarConfig());
    void InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config = AstarConfig());
    void PathPlanning(Point_3D _startPoint, Point_3D _targetPoint, vector<Point_3D>& path);
    bool CheckCollision(Mat _LabelMap, int x, int y, double theta);
    bool CheckData( int x, int y);
    void RobotKinematics(Mat neighbor,Mat _LabelMap,int &x,int &y, double &theta_,int cx,int cy,double ct,int k);
    void DrawPath(Mat& _Map, vector<Point_3D>& path, InputArray Mask = noArray(), Scalar color = Scalar(0, 0, 255),
            int thickness = 1, Scalar maskcolor = Scalar(255, 255, 255));
    void Smooth(vector<Point_3D>& path);
private:
    void MapProcess(Mat& Mask);
    Node* FindPath();
    void GetPath(Node* TailNode, vector<Point_3D>& path);

private:
    //Object
    Mat Map;
    Point_3D startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    AstarConfig config;
    vector<Node*> OpenList;  // open list
    vector<Node*> PathList;  // path list
};

}




#endif //ASTAR_H
