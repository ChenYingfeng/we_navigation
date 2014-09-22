#ifndef UTIL_H
#define UTIL_H

#include <math.h>
#include <valarray>
#include <vector>
#include <ros/ros.h>
#include "xform.h"
#include <sensor_msgs/LaserScan.h>
using namespace std;
struct Point
{
    float x,y;
    Point(float _x,float _y)
    {
        x=_x;
        y=_y;
    }
    Point()
    {}
};

float sign(float value);
float SlopeLine(Point p1,Point p2);
float PointToLineDist(Point pp,Point p1,Point p2);
float PointToLineDist(Point pp,float k,float b);
bool LineFit(float &k,float &b,float &ee,std::valarray<float> &data_x,std::valarray<float> &data_y);
void Smooth(float * const x,float *const y,uint windowSize,uint num,char * const valid);
void DetectCornerPoint(float * const x,float *const y,float * const kDiff,char * const flag,char * const valid,uint num,uint r,float threshold);
vector <Point> FilterCornerPoint(vector <int> &index,float *const x,float * const y,float *const kDiff,char *const flag,uint num,uint r,float threshold);

#endif
