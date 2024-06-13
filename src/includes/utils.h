#ifndef __UTILS_H__
#define __UTILS_H__

struct Params{
    double fx;
    double fy;
    double cx;
    double cy;
    double skew;
    double d[4];
    double radius;
};

struct Point{
    double x;
    double y;
    Point(double _x, double _y) : x(_x), y(_y){};
};

#endif