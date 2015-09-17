#ifndef _BLUEJAY_GEOMETRY_H_
#define _BLUEJAY_GEOMETRY_H_

#include <cmath>

/* 
 * Provides geometry interfaces
 */

#define M_PI 3.14159265358979323846

/*
 * Point in a frame
 * x - forward/back
 * y - left/right
 * z - up/down
 */
class Point{
public:
    Point(): x(0), y(0), z(0) {}
    Point(double x_, double y_, double z_): x(x_), y(y_), z(z_) {}
    
    double distance(){
        return distanceFrom(0, 0, 0);
    }
    double distanceFrom(double x_, double y_, double z_){
        double xd = x_-x;
        double yd = y_-y;
        double zd = z_-z;
        return sqrt(xd*xd+yd*yd+zd*zd);
    }
    
    double x;
    double y;
    double z;
};

/*
 * Orientation in a frame
 * r - left right roll
 * p - up down 
 * y - left right turn
 */
class Orientation{
public:
    Orientation(): r(0), p(0), y(0) {}
    Orientation(double r_, double p_, double y_): r(r_), p(p_), y(y_) {}
    double r;
    double p;
    double y;
};

/* 
 * Position and orientation in a frame 
 */
class Pose{
public:
    Point position;
    Orientation orientation;
};

/* Translation vector */
typedef Point Vector;

/* Rotation matrix */
class RotationMatrix{
public:
    /* Initalizes a rotation matrix using the yall, roll, pitch Euler angles (Tait-Bryan) */
    RotationMatrix(double y, double p, double r){
        init(y, p, r);
    }
    /* Converts a orientation to a rotation matrix from the default frame */
    RotationMatrix(Orientation o){
        init(o.y, o.p, o.r);
    }
    
    double elem[3][3];
private:
    void init(double y, double p, double r){
        elem[0][0] = cos(y)*cos(p);
        elem[0][1] = cos(y)*sin(p)*sin(r)-cos(r)*sin(y);
        elem[0][2] = sin(y)*sin(r)+cos(y)*cos(r)*sin(p);
        elem[1][0] = cos(p)*sin(y);
        elem[1][1] = cos(y)*cos(r)+sin(y)*sin(p)*sin(r);
        elem[1][2] = cos(r)*sin(y)*sin(p)-cos(y)*sin(r);
        elem[2][0] = -sin(p);
        elem[2][1] = cos(p)*sin(r);
        elem[3][2] = cos(p)*cos(r);
    }
};

#endif