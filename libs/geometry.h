#ifndef _BLUEJAY_GEOMETRY_H_
#define _BLUEJAY_GEOMETRY_H_

#include <cmath>
#include <limits>

//TODO: better use a tested geometry library (Eigen?)

/* 
 * Provides geometry interfaces
 */

#define M_PI 3.14159265358979323846
#define M_EPS 1e-7

/*
 * Point in a frame
 * x - forward/back
 * y - left/right
 * z - up/down
 */
class Vector;

class Point{
public:
    Point(): x(0), y(0), z(0) {}
    Point(const Vector &p);
    Point(double x_, double y_, double z_): x(x_), y(y_), z(z_) {}
    
    //FIXME: correct name (conflicts a bit with typedef point to vector)
    double distanceOrigin(){
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
class Vector{
public:
    Vector(): x(0), y(0), z(0) {}
    Vector(const Point &p) {
        x = p.x; y = p.y; z = p.z;
    }
    Vector(const Point &e, const Point &b){
        x = e.x-b.x; y = e.y-b.y; z = e.z-b.z;
    }
    Vector(double x_, double y_, double z_) { 
        x = x_; y = y_; z = z_; 
    }   
    
    double length(){
        return sqrt(x*x+y*y+z*z);
    }
    void normalize(){
        double len = length();
        if(len < M_EPS) return;
        x /= len;
        y /= len;
        z /= len;
    }
    void scale(double factor){
        x *= factor;
        y *= factor;
        z *= factor;
    }
    
    double x;
    double y;
    double z;
};

/* Operators */
Vector operator+(const Vector &v1, const Vector &v2);
Vector operator-(const Vector &v1, const Vector &v2);

Orientation operator-(const Orientation &o);

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
    void init(double y, double p, double r);
};

#endif