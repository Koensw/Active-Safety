#include "geometry.h"

Point::Point(const Vector &p) {
    x = p.x; y = p.y; z = p.z;
}

Vector operator+(const Vector &v1, const Vector &v2){
    Vector vec;
    vec.x = v1.x + v2.x;
    vec.y = v1.y + v2.y;
    vec.z = v1.z + v2.z;
    return vec;
}

Vector operator-(const Vector &v1, const Vector &v2){
    Vector temp_vec = v2;
    temp_vec.scale(-1);
    return v1+temp_vec;
}

Orientation operator-(const Orientation &o){
    Orientation n;
    n.r = -o.r;
    n.p = -o.p;
    n.y = -o.y;
    return n;
}

void RotationMatrix::init(double y, double p, double r){
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