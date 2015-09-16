#ifndef _BLUEJAY_GEOMETRY_H_
#define _BLUEJAY_GEOMETRY_H_

/* 
 * Provides geometry classes
 */

class Point{
public:
    Point(): x(0), y(0), z(0) {}
    Point(int x_, int y_, int z_): x(x_), y(y_), z(z_) {}
    double x;
    double y;
    double z;
};

#endif