#include "Potential.h"

void Potential::translate(Vector vec){
    _pos.x += vec.x;
    _pos.y += vec.y;
    _pos.z += vec.z;
}

void Potential::rotate(RotationMatrix rot){
    Point new_pos;
    new_pos.x = _pos.x*rot.elem[0][0] + _pos.y*rot.elem[0][1] + _pos.z*rot.elem[0][2];
    new_pos.y = _pos.x*rot.elem[1][0] + _pos.y*rot.elem[1][1] + _pos.z*rot.elem[1][2];
    new_pos.z = _pos.x*rot.elem[2][0] + _pos.y*rot.elem[2][1] + _pos.z*rot.elem[1][2];
    _pos = new_pos;
}

std::pair<double, double> Potential::getYawPitch(){
    double pot_pitch = acos(_pos.z/_pos.distance());
    double pot_yaw = acos(_pos.x/(_pos.distance()*sin(pot_pitch)));
    pot_pitch = 0.5*M_PI-pot_pitch;
    return std::make_pair(pot_yaw, pot_pitch);
}