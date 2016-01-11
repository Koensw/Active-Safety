#include "Potential.h"

Vector Potential::getGradient(Point _point){
    //get distance between point and potential
    Vector vector(_point - _pos);
    
    //convert distance to potential gradient
    double potential_gradient = -_potential_function->getGradient(vector.norm());
    
    //normalize and scale between vector with gradient
    vector.normalize();
    vector *= potential_gradient;
    
    //scale with potential strength
    vector *= _strength;
    
    return vector;
}

void Potential::translate(Vector vec){
    _pos.x() += vec.x();
    _pos.y() += vec.y();
    _pos.z() += vec.z();
}

void Potential::rotate(RotationMatrix rot){
    _pos = rot*_pos;
}

std::pair<double, double> Potential::getYawPitch(){
    double pot_pitch = acos(_pos.z()/_pos.norm());
    double pot_yaw = acos(_pos.x()/(_pos.norm()*sin(pot_pitch)));
    pot_pitch = 0.5*M_PI-pot_pitch;
    return std::make_pair(pot_yaw, pot_pitch);
}