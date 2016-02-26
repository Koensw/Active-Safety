#include "potential_functions.h"

#include <iostream>

int main(){    
    QuadraticLinearPotentialFunction pot(0.5);
    for(int i=0; i<100; ++i){
        std::cout << i/10.0 << " " << pot.getValue(i/10.0) << " " << pot.getGradient(i/10.0) << std::endl;
    }
}