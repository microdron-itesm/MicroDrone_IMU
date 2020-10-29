#include "SinglePoleIIR.h"
#include <cmath>

SinglePoleIIR::SinglePoleIIR(float timeConstant, float period){
    gain = std::exp(-period / timeConstant);
}

float SinglePoleIIR::update(float meas){
    float newVal = (1.0f - gain) * meas + gain * lastOut;
    lastOut = newVal;
    return newVal;
}