#pragma once

class SinglePoleIIR{
public:
    SinglePoleIIR(float timeConstant, float period);

    float update(float meas);

private:
    float lastOut{0};
    float gain{0};
};