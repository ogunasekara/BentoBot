#pragma once

#include "Arduino.h"

class MotorModel
{
    public:
        MotorModel(
            double cutoff,
            double slope
        );
        double getPWMFeedForward(double ang_vel);
        double getCutoff();
    
    private:
        double cutoff;
        double slope;
};
