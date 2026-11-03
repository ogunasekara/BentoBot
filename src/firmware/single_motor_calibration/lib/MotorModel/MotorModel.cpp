#include "MotorModel.h"
#include <cmath>

MotorModel::MotorModel(
    double cutoff,
    double slope
):
    cutoff { cutoff },
    slope { slope }
{}

double MotorModel::getPWMFeedForward(
    double ang_vel
) {
    return std::copysign(cutoff, ang_vel) + ang_vel * slope;
}

double MotorModel::getCutoff()
{
    return cutoff;
}
