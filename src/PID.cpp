#include "Inc/PID.h"

PID::PID() {}

PID::~PID() {}

/*
 * Consturctor
 */
PID::PID(double Kp, double Ki, double Kd) {
    K_ = {Kp,Ki,Kd};
    error_ = {0,0,0};
}

double PID::step(double cte) {
    error_[2] = cte - error_[0];
    error_[0] = cte;
    error_[1] += cte;
    return (-K_[0] * error_[0] - K_[2] * error_[2] - K_[1] * error_[1]);
}
