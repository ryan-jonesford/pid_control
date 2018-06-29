#include "Inc/PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

/*
 * Consturctor
 */
PID::PID(double Kp, double Kd, double Ki) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    p_error_ = 0;
    i_error_ = 0;
    d_error_ = 0;
    Dp_ = 1;
    Di_ = 1;
    Dd_ = 1;
    count_ = 0;
    twiddle_active = true;
}

double PID::step(double cte) {
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;
    return (-Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_);
}

bool PID::twiddle(double tolerance, int steps, double cte) {
    bool twiddle_done = false;
    ++count_;
    // if( count_ > steps ){
    //     adjust(kp,kd,ki);
    //     count_ = 0;
    //     twiddle_active = false;
    // }
    // else{
    //     twiddle_active = true;
    // }
    if (twiddle_active) {
        best_cte_ = cte;
    }
    double sum_Dp = 0;
    for (int d = 0; d < 3; ++d) {
        sum_Dp++;
    }
    if (sum_Dp < tolerance) {
        for (int i = 0; i < 3; ++i) {
        }
    }
    return twiddle_done;
}