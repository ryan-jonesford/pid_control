#ifndef PID_H
#define PID_H
#include <vector>

class PID {
   public:
    /*
     * Errors
     */
    std::vector<double> error_;

    /*
     * Coefficients
     */
    std::vector<double> K_;
    /*
     * Constructor
     */
    PID();

    /*
     * Destructor.
     */
    virtual ~PID();

    /*
     * Initialize PID.
     */
    PID(double Kp, double Kd, double Ki);

    /*
     * Run a single step of the PID controller
     */
    double step(double cte);
};

#endif /* PID_H */
