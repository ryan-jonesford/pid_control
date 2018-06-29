#ifndef PID_H
#define PID_H

class PID {
   public:
    /*
     * Errors
     */
    double p_error_;
    double i_error_;
    double d_error_;

    /*
     * Coefficients
     */
    double Kp_;
    double Ki_;
    double Kd_;

    double Dp_;
    double Di_;
    double Dd_;

    double tolerance_; 

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

    /*
     * Adjust the coefficients using the twiddle algorithm
     */
    bool twiddle(double tolerance, int steps, double cte);

   private:
    int count_;
    bool twiddle_active;
    double best_cte_;
};

#endif /* PID_H */
