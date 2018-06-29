#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"

class twiddle {
   public:
    void init(double thresh);
    void run(PID &pid);

   private:
    double thresh_;
};

#endif /* TWIDDLE_H */