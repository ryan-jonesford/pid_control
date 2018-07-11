# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## PID Tuning and Opimization

I first manually tuned the steering to have these values: 

	  K_p = .07
	  K_i = 0.001
	  K_d =10

I arrived at these values by first setting the integral and derivative coefficients to 0 and slowly changed the proportional coefficient until the car started oscillating without the oscillations seeming like they were getting bigger. This happens because the proportional coefficient (aka gain) of the system is proportional to the error; the higher the error, the higher the correction. Because the output lags the input of our system we can easily get into oscillations.

To dampen these oscillations I then tuned the derivative coefficient until the oscillations disappeared. 
Once the oscillations were controlled the only thing left was to remove the stead-state offset. Using print statements I could see that the CTE was always off by a small fixed amount and adjusted the integral coefficient until this offset was minimized. 

After manually tuning the PID I used the Twiddle algorithm to better optimize the PID coefficients. First I used it to optimized for a low CTE for 750 samples at a speed set point of 35mph. After the optimization at this level I increased the speed set point and did it again. 

Finally I changed the criteria for Twiddle to use the number of data-points that were collected before the car’s CTE exceeded 2, a value that I determined means the car has gone off (or close to off) the track. The final values that I ended with are:

	K_p = 0.174175 
	K_i =  0.00014839
	K_d = 9.03749

with a speed set-point of 60mph.

A small hack I put in is, if the car’s CTE is more than 1, is to quickly reduce the speed set-point, keeping the car on the track when traversing a corner at unsafe speeds. 

## Throttle PID

The throttle PID controller I determined a different way. Using a cruise control model, I determined a transfer function, and used GNU Ovtave to determine a suitable step response. The cruise control model I used was M*dv/dt(t) + b*v(t) = u(t). This gives a transfer function of 1/(Ms+b) in the s-domain. I perused the source code of the simulator and found the set characteristics of the vehicle for this course; the mass M = 1000kg and the viscous friction coefficient b = .1. The code for this can be found in pid_tuning.m.

