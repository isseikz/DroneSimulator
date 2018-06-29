# DroneSimulator
Flight Simulator and Implementation of Controller for Quadrotors

## Environment
+ Ubuntu 16.04
+ Python 3.6.5 :: Anaconda, Inc.
  - matplotlib
  - scipy
  - numpy

## File Structure
+ `Quadrotor.py` expresses the Model of a quadrotor.  The model includes the inertial parameters, the attitudes and the controller to be used.
+ `PD.py` expresses the implementation of a Proportional-Derivative controller. the equation is defined with the parameter $\omega$, $\zeta$, such that:
<img src="https://latex.codecogs.com/gif.latex?\dot{x}&space;=&space;-2\zeta\omega{x}-\omega^2{x}"/>
