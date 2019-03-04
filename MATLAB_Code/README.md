# MATLAB Version: Multi_Car_Parking

This project realizes autonomous valet parking by using Model Predictive Control (MPC) Method. This folder this the MATLAB version whiche is only able to plan and follow the path. Two cars are executed sequentially and not communicating. 

<img src="https://github.com/XuShenLZ/Multi_Car_Parking/blob/master/MATLAB_Code/imgs/Effect.png" width="700" />

## Current Features

1. Car model is kinematic bicycle model and the linearization around previous MPC prediction;
2. The target parking position can be specified by user and corresponding vehicle can finish the parking automatically;

## Update Log

### 02/20/2019

1. Linearized the kinematic bicycle model around the optimal solution solved by the last MPC iteration. After the linearization, the path following ability is not influenced but the calculating speed is dramatically accelerated;
2. Removed the car simulating plot action after the path-planning and directly plotted the reference path after the sampling step;
3. Fixed some bugs in speed profile of accelerating & decelerating stages;
4. Two lanes are now elements of cell array `lane` and vehicles are elements of cell array `car` .

## References

1. The parking maneuver is generated using the algorithm of OBCA: Zhang, Xiaojing, Alexander Liniger, and Francesco Borrelli. "Optimization-based collision avoidance." *arXiv preprint arXiv:1711.03449* (2017).
