# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This project implements an extended Kalman filter in C++.  The input is in the form of simulated lidar and radar measurements detecting a bicycle that travels around a vehicle. The objective is to use a Kalman filter, the lidar measurements and the radar measurements to track the bicycle's position and velocity.

This project requires Udacity Simulator which provides the inputs to the Kalman filter.

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF`

## Files in src Folder

**main.cpp** - communicates with Udacity Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE.

**FusionEKF.cpp** - initializes the filter, calls the predict function, calls the update function.

**kalman_filter.cpp**- defines the predict function, the update function for lidar, and the update function for radar.

**tools.cpp**- function to calculate RMSE and the Jacobian matrix.
