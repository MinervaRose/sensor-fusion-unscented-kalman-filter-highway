# Unscented Kalman Filter

## Overview

This project implements an Unscented Kalman Filter (UKF) using the CTRV motion model to track multiple vehicles on a simulated highway. The filter fuses lidar and radar measurements to estimate position and velocity.

The goal was to follow the UKF algorithm from the lessons and achieve RMSE values below the rubric thresholds.

## Process & Tuning

I implemented:

* Initialization of the state vector and covariance

* Augmented sigma point generation

* Sigma point prediction using CTRV

* Mean and covariance prediction

* Lidar update (linear)

* Radar update (non-linear, using UKF measurement space transform)

* Angle normalization and optional NIS checks

The main tuning focused on the process noise values:

std_a_     = 1.5;
std_yawdd_ = 0.9;

## RMSE Results

After more than 1 second of runtime, the RMSE values for all tracked cars were:

X  = 0.05-0.06
Y  = 0.04-0.08
Vx = 0.22-0.44
Vy = 0.43-0.56

All values are well below the required thresholds:

[0.30, 0.16, 0.95, 0.70]

## Conclusion

The UKF implementation:

* Correctly fuses lidar and radar

* Tracks all vehicles accurately

* Produces stable RMSE values below the rubric limits

* Follows the algorithm described in the course