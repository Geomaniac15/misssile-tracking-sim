# Missile Tracking Simulation

## Overview

This project simulates the detection and tracking of a ballistic missile
using:

-   Physics-based trajectory modelling
-   A noisy ground-based radar sensor
-   A Constant-Velocity Kalman Filter for state estimation
-   Uncertainty visualisation via covariance ellipses

The system demonstrates how noisy sensor measurements can be fused into
a smooth and probabilistically grounded trajectory estimate.

------------------------------------------------------------------------

## Features

### 1. Physics-Based Missile Model

The missile trajectory includes:

-   Thrust-based boost phase
-   Mass depletion during burn
-   Gravity-driven ballistic phase
-   Time-stepped numerical integration

State variables: - Position: (x, y) - Velocity: (vx, vy) - Mass - Pitch
schedule during boost

------------------------------------------------------------------------

### 2. Radar Sensor Simulation

Radar measurements are converted from polar (range, bearing) to Cartesian coordinates before filtering, enabling use of a linear Kalman filter.

The radar:

-   Measures range and bearing
-   Operates at discrete scan intervals
-   Includes Gaussian noise
-   Has configurable maximum range

Measurement model:

$Range: ρ = \sqrt{(x - xr)^2 + (y - yr)^2}$

Bearing: $φ = atan2(y - yr, x - xr)$

Noise is applied to both range and bearing before converting back to
Cartesian coordinates.

------------------------------------------------------------------------

### 3. Kalman Filter Tracking

The motion model assumes constant velocity between measurements.

A constant-velocity Kalman filter estimates:

$x = \[x, y, vx, vy\]\^T$

The filter includes:

-   Predict step using motion model
-   Update step using radar measurements
-   Process noise (acceleration uncertainty)
-   Measurement noise (position uncertainty)

The filter smooths noisy radar observations and reconstructs a stable
trajectory estimate.

------------------------------------------------------------------------

### 4. Uncertainty Ellipse Visualisation

The top-left 2×2 block of the covariance matrix represents position
uncertainty.

Eigen-decomposition is used to compute:

-   Major axis
-   Minor axis
-   Orientation

Ellipses represent 95% confidence regions based on the chi-squared
distribution ($χ²_{0.95,2} = 5.991$).

------------------------------------------------------------------------

## Project Structure

```bash
missile.py # Missile physics model 
radar.py # Radar sensor model
kalman.py # Constant-velocity Kalman filter 
simulator.py # Main simulation loop and plotting
```
------------------------------------------------------------------------

## How to Run

1.  Install dependencies: ```bash pip install numpy matplotlib```

2.  Run: ```bash python simulator.py```

------------------------------------------------------------------------

## Educational Objectives

This project demonstrates:

-   Nonlinear physics simulation
-   Sensor modelling under uncertainty
-   Linear state-space estimation
-   Covariance propagation
-   Probabilistic reasoning in dynamic systems

------------------------------------------------------------------------

## Future Extensions

-   Extended Kalman Filter using range/bearing directly
-   Multi-target tracking and data association
-   False alarm generation and track management
-   Atmospheric drag modelling
-   Impact point prediction
