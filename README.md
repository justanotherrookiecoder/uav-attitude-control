# UAV Attitude Control: Controller Comparison

A MATLAB-based simulation study comparing three control strategies for UAV attitude stabilization under disturbances and variable moments of inertia.

## Overview

This project investigates how different controllers regulate the roll and pitch angles of an unmanned aerial vehicle (UAV) when its dynamics change and external disturbances act on the system.

The controllers under comparison are:

- **Fixed-Gain PID:** Uses constant proportional, integral, and derivative gains.
- **Gain-Scheduled PID:** Adjusts selected controller gains based on changes in the estimated moment of inertia.
- **Model Predictive Control (MPC):** Optimizes control inputs over a finite prediction horizon while respecting actuator torque limits.

## Simulation Setup

- **Environment:** MATLAB
- **Controlled axes:** Roll and pitch
- **Reference angles:** Roll = 10°, Pitch = −5°
- **Actuator torque limits:** ±0.5 N·m
- **Simulation duration:** 10 seconds
- **Disturbance tests:** Multiple disturbance amplitudes and changes in moment of inertia

## Current Experiments

- Baseline attitude stabilization
- Variable-inertia response
- Comparison of controller performance
- Robustness testing under different disturbance amplitudes

## Performance Evaluation

The simulations evaluate controller performance using:

- Mean squared attitude error
- Peak absolute attitude error
- Root mean square control torque
- Solver success and failure counts for MPC

## Scope and Limitations

This is a simplified simulation of independent roll and pitch axes, not a complete six-degree-of-freedom UAV model. The results are simulation-based and do not establish real-world flight performance.

The gain-scheduled controller assumes the moment of inertia is known. MPC uses a simplified prediction model and does not have advance knowledge of future disturbances.

## Project Status

The initial controller implementations and disturbance-sweep simulations are complete. Further validation, result visualization, and analysis are ongoing.

## Repository Contents

The repository contains MATLAB scripts for baseline control, gain-scheduled control, MPC, variable-inertia simulations, controller comparison, robustness evaluation, and disturbance sweeps.

## Research Objective

To compare the simulated attitude-tracking performance and control effort of fixed-gain PID, gain-scheduled PID, and MPC under the same simplified UAV model and disturbance conditions.
