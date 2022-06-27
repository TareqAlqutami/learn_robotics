# **Learn Robotics**
This repository contains notes, simulations and explanations of multiple robotic concepts. It is organized such that concepts are incremental in complexity.
Read the readme within each subfolder for info on its content.

## **Requirements**
1. MATLAB R2021a or above
2. Install the Robotics Toolbox by Peter Corke. The correct version is provided with this repo `RTB.mltbx`
3. Add required folders to MATLAB path
   - `library` folder and subfolders

## **Basics**
### 1. symbolic math (MATLAB)
- Symbolic math vs numeric math
- Symbolic math operations in MATLAB
- Case study: Spring mass damper
## **Kinematics**
###  1. Planar Manipulators
###  2. Forward and inverse kinematics of three-link planar
###  3. Three-link planar from URDF and waypoint tracking

## **Dynamics**
### 1. Rigid body dynamics
- Intro rigid body dynamics and equation of motions
- Simulate dynamics of a two-link planar arm using Robotics Toolbox
### 2. Two-link Planar dynamics
- Formulate the dynamics using Lagrange formulation
- Simulate the dynamics in Simulink
- Compare derived dynamics to SimScape multibody model

![](media/twolink_planar_dynamics_1x_friction.gif)

## **Control**
### 1- Introduction to joint control: 
- transfer function of a motor and simple control
- Simulink simulation
### 2- Two-link planar control
- Ideal vs realistic simulation
- Examples of different controllers:
    - PD control
    - PID control
    - Full state feedback control (Pole Placement and LQR)
    - Gravity compensated control
    - Inverse Dynamics control
    - Robust control

## **docs**
  Contains reports that analyze and summarize results

## **Drawings**
Contains some drawing files and block diagrams

## **Media**
Contains videos and images of sample results.

## **library**
Contains scripts and functions that are used in other modules