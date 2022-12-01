# **Learn Robotics**
This repository contains notes, simulations and explanations of multiple robotic concepts. It is organized such that concepts are incremental in complexity.
Read the readme within each subfolder for info on its content.

# **Requirements**
1. MATLAB R2021a or above
2. Install the Robotics Toolbox by Peter Corke. The correct version is provided with this repo `RTB.mltbx`
3. Add required folders to MATLAB path
   - `library` folder and subfolders

# **Basics**
### 1. symbolic math (MATLAB)
- Symbolic math vs numeric math
- Symbolic math operations in MATLAB
- Case study: Spring mass damper
# **Manipulator Kinematics**
###  1. Planar Manipulators
###  2. Forward and inverse kinematics of three-link planar
###  3. Three-link planar from URDF and waypoint tracking

# **Manipulator Dynamics**
### 1. Rigid body dynamics
- Intro to rigid body dynamics and equation of motions
- Simulate dynamics of a two-link planar arm using Robotics Toolbox
### 2. Two-link Planar (RR) dynamics
- Formulate the dynamics using Lagrange formulation
- Simulate the dynamics in Simulink
- Compare derived dynamics to SimScape multibody model

![](media/twolink_planar_dynamics_1x_friction.gif)

# **Manipulator Control**
### 1- Introduction to joint control: 
- transfer function of a motor and simple control
- Simulink simulation
### 2- Joint control
- Ideal vs realistic simulations on Two-link planar (RR)
- Examples of different controllers:
    - PD control
    - PID control
    - Full state feedback control (Pole Placement and LQR)
    - Gravity compensated control
    - Inverse Dynamics control
    - Robust control
### 3- Task control
- Ideal simulations on Two-link planar (RR)
- Examples of different controllers:
    - PID control
    - Gravity compensated PD control
    - Full state feedback control (Pole Placement and LQR)
    - Inverse Dynamics control
    - Robust control (Sliding mode control)

### 4- Impedance/compliance control
- Simulations of Two-link planar (RR) interacting with different environments and target trajectories (No direct force regulation)
- Examples of different controllers:
    - Impedance control with force/torque (F/T) sensor 
    - Impedance control without (F/T sensor 
- Examples of different environments:
    - Straight Wall environment
    - Curved environment
    - Extruded environment

<img src="media/impedance_control/sim_4_twolink_impedance_control_curved.gif" width="300" height="200"/> <img src="media/impedance_control/sim_4_twolink_impedance_control_extruded.gif" width="300" height="200"/>


### 4- Force control
- Simulations of Two-link planar (RR) interacting with different environments and focused on both motion and force regulations.
- Examples of different controllers:
    - Hybrid position/force control
- Examples of different environments:
    - Straight Wall environment
    - Curved environment
    - Extruded environment
    - Different environment stiffness

<img src="media/force_control/sim_5_twolink_force_control_wall.gif" width="300" height="200"/> <img src="media/force_control/sim_5_twolink_force_control_curved.gif" width="300" height="200"/>

# **Drones**
### 1- Attitude_representation
### 2 Drone (Quadcopter) modeling
### 3- Attitude control
### 4- Position control

# **docs**
  Contains reports that analyze and summarize results

# **Drawings**
Contains some drawing files and block diagrams

# **Media**
Contains videos and images of sample results.

# **library**
Contains scripts and functions that are used in other modules

# **Notes**
Each module should contain a citation of all references consulted and external materials/code used.

# **Good references**
## Manipulators
- [Designing Robot Manipulator Algorithms](https://www.mathworks.com/videos/matlab-and-simulink-robotics-arena-designing-robot-manipulator-algorithms-1515776491590.html?s_tid=vid_pers_recs)

- [Controlling Robot Manipulator Joints](https://www.mathworks.com/videos/matlab-and-simulink-robotics-arena-controlling-robot-manipulator-joints-1521714030608.html)
-  [Trajectory Planning for Robot Manipulators](https://www.mathworks.com/videos/trajectory-planning-for-robot-manipulators-1556705635398.html)

- [Trajectory Tracking Control via Decentralized Joint-Level Schemes](https://github.com/RickyMexx/ttc-decentralized)

## Drones
- [Robotics: Aerial Robotics course in Coursera](https://www.coursera.org/learn/robotics-flight)
- [Modelling, Simulation, and Control of a Quadcopter, Mathworks webinar](https://www.mathworks.com/videos/modelling-simulation-and-control-of-a-quadcopter-122872.html)
- [Quadrotor Dynamics and Control, Randal Beard](https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub)