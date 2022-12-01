# **Learn Robotics - Control**
This repository contains notes and examples of control strategies of robots.

## **Content**
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
- Simulations of Two-link planar (RR) interacting with different environments and target trajectories
- Examples of different controllers:
    - Impedance control with force/torque (F/T) sensor 
    - Impedance control without (F/T sensor 
- Examples of different environments:
    - Straight Wall environment
    - Curved environment
    - Extruded environment

<img src="../media/impedance_control/sim_4_twolink_impedance_control_curved.gif" width="300" height="200"/> <img src="../media/impedance_control/sim_4_twolink_impedance_control_extruded.gif" width="300" height="200"/>


### 4- Force control
- Simulations of Two-link planar (RR) interacting with different environments and focused on both motion and force regulations.
- Examples of different controllers:
    - Hybrid position/force control
    - Parallel position and force control
    - Admittance control
- Examples of different environments:
    - Straight Wall environment
    - Curved environment
    - Extruded environment
    - Different environment stiffness

<img src="../media/force_control/sim_5_twolink_force_control_wall.gif" width="300" height="200"/> <img src="../media/force_control/sim_5_twolink_force_control_curved.gif" width="300" height="200"/>

## **Notes**
- Don't forget to  add the contents of `library` folder to MATLAB path. 
- Run the live script before running the simulation to set necessary parameters

## TODO
- Simulation example of robot control in ROS.
- Add control simulation of mobile robots
- Add control simulation of drones.
