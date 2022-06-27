%% gravity
g = 9.80665;  % m/sec^2

%% geometry parameters 
% assuming a selender. It will be represented as prism(or BRICK) in simscape. assume that each link has the same density.
density = 3700; % kg/m3

L1x     = 0.3; % m
L1y     = 0.050; % m
L1z     = 0.05; % m
L2x     = 0.15; % m
L2y     = 0.05; % m
L2z     = 0.05; % m

m1   = density * L1x*L1y*L1z; % kg
m2   = density * L2x*L2y*L2z; % kg


%% Define some damping at each joint: assume that each pivot point experiences viscous damping, ie: .   
% damping torque is a function of the relative joint velocity (viscous frcition) 
b1 = 0.1; %0.1 (N.m/(rad/sec));
b2 = 0.1; %0.1 (N.m/(rad/sec));

%% initial conditions
theta1_0     = 0;                            % rad
theta1_dot_0 = 0;                            % rad/sec 
theta2_0     = 0;                            % rad
theta2_dot_0 = 0;                            % rad/sec
alpha_0      = theta1_0 + theta2_0;          % rad
alpha_dot_0  = theta1_dot_0 + theta2_dot_0;  % rad/sec
