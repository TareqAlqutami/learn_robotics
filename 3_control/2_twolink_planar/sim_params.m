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

%% Limits and measurements
Tsample = 0.001; % sampling time (seconds)
max_joint_vel       =  pi; % rad/s
max_joint_torque1   =  20; % Nm
max_joint_torque2   =  10; % Nm
joint_pos_precision =  0.005; % rad 
gyro_precision =  0.005; % rad/s
torque_precision    =  0.2; % Nm
joint_acc_variance  = 0;
joint_vel_variance  = 0;
joint_pos_variance  = joint_pos_precision^2;

%% Define some damping at each joint: assume that each pivot point experiences viscous damping, ie: .   
% damping torque is a function of the relative joint velocity (viscous frcition) 
b1 = 0.01; %0.1 (N.m/(rad/sec));
b2 = 0.01; %0.1 (N.m/(rad/sec));

%% robot representation using Robotics toolbox,
% this is to use IK and FK of robotics toolbox

RR_robot = rigidBodyTree;
link1 = rigidBody('link1');
jnt1 = rigidBodyJoint('theta1','revolute');
link2 = rigidBody('link2');
jnt2 = rigidBodyJoint('theta2','revolute');

dhparams = [L1x 0 0 0;
            L2x 0 0 0];

setFixedTransform(jnt1,dhparams(1,:),'dh');
link1.Joint = jnt1;
addBody(RR_robot,link1,'base')

setFixedTransform(jnt2,dhparams(2,:),'dh');
link2.Joint = jnt2;
addBody(RR_robot,link2,'link1')
end_effector = rigidBody('end_effector');
addBody(RR_robot,end_effector,'link2');
%% initial conditions
theta1_0     = 0;                            % rad
theta1_dot_0 = 0;                            % rad/sec 
theta2_0     = 0;                            % rad
theta2_dot_0 = 0;                            % rad/sec
alpha_0      = theta1_0 + theta2_0;          % rad
alpha_dot_0  = theta1_dot_0 + theta2_dot_0;  % rad/sec


%% State Space matrices
[A,B,C,D] = gen_twolink_ss(L1x,L1y,L2x,L2y,b1,b2,m1,m2);

%% Electric motor parameters
% some DC motor parameters
% P_mot.Vmax     = 12;
% P_mot.ID       = 0;        % (-)           ID
% P_mot.Ka       = 40e-3;    % (N.m/A)       Torque constant
% P_mot.Ke       = 40e-3;    % (V.sec/rad)   Back EMF constant
% P_mot.b        = [];       % (N.m.sec/rad) Viscous friction co-efficient
% P_mot.I        = [];       % (kg.m^2)      Rotor ONLY Inertia
% P_mot.R        = 5;        % (Ohms)        Armature resistamce
% P_mot.L        = 0.63e-3;  % (Henry)       Armature Inductance
% P_mot.G        = 100;      % gearbox ratio = w_in/w_out = TQ_out/TQ_in
% P_mot.TQ_G_max = 3;        % (N.m)max torque from gearhead output
% P_mot.TQ_m_max = P_mot.TQ_G_max/P_mot.G;% (N.m) max torque from motor shaft