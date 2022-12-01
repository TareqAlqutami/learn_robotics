%% Vehicle params

% The inertia of the airframe AND propellers
I = [ ...
   0.005831943165131                   0                   0;
                   0   0.005831943165131                   0;
                   0                   0   0.011188595733333;
           ]; % (kg.m^2)
       
m           = 0.9272;    % (kg)
L           = 0.2;     % (m) (arm length) distance from Center of Gravity to motor
Xe_init     = [0;0;0]; % (m)       Initial position in INERTIAl axes
Vb_init     = [0;0;0]; % (m/sec)   Initial velocity in BODY axes
wb_init     = [0;0;0]; % (rad/sec) Initial body rates
eul_init    = [0;0;0]; % (rad)     Initial attitude in EULER angles [roll,pitch,yaw]
q_init      = [1;0;0;0]; % (rad)   Initial attitude in quaternion 
g = 9.81;

kt         = 0.9e-5;      % LIFT per w2 in (N  /( (rad/sec)^2 ))
km         = 1.2e-7;      % TWIST per w2 in (N.m/( (rad/sec)^2 ))
thrust_max_per_motor  = 8;        % (N), max thrust that 1 propeller can generate
thrust_max = thrust_max_per_motor*4; % in N, max total thrust 
%w_max      = 108*2*pi;    % (rad/sec) the max possible propeller rotational speed
w_max       = sqrt(thrust_max_per_motor/kt);    % (rad/sec) the max possible propeller rotational speed
TQ_max_per_motor  = km*(w_max.^2); % max torque (N.m) 
vel_xy_max = 15; %vel in m/s
vel_z_max = 10; %vel in m/s
rate_p_max = 3.5; % body rate (p) max in rad/s
rate_q_max = 3.5; % body rate (p) max in rad/s
rate_r_max = 2.5; % body rate (p) max in rad/s
tilt_max = pi/6; % max allowed angle the drone can tilt


%% control gains

% position P controller gains
Kp_pos = [7,7,3]';

% velocity PID controller gains
Kp_vel = [3,3,5]';
Ki_vel = [0.1,0.1,0.1]';
Kd_vel = [0.8,0.8,0.3]';


% attitude P controller gains (roll/pitch/yaw)
Kp_att = [6,6,6]'; %[roll,pitch, yaw

% rates PD controller gains
% Kp_rates = [1.5,1.5,1.0];
% Kd_rates = [0.04,0.04,0.1];
Kp_rates = [10,10,10]';
Kd_rates = [0.05,0.05,0.1]';


