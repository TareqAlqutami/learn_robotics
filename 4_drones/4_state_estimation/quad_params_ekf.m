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

Ts = 0.001; % sampling time
Ts_IMU = 0.001;


% IMU sensors characteristics
IMU.noiseSeeds =   [ 41    41    41    41    41    41   41   41   41]';
IMU.noisePower = [ 2.183e-06    1.864e-06     3.725e-06     1.0652e-08    1.3021e-08    1.1929e-08  1.0e-06    1.3021e-06    1.1929e-06];

IMU.accScaleCross = [1.00       0         0;
                        0    1.00         0;
                        0       0      1.00];
IMU.accBias = [0.0900   -0.0600    0.3370];
IMU.accLimits = [-50   -50   -50    50    50    50];

IMU.gyroScaleCross = [1.00         0         0;
                         0      1.00         0;
                         0         0      1.00];                             
IMU.gyroBias = [ -0.0095   -0.0075    0.0015];
IMU.gyroLimits =  [-10   -10   -10    10    10    10];

IMU.magInc = 0; % site specific inclination angle, assumed to be zero 
IMU.magF = [1  0  0]'; % Magnetic field vector magnitiude in local navigation coordinate system (μT)
IMU.magBn = IMU.magF.*[cos(IMU.magInc); 0; sin(IMU.magInc)]; % 
IMU.magBias = [0.0095   0.0075    0.0015];
IMU.magScaleCross = [1.00       0       0;
                        0    1.00       0;
                        0       0    1.00];
IMU.magLimits =  [-1200   -1200   -1200    1200    1200    1200];  

% filters
accel_filter_coef = [    0.0264    0.1405    0.3331    0.3331    0.1405    0.0264];
gyro_filter_num = [ 0.2821    1.2725    2.4208    2.4208    1.2725    0.2821];
gyro_filter_den = [ 1.0000    2.2287    2.5245    1.5773    0.5410    0.0796];

% Assuming calibration is done, estimated bias and scaling factors
% these are the values from calibration
% we will assume we don't know the gyro bias (all zeros) to simulate drift
% and use EKF to fix this
IMU.gyroBiasEst = [0,0,0];%[ -0.0095   -0.0075    0.0015];   
IMU.accBiasEst = [0.091,-0.061,0.34]; %[0.0900   -0.0600    0.3370];
IMU.magBiasEst = [0.005,0.008,0.002]; %[0.0095   0.0075    0.0015];
IMU.inverseAccGain = 1./diag(IMU.accScaleCross);
IMU.inverseGyroGain = 1./diag(IMU.gyroScaleCross);
IMU.inverseMagGain = 1./diag(IMU.magScaleCross);


%% EKF params
Q = diag([0.015,0.0150,0.0150,0.0150,0.001,0.001,0.001]);
R = diag([0.35,0.35,0.35,0.05,0.05,0.05]); % accelerometer has higher variance due to external forces and movement
x_0 = [1 0 0 0 0 0 0]';  %  initial state
P_init = 0.01*eye(7);    % initial covariance, we are quite sure of our initial state so covariance is low

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


