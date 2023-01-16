function A = ekf_process_jacobian(x,w)
% Jacobian of process model for the EKF
% we don't need the gyro rates as inputs since the process model is linear
% We put it as input to comply with Simulink EKF convention


Ts = 0.001; % update interval
q0     = x(1);
q1     = x(2);
q2     = x(3);
q3     = x(4);

Sq =  [
                -q1, -q2, -q3;
                 q0, -q3,  q2;
                 q3,  q0, -q1;
                -q2,  q1,  q0;
       ];



A = [
        eye(4)       -(Ts/2)*Sq;
        zeros(3,4)   eye(3);
    ];
end
