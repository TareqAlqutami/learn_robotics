function H = ekf_measurement_jacobian(x,u)
% Jacobian of measurement model 
% Inputs:
%    xk - States x[k]
%    u - reference acc and mag vectors
ref_acc = u(1:3);
ref_mag = u(4:6);

% get current attitude quaternion
q0    = x(1);
q1    = x(2);
q2    = x(3);
q3    = x(4);

g = ref_acc(3);
M = ref_mag(1);

h_acc = 2*g*[
                -q2  q3 -q0 q1;
                 q1 q0   q3 q2;
                 q0 -q1 -q2 q3;
             ];
h_mag = 2*M*[
                 q0  q1 -q2 -q3;
                -q3  q2  q1 -q0;
                 q2  q3  q0  q1;
             ];         


H = [
    h_acc zeros(3);
    h_mag zeros(3);
    ];

