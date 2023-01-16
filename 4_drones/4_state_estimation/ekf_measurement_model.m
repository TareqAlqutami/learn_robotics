function y = ekf_measurement_model(x,u)
% measurement model, assuming additive noise
% z[k] = h(x[k],u[k])
% sensors: magnetormeter and accelerometer
% u: references for mag (site-specific mag field from calibration) and acc (gravity vector)
ref_acc = u(1:3);
ref_mag = u(4:6);

% get current attitude quaternion
q0    = x(1);
qx    = x(2);
qy    = x(3);
qz    = x(4);

% convert quaternion to DCM (body to world)
C_11  = qx^2 + q0^2 - qy^2 - qz^2;
C_12  = 2*(qx*qy + qz*q0);
C_13  = 2*(qx*qz - qy*q0);

C_21  = 2*(qx*qy - qz*q0);
C_22  = qy^2 + q0^2 - qx^2 - qz^2;
C_23  = 2*(qy*qz + qx*q0);

C_31  = 2*(qx*qz + qy*q0);
C_32  = 2*(qy*qz - qx*q0);
C_33  = qz^2 + q0^2 - qx^2 - qy^2;
bRg = [ C_11, C_12, C_13;
        C_21, C_22, C_23;
        C_31, C_32, C_33; ];   

% acc measurement model
z_acc = bRg*ref_acc ;

% mag measurement model
z_mag = bRg*ref_mag;
y = [z_acc;z_mag];

