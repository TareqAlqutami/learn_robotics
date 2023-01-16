function xk = ekf_process_model(x,w)
% process model of quaternion attitude as per quaternion integration
% w is the body angular rates measurement from gyro 

% q_dot=0.5*S(w)*q=0.5*S(q)*w
% the gyro bias is added resulting in 
% q_dot=0.5*S(w-b)*q=0.5*S(q)*(w-b)
% Converting to matrix form and using first order integration
% x(k+1) = F*x(k)+B*w
% where w is coming from gyro measurements
% note that this is a linear model


% q_dot  = 0.5 * S(w-b) * q;

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

B = [
        (Ts/2)*Sq;
        zeros(3);
    ];

w = w(:);
xk  = A*x+B*w;

% normalize quaternion
%xk(1:4) =  quaternion(xk(1:4)').normalize().compact;
xk(1:4) = xk(1:4)/norm(xk(1:4));


%%  ignoring bias
% xk = zeros(7,1);
% q_dot  = 0.5 * Sq * w;
% xk(1:4) = [q0;q1;q2;q3]+q_dot*Ts; % integrate
% xk(1:4) = xk(1:4)/norm(xk(1:4));
end
