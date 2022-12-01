function qDOT = quad_6dof_eoms( t, q, m, I, FbMb_at_t )
    % get the excitation Forces and Moments
    FbMb = FbMb_at_t(t);
    F    = FbMb(1:3);    % (N),   Body frame, Force  vector
    M    = FbMb(4:6);    % (N.m), Body frame, Moment vector

    % extract components from the STATE vector
    vB    = q( 1:3);   % (m/s),   Body frame, translational vel 
    w     = q( 4:6);   % (rad/s), Body frame, angular velocity       
    e     = q( 7:9);   % (rad),   Euler angles [yaw,pitch,roll]
    xyzE  = q(10:12);  % (m),     INERTIAL frame, position

    %  F = m*(vDOT + w_x_v)   
    vDOT = F/m - cross(w,vB); 

    % M  = I*wDOT + w_x_(I*w) 
    wDOT = inv(I) * (M - cross(w, I*w)); 

    % euler rates from body rates
    eDOT = LOC_get_eDOT(w, e); 

    % Inertial velocity
    bRg     = LOC_get_bRg(e);
    gRb     = bRg.'; 
    vE      = gRb * vB;
    xyzEDOT = vE;

    % assemble the final derivative vector
    qDOT = [     vDOT;
                 wDOT;
                 eDOT;
              xyzEDOT;
            ];
end
%_#########################################################################

function bRg = LOC_get_bRg(e)
    
    phi   = e(1); %yaw
    theta = e(2); % pitch 
    psi   = e(3); % roll

bRg = [ ...
                              cos(phi)*cos(theta),                              cos(theta)*sin(phi),         -sin(theta);
 cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi);
 sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), cos(psi)*cos(theta);
 ];

end
%_#########################################################################
function euler_rates = LOC_get_eDOT(w, e)

    p     = w(1);
    q     = w(2);
    r     = w(3);

    phi   = e(1); %yaw
    theta = e(2); %pitch
    psi   = e(3); %roll

    euler_rates = [ ...
        (r*cos(psi) + q*sin(psi))/cos(theta);
        q*cos(psi) - r*sin(psi);
        (p*cos(theta) + r*cos(psi)*sin(theta) + q*sin(psi)*sin(theta))/cos(theta);
        ];
end



