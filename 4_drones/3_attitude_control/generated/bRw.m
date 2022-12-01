function bRw_new = bRw(phi_t,theta_t,psi_t)
%BRW
%    BRW_NEW = BRW(PHI_T,THETA_T,PSI_T)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    01-Dec-2022 15:20:55

bRw_new = reshape([cos(psi_t).*cos(theta_t),-cos(phi_t).*sin(psi_t)+cos(psi_t).*sin(phi_t).*sin(theta_t),sin(phi_t).*sin(psi_t)+cos(phi_t).*cos(psi_t).*sin(theta_t),cos(theta_t).*sin(psi_t),cos(phi_t).*cos(psi_t)+sin(phi_t).*sin(psi_t).*sin(theta_t),-cos(psi_t).*sin(phi_t)+cos(phi_t).*sin(psi_t).*sin(theta_t),-sin(theta_t),cos(theta_t).*sin(phi_t),cos(phi_t).*cos(theta_t)],[3,3]);
