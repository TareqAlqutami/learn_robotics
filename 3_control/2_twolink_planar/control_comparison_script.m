controllers_labels = {
     'PD_control','PID_control','PolePlacement_control',...
     'PolePlacement_D_control','PD_plus_gravity','PD_plus_gravity_plus_feadforward','Inverse_dynamics_control','robust_control'};
controller_display_names = {
      'PD','PID','Pole Placement',...
     'Pole Placement+D','Gravity+PD','Gravity+PD+FF','Inverse dynamics','Robust'};
plot_symbols = {'--','-.','--','-.','--',':','-.'};
n_controllers = length(controllers_labels);

trajectory = 'step_0_close'; 

%% compare ideal vs realistic for each controller
for i=1:n_controllers
    controller = controllers_labels{i};
    trajectory = 'sine'; % change this depending on the commanded trajectory (step,sine, trapz)
    plot_symbols = {'r--','m-','k-.','b--','r-','k:','k-'};
    if strcmp(trajectory,'sine')
        legend_loc = 'northeast';
    else
        legend_loc = 'northwest';
    end
    
    % === params for ideal ====
    
    vis_dev    = 1;
    length_dev = 1;
    m_dev      = 1;
    
    Kp1 = 30;
    Ki1 = 20;
    Kd1 = 2.5;
    N1 = 100;
    Kp2 = 7;
    Ki2 = 15;
    Kd2 = 0.5;
    N2 = 100;

    sys = ss(A,B,C,D);
    p = [-90, -80, -70, -10.8];
    Kpp = place(A,B,p);

    Kp1_grav  = 15;
    Kd1_grav  = 2.5;
    N1_grav  = 100;

    Kp2_grav  = 3.5;
    Kd2_grav  = 0.5;
    N2_grav  = 100;
    
    Kp1_inv  = 25;
    Kd1_inv  = 12;
    N1_inv  = 50;

    Kp2_inv  = 50;
    Kd2_inv  = 15;%48
    N2_inv  = 50; 

    n = 2;
    P = eye(2*n)/10^10;
    epsilon = 0.1; % 0.01
    rho = [1;1];
    D = [zeros(n); eye(n)];

    Kp1_robust  = 25;
    Kd1_robust  = 7;
    N1_robust   = 50;

    Kp2_robust   = 45;
    Kd2_robust   = 35;
    N2_robust   = 50;

    Kp_r = eye(n).*[Kp1_robust ;Kp2_robust ];
    Kd_r = eye(n).*[Kd1_robust ;Kd2_robust ];
    telda_H = [zeros(n)   ones(n);
                  -Kp_r     -Kd_r;]; % telda_H must have eigenvalues with negative real parts

    Q =  lyap(telda_H,P);
    D_T_Q = D'*Q;

    set_param('sim_1_twolink_joint_control_ideal/Controller','LabelModeActiveChoice', controller);
    sim('sim_1_twolink_joint_control_ideal');
    % get comanded trajectory, time, and responses
    SP_q1_ideal     = response{1}.Values.Data(:,2);
    SP_q2_ideal     = response{2}.Values.Data(:,2);
    SP_q1_d_ideal   = response{3}.Values.Data(:,2);
    SP_q2_d_ideal   = response{4}.Values.Data(:,2);
    tau1_ideal      = response{5}.Values.Data(:,1);
    tau2_ideal      = response{5}.Values.Data(:,2);
    t_ideal         = response{1}.Values.Time;
    PV_q1_ideal     = response{1}.Values.Data(:,1);
    PV_q2_ideal     = response{2}.Values.Data(:,1);
    PV_q1_d_ideal   = response{3}.Values.Data(:,1);
    PV_q2_d_ideal   = response{4}.Values.Data(:,1);


    % === realistic params ====
    Kp1 = 30; 
    Ki1 = 20;
    Kd1 = 2.5;
    N1 = 100;
    Kp2 = 7;
    Ki2 = 15;
    Kd2 = 0.5;
    N2 = 100;
    % no param dev

     sys = ss(A,B,C,D);
    p = [-25, -24, -23, -5.8];
    Kpp = place(A,B,p);
    
    Kp1_grav  = 15;
    Kd1_grav  = 2.5;
    N1_grav  = 100;

    Kp2_grav  = 3.5;
    Kd2_grav  = 0.5;
    N2_grav  = 100;

    Kp1_inv  = 25;
    Kd1_inv  = 12;
    N1_inv  = 50;

    Kp2_inv  = 50;
    Kd2_inv  = 15;%48
    N2_inv  = 50;

    n = 2;
    P = eye(2*n)/10^10;
    epsilon = 0.1; % 0.01
    rho = [1;1];
    D = [zeros(n); eye(n)];

    Kp1_robust  = 25;
    Kd1_robust  = 7;
    N1_robust   = 50;

    Kp2_robust   = 45;
    Kd2_robust   = 35;
    N2_robust   = 50;

    Kp_r = eye(n).*[Kp1_robust ;Kp2_robust ];
    Kd_r = eye(n).*[Kd1_robust ;Kd2_robust ];
    telda_H = [zeros(n)   ones(n);
                  -Kp_r     -Kd_r;];        
    Q =  lyap(telda_H,P);
    D_T_Q = D'*Q;

    vis_dev    = 1;
    length_dev = 1;
    m_dev      = 1;
    set_param('sim_2_twolink_joint_control_real/Controller','LabelModeActiveChoice', controller);
    sim('sim_2_twolink_joint_control_real');
    % get comanded trajectory, time, and responses
    SP_q1_no_dev     = response{1}.Values.Data(:,2);
    SP_q2_no_dev     = response{2}.Values.Data(:,2);
    SP_q1_d_no_dev   = response{3}.Values.Data(:,2);
    SP_q2_d_no_dev   = response{4}.Values.Data(:,2);
    tau1_no_dev      = response{5}.Values.Data(:,1);
    tau2_no_dev      = response{5}.Values.Data(:,2);
    t_no_dev         = response{1}.Values.Time;
    PV_q1_no_dev     = response{1}.Values.Data(:,1);
    PV_q2_no_dev     = response{2}.Values.Data(:,1);
    PV_q1_d_no_dev   = response{3}.Values.Data(:,1);
    PV_q2_d_no_dev   = response{4}.Values.Data(:,1);

    % with param dev
    param_dev = 15; % within 10%
    rng(1000) %fix random seed
    length_dev = (100+randi([-param_dev,param_dev],1))/100.0;
    vis_dev = (100+randi([-param_dev,param_dev],1))/100.0;
    m_dev = (100+randi([-param_dev,param_dev],1))/100.0;
    set_param('sim_2_twolink_joint_control_real/Controller','LabelModeActiveChoice', controller);
    sim('sim_2_twolink_joint_control_real');
    % get comanded trajectory, time, and responses
    SP_q1_dev     = response{1}.Values.Data(:,2);
    SP_q2_dev     = response{2}.Values.Data(:,2);
    SP_q1_d_dev   = response{3}.Values.Data(:,2);
    SP_q2_d_dev   = response{4}.Values.Data(:,2);
    tau1_dev      = response{5}.Values.Data(:,1);
    tau2_dev      = response{5}.Values.Data(:,2);
    t_dev         = response{1}.Values.Time;
    PV_q1_dev     = response{1}.Values.Data(:,1);
    PV_q2_dev     = response{2}.Values.Data(:,1);
    PV_q1_d_dev   = response{3}.Values.Data(:,1);
    PV_q2_d_dev   = response{4}.Values.Data(:,1);

    % plot and save
    figure('units','normalized','outerposition',[0 0 0.7 0.7])
    plot(t_ideal,SP_q1_ideal,'b-','LineWidth',2); % commanded position
    hold on;
    plot(t_ideal,PV_q1_ideal,plot_symbols{1},'LineWidth',1.2);
    plot(t_no_dev,PV_q1_no_dev,plot_symbols{2},'LineWidth',1.2);
    plot(t_dev,PV_q1_dev,plot_symbols{3},'LineWidth',1.2);
    legend('q_1','ideal','real - no dev','real - with dev','Location',legend_loc);
    hold off;
    grid on;
    print('-r600',['results\q1_',controller,'_',trajectory,'.png'],'-dpng')
    close

    figure('units','normalized','outerposition',[0 0 0.7 0.7])
    plot(t_ideal,SP_q2_ideal,'b-','LineWidth',2); % commanded position
    hold on;
    plot(t_ideal,PV_q2_ideal,plot_symbols{1},'LineWidth',1.2);
    plot(t_no_dev,PV_q2_no_dev,plot_symbols{2},'LineWidth',1.2);
    plot(t_dev,PV_q2_dev,plot_symbols{3},'LineWidth',1.2);
    hold off;
    grid on;
    legend('q_2','ideal','real - no dev','real - with dev','Location',legend_loc);
    print('-r600',['results\q2_',controller,'_',trajectory,'.png'],'-dpng')
    close

    figure('units','normalized','outerposition',[0 0 0.7 0.7])
    plot(t_ideal,SP_q1_d_ideal,'b-','LineWidth',2); % commanded position
    hold on;
    plot(t_ideal,PV_q1_d_ideal,plot_symbols{1},'LineWidth',1.2);
    plot(t_no_dev,PV_q1_d_no_dev,plot_symbols{2},'LineWidth',1.2);
    plot(t_dev,PV_q1_d_dev,plot_symbols{3},'LineWidth',1.2);
    legend('q_1_d','ideal','real - no dev','real - with dev','Location',legend_loc);
    hold off;
    grid on;
    print('-r600',['results\q1d_',controller,'_',trajectory,'.png'],'-dpng')
    close

    figure('units','normalized','outerposition',[0 0 0.7 0.7])
    plot(t_ideal,SP_q2_d_ideal,'b-','LineWidth',2); % commanded position
    hold on;
    plot(t_ideal,PV_q2_d_ideal,plot_symbols{1},'LineWidth',1.2);
    plot(t_no_dev,PV_q2_d_no_dev,plot_symbols{2},'LineWidth',1.2);
    plot(t_dev,PV_q2_d_dev,plot_symbols{3},'LineWidth',1.2);
    legend('q_2_d','ideal','real - no dev','real - with dev','Location',legend_loc);
    hold off;
    grid on;
    print('-r600',['results\q2d_',controller,'_',trajectory,'.png'],'-dpng')
    close

    figure('units','normalized','outerposition',[0 0 0.7 0.7])
    plot(t_no_dev,tau1_no_dev,plot_symbols{5},'LineWidth',1.2);
    hold on;
    plot(t_no_dev,tau2_no_dev,plot_symbols{5},'LineWidth',1.2);
    plot(t_dev,tau1_dev,plot_symbols{3},'LineWidth',1.2);
    plot(t_dev,tau2_dev,plot_symbols{3},'LineWidth',1.2);
    plot(t_ideal,tau1_ideal,'b-','LineWidth',2);
    plot(t_ideal,tau2_ideal,'b-','LineWidth',2);
    legend('u1 - no dev','u2 - no dev','u1 - with dev','u2 - with dev','u1 - ideal','u2 ideal','Location',legend_loc);
    hold off;
    grid on;
    print('-r600',['results\u_',controller,'_',trajectory,'.png'],'-dpng')
    close
end

%% compare controllers
controllers_labels = {
     'PolePlacement_D_control','LQR_D_control','Inverse_dynamics_PID_control','PID_plus_gravity','robust_PID_control'};
controller_display_names = {
      'Pole Placement+D','LQR+D','Gravity+PID','Inverse dynamics+PID','Robust+PID'};
plot_symbols = {'--','-.','--','-.','--',':','-.'};
trajectory = 'sine';
tag = '_PID_lqr';
plot_symbols    = {'r--','m-','k-.','b--','r-','k:','k-'};
if strcmp(trajectory,'sine')
    legend_loc = 'northeast';
else
    legend_loc = 'northwest';
end

% === params ====
Kp1 = 30; 
Ki1 = 20;
Kd1 = 2.5;
N1 = 100;
Kp2 = 7;
Ki2 = 15;
Kd2 = 0.5;
N2 = 100;

sys = ss(A,B,C,D);
p = [-20, -24, -23, -5.8];
Kpp = place(A,B,p);
Kdc = dcgain(syscl);
Kr = 1./Kdc; % scaling terms to eliminate sse
Kr(isinf(Kr))=0;

Q = diag([100,50,1,1]);
R = diag([0.1,0.1]);
Klqr = lqr(A,B,Q,R); 
A_cl = A-B*Klqr;
syscl = ss(A_cl,B,C,D);
Kdc = round(dcgain(syscl),2);
Kr_lqr = 1./Kdc; % scaling terms to eliminate sse
Kr_lqr(isinf(Kr_lqr))=0;

Kp1_grav  = 15;
Kd1_grav  = 2.5;
N1_grav  = 100;

Kp2_grav  = 3.5;
Kd2_grav  = 0.5;
N2_grav  = 100;

Kp1_inv  = 25;
Kd1_inv  = 12;
N1_inv  = 50;

Kp2_inv  = 50;
Kd2_inv  = 15;%48
N2_inv  = 50;

n = 2;
P = eye(2*n)/10^10;
epsilon = 0.1; % 0.01
rho = [1;1];
D = [zeros(n); eye(n)];

Kp1_robust  = 25;
Kd1_robust  = 7;
N1_robust   = 50;

Kp2_robust   = 45;
Kd2_robust   = 35;
N2_robust   = 50;

Kp_r = eye(n).*[Kp1_robust ;Kp2_robust ];
Kd_r = eye(n).*[Kd1_robust ;Kd2_robust ];
telda_H = [zeros(n)   ones(n);
              -Kp_r     -Kd_r;];        
Q =  lyap(telda_H,P);
D_T_Q = D'*Q;

% with param dev
param_dev = 15; % within 10%
rng(1000) %fix random seed
length_dev = (100+randi([-param_dev,param_dev],1))/100.0;
vis_dev = (100+randi([-param_dev,param_dev],1))/100.0;
m_dev = (100+randi([-param_dev,param_dev],1))/100.0;


% change actual plant parameters
% L1x     = 0.32; % m
% L1y     = 0.051; % m
% L1z     = 0.051; % m
% L2x     = 0.19; % m
% L2y     = 0.051; % m
% L2z     = 0.052; % m
% 
% m1   = density * L1x*L1y*L1z; % kg
% m2   = density * L2x*L2y*L2z; % kg
% b1 = 0.1;
% b2 = 0.1;

joint_pos_precision =  0.01; % rad 
joint_pos_variance  = joint_pos_precision^2;
m2=m2*1.25;
m1=m1*1.25;

for i=1:length(controllers_labels)
    controller = controllers_labels{i};
    set_param('sim_2_twolink_joint_control_real/Controller','LabelModeActiveChoice', controller);
    sim('sim_2_twolink_joint_control_real');
    % get comanded trajectory, time, and responses
    SP_q1_ctr{i}     = response{1}.Values.Data(:,2);
    SP_q2_ctr{i}     = response{2}.Values.Data(:,2);
    SP_q1_d_ctr{i}   = response{3}.Values.Data(:,2);
    SP_q2_d_ctr{i}   = response{4}.Values.Data(:,2);
    tau1_ctr{i}      = response{5}.Values.Data(:,1);
    tau2_ctr{i}      = response{5}.Values.Data(:,2);
    t_ctr{i}         = response{1}.Values.Time;
    PV_q1_ctr{i}     = response{1}.Values.Data(:,1);
    PV_q2_ctr{i}     = response{2}.Values.Data(:,1);
    PV_q1_d_ctr{i}   = response{3}.Values.Data(:,1);
    PV_q2_d_ctr{i}   = response{4}.Values.Data(:,1);
end

 % joint pos 
figure('units','normalized','outerposition',[0 0 0.7 0.7])
plot(t_ctr{1},SP_q1_ctr{1},'b-','LineWidth',2); % commanded position
hold on;
 for i=1:length(controllers_labels)
    plot(t_ctr{i},PV_q1_ctr{i},plot_symbols{i},'LineWidth',1.2);
 end
legend('q_1',controller_display_names{:},'Location',legend_loc);
hold off;
grid on;
print('-r600',['results\loaded\q1_compare_',tag,'_',trajectory,'.png'],'-dpng')
close

figure('units','normalized','outerposition',[0 0 0.7 0.7])
plot(t_ctr{1},SP_q2_ctr{1},'b-','LineWidth',2); % commanded position
hold on;
 for i=1:length(controllers_labels)
    plot(t_ctr{i},PV_q2_ctr{i},plot_symbols{i},'LineWidth',1.2);
 end
legend('q_2',controller_display_names{:},'Location',legend_loc);
hold off;
grid on; 
print('-r600',['results\loaded\q2_compare_',tag,'_',trajectory,'.png'],'-dpng')
close

 % joint vel
 figure('units','normalized','outerposition',[0 0 0.7 0.7])
plot(t_ctr{1},SP_q1_d_ctr{1},'b-','LineWidth',2); % commanded position
hold on;
 for i=1:length(controllers_labels)
    plot(t_ctr{i},PV_q1_d_ctr{i},plot_symbols{i},'LineWidth',1.2);
 end
legend('q_1_d',controller_display_names{:},'Location',legend_loc);
hold off;
grid on;
print('-r600',['results\loaded\q1d_compare_',tag,'_',trajectory,'.png'],'-dpng')
close

figure('units','normalized','outerposition',[0 0 0.7 0.7])
plot(t_ctr{1},SP_q2_d_ctr{1},'b-','LineWidth',2);
hold on;
 for i=1:length(controllers_labels)
    plot(t_ctr{i},PV_q2_d_ctr{i},plot_symbols{i},'LineWidth',1.2);
 end
legend('q_2_d',controller_display_names{:},'Location',legend_loc);
hold off;
grid on; 
print('-r600',['results\loaded\q2d_compare_',tag,'_',trajectory,'.png'],'-dpng')
close

%% simulate and store results all controllers and plot response together
% for i=1:n
%     controller_label = controllers_labels{i};
%     set_param('sim_2_twolink_joint_control_real/Controller','LabelModeActiveChoice', controller_label);
%     sim('sim_2_twolink_joint_control_real');
% 
%     % get comanded trajectory, time, and responses
%     SP_q1     = response{1}.Values.Data(:,2);
%     SP_q2     = response{2}.Values.Data(:,2);
%     SP_q1_d   = response{3}.Values.Data(:,2);
%     SP_q2_d   = response{4}.Values.Data(:,2);
%     tau1      = response{5}.Values.Data(:,1);
%     tau2      = response{5}.Values.Data(:,2);
%     effort1   = response{6}.Values.Data(:,1);
%     effort2   = response{6}.Values.Data(:,2);
% 
%     t         = response{1}.Values.Time;
% 
%     PV_q1     = response{1}.Values.Data(:,1);
%     PV_q2     = response{2}.Values.Data(:,1);
%     PV_q1_d   = response{3}.Values.Data(:,1);
%     PV_q2_d   = response{4}.Values.Data(:,1);
% 
% 
%     % store for comparison later
%     control_response{i} = [
%         t,...
%         SP_q1,PV_q1,      SP_q2,PV_q2,...
%         SP_q1_d,PV_q1_d,  SP_q2_d,PV_q2_d,...
%         tau1,tau2
%         ];
%  
% end

% %plotting
% 
% % q1
% figure('units','normalized','outerposition',[0 0 0.7 0.7])
% plot(control_response{1}(:,1),control_response{1}(:,2),'b-','LineWidth',2); % commanded position
% hold on;
% for i=1:n
%     plot(control_response{i}(:,1),control_response{i}(:,3),plot_symbols{i},'LineWidth',1);
% end
% legend('q_1',controller_display_names{:},'Location','northwest');
% hold off;
% % saveas(gcf,'results\q1.png')
% print(['-r600','results\q1_',trajectory,'.png'],'-dpng')
% close
% 
% % q2
% figure('units','normalized','outerposition',[0 0 0.7 0.7])
% plot(control_response{1}(:,1),control_response{1}(:,4),'b-','LineWidth',2); % commanded 
% hold on;
% for i=1:n
%     plot(control_response{i}(:,1),control_response{i}(:,5),plot_symbols{i},'LineWidth',1);
% end
% legend('q_2',controller_display_names{:},'Location','northwest');
% hold off;
% print('-r600',['results\q2_',trajectory,'.png'],'-dpng')
% close
% 
% % q1_d
% figure('units','normalized','outerposition',[0 0 0.7 0.7])
% plot(control_response{1}(:,1),control_response{1}(:,6),'b-','LineWidth',2); % commanded
% hold on;
% for i=1:n
%     plot(control_response{i}(:,1),control_response{i}(:,7),plot_symbols{i},'LineWidth',1); 
% end
% legend('q_1_d',controller_display_names{:});
% hold off;
% print('-r600',['results\q1_d_',trajectory,'.png'],'-dpng')
% close
% 
% % q1_d
% figure('units','normalized','outerposition',[0 0 0.7 0.7])
% plot(control_response{1}(:,1),control_response{1}(:,8),'b-','LineWidth',2); % commanded
% hold on;
% for i=1:n
%     plot(control_response{i}(:,1),control_response{i}(:,9),plot_symbols{i},'LineWidth',1);
% end
% legend('q_2_d',controller_display_names{:});
% hold off;
% print('-r600',['results\q2_d_',trajectory,'.png'],'-dpng')
% close
% 
% 
