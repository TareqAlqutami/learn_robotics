syms a1 a2 g real
syms m1 m2 real
syms Izz1 Iyy1 Ixx1 Izz2 Iyy2 Ixx2 real
syms b1 b2 real

twolink = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'm', m1, 'r', [a1/2 0 0], 'I', [Ixx1 Iyy1 Izz1], 'B', b1, 'G', 1, 'Jm', 0, 'standard')
    Revolute('d', 0, 'a', a2, 'alpha', 0, 'm', m2, 'r', [a2/2 0 0], 'I', [Ixx2 Iyy2 Izz2], 'B', b2, 'G', 1, 'Jm', 0, 'standard')
    ], ...
    'name', 'two link', ...
    'comment', 'two link planar on x-y');
twolink = twolink.sym();
twolink.gravity = [0; g; 0];
syms q1 q2 q1d q2d q1dd q2dd real
syms Q1 Q2 real

qz = [0 0]; % joint velocity


%% get the acceleration closed form equation
acc_eq = twolink.accel([q1 q2],[q1d q2d], [Q1 Q2])

%% write into function
Vars = {'Ixx1', 'Iyy1', 'Izz1', 'Ixx2', 'Iyy2','Izz2',  ...
        'a1', 'a2', 'g', 'm1', 'm2', 'b1', 'b2', ...
        'q1', 'q2', 'q1d' 'q2d', ...
                    'Q1', 'Q2'}
matlabFunction(acc_eq(1),acc_eq(2), 'File', 'gen_mdl_eq.m', ...
                     'Optimize', false, ...
                     'Vars',     Vars, ...
                     'Outputs', {'theta1_dd','theta2_dd'}   ); 