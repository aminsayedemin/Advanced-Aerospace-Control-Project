clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% System definition
% x = [v p \phi]^T
% y = [p \phi]^T

Ts = 0.004; % Sampling interval
g = 9.81;

Y_v = ureal('yv', -0.264,'Perc', 4.837);
Y_p = 0;
L_v = ureal('lv', -7.349,'Perc', 4.927);
L_p = 0;
Y_d = ureal('yd', 9.568,'Perc', 4.647);
L_d = ureal('ld', 1079.339,'Perc', 2.762);

A = [Y_v    Y_p     g;
    L_v     L_p     0;
    0       1       0];
N = size(A);
Anom = A.NominalValue; % The matrix A with its nominal value

B = [Y_d;
    L_d;
    0];
Bnom = B.NominalValue; % The vector B with its nominal value

C = [0      1       0;
    0       0       1];

D = [0;
    0];

%% Nominal plant
ld = ss(Anom, Bnom, C, D); % Nominal Plant
ld_dis = c2d(ld, Ts, 'foh');
% ld_dis = ss(Anom, Bnom, C, D, Ts); % Nominal Plant

G = tf(ld_dis);
G.u = {'delta_lat'};
G.y = {'p', 'phi'};

%% Uncertain plant
ld_un_dis = ss(A, B, C, D, Ts);
G_un_dis = tf(ld_un_dis);

%% Controller: R_p
b = realp('b', 1);
c1 = realp('c1',1);
c2 = realp('c2',1);
d1 = realp('d1',1);
d2 = realp('d2',1);

Ap = [1 0; 0 0];
Bp = [b -b; 0 0.5];
Cp = [c1 c2];
Dp = [d1 d2];

Rp = ss(Ap, Bp, Cp, Dp, Ts);

Rp.u = {'p_0', 'p'};
Rp.y = {'delta_lat'};

%% Controller: R_phi
d3 = realp('d3', 1);
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

% Assembly
% AP = AnalysisPoint('delta_lat');
% AP.u = 'delta_lat';
% AP.y = 'delta_lat';

Sum = sumblk('e_phi = phi_0 - phi');

T0 = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'}, {'delta_lat'});

% Desired dynamics
csi = 0.9;
om = 10;

s = zpk('s');
F2 = om^2/(s^2 + 2*csi*om*s + om^2);
refsys = F2;
F2 = c2d(F2, Ts, 'foh');
refsys = F2;

% hard = TuningGoal.Transient('phi_0', 'phi', refsys, 'step');
% soft = TuningGoal.Gain('phi_0', 'delta_lat', 1);

h = TuningGoal.Transient('phi_0', 'phi', refsys, 'step');
s = TuningGoal.Gain('phi_0', 'delta_lat', 1);

options = systuneOptions;
options.RandomStart = 10;
[K, FSOFT, GHARD, INFO] = systune(T0, h, s, options); % Soft: Objective, Hard: Constraint
FSOFT, GHARD
subplot(211), step(K(1));
subplot(212), step(K(2));

%% Redefinition
% Controller: R_p
b = K.Blocks.b.Value;
c1 = K.Blocks.c1.Value;
c2 = K.Blocks.c2.Value;
d1 = K.Blocks.d1.Value;
d2 = K.Blocks.d2.Value;

Ap =    [1  0;
        0   0];
    
Bp =    [b  -b;
        0   0.5];
    
Cp =    [c1 c2];

Dp =    [d1 d2];

Rp = ss(Ap, Bp, Cp, Dp, Ts);
Rp.u = {'p_0', 'p'};
Rp.y = {'delta_lat'};

% Controller: R_phi
d3 = K.Blocks.d3.Value;
Dphi = d3;

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

% Reassembly
Loop = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});
% figure
% pzmap(Loop);
figure;
step(Loop);

%% Plots
% Step response and comparison
Tf = 4; % Final time of the Step plot
figure
y = step(Loop(2), Tf); % Step of our system
plot(0:Ts:Tf, y);
hold on
f2 = step(F2, Tf);
plot(0:Ts:Tf, f2);
legend('Actual response', 'Desired response', 'Location', 'southeast');
% xlabel('Time [s]', 'Interpreter', 'Latex');
% ylabel ('Amplitude', 'Interpreter', 'Latex');
% axis ([0 Tf -0.2 1.2]);
grid on

% Sensitivity
S = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'});
S_des = 1 - F2;
figure
% bode(S, W1inv, S_des);
bode(S, S_des)
grid on;
legend('S', 'S_{des}', 'Interpreter', 'Latex')

% % Complementary Sensitivity
F = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'phi'});
% figure
% bode(F, W3inv);
% grid on;
% legend('Loop', '1/W3');

% Control Sensitivity
Q = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'delta_lat'});
figure
% bode(Q, W2inv);
% figure
opt = stepDataOptions('StepAmplitude', 10);
step(Q, opt)
grid on;
legend('Step Q')

%% END OF CODE