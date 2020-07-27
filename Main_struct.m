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
Anom = A.NominalValue; % The matrix with its nominal value

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

G = tf(ld_dis);
G.u = {'delta_lat'};
G.y = {'p', 'phi'};

%% Uncertain plant
ld_un_dis = ss(A, B, C, D, Ts);
G_un_dis = tf(ld_un_dis);

%% Controller: R_p
b = realp('b', 1);
c1 = realp('c1', 1);
c2 = realp('c2', 1);
d1 = realp('d1', 1);
d2 = realp('d2', 1);

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

%% Weights
csi = 0.9;
om = 10;

F2 = tf([om^2], [1, 2*csi*om, om^2]);
F2 = c2d(F2, Ts, 'foh');
S_des = 1 - F2;

A = 1e-3; omb = 10; M = 2;
s = zpk('s');
S_des = (s + A * omb)/(s/M + omb);
S_des = c2d(S_des, Ts, 'foh');

W1inv = S_des;
W1 = 1/S_des;
W1.u = {'e_phi'};
W1.y = {'z_1'};

% Plot
figure;
margin(W1inv);
grid

W2inv = tf(200, 1, Ts);
W2 = 1/W2inv; % Weight on the control sensitivity

W2 = tf(0);

W2.u = {'delta_lat'};
W2.y = {'z_2'};

% Weight of the complementary sensitivity (0 if nominal design)
W3inv = F2;
W3 = 1/W3inv;

W3 = tf(0);

W3.u = {'phi'};
W3.y = {'z_3'};

%% Assembly
Sum = sumblk('e_phi = phi_0 - phi');

OPT = connectOptions('Simplify', false);
% Loop = connect(G, Rp, Rphi, W1, W2, W3, Sum, {'phi_0'}, {'p', 'phi', 'z_1', 'z_2', 'z_3'}, OPT);
CL0 = connect(G, Rp, Rphi, W1, Sum, {'phi_0'}, {'p', 'phi', 'z_1'}, OPT);
% CL0 = connect(G, Rp, Rphi, W1, W2, Sum, {'phi_0'}, {'p', 'phi', 'z_1', 'z_2'}, OPT);

opt = hinfstructOptions('Display', 'final', 'RandomStart', 10);
[K, GAM, INFO] = hinfstruct(CL0, opt);

% [K.Blocks.b.Value; K.Blocks.c1.Value; K.Blocks.c2.Value;
%     K.Blocks.d1.Value; K.Blocks.d2.Value; K.Blocks.d3.Value]

%% Redefinition
% Controller: R_p
b = K.Blocks.b.Value;
c1 = K.Blocks.c1.Value;
c2 = K.Blocks.c2.Value;
d1 = K.Blocks.d1.Value;
d2 = K.Blocks.d2.Value;

Ap = [1 0; 0 0];
Bp = [b -b; 0 0.5];
Cp = [c1 c2];
Dp = [d1 d2];

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
L = connect(G, Rp, Rphi, {'e_phi'}, {'phi'}, OPT);
Loop = connect(G, Rp, Rphi, Sum, 'phi_0', {'p', 'phi'}, OPT);

S = 1/(1+L);
F = L/(1+L);
Q = Rphi/(1+L);

%% Plots
% figure, bode(G, K, G*K), grid, legend('G','K','G*K');
figure, bode(S, W1inv), grid, legend('S', '1/W1');

% Requirement
figure;
subplot(211)
step(Loop, 5);
grid minor

subplot(212);
step(F2, 5);
grid minor
legend('Desired');

% figure, bode(Q, W2inv),grid, legend('Q','1/W2');
% figure, bode(F, W3inv),grid, legend('F','1/W3');

%% END OF CODE