clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% System definition
% x = [v p \phi]^T
% y = [p \phi]^T

Ts = 0.004; % Sampling interval

Y_v = ureal('yv', -0.264,'Perc', 4.837);
Y_p = 0;
L_v = ureal('lv', -7.349,'Perc', 4.927);
L_p = 0;
Y_d = ureal('yd', 9.568,'Perc', 4.647);
L_d = ureal('ld', 1079.339,'Perc', 2.762);

g = 9.81;

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
% P_ld = tf(ld); % Nominal Plant continuous time t.f.

ld_dis = c2d(ld, Ts, 'foh');
G = tf(ld_dis);
G.u = {'delta_lat'};
G.y = {'phi', 'p'};

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
Rp.u = {'p_0','p'};
Rp.y = {'delta_lat'};

%% Controller: R_phi
d3 = realp('d3', 1);
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

%% Weights
csi = 0.9;
max = exp(-csi*pi/sqrt(1-csi^2)) + 1;

M = max; % Peak of Sensitivity function
omb = 10; % [rad/s] % Lower bound on crossover frequency of S
ampl = 1e-3; % Max value of S at steady state

% Plant
W1inv = makeweight(2, 0.2, 0.9);
W1inv = c2d(W1inv, Ts, 'foh');
W1 = 1/W1inv;
W1 = makeweight(10,[1 0.1],0.01, Ts);
W1inv = 1/W1;
W1.u = {'e_phi'};
W1.y = {'z_1'};

W2inv = tf(200,1);
W2inv = c2d(W2inv, Ts, 'foh');
W2 = 1/W2inv; % Weight on the control sensitivity
W2 = makeweight(0.1,[32 0.32], 1, Ts);
W2inv = 1/W2;
W2.u = {'delta_lat'};
W2.y = {'z_2'};

% Weight of the complementary sensitivity (0 if nominal design)
csi = 0.9;
om = 10;
s = zpk('s');
F2 = 10000 * om^2/(s^2 + 2*csi*om*s + om^2);
F2 = c2d(F2, Ts, 'foh');
W3inv = F2;
W3 = 1/W3inv;
W3.u = {'phi'};
W3.y = {'z_3'};

% Closed-loop results
% figure(1), bode(G, K, G*K), grid, legend('G','K','G*K');
% figure(2), bode(1/(1 + G(1)*K(1)), W1inv),grid, legend('S','1/W1');
% figure(3), bode(K(1)/(1 + G(1)*K(1)), W2inv),grid, legend('Q','1/W2');

%% Assembly
Sum = sumblk('e_phi = phi_0 - phi');
Loop = connect(G, Rp, Rphi, Sum, W3, {'phi_0'}, {'z_3', 'p'}); %, W1, W2  'z_1', 'z_2',
K = hinfstruct(Loop);

[K.Blocks.b.Value; K.Blocks.c1.Value; K.Blocks.c2.Value;
    K.Blocks.d1.Value; K.Blocks.d2.Value; K.Blocks.d3.Value]

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
Rp.u = {'p_0','p'};
Rp.y = {'delta_lat'};

% Controller: R_phi
d3 = K.Blocks.d3.Value;
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

L = connect(G, Rp, Rphi, {'e_phi'}, {'phi'});
% L_fed = feedback(L,1);
% step(L_fed)
S = 1/(1+L);
F = L/(1+L);

% ??
% Q = L/G(1); % ??
% ??

% figure(1), bode(G, K, G*K), grid, legend('G','K','G*K');
% figure(2), bode(S, W1inv),grid, legend('S','1/W1');
% figure(3), bode(Q, W2inv),grid, legend('Q','1/W2');
figure(4), bode(F, W3inv),grid, legend('F','1/W3');