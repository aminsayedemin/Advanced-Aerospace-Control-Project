clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% System definition
% x = [v p \phi]^T
% y = [p \phi]^T

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
P_ld = tf(ld); % Nominal Plant continuous time t.f.
G0 = [P_ld(1); P_ld(2)];
gain = [0; 214.6/72.09]; % 0 and 214.6/72.09

%% Uncertain plant
Ts = 0.004; % Sampling interval

ld_un = ss(A, B, C, D);
% P_ld_un = tf(ld_un); % Uncertain Plant continuous time t.f.
% G1 = [P_ld_un(1); P_ld_un(2)];

% Trand = usample(ld_un, 10); % Random samples of uncertain model T

% time = 0 : Ts : 5;

% Frequency response
% figure
% bodemag(Trand);  % Bode Plots with uncertainty
% 
% figure
% bode(ld_un);
% 
% figure
% step(ld_un, time);

% figure;
% hold on
% sigma(ld_un(1));
% sigma(ld_un(2));
% legend('t.f. with output $\phi$', 't.f. with output $p$', 'interpreter', 'latex');
% hold off

% Poles & Zeros
% figure;
% hold on
% ax = gca;
% ax.XAxisLocation = 'origin';
% ax.YAxisLocation = 'origin';
% iopzplot(ld_un(1));
% iopzplot(ld_un(2));
% hold off

% figure;
% hold on
% ax = gca;
% ax.XAxisLocation = 'origin';
% ax.YAxisLocation = 'origin';
% p1 = plot(real(poles), imag(poles), 'x', 'linewidth', 2);
% p2 = plot([zeros, 0, 0], [0, 0, 0], 'o', 'linewidth', 2);
% % (Two zeros at the origin for plotting purposes)
% 
% h = [p1, p2];
% title ('All nominal Poles and Zeros', 'Interpreter', 'Latex');
% xlabel('Re, rad/s', 'Interpreter', 'Latex');
% ylabel('Im, rad/s', 'Interpreter', 'Latex');
% legend(h, 'Poles', 'Zeros', 'Interpreter', 'Latex');
% axis([-5 3 -4 4]);
% grid on
% grid minor
% hold off

%% Controller: R_p
b = 1; c1 = 1; c2 = 1; d1 = 1; d2 = 1;
Ap = [1 0; 0 0];
Bp = [b -b; 0 0.5];
Cp = [c1 c2];
Dp = [d1 d2];

% Mixed Sensitivity Synthesis
ld_dis = c2d(ld, Ts, 'foh');
G = tf(ld_dis);

csi = 0.9;
max = exp(-csi*pi/sqrt(1-csi^2)) + 1;

M = max; % Peak of Sensitivity function
omb = 10; % [rad/s] % Lower bound on crossover frequency of S
ampl = 1e-3; % Max value of S at steady state

% Plant
W1inv = makeweight(2, 0.2, 0.9);
W1inv = c2d(W1inv, Ts, 'foh');
W1 = 1/W1inv;

W2inv = tf(200,1);
W2inv = c2d(W2inv, Ts, 'foh');
W2 = 1/W2inv; % Weight on the control sensitivity

W3 = []; % Weight of the complementary sensitivity (0 if nominal design)

P = augw(G, W1, W2, W3); % SIMO vector to be optimized

% Synthesize the controller: Structured synthesis
K0 = tunablePID('R_p', 'PI', Ts);
K = hinfstruct(P, K0);

% Closed-loop results
figure(1), bode(G, K, G*K), grid, legend('G','K','G*K');
figure(2), bode(1/(1 + G(1)*K(1)), W1inv),grid, legend('S','1/W1');
figure(3), bode(K(1)/(1 + G(1)*K(1)), W2inv),grid, legend('Q','1/W2');

%% Controller: R_phi
d3 = tunableGain('d3', 1, 1);
d3 = 1;
D_phi = ss(d3);

%% Trial for R_p
b = realp('b', 0);
c1 = realp('c1',0);
c2 = realp('c2',0);
d1 = realp('d1',0);
d2 = realp('d2',0);

Ap = [1 0; 0 0];
Bp = [b -b; 0 0.5];
Cp = [c1 c2];
Dp = [d1 d2];

rp_dis = ss(Ap, Bp, Cp, Dp, Ts); % Model of class "genss"
Rp = hinfstruct(P, rp_dis);

%% 2nd Order Response
% csi = 0.9;
% om = 10;
% 
% s = zpk('s');
% F = om^2/(s^2 + 2*csi*om*s + om^2);
% step(F)