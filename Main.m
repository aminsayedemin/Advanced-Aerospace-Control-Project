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

%% Uncertain Plant
Ts = 0.004; % Sampling interval

ld_un = ss(A, B, C, D);
P_ld_un = tf(ld_un); % Uncertain Plant continuous time t.f.
G1 = [P_ld_un(1); P_ld_un(2)];

Trand = usample(P_ld_un, 10); % Random samples of uncertain model T

time = 0:Ts:5;

% Frequency response
% figure
% bodemag(Trand);  % Bode Plots with uncertainty

figure
bode(P_ld_un);

figure
step(P_ld_un, time);

figure;
hold on
sigma(P_ld_un(1));
sigma(P_ld_un(2));
legend('t.f. with output $\phi$', 't.f. with output $p$', 'interpreter', 'latex');
hold off

% Poles & Zeros
poles = pole(P_ld_un);
zeros = tzero(P_ld_un);

figure;
hold on
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
p1 = plot(real(poles), imag(poles), 'x', 'linewidth', 2);
p2 = plot([zeros, 0, 0], [0, 0, 0], 'o', 'linewidth', 2);
% (Two zeros at the origin for plotting purposes)

h = [p1, p2];
title ('All Poles and Zeros', 'Interpreter', 'Latex');
xlabel('Re, rad/s', 'Interpreter', 'Latex');
ylabel('Im, rad/s', 'Interpreter', 'Latex');
legend(h, 'Poles', 'Zeros', 'Interpreter', 'Latex');
axis([-5 3 -4 4]);
grid on
grid minor
hold off

%% Controller: R_p
b = 1; c1 = 1; c2 = 1; d1 = 1; d2 = 1;
Ap = [1 0; 0 0];
Bp = [b -b; 0 0.5];
Cp = [c1 c2];
Dp = [d1 d2];

rp = ss(Ap, Bp, Cp, Dp); % Nominal Plant
rp_dis = c2d(rp, Ts, 'foh'); % NP in discrete time
P_p = tf(rp_dis);

%% Controller: R_phi
d3 = tunableGain('d3', 1, 1);
% d3 = 1;
D_phi = ss(d3);

% %% LQR Problem
% % Performance: \dot{p}
% Cz = A(2,:);
% Dzu = B(2);
% 
% Wzz = 1;
% Wuu = 1; % Rho
% 
% % Riccati equation
% Q = Cz' * Wzz * Cz;
% S = Cz' * Wzz * Dzu;
% R = Dzu' * Wzz * Dzu + Wuu;
% invR = inv(R);
% 
% AA = A - B * invR * S';
% RR = B * (invR * B');
% QQ = Q - S *(invR * S');
% P = are(AA, RR, QQ);
% 
% G0 = invR *(S' + B' * P); % Gain matrix
% Ac = A - B * G0;
% 
% %% Transfer functions
% Ts = 0.004; % Sampling interval (given)
% 
% % sC = ss(Ac, B, C, D, Ts); % Discrete time
% % sNC = ss(A, B, C, D, Ts);
% sC = ss(Ac, B, C, D); % Continous time
% sNC = ss(A, B, C, D);
% eigAc = eig(Ac);
% 
% % Response plot
% t = 0:Ts:1; % Time
% t = t';
% n = sqrt(0.01) * randn(size(t)); % White noise of W = 0.01
% 
% yC = lsim(sC, n, t);
% yNC = lsim(sNC, n, t);
% 
% 
% figure;
% 
% subplot(1,2,1);
% plot(t, yC(:,1), t, yNC(:,1));
% grid on
% xlabel('t [s]', 'Interpreter', 'Latex');
% ylabel('p [rad/s]', 'Interpreter', 'Latex');
% legend('Controlled', 'Not Controlled', 'Interpreter', 'Latex');

% subplot(1,2,2);
% plot(t, yC(:,2), t, yNC(:,2));
% grid on
% xlabel('t [s]', 'Interpreter', 'Latex');
% ylabel('$\phi$ [rad]', 'Interpreter', 'Latex');
% legend('Controlled', 'Not Controlled', 'Interpreter', 'Latex');