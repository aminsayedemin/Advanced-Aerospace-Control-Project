clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% System definition
% x = [v p \phi]^T
% y = [p \phi]^T

g = 9.81;

Y_v = -0.264;                   % Stability derivatives
Y_p = 0;
L_v = -7.349;
L_p = 0;

Y_d = 9.568;                    % Control derivatives
L_d = 1079.339;
%--------------------------------------------------------------------------
A = [Y_v    Y_p     g;
    L_v     L_p     0;
    0       1       0];
N = size(A);
eigA = eig(A);

B = [Y_d;
    L_d;
    0];

C = [0      1       0;
    0       0       1];

D = [0;
    0];

%% LQR Problem
% Performance: \dot{p}
Cz = A(1,:);
Dzu = B(1);

Wzz = 1;
Wuu = 1; % Rho

% Riccati equation
Q = Cz' * Wzz * Cz;
S = Cz' * Wzz * Dzu;
R = Dzu' * Wzz * Dzu + Wuu;
invR = inv(R);

AA = A - B * invR * S';
RR = B * (invR * B');
QQ = Q - S *(invR * S');
P = are(AA, RR, QQ);

G0 = invR *(S' + B' * P); % Gain matrix
Ac = A - B * G0;

%% Transfer functions
Ts = 0.004; % Sampling interval (given)

% sC = ss(Ac, B, C, D, Ts); % Discrete time
% sNC = ss(A, B, C, D, Ts);
sC = ss(Ac, B, C, D); % Continous time
sNC = ss(A, B, C, D);
eigAc = eig(Ac);

%% Plots
% Eigenvalues Plot

figure;

hold on
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
p1 = plot(real(eigAc), imag(eigAc), 'x', 'linewidth', 2); % Controlled
p2 = plot(real(eigA), imag(eigA), 'o', 'linewidth', 2); % Not controlled

h = [p1, p2];
title ('Eigenvalues Analysis', 'Interpreter', 'Latex');
xlabel('Re, rad/s', 'Interpreter', 'Latex');
ylabel('Im, rad/s', 'Interpreter', 'Latex');
legend(h, 'Controlled', 'Not Controlled', 'Interpreter', 'Latex');
grid on
grid minor
hold off

% Response plot
t = 0:Ts:1; % Time
t = t';
n = sqrt(0.01) * randn(size(t)); % White noise of W = 0.01

yC = lsim(sC, n, t);
yNC = lsim(sNC, n, t);


figure;

subplot(1,2,1);
plot(t, yC(:,1), t, yNC(:,1));
grid on
xlabel('t [s]', 'Interpreter', 'Latex');
ylabel('p [rad/s]', 'Interpreter', 'Latex');
legend('Controlled', 'Not Controlled', 'Interpreter', 'Latex');

subplot(1,2,2);
plot(t, yC(:,2), t, yNC(:,2));
grid on
xlabel('t [s]', 'Interpreter', 'Latex');
ylabel('$\phi$ [rad]', 'Interpreter', 'Latex');
legend('Controlled', 'Not Controlled', 'Interpreter', 'Latex');