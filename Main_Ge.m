clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici
%% Section: Main

%% Initial Data
Ts = 0.004; % Sampling interval
g = 9.81;

%% Lateral Dynamics Block
% x = [v p \phi]^T; y = [p \phi]^T
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
ld = ss(Anom, Bnom, C, D);

% figure
% bode(ld)
% title('Continuous Nominal Plant')
% grid on

ld_dis = c2d(ld, Ts, 'foh');

% figure
% bode(ld_dis)
% title('Discrete Nominal Plant')
% grid on

G = tf(ld_dis);
G.u = {'delta_lat'};
G.y = {'p', 'phi'};

%% Uncertain plant
ld_un = ss(A, B, C, D);

% figure
% bode(ld_un)
% title('Continuous Uncertain Plant')
% grid on

% ld_un_dis = c2d(ld, Ts, 'foh') %da aggiustare
% 
% figure
% bode(ld_un_dis)
% grid on

% G_un = tf(ld_un_dis);
% G_un.u = {'delta_lat'};
% G_un.y = {'p', 'phi'};

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

%% Weights for "hinfstruct"

% Weight on the sensitivity function
csi_w1 = 0.9
om_w1 = 10; %crossover frequency
A_w1 = 1e-4; %steady-state error
M_w1 = 1.5;
s = zpk('s');
F_w1 = om_w1^2/(s^2 + 2*csi_w1*om_w1*s + om_w1^2);
W1inv = 1 - F_w1;
% W1inv = s/om_w1 * 1/(s/M_w1/om_w1 + 1);
% W1inv = (s + om_w1 * A_w1)/(s/M_w1 + om_w1);
W1inv = c2d(W1inv, Ts, 'foh');
% % W1inv = makeweight(0, 50, 1.5, Ts);

% figure
% bode(W1inv)
% title ('1/W1')
% grid on

W1 = 1/W1inv;
W1.u = {'e_phi'};
W1.y = {'z_1'};

% % Weight on the control sensitivity
W2inv = tf([1],[1/30 1]);
W2inv = c2d(W2inv, Ts, 'foh');

% figure
% bode(W2inv)
% title ('1/W2')
% grid on

W2 = 1/W2inv;
W2.u = {'delta_lat'};
W2.y = {'z_2'};

% % Weight on the complementary sensitivity
% W3 = tf(0);
% W3.u = {'phi'};
% W3.y = {'z_3'};

%% Assembly
Sum = sumblk('e_phi = phi_0 - phi');

P = connect(Rp, W1, Sum, Rphi, G, {'phi_0'}, {'z_1'});
% P = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'}, {'delta_lat'});

%% Tuning
% Desired dynamics
csi = 0.9;
om = 10;
s = zpk('s');
F0 = om^2/(s^2 + 2*csi*om*s + om^2);
F0 = c2d(F0, Ts, 'foh');

% figure
% bode(F0)
% title ('Desired F')
% grid on

S0 = 1 - F0;

% figure
% bode(S0)
% title ('Desired S')
% grid on

% 
% soft = TuningGoal.Transient('phi_0', 'phi', F0, 'step');
% hard = TuningGoal.Gain('phi_0', 'delta_lat', 1);
% 
% options = systuneOptions;
% options.RandomStart = 20;
% [K, GSOFT, GHARD, INFO] = systune(P, soft, hard, options); % Soft: Objective, Hard: Constraint
% GSOFT, GHARD

opt = hinfstructOptions('Display', 'final', 'RandomStart', 30);
[K, gamma, info] = hinfstruct(P, opt);

[K.Blocks.b.Value; K.Blocks.c1.Value; K.Blocks.c2.Value;
    K.Blocks.d1.Value; K.Blocks.d2.Value; K.Blocks.d3.Value]

%% Tuned system redefinition
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

% Assembly
Loop = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});
F = tf(Loop(2))% Complementary Sensitivity
S = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'}); % Sensitivity
Q = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'delta_lat'});% Control sensitivity

%% Plots
% inutili = 0; % Show(1)/Hide(0) some graphs
Tf = 3; % Final time of the step plots

% Step response wrt phi and p
figure
% hold on
step(Loop(1), Tf)

figure
step(Loop(2), Tf)
hold on
step(F0, Tf)
legend ('Phi', 'Desired Phi')

% subplot(2,1,1), plot(t_L1, squeeze(s_L1), 'LineWidth',2);
% xlabel('Time [s]', 'Interpreter', 'Latex');
% ylabel ('Amplitude', 'Interpreter', 'Latex');
% title('Output: p', 'Interpreter', 'Latex');
% axis ([0 Tf -0.2 4.5]);
% grid on
% 
% subplot(2,1,2), plot(t_L2, squeeze(s_L2), 'LineWidth',2);
% xlabel('Time [s]', 'Interpreter', 'Latex');
% ylabel ('Amplitude', 'Interpreter', 'Latex');
% title('Output: phi', 'Interpreter', 'Latex');
% axis ([0 Tf -0.2 1.2]);
% grid on
% 
% % Check of the requirement
% figure;
% hold on
% [y, t_L] = step(F, Tf); % Step of the system, with output phi
% [z, t_F] = step(F0, Tf); % Step response of the desired dynamics
% h1 = plot(t_L, squeeze(y), 'LineWidth', 2);
% h2 = plot(t_F, squeeze(z), '--', 'LineWidth', 2);
% legend([h1(1), h2(1)], 'Actual response', 'Desired response', 'Location', 'southeast', 'Interpreter', 'Latex');
% xlabel('Time [s]', 'Interpreter', 'Latex');
% ylabel ('Amplitude', 'Interpreter', 'Latex');
% axis ([0 Tf -0.2 1.2]);
% grid on
% 
% if inutili == 1
%     
% % Sensitivity
% figure;
% subplot(2,1,1), bode(S, S0);
% grid on;
% legend('S', 'Desired S', 'Interpreter', 'Latex');
% title ('Sensitivity Function', 'Interpreter', 'Latex');
% 
% subplot(2,1,2), margin(S);
% grid on;
% 
% % Complementary Sensitivity
% figure;
% subplot(2,1,1), bode(F, F0);
% grid on;
% legend('F', 'Desired F', 'Interpreter', 'Latex');
% title ('Complementary sensitivity Function', 'Interpreter', 'Latex');
% 
% subplot(2,1,2), margin(F);
% grid on;
% 
% end
% 
% % Control Sensitivity
figure;
opt = stepDataOptions('StepAmplitude', 10*pi/180);
[s_Q, t_Q] = step(Q, 50, opt);
plot(t_Q, squeeze(s_Q), 'LineWidth',2)
xlabel('Time', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
grid on;
legend('Output: $\delta_{lat}$', 'Interpreter', 'Latex');
title('Step of amplitude 10 response', 'Interpreter', 'Latex');
% 
% figure;
% bode(F, F0);
% grid on;
% title ('Control sensitivity Function', 'Interpreter', 'Latex');

%% Plots for "hinfstruct"
% % Sensitivity
figure
bode(S, W1inv, S0)
legend('S', '1/W1', 'Desired S')
grid on

% 
% % Complementary Sensitivity
% figure
% bode(F, W3inv, F0);
% legend('Loop', '1/W3', 'Desired F');
% grid on;
% 
% figure;
% bode(Loop(2) * W3)
% title('Multiplication between F and W3');
% grid on;
% 
% % Control Sensitivity
figure;
bode(Q, W2inv);
legend('Q', '1/W2');
grid on;

%% END OF CODE