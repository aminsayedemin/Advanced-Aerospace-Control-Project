clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% System definition

Ts = 0.004; % Sampling interval
g = 9.81;

Y_v = ureal('yv', -0.264, 'Perc', 3 * 4.837);
Y_p = 0;
L_v = ureal('lv', -7.349,'Perc', 3 * 4.927);
L_p = 0;
Y_d = ureal('yd', 9.568,'Perc', 3 * 4.647);
L_d = ureal('ld', 1079.339,'Perc', 3 * 2.762);

A = [Y_v    Y_p     g;
    L_v     L_p     0;
    0       1       0];
A_nom = A.NominalValue; % The matrix with its nominal value

B = [Y_d;
    L_d;
    0];
B_nom = B.NominalValue; % The vector B with its nominal value

C = [0      1       0;
    0       0       1];

D = [0;
    0];

%% Nominal Plant

G_nom = ss(A_nom, B_nom, C, D);
G_nom = c2d(G_nom, Ts, 'foh');
G_nom.InputName = {'delta_lat'};
G_nom.OutputName = {'p'; 'phi'};

%% Controller: R_p

b = realp('b', 1);
c1 = realp('c1', 1);
c2 = realp('c2', 1);
d1 = realp('d1', 1);
d2 = realp('d2', 1);

Ap =    [1  0;
        0   0];
    
Bp =    [b  b;
        0   0.5];
    
Cp = [c1    c2];

Dp = [d1    d2];

Rp = ss(Ap, Bp, Cp, Dp, Ts);
Rp.InputName = {'p_0'; 'p'};
Rp.OutputName = {'delta_lat'};

%% Controller: R_phi

d3 = realp('d3', 1);
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.InputName = {'e_phi'};
Rphi.OutputName = {'p_0'};

%% Weights

%Desired Dynamics
csi = 0.9;
om = 10;

F2 = tf([om^2], [1, 2*csi*om, om^2]);
F2 = c2d(F2, Ts, 'foh');
S_des = 1 - F2;

% Weight on the sensitivity function
csi = 1;
om = 5;

F_weight = tf([om^2], [1, 2*csi*om, om^2]);
F_weight = c2d(F_weight, Ts, 'foh');
% W1inv = 1 - F_weight;

W1inv = makeweight(0.001, 5, 1.5, Ts);
figure
bode(W1inv, S_des);
W1 = 1/W1inv;
W1.u = {'e_phi'};
W1.y = {'z_1'};

% Weight on the control sensitivity
% W2 = makeweight(0, [0 1], 0, Ts);
W2inv = tf(12);
W2 = 1/W2inv;
% W2 = tf(0);
W2.u = {'delta_lat'};
W2.y = {'z_2'};

% Weight on the complementary sensitivity
W3 = [F2];
W3 = F_weight;
W3 = makeweight(1.3, 5, 0.001, Ts);

% W3 = tf(0.8);

W3.u = {'phi'};
W3.y = {'z_3'};

%% Assembly

Sum = sumblk('e_phi = phi_0 - phi');
% P = connect(Sum, G_nom, W1, {'phi_0', 'delta_lat'}, {'z_1', 'phi', 'p', 'e_phi'});
% P = augw(G_nom, W1);
P = connect(Rp, W1, W2, W3, Sum, Rphi, G_nom, {'phi_0'}, {'z_1', 'z_2', 'z_3'});
% P = connect(Rp, W2, W3, Sum, Rphi, G_nom, {'phi_0'}, {'z_2', 'z_3'});

% K0 = connect(Rphi, Rp, {'e_phi'; 'p'},  {'delta_lat'});
opt = hinfstructOptions('Display', 'final', 'RandomStart', 40);
[K, gamma, info] = hinfstruct(P, opt);

%% Redefinition
% Controller: R_p
b = K.Blocks.b.Value
c1 = K.Blocks.c1.Value
c2 = K.Blocks.c2.Value
d1 = K.Blocks.d1.Value
d2 = K.Blocks.d2.Value

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
d3 = K.Blocks.d3.Value
Dphi = d3;

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

% Reassembly
Loop = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});
% Loop = connect(Rp, W1, W2, W3, Sum, Rphi, G_nom, {'phi_0'}, {'z_1', 'z_2', 'z_3'});
% figure
% pzmap(Loop)

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
S = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'});
figure
bode(S, W1inv, S_des);
grid on;
legend('S', '1/W1', 'S_des')

% % Complementary Sensitivity
F = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'phi'});
figure
bode(F, W3);
grid on;
legend('Loop', 'W3');

% Control Sensitivity
Q = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'delta_lat'});
figure
bode(Q, W2inv);
figure
opt = stepDataOptions('StepAmplitude', 10);
step(Q, opt)
grid on;
legend('Q', '1/W2')

%% END OF CODE