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
G_nom.u = {'delta_lat'};
G_nom.y = {'p', 'phi'};

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
Rp.u = {'p_0', 'p'};
Rp.y = {'delta_lat'};

%% Controller: R_phi

d3 = realp('d3', 1);
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

%% Weights

%Desired Dynamics
csi = 0.9;
om = 10;

F2 = tf([om^2], [1, 2*csi*om, om^2]);
F2 = c2d(F2, Ts, 'foh');
S_des = 1 - F2;

% Weight on the sensitivity function
csi = 0.95;
om = 10.5;

F = tf([om^2], [1, 2*csi*om, om^2]);
F = c2d(F, Ts, 'foh');
W1inv = 1 - F;
W1 = 1/W1inv;
W1.u = {'e_phi'};
W1.y = {'z_1'};

W2 = [];

W3 = [];

%% Assembly

Sum = sumblk('e_phi = phi_0 - phi');
P = connect(G_nom, Sum, W1, {'phi_0', 'delta_lat'}, {'phi', 'z_1', 'e_phi', 'p'});

K0 = connect(Rp, Rphi, {'e_phi', 'p'},  {'delta_lat'});

opt = hinfstructOptions('Display', 'final', 'RandomStart', 20);
[K, gamma, info] = hinfstruct(P, K0, opt)

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


%% Plots
% Step response and comparison
Tf = 20; % Final time of the Step plot
figure
step(Loop, Tf); % Step of our system
% s2 = step(F2, Tf); % Step of the desired system

% figure
% hold on
% plot(0:Ts:Tf, y, 'b');
% plot(0:Ts:Tf, s2, 'r');
% legend('Actual response', 'Desired response', 'Interpreter', 'Latex', 'Location', 'southeast');
% xlabel('Time [s]', 'Interpreter', 'Latex');
% ylabel ('Amplitude', 'Interpreter', 'Latex');
% axis ([0 Tf -0.2 1.2]);
% grid on

% Sensitivity
S = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'});
figure
bode(S, W1inv, S_des);
grid on;
legend('S', '1/W1', 'Interpreter', 'Latex');

%% END OF CODE
