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

%% Uncertain plant
ld_un = ss(A, B, C, D);

G = usample(ld_un, 10);
G_dis = c2d(G, Ts, 'foh');
G_dis.u = {'delta_lat'};
G_dis.y = {'p', 'phi'};

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
<<<<<<< HEAD
csi = 0.9;
om = 10;
=======
% % W1
csi = 0.9;
om = 15;
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b

F2 = tf([om^2], [1, 2*csi*om, om^2]);
F2 = c2d(F2, Ts, 'foh');
S_des = 1 - F2;

<<<<<<< HEAD
% Weight on the sensitivity function
=======
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b
W1inv = S_des;
W1 = 1/W1inv;

W1.u = {'e_phi'};
W1.y = {'z_1'};

<<<<<<< HEAD
% Weight on the control sensitivity
% W2inv = tf(0, 1);
% W2inv = c2d(W2inv, Ts, 'foh');
% W2 = 1/W2inv;
% W2.u = {'delta_lat'};
% W2.y = {'z_2'};

% Weight of the complementary sensitivity (0 if nominal design)
W3inv = F2;
W3 = W3inv;

=======
% % W2
% % W2inv = tf(0, 1);
% % W2inv = c2d(W2inv, Ts, 'foh');
% % W2 = 1/W2inv; % Weight on the control sensitivity
% % W2.u = {'delta_lat'};
% % W2.y = {'z_2'};
% 
% W3 % Weight of the complementary sensitivity (0 if nominal design)
W3inv = F2;
W3 = W3inv;
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b
W3.u = {'phi'};
W3.y = {'z_3'};

Wm = tf([om^2], [1, 2*csi*om, om^2]);
Wm = c2d(Wm, Ts, 'foh');
Wm.u = 'phi_0';
Wm.y = 'm';


%% Assembly
Sum1 = sumblk('e_phi = phi_0 - phi');
Sum2 = sumblk('tracking_error = phi - m');

<<<<<<< HEAD
CL0 = connect(G_dis, Rp, Rphi, W1, W3, Sum, {'phi_0'}, {'p', 'phi', 'z_1', 'z_3'});
=======
OPT = connectOptions('Simplify', false);
CL0 = connect(G_dis, Rp, Rphi, Wm, W1,Sum1, Sum2, {'phi_0'}, {'p', 'tracking_error','z_1'}, OPT);
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b

opt = hinfstructOptions('Display', 'final', 'RandomStart', 2);
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
<<<<<<< HEAD
=======
Sum = sumblk('e_phi = phi_0 - phi');
L = connect(G_dis, Rp, Rphi, {'e_phi'}, {'phi'}, OPT);
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b
Loop = connect(G_dis, Rp, Rphi, Sum, 'phi_0', {'p', 'phi'}, OPT);

%% Plots
<<<<<<< HEAD
csi = 0.9; om = 10; % Lower Bounds - Desired System
F_lim = tf([om^2], [1, 2*csi*om, om^2]); % Desired Complementary Sensitivity
F_lim = c2d(F_lim, Ts, 'foh');
=======
% figure, bode(G_dis, K, G*K), grid, legend('G','K','G*K');
% figure, bode(S, W1inv), grid, legend('S', '1/W1');
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b

figure;
<<<<<<< HEAD
s1 = [];
for i = 1:size(Loop(2,1,:,1),3)
    y = step(tf(Loop(2,1,i,1)), 0:Ts:10); % Step of our system
    s1 = [s1, y];
end
s2 = step(F_lim, 0:Ts:10); % Step of the desired system

hold on
h1 = plot(0:Ts:10, s1, 'b');
h2 = plot(0:Ts:10, s2, 'k');
h = [h1(1), h2(1)];
legend(h, 'Uncertain bundle', 'Lower bound', 'Interpreter', 'Latex');
xlabel('Time [s]', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
axis ([0 5 -0.2 1.2]);
grid on
=======
subplot(211)
step(Loop(2), 5);
grid minor

subplot(212);
step(Wm, 5);
grid minor
legend('Desired');

% figure;
% margin(S);
% grid on
% title('Margin of S');
>>>>>>> 0f40e70c9d0a7ae8c2f339190c04a687ea91039b

% Sensitivity
S = connect(G_dis, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'}, OPT);
figure;
bode(S, W1inv);
grid on;
legend('S', '1/W1', 'Interpreter', 'Latex');

%% END OF CODE
