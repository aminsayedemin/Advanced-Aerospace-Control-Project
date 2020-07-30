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

%% Plots desired
csi = 0.9;
om = 10;
F = tf([om^2], [1, 2*csi*om, om^2]); % complementary sensitivity, or closed loop tf
F = c2d(F, Ts, 'foh');
% F = L/(1+L), L is the open loop tf
L = F/(1-F);
S_ex = 1 - F;

% we set the proper weight
% M=2;
% omb=4;
% A=0.001;
% s = zpk('s');
% W1inv = (s+A*omb)/(s/M+omb);
% W1inv = c2d(W1inv, Ts, 'foh');
% 
% wc = 6
% HF = db2mag(6);
% LF = db2mag(-40);
% W1inv = makeweight(LF,wc,HF,Ts)
% W1 = 1/W1inv;

% w = 10;
% mag_w = db2mag(-10);
% LF = db2mag(-50);
% HF = db2mag(0);
% W2 = makeweight(LF,[w mag_w],HF,Ts,2)
% 
% bode(L, F, S) %?
% legend('L','F','S')
% figure;
% bode(W1,W2)
% legend('W1','W2')

%% Weights
% % Desired Dynamics
% csi = 0.9;
% om = 10;

% % Model Weight
% Wm = tf([om^2], [1, 2*csi*om, om^2]);
% Wm = c2d(Wm, Ts, 'foh');
% Wm.u = 'phi_0';
% Wm.y = 'phi_model';

% S Weight
M=2;
omb=18;
A=0.001;
s = zpk('s');
W1inv = (s+A*omb)/(s/M+omb);
W1inv = c2d(W1inv, Ts, 'foh');
W1 = 1/W1inv;
% wc = 5;
% HF = db2mag(6);
% LF = db2mag(-40);
% W1inv = makeweight(LF,wc,HF,Ts)
% W1 = 1/W1inv;
W1.u = 'e_phi';
W1.y = 'es';

% control weight
wctrl = 15;
mag_w = db2mag(-10);
LFc = db2mag(-60);
HFc = db2mag(0);
W2 = makeweight(LFc,[wctrl mag_w],HFc,Ts,2)
W2.u = 'delta_lat';
W2.y = 'ec';

%% Assembly
Sum_FB = sumblk('e_phi = phi_0 - phi');
%Sum_me = sumblk('e = phi - phi_model');

Loop2Tune = connect(G_nom, Rp,Rphi,W1,W2, Sum_FB, 'phi_0',{'es','ec'});
opt = hinfstructOptions('Display', 'final', 'RandomStart', 50);
[TunedLoop, gamma, info] = hinfstruct(Loop2Tune, opt)

%% Redefinition
% Controller: R_p
b = TunedLoop.Blocks.b.Value
c1 = TunedLoop.Blocks.c1.Value
c2 = TunedLoop.Blocks.c2.Value
d1 = TunedLoop.Blocks.d1.Value
d2 = TunedLoop.Blocks.d2.Value

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
d3 = TunedLoop.Blocks.d3.Value
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

% Reassembly
Sum = sumblk('e_phi = phi_0 - phi');
Loop = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'p','phi'});

%% Plots
figure;
step(Loop);
figure;
step(Loop(2),F)

S = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'});
figure;
bode(S, W1inv,S_ex);
legend('S','W1inv','S_ex')
%% END OF CODE
