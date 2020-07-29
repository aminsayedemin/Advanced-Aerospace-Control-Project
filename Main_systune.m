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
ld = ss(Anom, Bnom, C, D); % Nominal Plant
ld_dis = c2d(ld, Ts, 'foh');
% ld_dis = ss(Anom, Bnom, C, D, Ts); % Nominal Plant

G = tf(ld_dis);
G.u = {'delta_lat'};
G.y = {'p', 'phi'};

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

Rp.u = {'p_0', 'p'};
Rp.y = {'delta_lat'};

%% Controller: R_phi
d3 = realp('d3', 1);
Dphi = [d3];

Rphi = ss(0, 0, 0, Dphi, Ts);
Rphi.u = {'e_phi'};
Rphi.y = {'p_0'};

% Assembly
AP = AnalysisPoint('delta_lat');
AP.u = 'delta_lat';
AP.y = 'delta_lat';

Sum = sumblk('e_phi = phi_0 - phi');

T0 = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});

csi = 0.9;
om = 10;

s = zpk('s');
F2 = om^2/(s^2 + 2*csi*om*s + om^2);
% F2 = c2d(F2, Ts);
refsys = F2;

soft = TuningGoal.Transient('phi_0', 'phi', refsys, 'step');
% hard = TuningGoal.StepTracking('phi0', 'delta_lat', 1, 100); % ?
hard = TuningGoal.Overshoot('phi_0', 'p', 10);

options = systuneOptions;
options.RandomStart = 10;
[K, FSOFT, GHARD, INFO] = systune(T0, soft, hard, options); % Soft: Objective, Hard: Constraint
FSOFT, GHARD
subplot(211), step(K(1));
subplot(212), step(K(2));

% [K.Blocks.b.Value; K.Blocks.c1.Value; K.Blocks.c2.Value;
%     K.Blocks.d1.Value; K.Blocks.d2.Value; K.Blocks.d3.Value]

% With these,
%    0.375874005430437
%    0.008663389048878
%   -0.118536064022561
%    1.035930341948536
%   -2.004010706564398
%    2.066434147615174
% the two graph are respected (but FSOFT > 1).

%% Redefinition
% % Controller: R_p
% b = K.Blocks.b.Value;
% c1 = K.Blocks.c1.Value;
% c2 = K.Blocks.c2.Value;
% d1 = K.Blocks.d1.Value;
% d2 = K.Blocks.d2.Value;
% 
% Ap = [1 0; 0 0];
% Bp = [b -b; 0 0.5];
% Cp = [c1 c2];
% Dp = [d1 d2];
% 
% Rp = ss(Ap, Bp, Cp, Dp, Ts);
% 
% % Controller: R_phi
% d3 = K.Blocks.d3.Value;
% Dphi = d3;
% 
% Rphi = ss(0, 0, 0, Dphi, Ts);