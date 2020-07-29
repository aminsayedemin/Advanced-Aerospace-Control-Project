clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% System definition
% x = [v p \phi]^T
% y = [p \phi]^T

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

%% Uncertain plant

% smpls = 30; % Number of samples to use in "usample"
% 
% ld_un = ss(A, B, C, D);
% G = usample(ld_un, smpls);
% G_un = c2d(G, Ts, 'foh')
% G_un.u = {'delta_lat'};
% G_un.y = {'p', 'phi'};
% 
% % ld_un_dis = ss(A, B, C, D, Ts);
% % G_un = tf(ld_un_dis)
% % G_un.u = {'delta_lat'};
% % G_un.y = {'p', 'phi'};
% 
% % M - Delta decomposition
% [M, delta] = lftdata(ld_un);
% % G = Gnom * (1 + W * delta);
% err1 = (G_un(1,1,:,1) - G_nom(1,1,:,1)) / G_nom(1,1,:,1);
% err2 = (G_un(2,1,:,1) - G_nom(2,1,:,1)) / G_nom(2,1,:,1);
% 
% % [W, INFO] = ucover(G_dis(1,1,:,1), Gnom(1,1,:,1), 10);
% % figure
% % bode(err1, INFO.W1)
% 
% [W, INFO] = ucover(G_un(2,1,:,1), G_nom(2,1,:,1), 1);
% % figure
% % bode(err2, INFO.W1)
% W3 = INFO.W1;
% 
% % err = (G_dis - Gnom);
% % [sys, INFO] = ucover(G_dis, Gnom, 3);
% % figure
% % bode(err, sys)

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

csi = 0.9; %0.35
om = 10; %6
k = 10;

F2 = tf([k*om^2], [1, 2*csi*om, om^2]);
F2 = c2d(F2, Ts, 'foh');
S_des = 1 - F2;

% Weight on the sensitivity function
W1inv = S_des;
W1 = 1/W1inv;
W1.u = {'e_phi'};
W1.y = {'z_1'};

% Weight on the control sensitivity
% W2inv = tf(0, 1);
% W2inv = c2d(W2inv, Ts, 'foh');
% W2 = 1/W2inv;
% W2.u = {'delta_lat'};
% W2.y = {'z_2'};

% Weight of the complementary sensitivity (0 if nominal design)
% W3inv = F2;
% W3 = 1/W3inv;
% W3.u = {'phi'};
% W3.y = {'z_3'};

% Wm = tf([om^2], [1, 2*csi*om, om^2]);
% Wm = c2d(Wm, Ts, 'foh');
% Wm.u = 'phi_0';
% Wm.y = 'm';

%% Assembly

Sum1 = sumblk('e_phi = phi_0 - phi');

CL0 = connect(G_nom, Rp, Rphi, W1, Sum1, {'phi_0'}, {'p', 'phi', 'z_1'});
opt = hinfstructOptions('Display', 'final', 'RandomStart', 50);
[K, gamma, info] = hinfstruct(CL0, opt)

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
Loop = connect(G_nom, Rp, Rphi, Sum1, 'phi_0', {'p', 'phi'});

%% Robustness analysis
% T = connect(G_un, Rp, Rphi, Sum1, 'phi_0', {'p', 'phi'});
% 
% % Robust stability margin (>1 for stability)
% opt = robOptions('Display','on', 'Sensitivity','on');
% [StabilityMargin] = robstab(T, opt);
% 
% [PeakGain, wcu] = wcgain(T); % Worst peak gain over frequency
% Twc = usubs(T,wcu); % Worst-case closed-loop transfer T
% 
% Trand = usample(T,4);
% 
% figure;
% subplot(211), bodemag(Trand,'b',Twc,'r',{10 1000});
% subplot(212), step(Trand,'b',Twc,'r',0.2);

%% Plots
% Step response and comparison
Tf = 1; % Final time of the Step plot

csi = 0.9; om = 10; % Lower Bounds - Desired System
F_lim = tf([om^2], [1, 2*csi*om, om^2]); % Desired Complementary Sensitivity
F_lim = c2d(F_lim, Ts, 'foh');

figure;
s1 = [];
    y = step(Loop(2), Tf); % Step of our system
    s1 = [s1, y];
s2 = step(F_lim, Tf); % Step of the desired system

hold on
h1 = plot(0:Ts:Tf, s1, 'b');
h2 = plot(0:Ts:Tf, s2, 'k');
h = [h1(1), h2(1)];
legend(h, 'Uncertain bundle', 'Lower bound', 'Interpreter', 'Latex', 'Location', 'southeast');
xlabel('Time [s]', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
axis ([0 Tf -0.2 1.2]);
grid on

% Sensitivity
S = connect(G_nom, Rp, Rphi, Sum1, {'phi_0'}, {'e_phi'});
figure;
bode(S, W1inv);
grid on;
legend('S', '1/W1', 'Interpreter', 'Latex');

% Complementery sensitivity
F = connect(G_nom, Rp, Rphi, Sum1, {'phi_0'}, {'phi'});
figure;
bode(F, 1/W3);
grid on;
legend('F', '1/W3', 'Interpreter', 'Latex');

%% MC Analysis
close all

margin(Loop(2)),grid
figure
bode(Loop(2)*W3), grid
N = 500;

Y_v = zeros(N);
Y_p = 0;
L_v = zeros(N);
L_p = 0;
Y_d = zeros(N);
L_d = zeros(N);
for n = 1:N
    Y_v(n) = -0.264 + 4.837/100*(-0.264)*randn(1,1);
    L_v(n) = -7.349 + 4.927/100*(-7.349)*randn(1,1);
    Y_d(n) = 9.568 + 4.647/100*(9.568)*randn(1,1);
    L_d(n) = 1079.339 + 2.762/100*(1079.339)*randn(1,1);

A = [Y_v(n)    Y_p     g;
    L_v(n)     L_p     0;
    0       1       0];

B = [Y_d(n);
    L_d(n);
    0];

C = [0      1       0;
    0       0       1];

D = [0;
    0];

G_nom = ss(A, B, C, D);
G_nom = c2d(G_nom, Ts, 'foh');
G_nom.u = {'delta_lat'};
G_nom.y = {'p', 'phi'};
Loop = connect(G_nom, Rp, Rphi, Sum1, 'phi_0', {'p', 'phi'});
[Gm(n),Pm(n)] = margin(Loop(2));

t=(0:Ts:Tf);
    y = step(Loop(2),t);
    S = stepinfo(y,t,1);
    Sett(n)=S.SettlingTime;
    Over(n)=S.Overshoot;
end

% Plot the histogram of phase margin
figure
hist(Pm,100), grid, title('Phase margin')

% Plot the histogram of gain margin
figure
hist(Gm,100), grid, title('Gain margin')

% Plot the histogram of settling time
figure
hist(Sett,100), grid, title('Settling time')

% Plot the histogram of overshoot
figure
hist(Over,100), grid, title('Overshoot')

%% END OF CODE