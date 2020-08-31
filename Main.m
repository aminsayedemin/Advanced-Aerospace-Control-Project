clear all; close all; clc;
%% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici

%% Lateral Dynamics Block
% x = [v p \phi]^T; y = [p \phi]^T
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
Ts = 0.004; % Sampling interval

ld = ss(Anom, Bnom, C, D);
ld_dis = c2d(ld, Ts, 'foh');

G_nom = tf(ld_dis);
G_nom.u = {'delta_lat'};
G_nom.y = {'p', 'phi'};

%% Uncertain plant
ld_un = ss(A, B, C, D);

smpls = 100; % Number of samples to use in "usample"

G_un = usample(ld_un, smpls);
G_un = c2d(G_un, Ts, 'foh');
G_un.u = {'delta_lat'};
G_un.y = {'p', 'phi'};
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
csi_w1 = 0.91;
om_w1 = 12; %crossover frequency
s = zpk('s');
F_w1 = om_w1^2/(s^2 + 2*csi_w1*om_w1*s + om_w1^2);
W1inv = c2d((1 - F_w1), Ts, 'foh');
W1 = 1/W1inv;
W1.u = {'e_phi'};
W1.y = {'z_1'};

% Weight on the control sensitivity
W2inv = tf([2],[1/33 1]);
W2inv = c2d(W2inv, Ts, 'foh');
W2 = 1/W2inv;
W2.u = {'delta_lat'};
W2.y = {'z_2'};

% Weight on the complementary sensitivity
[W0, INFO] = ucover(G_un(2,1,:,1), G_nom(2), 1);
W3 = INFO.W1;
W3inv = 1/W3;

%% Assembly
Sum = sumblk('e_phi = phi_0 - phi');

P_h = connect(Rp, W1, W2, Sum, Rphi, G_nom, {'phi_0'}, {'z_1', 'z_2'});
P_s = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'}, {'delta_lat'});

%% Tuning
% Desired dynamics
csi = 0.9;
om = 10;

s = zpk('s');
F0 = om^2/(s^2 + 2*csi*om*s + om^2);
F0 = c2d(F0, Ts, 'foh');
S0 = 1 - F0;

soft = TuningGoal.Transient('phi_0', 'phi', F0, 'step');
hard = TuningGoal.Gain('phi_0', 'delta_lat', 1);

starts = 50;

% options = systuneOptions;
% options.RandomStart = starts;
% [K_s, GSOFT, GHARD, INFO] = systune(P_s, soft, hard, options); % Soft: Objective, Hard: Constraint
% % GSOFT, GHARD

opt = hinfstructOptions('Display', 'final', 'RandomStart', starts);
[K_h, gamma, info] = hinfstruct(P_h, opt);
% gamma

for i = 1:2 % Needed for the first plot
%     if i == 1
%         K = K_s;
%     elseif i == 2
        K = K_h;
%     end
    
% [K.Blocks.b.Value; K.Blocks.c1.Value; K.Blocks.c2.Value;
%     K.Blocks.d1.Value; K.Blocks.d2.Value; K.Blocks.d3.Value]

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
Loop = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});
F = Loop(2); % Complementary Sensitivity
S = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'e_phi'}); % Sensitivity
Q = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'delta_lat'});% Control sensitivity

if i == 1 % If systune
Loop_old = Loop;
F_old = F;
S_old = S;
Q_old = Q;
end

end
%% Step plot
Tf = 3; % Final time of the step plots

% Requirement
csi = 0.9;
om = 10;
s = zpk('s');
F0 = om^2/(s^2 + 2*csi*om*s + om^2);
S0 = 1- F0;

% Comparison between hinfstruct and systune (_old)
figure;
hold on
[y_h, t_Lh] = step(F, Tf); % Step of the system of hinfstruct with output phi
[y_sys, t_Lsys] = step(F_old, Tf); % Step of the system with output phi
[z_s, t_F] = step(F0, Tf); % Step response of the desired dynamics

h1 = plot(t_Lh, squeeze(y_h), 'LineWidth', 2);
h2 = plot(t_Lsys, squeeze(y_sys), 'LineWidth', 2);
h3 = plot(t_F, squeeze(z_s), '--', 'LineWidth', 2);

% legend([h3(1), h1(1), h2(1)], 'Desired response', '\textit{hinfstruct}', '\textit{systune}', 'Location', 'southeast', 'Interpreter', 'Latex');
legend([h3(1), h1(1)], 'Desired response', '\textit{hinfstruct}', 'Location', 'southeast', 'Interpreter', 'Latex');
xlabel('Time [s]', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
axis ([0 Tf -0.2 1.2]);
grid on

%% Plots
% Sensitivity
figure;
bode(S, W1inv, S0, {1e-20, 1e3});
grid on;
legend('S', '1/W1', 'Desired S', 'Interpreter', 'Latex');
title ('Sensitivity Function', 'Interpreter', 'Latex');

% Complementary Sensitivity
figure;
bode(F, W3inv, F0);
grid on;
legend('F', '1/W3', 'Desired F', 'Interpreter', 'Latex');
title ('Complementary Sensitivity Function', 'Interpreter', 'Latex');

% Robust Performance
RP1 =  W1 * S;
RP2 = W3 * F;
RP = RP1 + RP2;

figure
bode(RP1, RP2, RP, {1e-20, 1e+3})
grid on;
legend('$|W1 \cdot S|$', '$|W3 \cdot F|$', '$|W1 \cdot S| + |W3 \cdot F|$', 'Interpreter', 'Latex', 'location', 'southwest');
title ('Robust Performance', 'Interpreter', 'Latex');

% Control sensitivity
figure;
bode(Q, W2inv);
legend('Q', '1/W2', 'Interpreter', 'Latex');
grid on;
title ('Control Sensitivity Function', 'Interpreter', 'Latex');

% Step response: both phi and p
figure;
hold on
[s_L1, t_L1] = step(Loop(1), Tf);
[s_L2, t_L2] = step(Loop(2), Tf);

subplot(2,1,1), plot(t_L1, squeeze(s_L1), 'LineWidth',2);
xlabel('Time [s]', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
title('Output: p', 'Interpreter', 'Latex');
axis ([0 Tf -0.2 6]);
grid on

subplot(2,1,2), plot(t_L2, squeeze(s_L2), 'LineWidth',2);
xlabel('Time [s]', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
title('Output: phi', 'Interpreter', 'Latex');
axis ([0 Tf -0.2 1.2]);
grid on
    
% Step 10 of the control sensitivity
figure;
opt = stepDataOptions('StepAmplitude', deg2rad(10));
[s_Q, t_Q] = step(Q, 50, opt);
plot(t_Q, squeeze(s_Q), 'LineWidth',2)
xlabel('Time', 'Interpreter', 'Latex');
ylabel ('Amplitude', 'Interpreter', 'Latex');
grid on;
legend('Output: $\delta_{lat}$', 'Interpreter', 'Latex');
title('Step of amplitude 10 response', 'Interpreter', 'Latex');

%% MonteCarlo Analysis
T_final = 5;
N = 500;

Y_v = zeros(N);
L_v = zeros(N);
Y_d = zeros(N);
L_d = zeros(N);
Y_p = 0;
L_p = 0;

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

G = ss(A, B, C, D);
G = c2d(G, Ts, 'foh');
G.u = {'delta_lat'};
G.y = {'p', 'phi'};

Loop = connect(G, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});
[Gm(n), Pm(n)] = margin(Loop(2));

t= (0 : Ts : T_final);
    y = step(Loop(2), t);
    S = stepinfo(y, t, 1);
    Sett(n)= S.SettlingTime;
    Over(n)= S.Overshoot;
end

%% Plots
bars = 100; % Number of histograms

% Phase margin
figure
hist(Pm, bars), grid, title('Phase margin')

% Gain margin
figure
% hist(Gm, bars), grid, title('Gain margin') % Gm in mag
hist(mag2db(Gm), bars), grid, title('Gain margin')  % Gm in dB

% Settling time
figure
hist(Sett, bars), grid, title('Settling time')

% Overshoot
figure
hist(Over, bars), grid, title('Overshoot')

%% Function W (= W3)
err2 = (G_un(2,1,:,1) - G_nom(2,1,:,1)) / G_nom(2,1,:,1);

figure;
hold on
p_e = bodeplot(err2);
p_W = bodeplot(W3);
legend('Uncertain plant', 'W3', 'Interpreter', 'Latex');

%% M - Delta decomposition
[M, delta] = lftdata(ld_un);
maxnorm = wcnorm(delta); % Max: 1.001611 (OK)
Delta = usample(delta, smpls);

figure;
sigma(M), grid on

%% Singular values
% Goal: verify that the upper sigma of delta is <1
% G0 = [0 0; 0 214.6/72.09]; % At zero: 0
% [U, S, V] = svd(G0);
% sv = diag(S);

maxi = 0;
for i = 1:smpls
    sv = svd(Delta(:,:,i));
   
    if sv > max(maxi)
        maxi = sv;
        
        if max(maxi) >= 1
            disp('You"re fucked bro');
        end
    end
end

%% SSV (mu-Delta Analysis)
% There are no sigmas > 1, so now Goal: verify that mu<1 at all omega

N = size(delta, 1);
omega = logspace(-3, 2, 500);

BLK = [1 0]; % delta is real and diagonal
Mu = W3 * F;
bounds = mussv(frd(tf(Mu), omega), BLK, 'a');

% Plot
figure;
sigma(bounds), grid on
title('S.S.V.: $\mu$', 'Interpreter', 'Latex');
% Satisfied!

%% END OF CODE