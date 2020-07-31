clear all; close all; clc;
% Attitude Control System for ANT-R UAV
% Group: Romagnoli, Sayed, Selvatici
%% Section: Uncertainties & Monte Carlo Analysis

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
A_nom = A.NominalValue; % The matrix A with its nominal value

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
smpls = 30; % Number of samples to use in "usample"

ld_un = ss(A, B, C, D);
G_uc = tf(ld_un); % Continous t.f. of the uncertain system
G_un = usample(ld_un, smpls);
G_un = c2d(G_un, Ts, 'foh');
G_un.u = {'delta_lat'};
G_un.y = {'p', 'phi'};

%%I part: Monte Carlo Analysis
%% Results from Main.m

K.Blocks.b.Value =    1e+04 * 0.008742252290080;
K.Blocks.c1.Value =    1e+04 * - 0.003858240227559;
K.Blocks.c2.Value =    1e+04 * 0.055776527377366;
K.Blocks.d1.Value =    1e+04 * - 0.168661866654401;
K.Blocks.d2.Value =    1e+04 * 4.824852799852954;
K.Blocks.d3.Value =    1e+04 * 0.000555555533010;

%% Tuned system redefinition
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

% Assembly
Sum = sumblk('e_phi = phi_0 - phi');

Loop = connect(G_nom, Rp, Rphi, Sum, {'phi_0'}, {'p', 'phi'});
F = Loop(2);

%% MonteCarlo Analysis
Tf = 5;
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

G_nom = ss(A, B, C, D);
G_nom = c2d(G_nom, Ts, 'foh');
G_nom.u = {'delta_lat'};
G_nom.y = {'p', 'phi'};

Loop = connect(G_nom, Rp, Rphi, Sum, 'phi_0', {'p', 'phi'});
[Gm(n), Pm(n)] = margin(Loop(2));

t= (0 : Ts : Tf);
    y = step(Loop(2), t);
    S = stepinfo(y, t, 1);
    Sett(n)=S.SettlingTime;
    Over(n)=S.Overshoot;
end

%% Plots
bars = 100; % Number of histograms

% Phase margin
figure
hist(Pm, bars), grid, title('Phase margin')

% Gain margin
figure
% Note: Gm is in "mag", is it better in dB?
hist(Gm, bars), grid, title('Gain margin')
% histogram(mag2db(Gm), bars), grid, title('Gain margin')

% Settling time
figure
hist(Sett, bars), grid, title('Settling time')

% Overshoot
figure
hist(Over, bars), grid, title('Overshoot')

%%II part
%% Function W
% err1 = (G_un(1,1,:,1) - G_nom(1,1,:,1)) / G_nom(1,1,:,1);
err2 = (G_un(2,1,:,1) - G_nom(2,1,:,1)) / G_nom(2,1,:,1);

[W0, INFO] = ucover(G_un(2,1,:,1), G_nom(2,1,:,1), 1);
W = INFO.W1;

% Plot
figure;
hold on
p_e = bodeplot(err2);
p_W = bodeplot(INFO.W1);
legend('Uncertain plant', 'Function W', 'Interpreter', 'Latex');%% M - Delta decomposition
[M, delta] = lftdata(ld_un);
maxnorm = wcnorm(delta); % Max: 1.0016 (OK)
Delta = usample(delta, smpls);

% figure;
% sigma(M), grid on

%% Singular values
% Goal: verify that the upper sigma of delta is <1
% G0 = [0 0; 0 214.6/72.09]; % At zero: 0
% % [U, S, V] = svd(G0);
% % sv = diag(S);

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

BLK = [1 0];
Mu = W * F;
bounds = mussv(frd(tf(Mu), omega), BLK, 'a');

% Plot
figure;
sigma(bounds), grid on
title('S.S.V.: $\mu$', 'Interpreter', 'Latex');
% Satisfied!

%% END OF CODE