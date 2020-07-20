clear all; close all; clc;
%% MonteCarlo Verification

Y_v = ureal('yv', -0.264,'Perc', 4.837);
Y_p = 0;
L_v = ureal('lv', -7.349,'Perc', 4.927);
L_p = 0;
Y_d = ureal('yd', 9.568,'Perc', 4.647);
L_d = ureal('ld', 1079.339,'Perc', 2.762);
g = 9.81;

A = [Y_v    Y_p     g;
    L_v     L_p     0;
    0       1       0];
N = size(A);
Anom = A.NominalValue; % The matrix with its nominal value

B = [Y_d;
    L_d;
    0];

C = [0      1       0;
    0       0       1];

D = [0;
    0];

Ts = 0.004; % Sampling interval
sC = ss(A, B, C, D, Ts); % Discrete time
G = tf(sC);
% Y = usample(G,10);
% bode(G(1));

%% Second order model parameters
wn = 50;
xi = ureal('xi', 0.5, 'Range', [0.1 0.8]);      % uncertain damping ratio

% Uncertainty model for the plant
% Parray = usample(P,100);
% Pn = P.NominalValue;
% Wt = tf(0.4*[1 1],[1/60 1]);
% bodemag((Pn-Parray)/Pn, Wt, 'r')

% Nominal controller, nominal performance and robust stability
% Cn = tf([1, 1],[1 0]);
% margin(Pn*Cn), grid
% step(minreal(Pn*Cn/(1+Pn*Cn))), grid
% bode(minreal(Pn*Cn/(1+Pn*Cn)),1/Wt), grid
% bode(minreal(Pn*Cn/(1+Pn*Cn)*Wt)), grid

%% MonteCarlo
% N = 500; % Samples 
% 
% gamma_mc = zeros(N);
% tau_mc = zeros(N);
% xi_mc = zeros(N);
% 
% % Note: in the original script there was "rand" because the probability
% % density was uniform. Is randn sufficient to make it Gaussian?
% for n=1:N
%     % Resampling of uncertain parameters
%     gamma_mc(n)= 0.7*2 + (1.3-0.7)*2*randn(1,1);
%     tau_mc(n)= 0.7*1 + (1.3-0.7)*1*randn(1,1);
%     xi_mc(n)= 0.1 + (0.8-0.1)*randn(1,1);
%     
%     % Construction of resampled uncertain plant
%     P_mc(n) = tf(gamma_mc(n),[tau_mc(n) 1]) * tf(wn^2,[1 2*xi_mc(n)*wn wn^2]);
%     
%     % Computation of gain and phase margins
%     [Gm(n),Pm(n)] = margin(P_mc(n)*Cn);
%     
%     % Computation of step response 
%     t=(0:.01:10);
%     y=step(minreal(P_mc(n)*Cn/(1+P_mc(n)*Cn)),t);
%     
%     % Computation of step response characteristics
%     S = stepinfo(y,t,1);
%     
%     Sett(n)=S.SettlingTime;
%     Over(n)=S.Overshoot;
%     
% end
% 
% %% Results
% 
% % Plot the histogram of phase margin
% hist(Pm,100), grid, title('Phase margin')
% 
% % Plot the histogram of gain margin
% hist(Gm,100), grid, title('Gain margin')
% 
% % Plot the histogram of settling time
% hist(Sett,100), grid, title('Settling time')
% 
% % Plot the histogram of overshoot
% hist(Over,100), grid, title('Overshoot')
% 
% %% Studying worst cases
% % find MC case corresponding to worst settling time
% max_Sett=max(Sett)
% worst=find(Sett==max_Sett)
% 
% % Find worst case uncertainty
% gamma_w=gamma_mc(worst);
% tau_w=tau_mc(worst);
% xi_w=xi_mc(worst);
% 
% % Construct worst case uncertain plant 
% P_w = tf(gamma_w,[tau_w 1]) * tf(wn^2,[1 2*xi_w*wn wn^2]);
% step(P_w,5), grid
% bode(P_w), grid
% margin(P_w*Cn), grid