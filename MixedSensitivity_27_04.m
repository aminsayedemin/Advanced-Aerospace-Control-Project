clear all; close all; clc;
% Mixed Sensitivity Synthesis
%% Plant
% Design Requirements

M = 1.35; % Peak of sensitivity function
omb = 12; %30 %50 %[rad/s] % Lower bound on crossover frequency of S
A = 1e-3; % Max value of S at steady state

% Plant
s = zpk ('s');
G = 5*(0.2*s+1)/s/(0.1*s+1)^2/(0.01*s+1);
% G = 5*(-0.2*s+1)/s/(0.1*s+1)^2/(0.01*s+1);

W1inv = (s+A*omb)/(s/M+omb);
W1 = 1/W1inv; % Performance Weight

W2inv = tf(200, 1);
W2 = 1/W2inv; % Weight on the control sensitivity
W3 = []; % Weight of the complementary sensitivity (0 if nominal deisgn)

P = augw(G, W1, W2, W3); % SIMO vector to be optimized

%% Synthesize the controller

%  Unstructured (Optimal one)
opt = hinfsynOptions('METHOD', 'lmi'); % lmi is Linear Matrix Inequality
[K, CL, GAM] = hinfsyn(P, 1, 1, opt);

%  Structured Synthesis
% K0 = tunablePID('C', 'pid');
% opt = hinfstructOptions('Display', 'final', 'RandomStart', 5);
% K = hinfstruct(P, K0, opt);

%% Results (frequency)
figure(1), bode(G, K, G*K), grid on, legend('G','K','G*K');
figure(2), bode(1/(1+G*K), W1inv), grid on, legend('S','1/W1');
figure(3), bode(K/(1+G*K), W2inv), grid on, legend('Q','1/W2');

%% Results (time)
figure(4), step(G*K/(1+G*K)), grid on % Complementary Sensitivity
figure(5), step(K/(1+G*K)), grid on % Control Sensitivity