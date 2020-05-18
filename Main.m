clear all
close all
clc
%% Ciao sono Davide
plot([0,1], [10,12]);
%% Ciao
plot(3, 4)
aueeeee
g = 9.81;

Y_v = -0.264;                   % stability derivatives
Y_p = 0;
L_v = -7.349;
L_p = 0;

Y_d = 9.568;                    % control derivatives
L_d = 1079.339;
%--------------------------------------------------------------------------
A = [Y_v    Y_p     g;
    L_v     L_p     0;
    0       1       0];

B = [Y_d;
    L_d;
    0];

C = [0      1       0;
    0       0       1];

D = [0;
    0];
%--------------------------------------------------------------------------
sys = ss(A,B,C,D)