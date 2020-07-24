clc; close all; clear all;

% studio nominale
%% definizione del sistema
% plant: lateral dynamics
g = 9.81;

Y_v = -0.264;
Y_p = 0;
L_v = -7.349;
L_p = 0;
Y_d = 9.568;
L_d = 1079.339;

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

plant = ss(A,B,C,D)
plant_tf= tf(plant);
bode(plant)
step(plant)

% controllore: Rp PID
b = realp('b', 1);
c1 = realp('c1',1);
c2 = realp('c2',1);
d1 = realp('d1',1);
d2 = realp('d2',1);

Ap = [1 0; 0 0];
Bp = [b -b; 0 0.5];
Cp = [c1 c2];
Dp = [d1 d2];

Rp=ss(Ap,Bp,Cp,Dp)
Rp_tf = tf(Rp)

% controllore Rphi
d3 = realp('d3',1);
Aphi = [0];
Bphi = [0];
Cphi = [0];
Dphi = [d3];

Rphi = ss(Aphi, Bphi, Cphi, Dphi)
Rphi_tf = tf(Rphi)


% assemblo
Rphi.u = 'e_phi';
Rphi.y = 'p0';

Rp.u = {'p0';'p'};
Rp.y = 'delta_lat';

plant_tf.u = 'delta_lat';
plant_tf.y = {'phi'; 'p'};

sum = sumblk('e_phi = phi0 - phi');

T0 = connect(plant_tf, Rp, Rphi, sum, {'phi0'}, {'phi', 'p'});


csi = 0.95;
om = 20;
s = zpk('s');
refsys = om^2/(s^2 + 2*csi*om*s + om^2)

Req = TuningGoal.Transient('phi0','phi',refsys,'step')

options = systuneOptions;
options.RandomStart = 10;
[T,fSoft] = systune(T0,Req,options);
stepplot(T(1))



