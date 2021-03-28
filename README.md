# test
```matlab
theta_ref = pi/4;
tau_L_bias = 0.5;
tau_L_amp = 1;
tau_L_phase = 1;
tau_L_freq = 1;
white_noise = 3;
noise_power = 0.001;

%motor parameters
L = 20e-03;
R = 7.2;
B = 0.006;
lambda_m = 0.06;
n_p = 16/2;
j=0.55367e-04;
J = [0 -1; 1 0];

%control design parameters
g_2 = n_p;
g_3 = 1/j;
b= lambda_m*n_p/L;
barb = 20;
delta=( b-barb)*barb*(-1);
K = [1 3 3];
N = 10000000;
kappa = 1000;
b_0 = 1;
g_1 = 1;
g_4 = 1;
L_1 = 1;
L_2 = 1;
L_3 = 1;
b_1 = (L_1*b_0*g_4 + b_0*g_4)/g_3;
b_2 = (L_2*(g_4*sqrt(2)+2*b_1*b_0*g_3)+b_1*b_0*g_3)/g_2;
b_3 = (L_3*(g_3*sqrt(3)+3*b_2*b_1*b_0*g_2)+b_2*b_1*b_0*g_2)/g_1;
a = [b_3*b_2*b_1*b_0 b_3*b_2*b_1 b_3*b_2 b_3];
G = [0; 0; 1];
F = [0 1 0; 0 0 1; -1 -3 -3];
Gamma = [1 2 3];
```
