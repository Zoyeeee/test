# test
```matlab
M=[3 0 0;0 2 -0.5;0 -0.5 1];
D=[1 0 0;0 1 1;0 1 1];

omega_ref_x=1;
omega_ref_y=2;
omega_ref_psi=0.5;
amp_ref_x=1;
amp_ref_y=1;
amp_ref_psi=1;
bias_ref_x=2;
bias_ref_y=2;
bias_ref_psi=0;
phase_ref_x=0;
phase_ref_y=0;
phase_ref_psi=0;

S_ref_x=[0 0 0;0 0 1;0 -omega_ref_x^2 0];
S_ref_y=[0 0 0;0 0 1;0 -omega_ref_y^2 0];
S_ref_psi=[0 0 0;0 0 1;0 -omega_ref_psi^2 0];

S_ref=blkdiag(S_ref_x,S_ref_y,S_ref_psi);
w_ref_0=[bias_ref_x;amp_ref_x*sin(phase_ref_x);omega_ref_x*amp_ref_x*cos(phase_ref_x);
         bias_ref_y;amp_ref_y*sin(phase_ref_y);omega_ref_y*amp_ref_y*cos(phase_ref_y);
         bias_ref_psi;amp_ref_psi*sin(phase_ref_psi);omega_ref_psi*amp_ref_psi*cos(phase_ref_psi)];
H_ref_x=[1 1 0];
H_ref_y=[1 1 0];
H_ref_psi=[1 1 0];
H_ref=blkdiag(H_ref_x,H_ref_y,H_ref_psi);



omega_dist_x=0.75;
omega_dist_y=0.5;
omega_dist_psi=0.25;
amp_dist_x=9;
amp_dist_y=3;
amp_dist_psi=4;
bias_dist_x=2;
bias_dist_y=1;
bias_dist_psi=5;
phase_dist_x=9;
phase_dist_y=4;
phase_dist_psi=7;

S_dist_x=[0 0 0;0 0 1;0 -omega_dist_x^2 0];
S_dist_y=[0 0 0;0 0 1;0 -omega_dist_y^2 0];
S_dist_psi=[0 0 0;0 0 1;0 -omega_dist_psi^2 0];

S_dist=blkdiag(S_dist_x,S_dist_y,S_dist_psi);
w_dist_0=[bias_dist_x;amp_dist_x*sin(phase_dist_x);omega_dist_x*amp_dist_x*cos(phase_dist_x);
          bias_dist_y;amp_dist_y*sin(phase_dist_y);omega_dist_y*amp_dist_y*cos(phase_dist_y);
          bias_dist_psi;amp_dist_psi*sin(phase_dist_psi);omega_dist_psi*amp_dist_psi*cos(phase_dist_psi)];
H_dist_x=[1 1 0];
H_dist_y=[1 1 0];
H_dist_psi=[1 1 0];
H_dist=blkdiag(H_dist_x,H_dist_y,H_dist_psi);


F_x=[zeros(4,1) eye(4);-1 -5 -10 -10 -5];
F_y=[zeros(6,1) eye(6);-1 -7 -21 -35 -35 -21 -7];
F_psi=[zeros(6,1) eye(6);-1 -7 -21 -35 -35 -21 -7];
G_x=[zeros(4,1);1];
G_y=[zeros(6,1);1];
G_psi=[zeros(6,1);1];

S_x=blkdiag(S_dist_x,S_ref_x);
S_y=blkdiag(S_dist_y,S_ref_y,S_ref_psi);
S_psi=blkdiag(S_dist_psi,S_ref_psi,S_ref_y);

theta_1x=omega_dist_x^2;
theta_2x=omega_ref_x^2;

theta_1y=omega_dist_y^2;
theta_2y=omega_ref_y^2;

theta_1psi=omega_dist_psi^2;
theta_2psi=omega_ref_psi^2;

bartheta_1x=theta_1x+theta_2x;
bartheta_2x=theta_1x*theta_2x;

bartheta_1ypsi=theta_1y+theta_2y+theta_2psi;
bartheta_2ypsi=theta_1y*theta_2y+theta_1y*theta_2psi+theta_2y*theta_2psi;
bartheta_3ypsi=theta_1y*theta_2y*theta_2psi;

F=blkdiag(F_x,F_y,F_psi);
G=blkdiag(G_x,G_y,G_psi);

Gamma_x=-F_x(5,:)-[0 bartheta_2x 0 bartheta_1x 0];
Gamma_y=-F_y(7,:)-[0 bartheta_3ypsi 0 bartheta_2ypsi 0 bartheta_1ypsi 0];
Gamma_psi=-F_psi(7,:)-[0 bartheta_3ypsi 0 bartheta_2ypsi 0 bartheta_1ypsi 0];
Gamma=blkdiag(Gamma_x,Gamma_y,Gamma_psi);



Hp=[10 1 1;1 10 1;1 1 10];
Hv=[30 3 3;3 30 3;3 3 30];
Hs=[30 3 3;3 30 3;3 3 30];

k1=1;
k2=1;
k3=1;
k4=1;
k5=1;
k6=1;

N=100000;

K=[k1 0 0 k2 0 0;0 k3 0 0 k4 0;0 0 k5 0 0 k6];
kappa=50;
M0=eye(3);

A_i=[0 1;-1 -1];
B_i=[0;1];
C_i=[1 0];
A_o=blkdiag(A_i,A_i,A_i);
B_o=blkdiag(B_i,B_i,B_i);
C_o=blkdiag(C_i,C_i,C_i);

%mu=0.008;
mu=0.01;
```
