# test
```matlab
qref_1=1;
qref_2=2;



A_1=1;
A_2=1;
psi_1=0;
psi_2=0;
omegaref_1=1;
omegaref_2=2;

wref_0=[A_1*sin(psi_1) omegaref_1*A_1*cos(psi_1) A_2*sin(psi_2) omegaref_2*A_2*cos(psi_2)];
Sref=[0 1 0 0;-omegaref_1^2 0 0 0;0 0 0 1;0 0 -omegaref_2^2 0];
Qrefp=[1 0 0 0;0 0 1 0];
Qrefv=[0 1 0 0;0 0 0 1];
Qrefa=[-omegaref_1^2 0 0 0;0 0 -omegaref_2^2 0];

%model parameters
m_1=1;
m_2=1;
l_1=1;
l_2=1;
l_c2=1/2;
l_c1=1/2;
l_1=1;
l_2=1;
g=9.81;

rho_1=m_1*l_c1^2+m_2*l_1^2+2*m_2*l_1*l_c2^2+l_1;
rho_2=m_2*l_c2^2+l_2;
rho_3=m_2*l_1*l_c2;
rho_4=(m_1*l_c1+m_2*l_1)*g;
rho_5=m_2*l_c2*g;
rho_6=m_2*l_c2;

F_fric=[0.1 0;0 0.1];


H_p=[10 1;1 10];
H_v=[30 3;3 30];
H_s=[30 3;3 30];
P=[-H_p eye(2) zeros(2);-H_v zeros(2) eye(2);-H_s zeros(2) zeros(2)];
eig(P)


kappa=1000;
K=[1 0 1 0;0 1 0 1];
N=inf;%change
M_0=[rho_1+rho_2+2*rho_3*cos(qref_2) rho_2+rho_3*cos(qref_2);rho_2+rho_3*cos(qref_2) rho_2];

omega_dist_1=1;
omega_dist_2=2;
amp_dist_1=30;
amp_dist_2=40;
bias_dist_1=5;
bias_dist_2=6;
phase_dist_1=7;
phase_dist_2=8;

S_dist_1=[0 0 0;0 0 1;0 -omega_dist_1^2 0];
S_dist_2=[0 0 0;0 0 1;0 -omega_dist_2^2 0];

S_dist=blkdiag(S_dist_1,S_dist_2);
w_dist_0=[bias_dist_1;amp_dist_1*sin(phase_dist_1);omega_dist_1*amp_dist_1*cos(phase_dist_1);
          bias_dist_2;amp_dist_2*sin(phase_dist_2);omega_dist_2*amp_dist_2*cos(phase_dist_2)];

H_dist_1=[1 1 0];
H_dist_2=[1 1 0];
H_dist=blkdiag(H_dist_1,H_dist_2);

%internal model parameters
F_1=[zeros(2,1) eye(2);-1 -3 -3];
F_2=[zeros(2,1) eye(2);-1 -3 -3];

G_1=[zeros(2,1);1];
G_2=[zeros(2,1);1];

F=blkdiag(F_1,F_2);
G=blkdiag(G_1,G_2);

Gamma_1=-F_1(3,:)-[0 omega_dist_1^2 0];
Gamma_2=-F_2(3,:)-[0 omega_dist_2^2 0];
Gamma=blkdiag(Gamma_1,Gamma_2);
```
