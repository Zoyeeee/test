# test
```matlab
syms x y z;

g=9.81;
r0=5;
r1=10;
alpha=0.9;
L=alpha*g;
bar_B=[g,0;0,-g];
N=100;
kappa=5;
k=[-100 -100 -100 -10];
a=[0.01 1 1000 1000 10];
init=[1 1 1 pi/6 pi/6 ];


%%

%  fprintf('Program paused. Press enter to continue.\n');
%  pause;
  init=[1 2 3 pi/6 pi/4 ];
 
 
%  fprintf('Program paused. Press enter to continue.\n');
%  pause;
  init=[-1 -1 -1 -pi/6 -pi/6 ];

 
  
%  fprintf('Program paused. Press enter to continue.\n');
%  pause;
 init=[-1 -2 -3 -pi/6 -pi/4 ];


```
