# test
```matlab
clear all;clc
x=[20 40 60 80 100 120];
y=[0.045 0.043 0.041 0.039 0.037 0.034];
save_y=y;
plot(x,y);
hold on;
xlabel('W(g)');
ylabel('V(V)');
hold off;
[k,t]=TWOby(x,y);
y=k.*x+t;
g=(0.041-t)/k
```
