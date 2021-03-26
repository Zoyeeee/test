# test
```matlab
x=[10 10.5 11 11.5 12 12.5 13 13.5 14 14.5];
y=[1 6 10 15 19 23 27 31 35 39];
save_y=y;
plot(x,y);
hold on;
xlabel('X(mm)');
ylabel('V(mv)');
plot(x,y,'r');
hold off;
[k,t]=TWOby(x,y);
y=k.*x+t;
k

```
