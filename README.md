# test
```matlab
clear all;clc
x=[10 10.5 11 11.5 12 12.5 13 13.5 14 14.5];
y=nan(3,10);
y(1,:)=[0 -14 -26 -35 -48 -60 -72 -84 -95 -104];
y(2,:)=[0 -24 -47 -76 -97 -119 -142 -164 -186 -210];
y(3,:)=[0 -54 -111 -163 -214 -268 -320 -370 -421 -469];
save_k=nan(1,3);
save_t=nan(1,3);
savey=y;
maxdeltay=nan(1,3);
for i=1:3
    [k,t]=TWOby(x,y(i,:));
    y(i,:)=k.*x+t;
    save_k(i)=k;
    save_t(i)=t;
    maxdeltay(i)=max(y(i,:)-savey(i,:));
end
plot(x,y(1,:),'-r');
hold on;
plot(x,y(2,:),'--g');
plot(x,y(3,:),':b');
legend('正常(最小二乘法拟合)','半桥(最小二乘法拟合)','全桥(最小二乘法拟合)');
xlabel('X(mm)');
ylabel('V(mv)');
hold off;
save_k 
maxdeltay;
save_t
```
