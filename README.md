# test
```matlab
x=[10 10.5 11 11.5 12 12.5 13 13.5 14 14.5];
y=[0 -14 -26 -35 -48 -60 -72 -84 -95 -104];
save_y=y;
plot(x,y);
hold on;
xlabel('X(mm)');
ylabel('V(mv)');
[k,t]=TWOby(x,y);
y=k.*x+t;
plot(x,y,'r');
legend('实际线','最小二乘拟合线')
hold off;
maxdeltay=max(save_y-y);
k
t

function [k,t] = TWOby(ind,dp)
if length(ind)~= length(dp)
    disp('数组长度不等，请检查！');       % 要求 x,y 一一对应
end
% 最小二乘法的系数设置，参考数学模型
    a = ind*ind';
    b = sum(ind);
    c = ind*dp';
    d = sum(dp);
% 求解斜率k
k = (length(ind).*c-b*d)./(length(ind).*a-b*b);
% 求解截距t
t = (a.*d-c.*b)/(a*length(ind)-b.*b);
end
```
