# test
```matlab
clear;close all;clc
%load fcmdata.dat

rng('default')
X_zhen=5*ones(2,200)+4*(rand(2,200)-0.5);
X_zhen(1,:)=X_zhen(1,:)+2.*rand(1,200);
figure(1)
plot(X_zhen(1, :), X_zhen(2, :), 'ro');
hold on
X_san=10*ones(2,200);
X_san(1,:)=X_san(1,:)+rand(1,200);
X_san_up=X_san(1,:)*(-1)+21;
X_san(2,:)=10+(X_san_up-10).*rand(1,200);
plot(X_san(1, :), X_san(2, :), 'ro');
hold off

data=[X_zhen, X_san];
data=data';
[centers,U]=fcm(data,2);

maxU = max(U);                     %u代表隶属度
index1 = find(U(1,:) == maxU);
index2 = find(U(2,:) == maxU);
figure(2)
plot(data(index1,1),data(index1,2),'ob')
hold on
plot(data(index2,1),data(index2,2),'or')
plot(centers(1,1),centers(1,2),'xr','MarkerSize',15,'LineWidth',3)
plot(centers(2,1),centers(2,2),'xb','MarkerSize',15,'LineWidth',3)
hold off
```
