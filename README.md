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

function [center, U, obj_fcn] = fcm(data, cluster_n, options)
  if nargin ~= 2 && nargin ~= 3
    error(message("fuzzy:general:errFLT_incorrectNumInputArguments"))
  end

  data_n = size(data, 1);

  % Change the following to set default options
  default_options = [2;	% exponent for the partition matrix U
      100;	% max. number of iteration
      1e-5;	% min. amount of improvement
      1];	% info display during iteration 

  if nargin == 2
    options = default_options;
  else
    % If "options" is not fully specified, pad it with default values.
    if length(options) < 4
      tmp = default_options;
      tmp(1:length(options)) = options;
      options = tmp;
    end
    % If some entries of "options" are nan's, replace them with defaults.
    nan_index = find(isnan(options)==1);
    options(nan_index) = default_options(nan_index);
    if options(1) <= 1
      error(message("fuzzy:general:errFcm_expMustBeGtOne"))
    end
  end

  expo = options(1);		% Exponent for U
  max_iter = options(2);		% Max. iteration
  min_impro = options(3);		% Min. improvement
  display = options(4);		% Display info or not

  obj_fcn = zeros(max_iter, 1);	% Array for objective function

  U = initfcm(cluster_n, data_n);			% Initial fuzzy partition
  % Main loop
  for i = 1:max_iter
    [U, center, obj_fcn(i)] = stepfcm(data, U, cluster_n, expo);
    if display
      fprintf('Iteration count = %d, obj. fcn = %f\n', i, obj_fcn(i));
    end
    % check termination condition
    if i > 1
      if abs(obj_fcn(i) - obj_fcn(i-1)) < min_impro, break; end
    end
  end

  iter_n = i;	% Actual number of iterations 
  obj_fcn(iter_n+1:max_iter) = [];
end

```
