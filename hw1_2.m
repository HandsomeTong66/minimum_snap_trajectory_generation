clc;clear;close all;
path = ginput() * 100.0;

n_order = 7;% order of poly(多项式的阶数)
n_seg = size(path, 1) - 1;% segment number(段数)
n_poly_perseg = n_order + 1; % coef number of perseg(多项式系数)

ts = zeros(n_seg, 1);%返回n_seg行，1列的零矩阵
% calculate time distribution based on distance between 2 points(给定一个总时长，根据路径点之间的距离来分配时间)
%dist = zeros(n_seg, 1);
%dist_sum = 0;
%T = 25;

%t_sum = 0;
%for i = 1:n_seg
  %  dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
 %   dist_sum = dist_sum + dist(i);
%end
%for i = 1:n_seg-1
  %  ts(i) = dist(i) / dist_sum * T;
 %   t_sum = t_sum + ts(i);
%end
%ts(n_seg) = T - t_sum;
% or you can simply average the time
 for i = 1:n_seg
     ts(i) = 1.0;
 end

%path(:, 1)表示所有点的x
poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);

% display the trajectory(可视化的部分)
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;%间隔

 a = 1;
 b = 8;
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis(获得x轴和y轴的第i段的系数)
    % and y-axis
    for t = 0:tstep:ts(i+1) %每一段时间都从0开始
        Pxi = poly_coef_x(a:b);
        x = flipud(Pxi);
        Pyi = poly_coef_y(a:b);
        y = flipud(Pyi);
        X_n(k)  = polyval(x, t);
        Y_n(k)  = polyval(y, t);
        k = k + 1;
    end
    a = a + 8;
    b = b + 8;
end

plot(X_n, Y_n ,'Color',[0 1.0 0],'LineWidth',2);
hold on
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    C = Ct';
    % inv():矩阵求逆
    R = C * inv(M)' * Q * inv(M) * Ct;
    % mat2cell()：矩阵分块
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    %#####################################################
    % STEP 3: compute dF
    dF = zeros(n_seg+7,1);
    dF(1) = waypoints(1);
    for i=1:n_seg
        dF(i+4)= waypoints(i+1);
    end
    dP = -inv(R_pp) * R_fp' * dF;
    poly_coef = inv(M) * Ct * [dF;dP];
end