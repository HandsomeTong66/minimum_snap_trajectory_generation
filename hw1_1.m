clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;% order of poly(多项式的阶数)
n_seg         = size(path,1)-1;% segment number(段数)
n_poly_perseg = (n_order+1); % coef number of perseg(多项式系数)

ts = zeros(n_seg, 1); %返回n_seg行，1列的零矩阵
% calculate time distribution in proportion to distance between 2 points(给定一个总时长，根据路径点之间的距离来分配时间)
%dist     = zeros(n_seg, 1);
%dist_sum = 0;
%T        = 25;
%t_sum    = 0;

%for i = 1:n_seg
 %  dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
  % dist_sum = dist_sum+dist(i);
%end
%for i = 1:n_seg-1
   %ts(i) = dist(i)/dist_sum*T;
   %t_sum = t_sum+ts(i);
%end
%ts(n_seg) = T - t_sum;

% or you can simply set all time distribution as 1(给每段轨迹分配一个固定的时间)
for i = 1:n_seg
    ts(i) = 1.0;%每段时间都是1s，这样就不用算T_j^i+l-7 - T_j-1^i+l-7了(已经算好了)
end

%path(:, 1)表示所有点的x
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);%poly_coef_x是一个列向量，存的是各段多项式的全部系数
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);


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
                        %[0 1.0 0]是RGB颜色空间的值，0 1 0表示绿色
plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on
% scatter：绘制气泡图
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));

function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    %传入的waypoints分别是所有点的x坐标，y坐标
    
    %start_cond[p0, v0, a0, j0]
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp(求解Q矩阵)(因为用的时间ts都是1s，所以Q_k矩阵都一样)
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq (利用等式约束求Aeq,beq矩阵)
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);%f没有用，不用管
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);%poly_coef:存的是全部多项式的系数
end