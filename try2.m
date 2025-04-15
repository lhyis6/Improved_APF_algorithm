clc;
clear all;
close all;
%% 参数设置（改成文献单机的了）
Katt = 15; % 引力因子
Krep = 1; % 斥力因子
Krep_uav = 1; % 无人机之间的斥力因子
d = 10; % 引力势场函数的划分阈值
rho0 = 1.5; % 障碍物的影响半径
rho1 = 0.75; % 无人机之间的安全距离
step_size = 0.1; % 步长
max_iter = 2000; % 最大迭代次数

% 无人机初始位置（文献中的无人机初始位置）
X_uav = [0, 0, 0.5; 
         0.25, 0.25, 0.25; 
         0, 0.5, 0; 
         0.5, 0, 0];

% 目标点（文献中的目标点）
X_goals = [4, 4, 4; 
           4.5, 4, 4; 
           4.25, 4.25, 4.25; 
           4, 4.5, 4];

% 障碍物位置列表（文献中的障碍物坐标）
X_obs_list = [1, 0.6, 1; 
              2, 2.5, 2; 
              1.4, 2.1, 1.4; 
              3.2, 2, 3.2; 
              2.5, 1, 2.5; 
              1, 3.5, 1; 
              1.5, 2.8, 1.6; 
              3, 3.5, 3; 
              2.5, 2, 2; 
              2.5, 2.6, 2.5; 
              2.5, 3.3, 1.8; 
              3.5, 3.2, 3.6; 
              2, 2.6, 1.6; 
              3, 2, 0; 
              4, 2, 0; 
              1.5, 3, 0];

% 障碍物半径（文献中的半径）
obs_radii = [0.2, 0.2, 0.2, 0.4, 0.4, 0.3, 0.2, 0.2, 0.3, 0.2, 0.3, 0.2, 0.3, 0.8, 0.5, 0.6];

% 无人机优先级（自己给的）
priority = [1, 2, 3, 4];

% 路径记录
Paths = cell(size(X_uav, 1), 1);
for i = 1:size(X_uav, 1)
    Paths{i} = X_uav(i, :)';
end

% 无人机之间的最小距离记录
min_distances = [];
mmin_distances = 100
% 无人机到达目标flag
X_uav_flag = [0,0,0,0];

% size(X_uav, 1)是行数
% Paths = 
%     [3×1 double]  % 无人机 1 的初始位置 [0; 0; 0.5]
%     [3×1 double]  % 无人机 2 的初始位置 [0.25; 0.25; 0.25]
%     [3×1 double]  % 无人机 3 的初始位置 [0; 0.5; 0]
%     [3×1 double]  % 无人机 4 的初始位置 [0.5; 0; 0]

%% 主循环
for iter = 1:max_iter
    for i = 1:size(X_uav, 1)
        % 当前无人机位置
        X = X_uav(i, :)';

        % 当前无人机的目标点
        X_goal = X_goals(i, :)';

        % 计算引力
        Fatt = AttractiveForceImproved(X, X_goal, Katt, d);

        % 计算障碍物的斥力
        Frep_obs = [0; 0; 0];
        for j = 1:size(X_obs_list, 1)
            X_obs = X_obs_list(j, :)';
            Frep_obs = Frep_obs + RepulsiveForceImproved(X, X_obs, Krep, rho0, obs_radii(j));
        end

        % 计算无人机之间的斥力
        Frep_uav = [0; 0; 0];
        for j = 1:size(X_uav, 1)
            if j ~= i
                X_other = X_uav(j, :)';
                if priority(i) <= priority(j)
                    Frep_uav = Frep_uav + RepulsiveForceUAV(X, X_other, Krep_uav, rho1, X_goal);
                end
            end
        end

        % 计算合力
        Ftotal = Fatt + Frep_obs + Frep_uav;

        % 更新位置
        X_new = X + step_size * Ftotal / norm(Ftotal);

        % 记录路径
        Paths{i} = [Paths{i}, X_new];

        % 更新无人机位置
        X_uav(i, :) = X_new';
        
        % 检查是否到达目标
        if norm(X_new - X_goal) < 0.1
            if X_uav_flag(i) == 0
                fprintf('无人机 %d 到达目标！\n', i);
                X_uav_flag(i) = 1;
            end
        end
    end
    % 计算当前迭代中无人机之间的最小距离
    min_distance = inf;
    for j = 1:size(X_uav, 1)
        for k = j+1:size(X_uav, 1)
            distance = norm(X_uav(j, :) - X_uav(k, :));
            if distance < min_distance
                min_distance = distance;
                if min_distance < mmin_distances
                    mmin_distances = min_distance;
                end
            end
        end
    end
    min_distances = [min_distances, min_distance];
    
    if X_uav_flag == [1,1,1,1]
        fprintf('一共步数 %d \n', iter);
        fprintf('uav间最短距离 %f \n', mmin_distances);
       break
    end
end

%% 绘制图
figure;
hold on;
colors = ['r', 'g', 'b', 'm'];
% 目标点
for i = 1:size(X_uav, 1)
    scatter3(X_goals(i, 1), X_goals(i, 2), X_goals(i, 3), 100, colors(i), 'filled'); 
end
% 障碍物
for i = 1:size(X_obs_list, 1)
    [x, y, z] = sphere;
    r = obs_radii(i);
    surf(x * r + X_obs_list(i, 1), y * r + X_obs_list(i, 2), z * r + X_obs_list(i, 3), 'EdgeColor', 'b', 'FaceAlpha', 1);
end
% 路径
for i = 1:size(X_uav, 1)
    plot3(Paths{i}(1, :), Paths{i}(2, :), Paths{i}(3, :), colors(i), 'LineWidth', 2);
end
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
grid on;
view(3);
hold off;

% 最短距离应大于2倍步长
figure;
plot(1:length(min_distances), min_distances,1:length(min_distances),2*step_size * ones(1,length(min_distances)),'LineWidth', 2);
xlabel('迭代次数');
ylabel('无人机之间的最小距离 (km)');
title('无人机之间的最小距离随迭代次数的变化');
grid on;
%% 改进后的引力函数
function Fatt = AttractiveForceImproved(X, X_goal, Katt, d)
    rho = norm(X - X_goal);
    if rho <= d
        Fatt = Katt * (X_goal - X);
    else
        A1 = d - 1;
        A2 = rho - d + 1;
        Fatt = Katt * (A1 + sqrt(A2)) * (X_goal - X) / rho;
    end
end

%% 改进后的斥力函数
function Frep = RepulsiveForceImproved(X, X_obs, Krep, rho0, obs_radius)
    rho = norm(X - X_obs) - obs_radius; % 考虑障碍物半径
    if rho <= rho0
        Frep = Krep * (1/rho^2 - 1/rho0^2)/2 * (X - X_obs) / rho; 
%         Frep = Krep * (1/rho - 1/rho0) * (X - X_obs) / rho^3;
%         Frep = Krep * (1/rho - 1/rho0)^2 * rho * (X - X_obs) / rho^3;
    else
        Frep = [0; 0; 0];
    end
end
% 三个依次是论文推的(所需步数少了),原始的，deepseek改进的、

%% 无人机之间的斥力函数
function Frep_uav = RepulsiveForceUAV(X, X_other, Krep_uav, rho1, X_goal)
    rho = norm(X - X_other);
    if rho <= rho1
        F1 = Krep_uav * (1/rho - 1/rho1) * norm(X - X_goal) / rho^2;
        F2 = 0.5 * Krep_uav * (1/rho - 1/rho1)^2;
        Frep_uav = (F1 + F2) * (X - X_other) / rho;
    else
        Frep_uav = [0; 0; 0];
    end
end