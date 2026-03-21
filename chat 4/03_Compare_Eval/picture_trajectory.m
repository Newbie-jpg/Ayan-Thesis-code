% PICTURE_EXP1_TRAJECTORY
% 绘制实验一：两种算法的传感器运动轨迹与目标真实轨迹对比图
%
% 使用方法：
%   先运行 chat4/01_Main_Env/main_exp1_performance.m 生成 Exp1_Result_*.mat
%   再运行本脚本生成轨迹对比图

clear; close all; clc;

%% 1. 路径准备与自动加载最新仿真数据
script_dir = fileparts(mfilename('fullpath')); % chat4/03_Compare_Eval
chat4_root = fileparts(script_dir);            
data_dir = fullfile(chat4_root, '04_Data');
fig_dir = fullfile(script_dir, 'Result_Fig_Ch4');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

disp('>>> 正在寻找最新的 Exp1 仿真结果数据...');
mat_files = dir(fullfile(data_dir, 'Exp1_Result_*.mat'));
if isempty(mat_files)
    error('未在 %s 目录下找到 Exp1_Result_*.mat。请先运行 main_exp1_performance.m', data_dir);
end
[~, latest_idx] = max([mat_files.datenum]);
target_file = fullfile(mat_files(latest_idx).folder, mat_files(latest_idx).name);
load(target_file);
disp(['成功加载数据: ', mat_files(latest_idx).name]);

%% 2. 构建待绘制的结果集
% 将基线算法和本文算法放入 cell 数组进行统一循环
methods_cell = {result_B, result_A};
method_names = {'基线算法：联合加权优化', '本文算法：任务分配两阶段管控'};

%% 3. 绘制算法轨迹示意图
figure('Position', [100, 150, 1100, 500], 'Name', '算法轨迹对比示意图', 'Color', 'w');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for a = 1:length(methods_cell)
    curr_result = methods_cell{a};
    
    nexttile;
    hold on;
    grid on;
    axis equal;
    
    % --- 3.1 真实目标轨迹 ---
    h_true = [];
    h_true_start = [];
    h_true_end = [];
    n_target = size(Xreal_target_time, 1);
    
    for j = 1:n_target
        traj = Xreal_target_time{j,1};
        if isempty(traj)
            continue;
        end
        
        h = plot(traj(1,:), traj(3,:), 'k-', 'LineWidth', 1.6);
        valid_idx = find(~isnan(traj(1,:)));
        if ~isempty(valid_idx)
            idx_start = valid_idx(1);
            idx_end = valid_idx(end);
            hs = plot(traj(1,idx_start), traj(3,idx_start), '^', 'Color', [0 0 0], ...
                'MarkerFaceColor', 'y', 'MarkerSize', 8);
            he = plot(traj(1,idx_end), traj(3,idx_end), 'o', 'Color', [0 0 0], ...
                'MarkerFaceColor', 'c', 'MarkerSize', 7);
            if isempty(h_true_start), h_true_start = hs; end
            if isempty(h_true_end), h_true_end = he; end
        end
        if isempty(h_true), h_true = h; end
    end
    
    % --- 3.2 传感器轨迹 ---
    h_sensor = [];
    h_start = [];
    h_decision = [];
    h_end = [];
    
    % 检查是否存在轨迹数据（兼容不同算法的输出结构）
    if isfield(curr_result, 'Sensor_traj_vis') && ~isempty(curr_result.Sensor_traj_vis)
        Sensor_traj_vis = curr_result.Sensor_traj_vis;
        
        % 兼容没有 decision_log 的情况（第四章新算法默认每步决策）
        if isfield(curr_result, 'decision_log') && isfield(curr_result.decision_log, 'times')
            decision_times = curr_result.decision_log.times;
        else
            % 默认从 t=3 开始决策
            decision_times = 3:size(Sensor_traj_vis, 2);
        end
        
        for i = 1:size(Sensor_traj_vis, 3)
            xs = squeeze(Sensor_traj_vis(1,:,i));
            ys = squeeze(Sensor_traj_vis(2,:,i));
            
            % 剔除全零未赋值的坐标避免连线回原点
            valid_s = find(xs ~= 0 | ys ~= 0); 
            if isempty(valid_s)
                continue;
            end
            
            hs = plot(xs(valid_s), ys(valid_s), 'LineWidth', 2);
            hst = plot(xs(valid_s(1)), ys(valid_s(1)), '^', 'Color', [0 0 0], 'MarkerFaceColor', 'y', 'MarkerSize', 8);
            hse = plot(xs(valid_s(end)), ys(valid_s(end)), 'o', 'Color', [0 0 0], 'MarkerFaceColor', 'c', 'MarkerSize', 7);
            
            % 找到合法的决策绘制点
            valid_dt = decision_times(decision_times >= valid_s(1) & decision_times <= valid_s(end));
            hsd = plot(xs(valid_dt), ys(valid_dt), 'ks', ...
                'MarkerFaceColor', [0.95 0.6 0.2], 'MarkerSize', 5);
                
            if isempty(h_sensor), h_sensor = hs; end
            if isempty(h_start), h_start = hst; end
            if isempty(h_decision), h_decision = hsd; end
            if isempty(h_end), h_end = hse; end
        end
        
        legend_items = [h_true, h_true_start, h_true_end, h_sensor, h_start, h_decision, h_end];
        legend_labels = {'真实目标轨迹', '目标起点', '目标终点', '传感器轨迹', '传感器起点', '决策时刻位置', '传感器终点'};
    else
        % 若缺少传感器轨迹数据，则只画真实目标
        legend_items = [h_true, h_true_start, h_true_end];
        legend_labels = {'真实目标轨迹', '目标起点', '目标终点'};
        title([method_names{a}, ' (无传感器轨迹数据)'], 'FontSize', 12, 'Color', 'r');
    end
    
    % 图表装饰
    xlabel('X / m', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel('Y / m', 'Interpreter', 'latex', 'FontSize', 12);
    
    if isempty(get(gca, 'Title').String)
        title(method_names{a}, 'FontSize', 14);
    end
    
    valid_legend_idx = isgraphics(legend_items);
    legend(legend_items(valid_legend_idx), legend_labels(valid_legend_idx), ...
        'Location', 'best', 'FontSize', 10);
        
    % 提取并统一坐标轴（可根据 config.m 实际范围微调）
    try 
        xlim([X_range(1) X_range(2)]); 
        ylim([Y_range(1) Y_range(2)]); 
    catch
        xlim([-2000 2000]); 
        ylim([-2000 2000]);
    end
    hold off;
end

%% 4. 保存图表
saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_Trajectory.jpg'));
print(fullfile(fig_dir, 'Fig_Exp1_Trajectory.jpg'), '-djpeg', '-r300');
disp(['>>> 轨迹对比图表生成完毕，已保存至：', fig_dir]);