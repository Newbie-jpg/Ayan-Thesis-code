% PICTURE_EXP1_OSPA_AND_DISCOVERY
% 生成：实验一（Exp1）OSPA距离曲线 + 发现目标统计柱状图（累计发现数量/平均发现延迟）
%
% 对应论文第四章 4.5 节实验一：
%  - OSPA 距离随时间变化曲线
%  - 任务分配效果：发现目标数量或发现延迟
%
% 使用方法：
%   先运行 chat4/01_Main_Env/main_exp1_performance.m 生成 Exp1_Result_*.mat
%   再运行本脚本生成图表

clear; close all; clc;

%% 0. 路径准备：以脚本位置为基准，避免工作目录不一致
script_dir = fileparts(mfilename('fullpath')); % chat4/03_Compare_Eval
chat4_root = fileparts(script_dir);            % chat4
data_dir = fullfile(chat4_root, '04_Data');
fig_dir = fullfile(script_dir, 'Result_Fig_Ch4');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

%% 1. 自动加载最新的实验一仿真结果
disp('>>> 正在寻找最新的 Exp1 仿真结果数据...');
mat_files = dir(fullfile(data_dir, 'Exp1_Result_*.mat'));
if isempty(mat_files)
    error('未在 %s 目录下找到 Exp1_Result_*.mat。请先运行 main_exp1_performance.m', data_dir);
end
[~, latest_idx] = max([mat_files.datenum]);
target_file = fullfile(mat_files(latest_idx).folder, mat_files(latest_idx).name);
load(target_file);
disp(['成功加载数据: ', mat_files(latest_idx).name]);

%% 2. OSPA 距离曲线
N_steps = length(result_A.OSPA_avg);
figure('Position', [150, 150, 900, 450], 'Color', 'w', 'Name', 'Exp1 OSPA Comparison');
hold on; grid on;

plot(1:N_steps, result_B.OSPA_avg, '--s', ...
    'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5, 'MarkerSize', 5, ...
    'MarkerFaceColor', [0, 0.4470, 0.7410]);
plot(1:N_steps, result_A.OSPA_avg, '-o', ...
    'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.5, 'MarkerSize', 5, ...
    'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);

set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 14);
xlabel('Time Step $k$', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('OSPA Distance (m)', 'Interpreter', 'latex', 'FontSize', 16);
title('OSPA Comparison: Joint Optimization vs. Task Allocation', ...
      'Interpreter', 'latex', 'FontSize', 16);
legend({'Algorithm B: Joint Optimization (Baseline)', ...
        'Algorithm A: Task Allocation & Grouping (Proposed)'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', 13);

% 标注：目标 1 出生时刻（若存在）
try
    tgt1_birth = Target_Init(7, 1);
    y_limits = ylim;
    plot([tgt1_birth, tgt1_birth], y_limits, 'k-.', 'LineWidth', 1.5);
    text(tgt1_birth + 1, y_limits(2) * 0.85, '\leftarrow Target 1 Born', ...
        'Interpreter', 'tex', 'FontSize', 12, 'FontWeight', 'bold');
catch
end

saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_OSPA.jpg'));
print(fullfile(fig_dir, 'Fig_Exp1_OSPA.jpg'), '-djpeg', '-r300');
%% 3. 发现目标统计（累计发现数量 & 平均发现延迟）
% discovery_cutoff：判定“发现”的距离阈值（与 ospa_dist 的 cutoff=120 相同更一致）
discovery_cutoff = 120;
% metric_type：'count' / 'delay' / 'both'
metric_type = 'both';

birth_times = Target_Init(7, :);
death_times = Target_Init(8, :);
n_target = size(Target_Init, 2);

methods = {result_A, result_B};
method_names = {'Algorithm A', 'Algorithm B'};
colors = {[0.8500, 0.3250, 0.0980], [0, 0.4470, 0.7410]};

discovery_time_all = zeros(2, n_target) * NaN;
avg_delay_all = zeros(1, 2) * NaN;
count_end_all = zeros(1, 2);

for mi = 1:2
    X_est_vis = methods{mi}.X_est_vis; % cell(N_steps,1)，每个元素为 6×num_est
    discovery_time = NaN(1, n_target);
    
    for tj = 1:n_target
        birth_k = birth_times(tj);
        death_k = death_times(tj);
        birth_k = max(1, round(birth_k));
        death_k = min(N_steps, round(death_k));
        if death_k < birth_k
            continue;
        end
        
        for k = birth_k:death_k
            % 真实目标在该时刻的位置（Xreal_time_target{k,1} 的第 tj 列）
            true_state_k = Xreal_time_target{k, 1};
            if size(true_state_k, 2) < tj
                continue;
            end
            if any(isnan(true_state_k([1,3,5], tj)))
                continue; % 目标尚未出现或已经消失
            end
            true_pos = true_state_k([1,3,5], tj);
            
            X_est_k = X_est_vis{k, 1};
            if isempty(X_est_k)
                continue;
            end
            if size(X_est_k, 1) < 5
                continue;
            end
            
            est_pos = X_est_k([1,3,5], :);
            valid_est = ~any(isnan(est_pos), 1);
            est_pos = est_pos(:, valid_est);
            if isempty(est_pos)
                continue;
            end
            
            min_dist = min(vecnorm(est_pos - true_pos, 2, 1));
            if min_dist <= discovery_cutoff
                discovery_time(tj) = k;
                break;
            end
        end
    end
    
    discovery_time_all(mi, :) = discovery_time;
    discovered_mask = ~isnan(discovery_time);
    count_end_all(mi) = sum(discovered_mask);
    
    if any(discovered_mask)
        avg_delay_all(mi) = mean(discovery_time(discovered_mask) - birth_times(discovered_mask));
    end
end

% 输出一些控制台信息
disp('>>> 发现目标统计结果（阈值：discovery_cutoff = 120m）');
disp(table(method_names', count_end_all', avg_delay_all', 'VariableNames', ...
    {'Method','DiscoveredCount','AvgDiscoveryDelay'}));

% 绘制柱状图
if strcmpi(metric_type, 'count')
    figure('Position', [200, 180, 650, 420], 'Color', 'w', 'Name', 'Discovery Count');
    b = bar(count_end_all);
    b.FaceColor = 'flat';
    b.CData = [colors{1}; colors{2}];
    set(gca, 'XTickLabel', method_names, 'TickLabelInterpreter', 'none', 'FontSize', 12);
    ylabel('Cumulative Discovered Targets', 'FontSize', 13);
    title(sprintf('Exp1 Discovery Count (cutoff=%dm)', discovery_cutoff), 'FontSize', 14);
    grid on;
    saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_DiscoveryCount.jpg'));
    print(fullfile(fig_dir, 'Fig_Exp1_DiscoveryCount.jpg'), '-djpeg', '-r300');
elseif strcmpi(metric_type, 'delay')
    figure('Position', [200, 180, 650, 420], 'Color', 'w', 'Name', 'Discovery Delay');
    b = bar(avg_delay_all);
    b.FaceColor = 'flat';
    b.CData = [colors{1}; colors{2}];
    set(gca, 'XTickLabel', method_names, 'TickLabelInterpreter', 'none', 'FontSize', 12);
    ylabel('Avg Discovery Delay (time steps)', 'FontSize', 13);
    title(sprintf('Exp1 Avg Discovery Delay (cutoff=%dm)', discovery_cutoff), 'FontSize', 14);
    grid on;
    saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_DiscoveryDelay.jpg'));
    print(fullfile(fig_dir, 'Fig_Exp1_DiscoveryDelay.jpg'), '-djpeg', '-r300');
else
    figure('Position', [120, 160, 1050, 420], 'Color', 'w', 'Name', 'Discovery Count & Delay');
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    
    nexttile;
    b1 = bar(count_end_all);
    b1.FaceColor = 'flat';
    b1.CData = [colors{1}; colors{2}];
    set(gca, 'XTickLabel', method_names, 'FontSize', 12);
    ylabel('Cumulative Discovered Targets', 'FontSize', 13);
    title(sprintf('Discovery Count (cutoff=%dm)', discovery_cutoff), 'FontSize', 14);
    grid on;
    
    nexttile;
    b2 = bar(avg_delay_all);
    b2.FaceColor = 'flat';
    b2.CData = [colors{1}; colors{2}];
    set(gca, 'XTickLabel', method_names, 'FontSize', 12);
    ylabel('Avg Discovery Delay (time steps)', 'FontSize', 13);
    title(sprintf('Avg Discovery Delay (cutoff=%dm)', discovery_cutoff), 'FontSize', 14);
    grid on;
    
    saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_DiscoveryCountAndDelay.jpg'));
    print(fullfile(fig_dir, 'Fig_Exp1_DiscoveryCountAndDelay.jpg'), '-djpeg', '-r300');
end

disp(['>>> 图表生成完毕，已保存至：', fig_dir]);

