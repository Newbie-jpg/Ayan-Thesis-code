% PICTURE_ROLE_SWITCH 实验一出图脚本：传感器任务角色状态切换甘特图
% 对应论文第四章 4.5 节：可视化分析 (Fig. 4.X)

clear; close all; clc;

%% 1. 数据加载与预处理
script_dir = fileparts(mfilename('fullpath')); % chat4/03_Compare_Eval
chat4_root = fileparts(script_dir);            % chat4
data_dir = fullfile(chat4_root, '04_Data');
fig_dir = fullfile(script_dir, 'Result_Fig_Ch4');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

disp('------------------------------------------------------');
disp('>>> 正在加载实验一结果数据 (.mat)...');
% [file, path] = uigetfile('04_Data/*.mat', '选择实验一仿真结果文件');
% if isequal(file, 0) || isequal(path, 0)
%    disp('用户取消选择。'); return;
% end
% load(fullfile(path, file));

% 自动寻找最新保存的 Exp1 文件
mat_files = dir(fullfile(data_dir, 'Exp1_Result_*.mat'));
if isempty(mat_files)
    error('未在 %s 目录下找到 Exp1_Result 文件。请先运行 main_exp1_performance.m', data_dir);
end
% 按时间排序，取最新的一个
[~, latest_idx] = max([mat_files.datenum]);
target_file = fullfile(mat_files(latest_idx).folder, mat_files(latest_idx).name);
load(target_file);
disp(['成功加载最新数据: ', mat_files(latest_idx).name]);


%% 2. 提取任务角色日志信息
% result_A.task_log 是一个 [N_steps x 1] 的 cell 数组，每步存放所有传感器的状态名
N_steps = length(result_A.task_log);
N_sensor = length(result_A.task_log{1});

% 创建一个数值矩阵用于绘图: Search 'S' -> 1, Tracking 'T' -> 2
role_num_matrix = zeros(N_sensor, N_steps);

for k = 1:N_steps
    current_roles_cell = result_A.task_log{k};
    for i = 1:N_sensor
        if strcmp(current_roles_cell{i}, 'S')
            role_num_matrix(i, k) = 1;
        elseif strcmp(current_roles_cell{i}, 'T')
            role_num_matrix(i, k) = 2;
        end
    end
end

%% 3. 提取目标的关键生命周期信息 (用于在图中添加标注)
% Target_Init_Ch4 格式为 N_attrs x N_targets
birth_times = Target_Init_Ch4(7, :); % 提取 t_start
death_times = Target_Init_Ch4(8, :); % 提取 t_end

% 提取 Target 1 (用于响应验证)
tgt1_birth = birth_times(1); % 应该是 20
tgt1_death = death_times(1); % 应该是 100

%% 4. 绘图：任务角色切换图 (imagesc 风格)
figure('Position', [100, 100, 1200, 600], 'Name', '传感器任务角色动态切换');
hold on;

% --- A. 主绘图区: 使用 imagesc 绘制热力甘特图 ---
% X轴：时间步 k，Y轴：传感器ID i
imagesc(1:N_steps, 1:N_sensor, role_num_matrix);

% --- B. 图形美化与坐标轴设置 ---
set(gca, 'YDir', 'normal'); % imagesc 默认颠倒Y轴，需手动正过来
set(gca, 'YTick', 1:N_sensor, 'YTickLabel', string(1:N_sensor)); % 显示所有传感器ID
axis tight; % 图像紧凑对齐

% 设置 LaTeX 字体 (学术风格)
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 14);
xlabel('Time Step $k$', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Sensor ID $i$', 'Interpreter', 'latex', 'FontSize', 16);
title('Dynamic Sensor Role Allocation (Experiment 1)', ...
      'Interpreter', 'latex', 'FontSize', 18);

% --- C. 设置自定义颜色图 ---
% 定义：Search 'S' (1) -> 柔和浅绿色; Tracking 'T' (2) -> 鲜艳橙红色
my_colormap = [
    0.70, 0.90, 0.70;  % Light Green (Search)
    1.00, 0.45, 0.40   % Bright Red/Orange (Tracking)
];
colormap(gca, my_colormap);
% 限制颜色范围为 [1, 2]，确保 colormap 匹配正确
caxis([1, 2]);

% --- D. 添加自定义图例 ---
% 建立哑点(dummy patches)用于生成图例
hold on;
h_s = patch(NaN, NaN, [0.70, 0.90, 0.70]); % S色块
h_t = patch(NaN, NaN, [1.00, 0.45, 0.40]); % T色块
legend([h_s, h_t], {'Search Node (S)', 'Tracking Node (T)'}, ...
       'Interpreter', 'latex', 'Location', 'bestoutside', 'FontSize', 14);
hold off;


%% 5. 添加环境变化事件标注 (关键：证明有效性)
hold on;

% A. 目标 1 在 k=20 出生
plot([tgt1_birth, tgt1_birth], [0.5, N_sensor + 0.5], 'k--', 'LineWidth', 2);
text(tgt1_birth + 2, N_sensor * 0.9, ...
     '\leftarrow Target 1 Born', ...
     'Interpreter', 'tex', 'FontSize', 14, 'Color', 'k', 'FontWeight', 'bold');

% B. 目标 1 在 k=100 消失 (如果仿真步长包含到了这里)
if tgt1_death <= N_steps
    plot([tgt1_death, tgt1_death], [0.5, N_sensor + 0.5], 'k--', 'LineWidth', 2);
    text(tgt1_death + 2, N_sensor * 0.9, ...
         '\leftarrow Target 1 Disappeared', ...
         'Interpreter', 'tex', 'FontSize', 14, 'Color', 'k', 'FontWeight', 'bold');
end

% C. 目标 2, 3, 4 在初始时刻 k=1 已出生 (可选标注)
plot([1, 1], [0.5, N_sensor + 0.5], ':', 'LineWidth', 1.5, 'Color', [0.3 0.3 0.3]);
text(1 + 2, N_sensor * 0.7, ...
     '\leftarrow Tgts 2,3,4 Active', ...
     'Interpreter', 'tex', 'FontSize', 14, 'Color', [0.3 0.3 0.3]);

hold off;

%% 6. 保存图片 (供论文使用)

% 保存为 fig 和 jpg (可调整 DPI)
saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_RoleSwitch.fig'));
print(fullfile(fig_dir, 'Fig_Exp1_RoleSwitch.jpg'), '-djpeg', '-r300'); % 300 DPI 学术质量

disp(['>>> 可视化图表生成完毕！已保存至: ', fullfile(fig_dir, 'Fig_Exp1_RoleSwitch.jpg')]);
disp('请运行 picture_exp1_ospa_and_discovery.m 来绘制 OSPA 与发现统计图。');