% PICTURE_EXP1_SCENARIO
% 绘制实验一仿真场景示意图：
%   - 传感器初始位置与视域半径
%   - 目标真实轨迹、起点与终点
%
% 使用方法：
%   直接运行本脚本，将通过 config.m 动态读取配置并生成场景图

clear; close all; clc;

%% 0. 路径准备：以脚本位置为基准
script_dir = fileparts(mfilename('fullpath')); % chat4/03_Compare_Eval
chat4_root = fileparts(script_dir);            % chat4
fig_dir = fullfile(script_dir, 'Result_Fig_Ch4');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

%% 1. 初始化配置与动态生成轨迹
disp('>>> 正在读取 config.m 配置并生成场景轨迹...');

% 添加主程序路径以调用 config.m 和相关函数
main_env_dir = fullfile(chat4_root, '01_Main_Env');
addpath(main_env_dir);

% 加载全局参数与场景配置 (包含 N, Target_Init, Sensor_distr, 边界范围等)
config; 

% 直接调用 targetset 生成目标真实轨迹，无需读取跑完的 .mat 文件
[Xreal_target_time, ~] = targetset(N, Target_Init);
disp('>>> 成功基于 config 配置生成目标真实轨迹。');

%% 2. 开始绘制场景图
figure('Position', [150, 150, 800, 600], 'Name', 'Exp1 仿真场景', 'Color', 'w');
hold on;
grid on;
axis equal;

% 2.1 传感器视域半径（圆形）
for i = 1:N_sensor
    cx = Sensor_distr(i).location(1);
    cy = Sensor_distr(i).location(2);
    R_detect = Sensor_distr(i).R_detect;
    theta_c = linspace(0, 2*pi, 100);
    x_c = cx + R_detect * cos(theta_c);
    y_c = cy + R_detect * sin(theta_c);
    plot(x_c, y_c, '--', 'Color', [0.5 0.5 0.9], 'LineWidth', 0.8);
end

% 2.2 传感器初始位置
h_sensor = [];
for i = 1:N_sensor
    xs = Sensor_distr(i).location(1);
    ys = Sensor_distr(i).location(2);
    h = plot(xs, ys, '^', 'Color', [0 0.5 0], 'MarkerFaceColor', [0.5 1 0.5], 'MarkerSize', 10);
    text(xs + 80, ys + 80, ['S', num2str(i)], 'FontSize', 11, 'FontWeight', 'bold');
    if isempty(h_sensor), h_sensor = h; end
end

% 2.3 目标轨迹、起点、终点
n_target = size(Xreal_target_time, 1);
h_traj = [];
h_start = [];
h_end = [];
colors = lines(n_target);

for j = 1:n_target
    traj = Xreal_target_time{j, 1};  % 6 x N
    if isempty(traj)
        continue;
    end
    px = traj(1, :);  % x 位置
    py = traj(3, :);  % y 位置
    
    valid = ~isnan(px);
    if sum(valid) == 0, continue; end
    px = px(valid);
    py = py(valid);
    
    % 轨迹线
    h = plot(px, py, '-', 'Color', colors(j,:), 'LineWidth', 1.5);
    % 起点
    hs = plot(px(1), py(1), 'o', 'Color', colors(j,:), 'MarkerFaceColor', colors(j,:), 'MarkerSize', 8);
    % 终点
    he = plot(px(end), py(end), 's', 'Color', colors(j,:), 'MarkerFaceColor', colors(j,:), 'MarkerSize', 7);
    
    if isempty(h_traj), h_traj = h; end
    if isempty(h_start), h_start = hs; end
    if isempty(h_end), h_end = he; end
end

% 2.4 图表美化与修饰
xlabel('X / m', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Y / m', 'Interpreter', 'latex', 'FontSize', 14);
title('实验一仿真场景：传感器、视域与目标轨迹', 'FontSize', 15);
legend([h_sensor, h_traj, h_start, h_end], {'传感器初始位置', '目标真实轨迹', '目标起点', '目标终点'}, ...
    'Location', 'best', 'FontSize', 12);
xlim(X_range);
ylim(Y_range);

%% 3. 保存图表
saveas(gcf, fullfile(fig_dir, 'Fig_Exp1_Scenario.jpg'));
print(fullfile(fig_dir, 'Fig_Exp1_Scenario.jpg'), '-djpeg', '-r300');
disp(['>>> 场景图表生成完毕，已保存至：', fig_dir]);