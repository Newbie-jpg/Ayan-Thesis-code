% MAIN_OSPA_CUSTOM_SCENE
% 支持两种模式：
% 1) single  : 单一轨迹 OSPA
% 2) compare : 双轨迹 OSPA 对比

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));
chat4_root = fileparts(this_dir);

%% 0) 运行模式
% 'single' 或 'compare'
run_mode = 'compare';

%% 1) 轨迹输入（按需替换）
% 支持 CoreData 风格，也支持 matlab.mat 这类变量名风格（Sensor_traj_vis / Xreal_time_target）
traj_input_A = 'CoreData_grid3norm_20260322_234505.mat';
traj_input_B = 'CoreData_cs3norm_20260322_234505.mat';
label_A = '本文算法';
label_B = '对比算法';

% 当 MAT 内含目标与传感器轨迹时，这两个输入可留空；
% 若 MAT 缺真值，可在此提供 Target_Init / Sensor_distr
Target_Init = [];
Sensor_distr = [];

%% 2) 可选参数
user_opt_base = struct();
user_opt_base.save_output = false;
user_opt_base.output_dir = fullfile(this_dir, 'Result_Fig_Ch4');
user_opt_base.monte_carlo = 100;   % 与 chat 4 主评估一致：多次蒙特卡洛取平均
user_opt_base.rng_seed = 10;       % 与 config.m 默认随机种子保持一致

% 如需覆盖默认评估参数可设置：
% user_opt_base.process_opt = struct('Vx_thre', 200, 'Vy_thre', 200, 'Vz_thre', 200);
% user_opt_base.control_opt = struct('match_threshold', 200, 'objective_mode', 'grid');

if ~exist(user_opt_base.output_dir, 'dir')
    mkdir(user_opt_base.output_dir);
end
save_stamp = datestr(now, 'yyyymmdd_HHMMSS');

%% 3) 运行并绘图
user_opt_A = user_opt_base;
user_opt_A.scenario_name = 'scene_A';
result_A = plot_ospa_from_scene_inputs(Target_Init, Sensor_distr, traj_input_A, user_opt_A);

if strcmpi(run_mode, 'single')
    % 单一轨迹模式
    figure('Color', 'w', 'Name', 'OSPA_SingleTraj');
    hold on; grid on;
    plot(1:numel(result_A.OSPA_avg), result_A.OSPA_avg, '-o', 'LineWidth', 1.6, 'MarkerSize', 4);
    xlabel('时刻 k');
    ylabel('OSPA');
    title(sprintf('单轨迹 OSPA (M=%d)', user_opt_base.monte_carlo));
    legend({label_A}, 'Location', 'best');
    hold off;

    save_file = fullfile(user_opt_base.output_dir, sprintf('OSPA_Single_%s.mat', save_stamp));
    save(save_file, 'traj_input_A', 'label_A', 'result_A', 'user_opt_base', 'run_mode');
    saveas(gcf, fullfile(user_opt_base.output_dir, sprintf('OSPA_Single_%s.png', save_stamp)));
    disp(['单轨迹 OSPA 已保存: ', save_file]);

elseif strcmpi(run_mode, 'compare')
    % 双轨迹对比模式
    user_opt_B = user_opt_base;
    user_opt_B.scenario_name = 'scene_B';
    result_B = plot_ospa_from_scene_inputs(Target_Init, Sensor_distr, traj_input_B, user_opt_B);

    N_plot = min(numel(result_A.OSPA_avg), numel(result_B.OSPA_avg));
    figure('Color', 'w', 'Name', 'OSPA_Compare_TwoTraj');
    hold on; grid on;
    plot(1:N_plot, result_A.OSPA_avg(1:N_plot), '-o', 'LineWidth', 1.6, 'MarkerSize', 4);
    plot(1:N_plot, result_B.OSPA_avg(1:N_plot), '-s', 'LineWidth', 1.6, 'MarkerSize', 4);
    xlabel('时刻 k');
    ylabel('OSPA');
    title(sprintf('两份轨迹 OSPA 对比 (M=%d)', user_opt_base.monte_carlo));
    legend({label_A, label_B}, 'Location', 'best');
    hold off;

    save_file = fullfile(user_opt_base.output_dir, sprintf('OSPA_Compare3norm_%s.mat', save_stamp));
    save(save_file, 'traj_input_A', 'traj_input_B', 'label_A', 'label_B', ...
        'result_A', 'result_B', 'user_opt_base', 'run_mode');
    saveas(gcf, fullfile(user_opt_base.output_dir, sprintf('OSPA_Compare3norm_%s.png', save_stamp)));
    disp(['双轨迹 OSPA 对比已保存: ', save_file]);
else
    error('run_mode 仅支持 ''single'' 或 ''compare''。当前值: %s', run_mode);
end
