% REPLOT_TWO_COREDATA_WITH_VALIDATION
% 读取两份 CoreData 轨迹，先做格式校验，再基于固定轨迹重放滤波重新绘制 OSPA 对比图。

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));      % chat 4/03_Compare_Eval
chat4_root = fileparts(this_dir);                 % chat 4
data_dir = fullfile(chat4_root, '04_Data');
output_dir = fullfile(this_dir, 'Result_Fig_Ch4');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save_stamp = datestr(now, 'yyyymmdd_HHMMSS');

%% 1) 输入文件（按需替换）
traj_input_A = fullfile(data_dir, 'CoreData_grid3search_20260322_234728.mat');
% 单轨迹模式：将 traj_input_B 置为空 [] 或 ''
%traj_input_B = fullfile(data_dir, 'CoreData_cs3norm_20260323_154003.mat');
traj_input_B = [];
label_A = '算法A (my)';
label_B = '算法B (cs)';

%% 2) 先做格式校验
report_A = validate_coredata_file(traj_input_A, 'A');
is_single_mode = isempty(traj_input_B) || ...
    (ischar(traj_input_B) && isempty(strtrim(traj_input_B))) || ...
    (isstring(traj_input_B) && strlength(traj_input_B) == 0);
if ~is_single_mode
    report_B = validate_coredata_file(traj_input_B, 'B');
end

fprintf('\n========== CoreData 格式校验通过 ==========\n');
fprintf('A: %s | N=%d, Ns=%d\n', report_A.file_name, report_A.N, report_A.Ns);
if ~is_single_mode
    fprintf('B: %s | N=%d, Ns=%d\n', report_B.file_name, report_B.N, report_B.Ns);
end
fprintf('===========================================\n');

if ~is_single_mode && report_A.N ~= report_B.N
    warning('两份轨迹时间长度不同（A=%d, B=%d）。绘图会取共同最短长度。', report_A.N, report_B.N);
end

%% 3) 重放评估参数
user_opt_base = struct();
user_opt_base.save_output = false;
user_opt_base.monte_carlo = 100;   % 与主实验一致可改
user_opt_base.rng_seed = 10;
% 如需覆盖评估参数，可打开下面两行：
% user_opt_base.process_opt = struct('Ps', 0.99, 'Vx_thre', 200, 'Vy_thre', 200, 'Vz_thre', 200);
% user_opt_base.control_opt = struct('match_threshold', 200, 'objective_mode', 'grid');

%% 4) 运行重放评估（复用 plot_ospa_from_scene_inputs）
opt_A = user_opt_base;
opt_A.scenario_name = 'coredata_A';
res_A = plot_ospa_from_scene_inputs([], [], traj_input_A, opt_A);
result_A = res_A; %#ok<NASGU>

if is_single_mode
    %% 5) 单轨迹 OSPA 曲线
    N_plot = numel(res_A.OSPA_avg);
    figure('Color', 'w', 'Name', 'OSPA_Single_Replayed_CoreData');
    hold on; grid on;
    plot(1:N_plot, res_A.OSPA_avg, '-o', 'LineWidth', 1.6, 'MarkerSize', 4);
    xlabel('时刻 k');
    ylabel('OSPA');
    title(sprintf('单轨迹重放 OSPA (M=%d)', user_opt_base.monte_carlo));
    legend({label_A}, 'Location', 'best');
    hold off;

    mat_file = fullfile(output_dir, sprintf('OSPA_Single3trace_%s.mat', save_stamp));
    save(mat_file, 'traj_input_A', 'label_A', 'result_A', 'user_opt_base');
    out_png = fullfile(output_dir, sprintf('OSPA_Single3trace_%s.png', save_stamp));
    out_fig = fullfile(output_dir, sprintf('OSPA_Single3trace_%s.fig', save_stamp));
    saveas(gcf, out_png);
    saveas(gcf, out_fig);
    fprintf('单轨迹结果已保存:\n  %s\n  %s\n  %s\n', mat_file, out_png, out_fig);
else
    opt_B = user_opt_base;
    opt_B.scenario_name = 'coredata_B';
    res_B = plot_ospa_from_scene_inputs([], [], traj_input_B, opt_B);
    result_B = res_B; %#ok<NASGU>

    %% 5) 双算法对比图（重放后的 OSPA）
    N_plot = min(numel(res_A.OSPA_avg), numel(res_B.OSPA_avg));
    figure('Color', 'w', 'Name', 'OSPA_Compare_Replayed_CoreData');
    hold on; grid on;
    plot(1:N_plot, res_A.OSPA_avg(1:N_plot), '-o', 'LineWidth', 1.6, 'MarkerSize', 4);
    plot(1:N_plot, res_B.OSPA_avg(1:N_plot), '-s', 'LineWidth', 1.6, 'MarkerSize', 4);
    xlabel('时刻 k');
    ylabel('OSPA');
    title(sprintf('两份 CoreData 重放 OSPA 对比 (M=%d)', user_opt_base.monte_carlo));
    legend({label_A, label_B}, 'Location', 'best');
    hold off;

    % 结果保存（字段格式对齐 OSPA_Compare_*.mat）
    mat_file = fullfile(output_dir, sprintf('OSPA_Compare3norm_%s.mat', save_stamp));
    save(mat_file, 'traj_input_A', 'traj_input_B', 'label_A', 'label_B', ...
        'result_A', 'result_B', 'user_opt_base');

    out_png = fullfile(output_dir, sprintf('OSPA_Compare3search_%s.png', save_stamp));
    out_fig = fullfile(output_dir, sprintf('OSPA_Compare3search_%s.fig', save_stamp));
    saveas(gcf, out_png);
    saveas(gcf, out_fig);
    fprintf('对比结果已保存:\n  %s\n  %s\n  %s\n', mat_file, out_png, out_fig);
end

%% ===== Local functions =====
function report = validate_coredata_file(file_path, alias)
    if ~isfile(file_path)
        error('输入文件不存在 (%s): %s', alias, file_path);
    end

    S = load(file_path);
    required_fields = {'Sensor_Traj', 'True_Target_Time'};
    for i = 1:numel(required_fields)
        f = required_fields{i};
        if ~isfield(S, f) || isempty(S.(f))
            error('文件 %s 缺少必要字段 %s: %s', alias, f, file_path);
        end
    end

    Sensor_Traj = S.Sensor_Traj;
    if ndims(Sensor_Traj) ~= 3
        error('文件 %s 的 Sensor_Traj 维度错误，应为 3D (2/3 x N x Ns)。', alias);
    end
    if ~(size(Sensor_Traj, 1) == 2 || size(Sensor_Traj, 1) == 3)
        error('文件 %s 的 Sensor_Traj 第一维应为 2 或 3，当前为 %d。', alias, size(Sensor_Traj, 1));
    end
    if size(Sensor_Traj, 2) < 3
        error('文件 %s 的 Sensor_Traj 时间长度过短，N=%d。', alias, size(Sensor_Traj, 2));
    end
    if size(Sensor_Traj, 3) < 1
        error('文件 %s 的 Sensor_Traj 传感器数量非法，Ns=%d。', alias, size(Sensor_Traj, 3));
    end
    if any(~isfinite(Sensor_Traj(:)))
        warning('文件 %s 的 Sensor_Traj 存在 NaN/Inf，后续重放可能受影响。', alias);
    end

    True_Target_Time = S.True_Target_Time;
    if ~iscell(True_Target_Time)
        error('文件 %s 的 True_Target_Time 必须是 cell(N,1)。', alias);
    end
    if numel(True_Target_Time) ~= size(Sensor_Traj, 2)
        error('文件 %s 维度不匹配：numel(True_Target_Time)=%d, Sensor_Traj 时间长度=%d。', ...
            alias, numel(True_Target_Time), size(Sensor_Traj, 2));
    end
    for t = 1:numel(True_Target_Time)
        Xt = True_Target_Time{t};
        if isempty(Xt)
            continue;
        end
        if ~isnumeric(Xt) || size(Xt, 1) < 6
            error('文件 %s 的 True_Target_Time{%d} 维度错误，应至少为 6xNt。', alias, t);
        end
    end

    report = struct();
    report.file_name = file_path;
    report.N = size(Sensor_Traj, 2);
    report.Ns = size(Sensor_Traj, 3);
    if isfield(S, 'OSPA_Metric') && ~isempty(S.OSPA_Metric) && isnumeric(S.OSPA_Metric)
        report.OSPA_Metric = S.OSPA_Metric(:).';
    else
        report.OSPA_Metric = [];
    end
end
