clear; clc; close all;

% 可选：手动指定结果文件；留空则自动读取 04_Data 下最新 Exp1_Result_*.mat
result_file = '';

this_dir = fileparts(mfilename('fullpath'));          % chat 4/03_Compare_Eval
chat4_root = fileparts(this_dir);                     % chat 4
data_dir = fullfile(chat4_root, '04_Data');

if isempty(result_file)
    files = dir(fullfile(data_dir, 'Exp1_Result_*.mat'));
    if isempty(files)
        error('未找到 Exp1_Result_*.mat，请先运行 main_exp1_performance.m');
    end
    [~, idx] = max([files.datenum]);
    result_file = fullfile(files(idx).folder, files(idx).name);
end

S = load(result_file);
if ~isfield(S, 'result_A') || ~isfield(S, 'result_B') || ~isfield(S, 'Xreal_target_time')
    error('结果文件缺少 result_A / result_B / Xreal_target_time 字段：%s', result_file);
end

result_A = S.result_A;
result_B = S.result_B;
Xreal_target_time = S.Xreal_target_time;
results = {result_A, result_B};

if ~isfield(results{1}, 'name') || isempty(results{1}.name), results{1}.name = '算法A'; end
if ~isfield(results{2}, 'name') || isempty(results{2}.name), results{2}.name = '算法B'; end
result_names = cellfun(@(r) r.name, results, 'UniformOutput', false);

R_detect = 500;
if isfield(S, 'Sensor_distr') && ~isempty(S.Sensor_distr) && isfield(S.Sensor_distr, 'R_detect')
    R_detect = S.Sensor_distr(1).R_detect;
end
k_confirm = 3;
if isfield(S, 'ch4_cfg') && isstruct(S.ch4_cfg) && ...
        isfield(S.ch4_cfg, 'k_steps') && ~isempty(S.ch4_cfg.k_steps)
    k_confirm = max(1, round(S.ch4_cfg.k_steps));
end

fprintf('已加载结果文件: %s\n', result_file);

%% 1) 目标与传感器轨迹（每个算法单独窗口）
for a = 1:numel(results)
    result_i = results{a};
    figure('Color', 'w', 'Name', result_i.name, 'Position', [150 + a*40, 120 + a*20, 980, 620]);
    hold on; grid on; axis equal;

    N_traj = size(result_i.Sensor_traj_vis, 2);
    decision_interval = 5;
    decision_times = 1:decision_interval:N_traj;

    h_true = gobjects(0);
    h_tgt_start = gobjects(0);
    h_tgt_end = gobjects(0);
    for j = 1:numel(Xreal_target_time)
        traj = Xreal_target_time{j};
        if isempty(traj) || size(traj, 1) < 3, continue; end
        valid_idx = find(~isnan(traj(1, :)));
        if isempty(valid_idx), continue; end

        ht = plot(traj(1, valid_idx), traj(3, valid_idx), 'k-', 'LineWidth', 1.6, 'HandleVisibility', 'off');
        hs = plot(traj(1, valid_idx(1)), traj(3, valid_idx(1)), '^k', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        he = plot(traj(1, valid_idx(end)), traj(3, valid_idx(end)), 'ok', 'MarkerFaceColor', 'c', 'MarkerSize', 7);

        if isempty(h_true), h_true = ht; end
        if isempty(h_tgt_start), h_tgt_start = hs; end
        if isempty(h_tgt_end), h_tgt_end = he; end
    end

    Sensor_traj_vis = result_i.Sensor_traj_vis;
    num_sensors = size(Sensor_traj_vis, 3);
    h_sens_lines = gobjects(1, num_sensors);
    h_sen_start = gobjects(0);
    h_sen_end = gobjects(0);
    h_decision = gobjects(0);
    color_list = lines(num_sensors);

    for i = 1:num_sensors
        xs = squeeze(Sensor_traj_vis(1, :, i));
        ys = squeeze(Sensor_traj_vis(2, :, i));
        valid_sen_idx = find(~isnan(xs) & ~isnan(ys));
        if isempty(valid_sen_idx)
            continue;
        end

        h_sens_lines(i) = plot(xs(valid_sen_idx), ys(valid_sen_idx), '-', ...
            'Color', color_list(i,:), 'LineWidth', 1.5);

        hst = plot(xs(valid_sen_idx(1)), ys(valid_sen_idx(1)), '^k', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        hse = plot(xs(valid_sen_idx(end)), ys(valid_sen_idx(end)), 'ok', 'MarkerFaceColor', 'c', 'MarkerSize', 7);

        valid_dec_idx = intersect(valid_sen_idx, decision_times);
        h_dec = plot(xs(valid_dec_idx), ys(valid_dec_idx), 'ks', ...
            'MarkerFaceColor', [0.95 0.6 0.2], 'MarkerSize', 5);

        if i == 1
            h_sen_start = hst;
            h_sen_end = hse;
            h_decision = h_dec;
        end
    end

    xlabel('X / m', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Y / m', 'FontSize', 11, 'FontWeight', 'bold');

    legend_handles = [];
    legend_labels = {};

    if ~isempty(h_true)
        legend_handles(end+1) = h_true; %#ok<AGROW>
        legend_labels{end+1} = '真实目标轨迹'; %#ok<AGROW>
    end

    for i = 1:num_sensors
        if i <= numel(h_sens_lines) && isgraphics(h_sens_lines(i))
            legend_handles(end+1) = h_sens_lines(i); %#ok<AGROW>
            legend_labels{end+1} = sprintf('传感器%d轨迹', i); %#ok<AGROW>
        end
    end

    if ~isempty(h_tgt_start) && isgraphics(h_tgt_start)
        legend_handles(end+1) = h_tgt_start; %#ok<AGROW>
        legend_labels{end+1} = '目标起点'; %#ok<AGROW>
    end
    if ~isempty(h_tgt_end) && isgraphics(h_tgt_end)
        legend_handles(end+1) = h_tgt_end; %#ok<AGROW>
        legend_labels{end+1} = '目标终点'; %#ok<AGROW>
    end
    if ~isempty(h_sen_start) && isgraphics(h_sen_start)
        legend_handles(end+1) = h_sen_start; %#ok<AGROW>
        legend_labels{end+1} = '传感器起点'; %#ok<AGROW>
    end
    if ~isempty(h_sen_end) && isgraphics(h_sen_end)
        legend_handles(end+1) = h_sen_end; %#ok<AGROW>
        legend_labels{end+1} = '传感器终点'; %#ok<AGROW>
    end
    if ~isempty(h_decision) && isgraphics(h_decision)
        legend_handles(end+1) = h_decision; %#ok<AGROW>
        legend_labels{end+1} = '决策时刻'; %#ok<AGROW>
    end

    legend(legend_handles, legend_labels, 'Location', 'best', 'FontSize', 10);
    xlim([-2000, 2000]);
    ylim([-2000, 2000]);
    hold off;
end

%% 2) OSPA 曲线
figure('Color', 'w', 'Name', 'OSPA对比', 'Position', [180, 150, 900, 420]);
hold on; grid on;
style_list = {'-o', '-s'};
color_list = lines(numel(results));
for a = 1:numel(results)
    ospa = results{a}.OSPA_avg;
    plot(1:numel(ospa), ospa, style_list{a}, 'Color', color_list(a,:), ...
        'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', color_list(a,:));
end
xlabel('时刻 k');
ylabel('OSPA');
legend(result_names, 'Location', 'best');
hold off;

%% 3) 累计时间阶梯曲线（每步耗时累加）
figure('Color', 'w', 'Name', '累计计算时间对比', 'Position', [180, 170, 900, 420]);
hold on; grid on;
for a = 1:numel(results)
    step_t = get_step_time_series(results{a});
    cum_t = cumsum(step_t);
    stairs(1:numel(cum_t), cum_t, 'LineWidth', 1.8, 'Color', color_list(a,:));
end
xlabel('时刻 k');
ylabel('累计计算时间 / s');
legend(result_names, 'Location', 'northwest');
hold off;

%% 4) 平均发现延迟柱状图
[avg_delay_A, found_A] = compute_discovery_delay_mc(results{1}, Xreal_target_time, R_detect, k_confirm);
[avg_delay_B, found_B] = compute_discovery_delay_mc(results{2}, Xreal_target_time, R_detect, k_confirm);

% 可改动的绘图变量（按需修改）
delay_plot_cfg = struct();
delay_plot_cfg.figure_name = '平均发现延迟';
delay_plot_cfg.figure_position = [220, 200, 700, 420];
delay_plot_cfg.scale_factor = 4.0; % 若不需要放大，改为 1
delay_plot_cfg.bar_width = 0.55;
delay_plot_cfg.bar_colors = [0.2 0.5 0.9; 0.9 0.45 0.2];
delay_plot_cfg.x_labels = {'本文算法', '基线算法'};
delay_plot_cfg.y_label = '平均发现延迟 / 步';

% 创建图形窗口
figure('Color', 'w', 'Name', delay_plot_cfg.figure_name, ...
    'Position', delay_plot_cfg.figure_position);

% 绘制柱状图
b = bar([avg_delay_A, avg_delay_B] * delay_plot_cfg.scale_factor, delay_plot_cfg.bar_width);
b.FaceColor = 'flat';
b.CData = delay_plot_cfg.bar_colors;

% 设置 X 轴标签，修复斜体问题，字体和字号保持 MATLAB 默认
set(gca, 'XTickLabel', delay_plot_cfg.x_labels, ...
    'XTickLabelRotation', 0, ...           % 标签较少，改为 0 度水平显示更清晰
    'TickLabelInterpreter', 'none', ...
    'FontAngle', 'normal');                % 强制字体正常显示（非斜体）

% 设置 Y 轴标签（同步字体格式）
ylabel(delay_plot_cfg.y_label, 'FontAngle', 'normal', 'FontName', 'Microsoft YaHei', 'FontSize', 12);

grid on;

% 精简后的控制台输出，仅展示存在的延迟数据
fprintf('\n===== 目标平均发现延迟对比 =====\n');
fprintf('%s: 平均发现延迟 = %.2f 步\n', results{1}.name, avg_delay_A);
fprintf('%s: 平均发现延迟 = %.2f 步\n', results{2}.name, avg_delay_B);

%% 5) 按决策时刻打印传感器状态 S/T（默认 1,6,11,16,...）
fprintf('\n===== 决策时刻传感器状态 (S/T) =====\n');
for a = 1:numel(results)
    result_i = results{a};
    fprintf('\n[%s]\n', result_i.name);
    role_log = get_role_log(result_i);
    if isempty(role_log)
        fprintf('  无角色日志（该结果未记录 S/T 状态）\n');
        continue;
    end
    N = numel(role_log);
    decision_times = get_decision_times(result_i, N);
    display_times = map_decision_times_for_display(decision_times);
    for idx_t = 1:numel(decision_times)
        t = decision_times(idx_t);
        t_show = display_times(idx_t);
        roles_t = role_log{t};
        if isempty(roles_t)
            continue;
        end
        msg = sprintf('  t=%3d (原始k=%3d) : ', t_show, t);
        for i = 1:numel(roles_t)
            msg = [msg, sprintf('S%d=%s ', i, char(roles_t{i}))]; %#ok<AGROW>
        end
        fprintf('%s\n', strtrim(msg));
    end
end

function d_times = get_decision_times(result_i, N)
    d_times = [];
    if isfield(result_i, 'decision_log') && isstruct(result_i.decision_log) && ...
            isfield(result_i.decision_log, 'times') && ~isempty(result_i.decision_log.times)
        d_times = unique(round(result_i.decision_log.times));
        d_times = d_times(d_times >= 1 & d_times <= N);
    else
        d_times = 1:5:N;
    end
end

function disp_times = map_decision_times_for_display(raw_times)
    disp_times = raw_times;
    if isempty(raw_times)
        return;
    end
    % PROCESS 的控制循环从 t=3 起步；显示时换算为 1,6,11,16,...
    if raw_times(1) == 3
        disp_times = raw_times - 2;
    end
end

function step_t = get_step_time_series(result_i)
    if isfield(result_i, 'step_time_series') && ~isempty(result_i.step_time_series)
        step_t = result_i.step_time_series(:).';
        return;
    end
    if isfield(result_i, 'decision_log') && isstruct(result_i.decision_log) && ...
            isfield(result_i.decision_log, 'step_time_series') && ~isempty(result_i.decision_log.step_time_series)
        step_t = result_i.decision_log.step_time_series(:).';
        return;
    end
    if isfield(result_i, 'OSPA_avg')
        N = numel(result_i.OSPA_avg);
    else
        N = 100;
    end
    if isfield(result_i, 'Time_avg') && ~isempty(result_i.Time_avg)
        step_t = zeros(1, N);
        step_t(3:end) = result_i.Time_avg;
    else
        step_t = zeros(1, N);
    end
end

function role_log = get_role_log(result_i)
    role_log = [];
    if isfield(result_i, 'task_log') && ~isempty(result_i.task_log)
        role_log = result_i.task_log;
        return;
    end
    if isfield(result_i, 'decision_log') && isstruct(result_i.decision_log) && ...
            isfield(result_i.decision_log, 'role_log') && ~isempty(result_i.decision_log.role_log)
        role_log = result_i.decision_log.role_log;
        return;
    end
end

function [avg_delay, avg_found_cnt] = compute_discovery_delay_mc(result_i, Xreal_target_time, R_detect, k_confirm)
    traj_list = {};
    if isfield(result_i, 'mc_sensor_traj') && ~isempty(result_i.mc_sensor_traj)
        traj_list = result_i.mc_sensor_traj;
    elseif isfield(result_i, 'Sensor_traj_vis') && ~isempty(result_i.Sensor_traj_vis)
        traj_list = {result_i.Sensor_traj_vis};
    end

    if isempty(traj_list)
        avg_delay = NaN;
        avg_found_cnt = 0;
        return;
    end

    K = numel(traj_list);
    delay_avg_each = nan(1, K);
    found_each = zeros(1, K);
    for kk = 1:K
        [delay_vec, found_cnt] = compute_discovery_delay_single(traj_list{kk}, Xreal_target_time, R_detect, k_confirm);
        delay_avg_each(kk) = mean(delay_vec, 'omitnan');
        found_each(kk) = found_cnt;
    end

    avg_delay = mean(delay_avg_each, 'omitnan');
    avg_found_cnt = mean(found_each);
end

function [delay_vec, found_cnt] = compute_discovery_delay_single(Sensor_traj, Xreal_target_time, R_detect, k_confirm)
    if nargin < 4 || isempty(k_confirm)
        k_confirm = 3;
    end
    k_confirm = max(1, round(k_confirm));

    N = size(Sensor_traj, 2);
    Ns = size(Sensor_traj, 3);
    Nt = numel(Xreal_target_time);
    delay_vec = nan(1, Nt);
    found_cnt = 0;

    for j = 1:Nt
        traj = Xreal_target_time{j};
        if isempty(traj) || size(traj, 1) < 5
            continue;
        end
        valid_idx = find(~isnan(traj(1, :)) & ~isnan(traj(3, :)));
        if isempty(valid_idx)
            continue;
        end
        t_birth = valid_idx(1);
        t_death = valid_idx(end);
        t_found = NaN;
        cover_streak = 0;

        for t = t_birth:min(t_death, N)
            tar = [traj(1, t); traj(3, t); traj(5, t)];
            covered_any = false;
            for i = 1:Ns
                s = Sensor_traj(:, t, i);
                if norm(s(1:3) - tar) <= R_detect
                    covered_any = true;
                    break;
                end
            end
            if covered_any
                cover_streak = cover_streak + 1;
            else
                cover_streak = 0;
            end

            if cover_streak >= k_confirm
                % 发现时刻记为“达到连续k步判定”的确认时刻
                t_found = t;
                break;
            end
        end

        if ~isnan(t_found)
            delay_vec(j) = t_found - t_birth;
            found_cnt = found_cnt + 1;
        else
            % 若未发现，按“到仿真末尾仍未发现”处理，便于平均值可比较
            delay_vec(j) = N - t_birth + 1;
        end
    end
end
