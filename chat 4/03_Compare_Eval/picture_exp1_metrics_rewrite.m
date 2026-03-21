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

    h_tar_start = gobjects(0);
    h_tar_end = gobjects(0);
    for j = 1:numel(Xreal_target_time)
        traj = Xreal_target_time{j};
        if isempty(traj) || size(traj, 1) < 3, continue; end
        valid_idx = find(~isnan(traj(1, :)) & ~isnan(traj(3, :)));
        if isempty(valid_idx), continue; end

        plot(traj(1, valid_idx), traj(3, valid_idx), 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        hs = plot(traj(1, valid_idx(1)), traj(3, valid_idx(1)), '^', ...
            'Color', [0 0 0], 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        he = plot(traj(1, valid_idx(end)), traj(3, valid_idx(end)), 'o', ...
            'Color', [0 0 0], 'MarkerFaceColor', 'c', 'MarkerSize', 7);

        if isempty(h_tar_start), h_tar_start = hs; end
        if isempty(h_tar_end), h_tar_end = he; end
    end

    Sensor_traj_vis = result_i.Sensor_traj_vis;
    num_sensors = size(Sensor_traj_vis, 3);
    h_sens_lines = gobjects(num_sensors, 1);
    h_sens_start = gobjects(0);
    h_sens_end = gobjects(0);
    h_decision = gobjects(0);

    d_times = get_decision_times(result_i, size(Sensor_traj_vis, 2));
    for i = 1:num_sensors
        xs = squeeze(Sensor_traj_vis(1, :, i));
        ys = squeeze(Sensor_traj_vis(2, :, i));
        h_sens_lines(i) = plot(xs, ys, 'LineWidth', 1.8);

        hst = plot(xs(1), ys(1), '^', 'Color', [0 0 0], ...
            'MarkerFaceColor', 'y', 'MarkerSize', 8);
        hse = plot(xs(end), ys(end), 'o', 'Color', [0 0 0], ...
            'MarkerFaceColor', 'c', 'MarkerSize', 7);

        if isempty(h_sens_start), h_sens_start = hst; end
        if isempty(h_sens_end), h_sens_end = hse; end

        if ~isempty(d_times)
            hsd = plot(xs(d_times), ys(d_times), 'ks', ...
                'MarkerFaceColor', [0.95 0.6 0.2], 'MarkerSize', 5);
            if isempty(h_decision), h_decision = hsd; end
        end
    end

    leg_h = [];
    leg_l = {};
    for i = 1:num_sensors
        leg_h = [leg_h; h_sens_lines(i)]; %#ok<AGROW>
        leg_l{end+1} = sprintf('传感器轨迹 %d', i); %#ok<AGROW>
    end
    if ~isempty(h_sens_start), leg_h = [leg_h; h_sens_start]; leg_l{end+1} = '传感器起点'; end %#ok<AGROW>
    if ~isempty(h_sens_end),   leg_h = [leg_h; h_sens_end];   leg_l{end+1} = '传感器终点'; end %#ok<AGROW>
    if ~isempty(h_tar_start),  leg_h = [leg_h; h_tar_start];  leg_l{end+1} = '目标起点'; end %#ok<AGROW>
    if ~isempty(h_tar_end),    leg_h = [leg_h; h_tar_end];    leg_l{end+1} = '目标终点'; end %#ok<AGROW>
    if ~isempty(h_decision),   leg_h = [leg_h; h_decision];   leg_l{end+1} = '决策时刻点'; end %#ok<AGROW>

    xlabel('X轴 坐标 / m');
    ylabel('Y轴 坐标 / m');
    title(['算法结果: ', result_i.name], 'Interpreter', 'none');
    legend(leg_h, leg_l, 'Location', 'eastoutside');
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
title('算法A/B OSPA曲线对比');
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
title('算法A/B 累计计算时间阶梯曲线');
legend(result_names, 'Location', 'northwest');
hold off;

%% 4) 平均发现延迟柱状图
[delay_A, found_A] = compute_discovery_delay(results{1}, Xreal_target_time, R_detect, k_confirm);
[delay_B, found_B] = compute_discovery_delay(results{2}, Xreal_target_time, R_detect, k_confirm);

avg_delay_A = mean(delay_A, 'omitnan');
avg_delay_B = mean(delay_B, 'omitnan');

figure('Color', 'w', 'Name', '平均发现延迟', 'Position', [220, 200, 700, 420]);
b = bar([avg_delay_A, avg_delay_B], 0.55);
b.FaceColor = 'flat';
b.CData = [0.2 0.5 0.9; 0.9 0.45 0.2];
set(gca, 'XTickLabel', {results{1}.name, results{2}.name}, 'XTickLabelRotation', 10);
ylabel('平均发现延迟 / 步');
title(sprintf('目标平均发现延迟对比（连续%d步覆盖判定）', k_confirm));
grid on;

fprintf('\n===== 目标发现统计 =====\n');
fprintf('判定规则：任一传感器 FoV 连续覆盖 >= %d 步，记为“发现”。\n', k_confirm);
fprintf('%s: 平均发现延迟 = %.2f 步，已发现目标 = %d/%d\n', ...
    results{1}.name, avg_delay_A, found_A, numel(Xreal_target_time));
fprintf('%s: 平均发现延迟 = %.2f 步，已发现目标 = %d/%d\n', ...
    results{2}.name, avg_delay_B, found_B, numel(Xreal_target_time));

%% 5) 每十步打印传感器状态 S/T
fprintf('\n===== 每10步传感器状态 (S/T) =====\n');
for a = 1:numel(results)
    result_i = results{a};
    fprintf('\n[%s]\n', result_i.name);
    role_log = get_role_log(result_i);
    if isempty(role_log)
        fprintf('  无角色日志（该结果未记录 S/T 状态）\n');
        continue;
    end
    N = numel(role_log);
    for t = 10:10:N
        roles_t = role_log{t};
        if isempty(roles_t)
            continue;
        end
        msg = sprintf('  t=%3d : ', t);
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
        d_times = 3:5:N;
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

function [delay_vec, found_cnt] = compute_discovery_delay(result_i, Xreal_target_time, R_detect, k_confirm)
    if nargin < 4 || isempty(k_confirm)
        k_confirm = 3;
    end
    k_confirm = max(1, round(k_confirm));

    Sensor_traj = result_i.Sensor_traj_vis;
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
