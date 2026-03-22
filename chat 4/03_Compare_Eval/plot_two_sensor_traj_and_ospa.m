% PLOT_TWO_SENSOR_TRAJ_AND_OSPA
% 读取：
% - 两份传感器轨迹数据（A/B）
% - 一份目标轨迹数据
% - 一份 OSPA 数据
% 并绘制三张图：
% 1) 传感器A轨迹 + 目标轨迹
% 2) 传感器B轨迹 + 目标轨迹
% 3) OSPA 曲线图（支持单曲线或多曲线）

clear; clc; close all;

%% ===== 1) 输入路径（按需修改） =====
target_file   = 'C:\Users\Scarecrow\Desktop\Ayan Thesis code\chat 4\04_Data\Xreal_target_time.mat';
sensor_file_A = 'C:\Users\Scarecrow\Desktop\Ayan Thesis code\chat 4\04_Data\CoreData_grid3norm_20260322_234505.mat';
sensor_file_B = 'C:\Users\Scarecrow\Desktop\Ayan Thesis code\chat 4\04_Data\CoreData_cs3norm_20260322_234505.mat';
ospa_file     = 'C:\Users\Scarecrow\Desktop\Ayan Thesis code\chat 4\03_Compare_Eval\Result_Fig_Ch4\OSPA_Compare3norm_20260322_234931.mat';

label_A = '算法A';
label_B = '算法B';

save_fig = false;
output_dir = 'C:\Users\Scarecrow\Desktop\Ayan Thesis code\chat 4\03_Compare_Eval\Result_Fig_Ch4';

%% ===== 2) 读取数据 =====
S_tar = load(target_file);
S_A = load(sensor_file_A);
S_B = load(sensor_file_B);
S_ospa = load(ospa_file);

True_Target_Traj = resolve_target_traj(S_tar);
Sensor_Traj_A = resolve_sensor_traj(S_A);
Sensor_Traj_B = resolve_sensor_traj(S_B);
[ospa_series, ospa_labels] = resolve_ospa_series(S_ospa);

%% ===== 3) 绘图：A 传感器轨迹 + 目标轨迹 =====
figure('Color', 'w', 'Name', ['轨迹图_', label_A]);
plot_target_and_sensor(True_Target_Traj, Sensor_Traj_A, label_A);

%% ===== 4) 绘图：B 传感器轨迹 + 目标轨迹 =====
figure('Color', 'w', 'Name', ['轨迹图_', label_B]);
plot_target_and_sensor(True_Target_Traj, Sensor_Traj_B, label_B);

%% ===== 5) 绘图：OSPA 曲线 =====
figure('Color', 'w', 'Name', 'OSPA 曲线');
hold on; grid on;
style_list = {'-o', '-s', '-^', '-d'};
color_list = lines(max(2, numel(ospa_series)));
for i = 1:numel(ospa_series)
    y = ospa_series{i}(:).';
    plot(1:numel(y), y, style_list{1 + mod(i-1, numel(style_list))}, ...
        'LineWidth', 1.5, 'Color', color_list(i,:), 'MarkerSize', 4);
end
xlabel('时刻 k');
ylabel('OSPA');
title('OSPA 曲线');
legend(ospa_labels, 'Location', 'best');
hold off;

%% ===== 6) 可选保存 =====
if save_fig
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    saveas(findobj('Type', 'figure', 'Name', ['轨迹图_', label_A]), fullfile(output_dir, sprintf('traj_%s_%s.png', sanitize_name(label_A), stamp)));
    saveas(findobj('Type', 'figure', 'Name', ['轨迹图_', label_B]), fullfile(output_dir, sprintf('traj_%s_%s.png', sanitize_name(label_B), stamp)));
    saveas(findobj('Type', 'figure', 'Name', 'OSPA 曲线'), fullfile(output_dir, sprintf('ospa_%s.png', stamp)));
end

%% ===== Local functions =====
function True_Target_Traj = resolve_target_traj(S)
    if isfield(S, 'True_Target_Traj') && ~isempty(S.True_Target_Traj)
        True_Target_Traj = S.True_Target_Traj;
        return;
    end
    if isfield(S, 'Xreal_target_time') && ~isempty(S.Xreal_target_time)
        True_Target_Traj = S.Xreal_target_time;
        return;
    end
    if isfield(S, 'True_Target_Time') && ~isempty(S.True_Target_Time)
        True_Target_Traj = time_to_traj(S.True_Target_Time);
        return;
    end
    if isfield(S, 'Xreal_time_target') && ~isempty(S.Xreal_time_target)
        True_Target_Traj = time_to_traj(S.Xreal_time_target);
        return;
    end
    error('目标文件中未找到可识别的目标轨迹字段。');
end

function Sensor_Traj = resolve_sensor_traj(S)
    if isfield(S, 'Sensor_Traj') && ~isempty(S.Sensor_Traj)
        Sensor_Traj = S.Sensor_Traj;
    elseif isfield(S, 'Sensor_traj_vis') && ~isempty(S.Sensor_traj_vis)
        Sensor_Traj = S.Sensor_traj_vis;
    elseif isfield(S, 'Sensor_traj') && ~isempty(S.Sensor_traj)
        Sensor_Traj = S.Sensor_traj;
    else
        error('传感器文件中未找到可识别的 Sensor_Traj 字段。');
    end

    if ndims(Sensor_Traj) ~= 3
        error('Sensor_Traj 应为 3D 数组 (2/3 x N x Ns)。');
    end
    if size(Sensor_Traj, 1) == 2
        Sensor_Traj = cat(1, Sensor_Traj, zeros(1, size(Sensor_Traj,2), size(Sensor_Traj,3)));
    end
    if size(Sensor_Traj, 1) ~= 3
        error('Sensor_Traj 第一维应为 3。');
    end
end

function [series, labels] = resolve_ospa_series(S)
    series = {};
    labels = {};

    if isfield(S, 'OSPA_Metric') && ~isempty(S.OSPA_Metric)
        series = {S.OSPA_Metric(:).'};
        labels = {'OSPA'};
        return;
    end
    if isfield(S, 'OSPA_avg') && ~isempty(S.OSPA_avg)
        series = {S.OSPA_avg(:).'};
        labels = {'OSPA_avg'};
        return;
    end
    if isfield(S, 'result_A') && isfield(S.result_A, 'OSPA_avg')
        series{end+1} = S.result_A.OSPA_avg(:).'; %#ok<AGROW>
        if isfield(S.result_A, 'name') && ~isempty(S.result_A.name)
            labels{end+1} = char(S.result_A.name); %#ok<AGROW>
        else
            labels{end+1} = 'result_A'; %#ok<AGROW>
        end
    end
    if isfield(S, 'result_B') && isfield(S.result_B, 'OSPA_avg')
        series{end+1} = S.result_B.OSPA_avg(:).'; %#ok<AGROW>
        if isfield(S.result_B, 'name') && ~isempty(S.result_B.name)
            labels{end+1} = char(S.result_B.name); %#ok<AGROW>
        else
            labels{end+1} = 'result_B'; %#ok<AGROW>
        end
    end
    if ~isempty(series)
        return;
    end
    if isfield(S, 'results') && isstruct(S.results)
        R = S.results;
        for i = 1:numel(R)
            if isfield(R(i), 'OSPA_avg') && ~isempty(R(i).OSPA_avg)
                series{end+1} = R(i).OSPA_avg(:).'; %#ok<AGROW>
                if isfield(R(i), 'name') && ~isempty(R(i).name)
                    labels{end+1} = char(R(i).name); %#ok<AGROW>
                elseif isfield(R(i), 'tag') && ~isempty(R(i).tag)
                    labels{end+1} = char(R(i).tag); %#ok<AGROW>
                else
                    labels{end+1} = sprintf('OSPA_%d', i); %#ok<AGROW>
                end
            end
        end
    end
    if isempty(series)
        error('OSPA 文件中未找到可识别的 OSPA 字段。');
    end
end

function plot_target_and_sensor(True_Target_Traj, Sensor_Traj, fig_title_text)
    hold on; grid on; axis equal;

    num_targets = numel(True_Target_Traj);
    num_sensors = size(Sensor_Traj, 3);

    h_true = [];
    h_tgt_start = [];
    h_tgt_end = [];
    for j = 1:num_targets
        traj = True_Target_Traj{j,1};
        if isempty(traj) || size(traj,1) < 3
            continue;
        end
        valid_idx = find(~isnan(traj(1,:)) & ~isnan(traj(3,:)));
        if isempty(valid_idx)
            continue;
        end
        ht = plot(traj(1, valid_idx), traj(3, valid_idx), 'k-', 'LineWidth', 1.5);
        hs = plot(traj(1, valid_idx(1)), traj(3, valid_idx(1)), '^k', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        he = plot(traj(1, valid_idx(end)), traj(3, valid_idx(end)), 'ok', 'MarkerFaceColor', 'c', 'MarkerSize', 7);
        if isempty(h_true), h_true = ht; end
        if isempty(h_tgt_start), h_tgt_start = hs; end
        if isempty(h_tgt_end), h_tgt_end = he; end
    end

    color_list = lines(num_sensors);
    h_sensor = gobjects(1, num_sensors);
    h_sen_start = [];
    h_sen_end = [];
    for i = 1:num_sensors
        xs = squeeze(Sensor_Traj(1,:,i));
        ys = squeeze(Sensor_Traj(2,:,i));
        valid = find(~isnan(xs) & ~isnan(ys));
        if isempty(valid), continue; end
        h_sensor(i) = plot(xs(valid), ys(valid), '-', 'Color', color_list(i,:), 'LineWidth', 1.6);
        hs = plot(xs(valid(1)), ys(valid(1)), '^k', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        he = plot(xs(valid(end)), ys(valid(end)), 'ok', 'MarkerFaceColor', 'c', 'MarkerSize', 7);
        if isempty(h_sen_start), h_sen_start = hs; end
        if isempty(h_sen_end), h_sen_end = he; end
    end

    xlabel('X / m');
    ylabel('Y / m');
    title(['传感器与目标轨迹 - ', fig_title_text]);
    xlim([-2000, 2000]);
    ylim([-2000, 2000]);

    legend_handles = [];
    legend_labels = {};
    if ~isempty(h_true), legend_handles(end+1) = h_true; legend_labels{end+1} = '目标轨迹'; end %#ok<AGROW>
    for i = 1:num_sensors
        if isgraphics(h_sensor(i))
            legend_handles(end+1) = h_sensor(i); %#ok<AGROW>
            legend_labels{end+1} = sprintf('传感器%d轨迹', i); %#ok<AGROW>
        end
    end
    if ~isempty(h_tgt_start), legend_handles(end+1) = h_tgt_start; legend_labels{end+1} = '目标起点'; end %#ok<AGROW>
    if ~isempty(h_tgt_end), legend_handles(end+1) = h_tgt_end; legend_labels{end+1} = '目标终点'; end %#ok<AGROW>
    if ~isempty(h_sen_start), legend_handles(end+1) = h_sen_start; legend_labels{end+1} = '传感器起点'; end %#ok<AGROW>
    if ~isempty(h_sen_end), legend_handles(end+1) = h_sen_end; legend_labels{end+1} = '传感器终点'; end %#ok<AGROW>
    legend(legend_handles, legend_labels, 'Location', 'best');
    hold off;
end

function True_Target_Traj = time_to_traj(True_Target_Time)
    N = numel(True_Target_Time);
    if N < 1
        True_Target_Traj = {};
        return;
    end
    X1 = True_Target_Time{1};
    if isempty(X1)
        % 找一个非空时刻推断目标数
        idx = find(~cellfun(@isempty, True_Target_Time), 1, 'first');
        if isempty(idx)
            True_Target_Traj = {};
            return;
        end
        X1 = True_Target_Time{idx};
    end
    Nt = size(X1, 2);
    True_Target_Traj = cell(Nt, 1);
    for j = 1:Nt
        True_Target_Traj{j,1} = nan(6, N);
    end
    for t = 1:N
        Xt = True_Target_Time{t};
        if isempty(Xt), continue; end
        nj = min(size(Xt,2), Nt);
        for j = 1:nj
            if size(Xt,1) >= 6
                True_Target_Traj{j,1}(:,t) = Xt(1:6,j);
            end
        end
    end
end

function out = sanitize_name(in)
    out = regexprep(char(in), '[^a-zA-Z0-9_-]', '_');
    if isempty(out), out = 'name'; end
end
