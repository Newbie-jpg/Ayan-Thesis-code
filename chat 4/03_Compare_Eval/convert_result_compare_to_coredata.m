% CONVERT_RESULT_COMPARE_TO_COREDATA
% 将 result_compare_*.mat 中各算法结果导出为 CoreData 风格文件：
%   True_Target_Traj, True_Target_Time, Sensor_Traj, OSPA_Metric

clear; clc;

this_dir = fileparts(mfilename('fullpath'));      % chat 4/03_Compare_Eval
chat4_root = fileparts(this_dir);                 % chat 4

%% 1) 输入文件
compare_file = fullfile(chat4_root, 'result_compare_20260318_134307.mat');

% 可选：真值轨迹来源（可配置多个，按顺序尝试）
% 若都不可用，将按顺序尝试：
%   A. compare_file 内字段
%   B. truth_source_files 中的外部文件
%   C. 运行 config.m + targetset() 生成
truth_source_files = { ...
    fullfile(chat4_root, '04_Data', 'Xreal_target_time.mat')
};
% truth_source_files = {};

% 输出目录
output_dir = chat4_root;

%% 2) 加载对比结果
if ~isfile(compare_file)
    error('未找到 compare 文件: %s', compare_file);
end
S = load(compare_file);
if ~isfield(S, 'results') || isempty(S.results)
    error('compare 文件缺少 results 字段: %s', compare_file);
end
results = S.results;
if ~isstruct(results)
    error('results 字段类型应为 struct 数组。');
end

N = infer_time_length_from_results(results);
[True_Target_Traj, True_Target_Time] = resolve_truth_data(S, truth_source_files, N, chat4_root);

%% 3) 按算法导出 CoreData
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save_stamp = datestr(now, 'yyyymmdd_HHMMSS');

for a = 1:numel(results)
    r = results(a);
    if ~isfield(r, 'Sensor_traj_vis') || isempty(r.Sensor_traj_vis)
        warning('results(%d) 缺少 Sensor_traj_vis，跳过。', a);
        continue;
    end
    if ~isfield(r, 'OSPA_avg') || isempty(r.OSPA_avg)
        warning('results(%d) 缺少 OSPA_avg，跳过。', a);
        continue;
    end

    Sensor_Traj = normalize_sensor_traj(r.Sensor_traj_vis);
    OSPA_Metric = r.OSPA_avg(:).';

    tag = '';
    if isfield(r, 'tag') && ~isempty(r.tag)
        tag = char(r.tag);
    end
    if isempty(tag)
        tag = sprintf('algo%d', a);
    end
    tag = sanitize_token(tag);

    out_file = fullfile(output_dir, sprintf('CoreData_%s3trace_%s.mat', tag, save_stamp));
    save(out_file, 'True_Target_Traj', 'True_Target_Time', 'Sensor_Traj', 'OSPA_Metric');
    fprintf('已导出: %s\n', out_file);
end

%% ===== Local functions =====
function N = infer_time_length_from_results(results)
    N = [];
    for i = 1:numel(results)
        if isfield(results(i), 'Sensor_traj_vis') && ~isempty(results(i).Sensor_traj_vis)
            N = size(results(i).Sensor_traj_vis, 2);
            return;
        end
        if isfield(results(i), 'OSPA_avg') && ~isempty(results(i).OSPA_avg)
            N = numel(results(i).OSPA_avg);
            return;
        end
    end
    error('无法从 results 推断时间长度 N。');
end

function [True_Target_Traj, True_Target_Time] = resolve_truth_data(S_compare, truth_source_files, N, chat4_root)
    % A) 优先从 compare 文件自身读取
    [ok, True_Target_Traj, True_Target_Time] = parse_truth_from_struct(S_compare, N);
    if ok
        return;
    end

    % B) 从外部 truth_source_files 读取
    if nargin >= 2 && ~isempty(truth_source_files)
        for ii = 1:numel(truth_source_files)
            one_file = truth_source_files{ii};
            if isempty(one_file) || ~isfile(one_file)
                continue;
            end
            S_truth = load(one_file);
            [ok, True_Target_Traj, True_Target_Time] = parse_truth_from_struct(S_truth, N);
            if ok
                fprintf('提示：真值轨迹来源 = %s\n', one_file);
                return;
            end
        end
    end

    % C) 最后兜底：运行 config + targetset 生成
    cfg_file = fullfile(chat4_root, '01_Main_Env', 'config.m');
    if ~isfile(cfg_file)
        error('缺少真值轨迹，且未找到 config.m 用于兜底生成。');
    end
    run(cfg_file); %#ok<RUN>
    if ~exist('Target_Init', 'var') || isempty(Target_Init)
        error('config.m 未提供 Target_Init，无法兜底生成真值轨迹。');
    end
    addpath(genpath(chat4_root));
    [True_Target_Traj, True_Target_Time] = targetset(N, Target_Init);
    fprintf('提示：已使用 config.m + targetset() 兜底生成真值轨迹。\n');
end

function [ok, True_Target_Traj, True_Target_Time] = parse_truth_from_struct(S, N)
    ok = false;
    True_Target_Traj = [];
    True_Target_Time = [];

    if isfield(S, 'True_Target_Traj') && ~isempty(S.True_Target_Traj)
        True_Target_Traj = S.True_Target_Traj;
        ok = true;
    elseif isfield(S, 'Xreal_target_time') && ~isempty(S.Xreal_target_time)
        True_Target_Traj = S.Xreal_target_time;
        ok = true;
    end

    if isfield(S, 'True_Target_Time') && ~isempty(S.True_Target_Time)
        True_Target_Time = S.True_Target_Time;
    elseif isfield(S, 'Xreal_time_target') && ~isempty(S.Xreal_time_target)
        True_Target_Time = S.Xreal_time_target;
    elseif ok
        True_Target_Time = target_time_to_time_target(True_Target_Traj, N);
    end

    if ok
        if isempty(True_Target_Time)
            ok = false;
            return;
        end
        % 统一长度到 N
        if numel(True_Target_Time) > N
            True_Target_Time = True_Target_Time(1:N);
        elseif numel(True_Target_Time) < N
            pad = cell(N - numel(True_Target_Time), 1);
            for ii = 1:numel(pad)
                pad{ii,1} = zeros(6,0);
            end
            True_Target_Time = [True_Target_Time(:); pad];
        else
            True_Target_Time = True_Target_Time(:);
        end
    end
end

function Xreal_time_target = target_time_to_time_target(Xreal_target_time, N)
    Nt = numel(Xreal_target_time);
    Xreal_time_target = cell(N, 1);
    for t = 1:N
        Xt = nan(6, Nt);
        for j = 1:Nt
            traj_j = Xreal_target_time{j};
            if isempty(traj_j) || size(traj_j, 1) < 6 || t > size(traj_j, 2)
                continue;
            end
            Xt(:, j) = traj_j(:, t);
        end
        Xreal_time_target{t,1} = Xt;
    end
end

function Sensor_Traj = normalize_sensor_traj(traj)
    Sensor_Traj = traj;
    if ndims(Sensor_Traj) ~= 3
        error('Sensor_traj_vis 维度应为 3D (2/3 x N x Ns)。');
    end
    if size(Sensor_Traj, 1) == 2
        Sensor_Traj = cat(1, Sensor_Traj, zeros(1, size(Sensor_Traj,2), size(Sensor_Traj,3)));
    end
    if size(Sensor_Traj, 1) ~= 3
        error('Sensor_Traj 第一维应为 3。');
    end
end

function out = sanitize_token(in)
    out = regexprep(in, '[^a-zA-Z0-9_-]', '_');
    if isempty(out)
        out = 'algo';
    end
end
