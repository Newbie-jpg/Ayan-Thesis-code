function result = plot_ospa_from_scene_inputs(target_param_mat, sensor_input, traj_input, user_opt)
% PLOT_OSPA_FROM_SCENE_INPUTS
% 复用“固定传感器轨迹 + 重放滤波”链路计算并绘制 OSPA。
%
% 输入:
%   target_param_mat : 9xNt 或 Nt x9 的目标参数矩阵（可为空；若轨迹包有真值则可不传）
%   sensor_input     : 3xNs / Nsx3 位置矩阵，或带 .location 的 Sensor_distr 结构体数组（可为空）
%   traj_input       : .mat 路径，或结构体(含 Sensor_traj/Sensor_Traj)
%   user_opt         : 可选参数结构体
%
% 输出:
%   result.OSPA_avg
%   result.OSPA_avg_sensor
%   result.Num_avg
%   result.X_est_vis
%   result.mc_ospa
%   result.mc_num
%   result.Xreal_time_target
%   result.Sensor_traj
%   result.figure_handle

    if nargin < 4 || isempty(user_opt)
        user_opt = struct();
    end
    user_opt = apply_default_opt(user_opt);

    this_dir = fileparts(mfilename('fullpath'));   % chat 4/03_Compare_Eval
    chat4_root = fileparts(this_dir);              % chat 4
    addpath(genpath(chat4_root));

    % 加载工程配置（用于 GridMap 与默认传感器参数模板）
    cfg_file = fullfile(chat4_root, '01_Main_Env', 'config.m');
    if ~exist(cfg_file, 'file')
        error('未找到配置文件: %s', cfg_file);
    end
    run(cfg_file); %#ok<RUN>

    traj_bundle = resolve_traj_input(traj_input);
    Sensor_traj = resolve_sensor_traj(traj_bundle);
    N = size(Sensor_traj, 2);
    Ns = size(Sensor_traj, 3);

    Xreal_time_target = resolve_true_targets(traj_bundle, target_param_mat, N);
    Sensor_init = build_sensor_init(sensor_input, Sensor_distr, Sensor_traj, GridMap, N, Ns);

    process_opt = user_opt.process_opt;
    control_opt = user_opt.control_opt;
    M = user_opt.monte_carlo;
    base_seed = user_opt.rng_seed;

    OSPA_total = zeros(1, N);
    OSPA_total_sensor = zeros(Ns, N);
    Num_total = zeros(1, N);
    X_est_vis = [];
    mc_ospa = cell(M, 1);
    mc_num = cell(M, 1);

    if M > 1
        fprintf('固定轨迹重放评估中，蒙特卡洛次数 M=%d ...\n', M);
    end
    for m = 1:M
        if ~isempty(base_seed)
            rng(base_seed + m - 1);
        end
        [OSPA_m, OSPA_sensor_m, Num_m, X_est_m] = evaluate_ospa_on_fixed_traj( ...
            Xreal_time_target, Sensor_init, Sensor_traj, N, GridMap, process_opt, control_opt);

        OSPA_total = OSPA_total + OSPA_m;
        OSPA_total_sensor = OSPA_total_sensor + OSPA_sensor_m;
        Num_total = Num_total + Num_m;
        mc_ospa{m} = OSPA_m;
        mc_num{m} = Num_m;
        if m == 1
            X_est_vis = X_est_m;
        end
    end

    OSPA_avg = OSPA_total / M;
    OSPA_avg_sensor = OSPA_total_sensor / M;
    Num_avg = Num_total / M;

    fig = figure('Color', 'w', 'Name', ['OSPA_', user_opt.scenario_name]); %#ok<NASGU>
    plot(1:N, OSPA_avg, '-o', 'LineWidth', 1.5, 'MarkerSize', 4, ...
        'MarkerIndices', unique([1:5:N, N]));
    grid on;
    xlabel('时刻 k');
    ylabel('OSPA');
    title(sprintf('场景 %s 的 OSPA 曲线（固定轨迹重放，M=%d）', user_opt.scenario_name, M));

    result = struct();
    result.OSPA_avg = OSPA_avg;
    result.OSPA_avg_sensor = OSPA_avg_sensor;
    result.Num_avg = Num_avg;
    result.X_est_vis = X_est_vis;
    result.mc_ospa = mc_ospa;
    result.mc_num = mc_num;
    % 兼容旧字段名
    result.OSPA = OSPA_avg;
    result.OSPA_sensor = OSPA_avg_sensor;
    result.Num_estimate = Num_avg;
    result.X_est_global = X_est_vis;
    result.Xreal_time_target = Xreal_time_target;
    result.Sensor_traj = Sensor_traj;
    result.figure_handle = gcf;
    result.scenario_name = user_opt.scenario_name;
    result.M = M;
    result.rng_seed = base_seed;

    if user_opt.save_output
        if ~exist(user_opt.output_dir, 'dir')
            mkdir(user_opt.output_dir);
        end
        save_name = fullfile(user_opt.output_dir, ...
            sprintf('%s_ospa_result.mat', user_opt.scenario_name));
        save(save_name, 'result');
        saveas(gcf, fullfile(user_opt.output_dir, ...
            sprintf('%s_ospa_curve.png', user_opt.scenario_name)));
        fprintf('OSPA 结果已保存: %s\n', save_name);
    end
end

function opt = apply_default_opt(opt)
    if ~isfield(opt, 'scenario_name') || isempty(opt.scenario_name)
        opt.scenario_name = 'custom_scene';
    end
    if ~isfield(opt, 'save_output') || isempty(opt.save_output)
        opt.save_output = false;
    end
    if ~isfield(opt, 'output_dir') || isempty(opt.output_dir)
        this_dir = fileparts(mfilename('fullpath'));
        opt.output_dir = fullfile(this_dir, 'results');
    end
    if ~isfield(opt, 'process_opt') || isempty(opt.process_opt)
        opt.process_opt = struct();
    end
    if ~isfield(opt, 'control_opt') || isempty(opt.control_opt)
        opt.control_opt = struct('match_threshold', 200, 'objective_mode', 'grid');
    end
    if ~isfield(opt, 'monte_carlo') || isempty(opt.monte_carlo)
        opt.monte_carlo = 1;
    end
    opt.monte_carlo = max(1, round(opt.monte_carlo));
    if ~isfield(opt, 'rng_seed') || isempty(opt.rng_seed)
        opt.rng_seed = [];
    end
end

function target_param_mat = normalize_target_param_mat(target_param_mat)
    if isempty(target_param_mat) || ~ismatrix(target_param_mat) || ~isnumeric(target_param_mat)
        error('target_param_mat 不能为空，且必须为数值矩阵。');
    end
    if size(target_param_mat, 1) == 9
        return;
    end
    if size(target_param_mat, 2) == 9
        target_param_mat = target_param_mat.';
        return;
    end
    error('target_param_mat 维度错误，期望 9xNt 或 Nt x9。');
end

function bundle = resolve_traj_input(traj_input)
    if ischar(traj_input) || (isstring(traj_input) && isscalar(traj_input))
        file_path = char(traj_input);
        if ~exist(file_path, 'file')
            error('轨迹文件不存在: %s', file_path);
        end
        bundle = load(file_path);
    elseif isstruct(traj_input)
        bundle = traj_input;
    else
        error('traj_input 必须是 .mat 路径字符串，或结构体。');
    end
end

function Sensor_traj = resolve_sensor_traj(bundle)
    Sensor_traj = [];
    if isfield(bundle, 'Sensor_traj') && ~isempty(bundle.Sensor_traj)
        Sensor_traj = bundle.Sensor_traj;
    elseif isfield(bundle, 'Sensor_Traj') && ~isempty(bundle.Sensor_Traj)
        Sensor_traj = bundle.Sensor_Traj;
    elseif isfield(bundle, 'Sensor_traj_vis') && ~isempty(bundle.Sensor_traj_vis)
        Sensor_traj = bundle.Sensor_traj_vis;
    elseif isfield(bundle, 'result_A') && isfield(bundle.result_A, 'Sensor_traj_vis')
        Sensor_traj = bundle.result_A.Sensor_traj_vis;
    elseif isfield(bundle, 'result_B') && isfield(bundle.result_B, 'Sensor_traj_vis')
        Sensor_traj = bundle.result_B.Sensor_traj_vis;
    elseif isfield(bundle, 'algo_result') && isfield(bundle.algo_result, 'Sensor_traj_vis')
        Sensor_traj = bundle.algo_result.Sensor_traj_vis;
    elseif isfield(bundle, 'result') && isfield(bundle.result, 'Sensor_traj_vis')
        Sensor_traj = bundle.result.Sensor_traj_vis;
    end

    if isempty(Sensor_traj) || ndims(Sensor_traj) ~= 3
        error('未在 traj_input 中找到有效的 Sensor_traj/Sensor_Traj (维度应为 3xN xNs 或 2xN xNs)。');
    end
    if size(Sensor_traj, 1) == 2
        Sensor_traj = cat(1, Sensor_traj, zeros(1, size(Sensor_traj, 2), size(Sensor_traj, 3)));
    end
    if size(Sensor_traj, 1) ~= 3
        error('Sensor_traj 第一维应为 3（x,y,z）。');
    end
end

function Xreal_time_target = resolve_true_targets(bundle, target_param_mat, N)
    if isfield(bundle, 'Xreal_time_target') && ~isempty(bundle.Xreal_time_target)
        Xreal_time_target = bundle.Xreal_time_target;
        return;
    end
    if isfield(bundle, 'True_Target_Time') && ~isempty(bundle.True_Target_Time)
        Xreal_time_target = bundle.True_Target_Time;
        return;
    end
    if isfield(bundle, 'Xreal_target_time') && ~isempty(bundle.Xreal_target_time)
        Xreal_time_target = target_time_to_time_target(bundle.Xreal_target_time, N);
        return;
    end
    if isfield(bundle, 'True_Target_Traj') && ~isempty(bundle.True_Target_Traj)
        Xreal_time_target = target_time_to_time_target(bundle.True_Target_Traj, N);
        return;
    end

    target_param_mat = normalize_target_param_mat(target_param_mat);
    [~, Xreal_time_target] = targetset(N, target_param_mat);
end

function Xreal_time_target = target_time_to_time_target(Xreal_target_time, N)
    Nt = numel(Xreal_target_time);
    Xreal_time_target = cell(N, 1);
    for t = 1:N
        Xt = zeros(6, Nt);
        Xt(:) = nan;
        for j = 1:Nt
            traj_j = Xreal_target_time{j};
            if isempty(traj_j) || size(traj_j, 1) < 6 || t > size(traj_j, 2)
                continue;
            end
            Xt(:, j) = traj_j(:, t);
        end
        Xreal_time_target{t, 1} = Xt;
    end
end

function Sensor_init = build_sensor_init(sensor_input, Sensor_template, Sensor_traj, GridMap, N, Ns)
    if nargin < 1 || isempty(sensor_input)
        loc_mat = squeeze(Sensor_traj(:, 1, :));
        if isvector(loc_mat)
            loc_mat = loc_mat(:);
        end
    else
        loc_mat = sensor_input_to_matrix(sensor_input);
    end
    if size(loc_mat, 2) ~= Ns
        error('传感器数量不匹配：sensor_input=%d, Sensor_traj=%d。', size(loc_mat, 2), Ns);
    end

    Sensor_init = repmat(Sensor_template(1), 1, Ns);
    for i = 1:Ns
        Sensor_init(i).serial = i;
        Sensor_init(i).location = loc_mat(:, i);
        Sensor_init(i).Z_polar_part = cell(N, 1);
        Sensor_init(i).Z_dicaer_global = cell(N, 1);
        Sensor_init(i).X_est_local = cell(N, 1);
        Sensor_init(i).X_est_fov = cell(N, 1);
        Sensor_init(i).GridProb = GridMap.omega0 * ones(GridMap.M, 1);
    end
end

function loc_mat = sensor_input_to_matrix(sensor_input)
    if isstruct(sensor_input)
        Ns = numel(sensor_input);
        loc_mat = zeros(3, Ns);
        for i = 1:Ns
            if ~isfield(sensor_input(i), 'location') || isempty(sensor_input(i).location)
                error('sensor_input(%d) 缺少 location 字段。', i);
            end
            p = sensor_input(i).location(:);
            if numel(p) == 2
                p = [p; 0];
            end
            if numel(p) ~= 3
                error('sensor_input(%d).location 维度应为 2 或 3。', i);
            end
            loc_mat(:, i) = p;
        end
        return;
    end

    if isnumeric(sensor_input) && ismatrix(sensor_input)
        if size(sensor_input, 1) == 3
            loc_mat = sensor_input;
            return;
        end
        if size(sensor_input, 2) == 3
            loc_mat = sensor_input.';
            return;
        end
    end

    error('sensor_input 必须是 3xNs/Nsx3 数值矩阵，或带 location 的结构体数组。');
end
