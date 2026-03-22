% MAIN_EXP2_SCALABILITY
% 第四章实验二：可扩展性与计算效率（规模递增实验）
% 对比算法A与算法B在不同传感器规模下的单步平均决策时间（蒙特卡洛平均）。

clear; close all; clc;

disp('>>> 正在初始化实验二配置...');
config;
[Xreal_target_time, Xreal_time_target] = targetset(N, Target_Init);

sensor_counts = [4, 6, 8];
if isfield(sim_cfg, 'scalability') && isfield(sim_cfg.scalability, 'sensor_counts') && ...
        ~isempty(sim_cfg.scalability.sensor_counts)
    sensor_counts = sim_cfg.scalability.sensor_counts;
end

time_avg_A = zeros(size(sensor_counts));
time_avg_B = zeros(size(sensor_counts));
result_A_by_scale = cell(size(sensor_counts));
result_B_by_scale = cell(size(sensor_counts));

for ss = 1:numel(sensor_counts)
    ns = sensor_counts(ss);
    disp('======================================================');
    disp(['>>> 实验二规模点: 传感器数量 = ', num2str(ns)]);
    disp('======================================================');

    Sensor_distr_ns = build_sensor_network(Sensor_distr, ns);

    sim_cfg_ns = sim_cfg;
    sim_cfg_ns.sensor.num = ns;
    ch4_cfg_ns = ch4_cfg;
    ch4_cfg_ns.fixed_search_target_groups = expand_fixed_groups(sim_cfg.fixed_search_target_groups, ns);
    sim_cfg_ns.algo_baseline.fixed_search_target_groups = ch4_cfg_ns.fixed_search_target_groups;

    % 算法A初始化角色
    Sensor_distr_A = Sensor_distr_ns;
    for i = 1:ns
        Sensor_distr_A(i).task_type = 'S';
        Sensor_distr_A(i).target_in_fov_count = 0;
        Sensor_distr_A(i).no_target_in_fov_count = 0;
    end

    ticA = tic;
    result_A = run_control_twostage_scheme(Xreal_time_target, Sensor_distr_A, N, GridMap, sim_cfg_ns, ch4_cfg_ns, M, sim_cfg.rng_seed);
    time_total_A = toc(ticA);
    result_A.name = ['算法A(', num2str(ns), '传感器)'];
    result_A.total_time = time_total_A;

    algo_cfg_B = struct('name', ['基线算法(', num2str(ns), '传感器)'], ...
                        'tag', 'grid', ...
                        'control_opt', sim_cfg_ns.algo_baseline);
    Sensor_distr_B = Sensor_distr_ns;
    default_action_idx = find(sim_cfg_ns.sensor.C == 0, 1, 'first');
    if isempty(default_action_idx), default_action_idx = 1; end
    selection = ones(1, ns) * default_action_idx;

    ticB = tic;
    result_B = run_control_scheme(algo_cfg_B, Xreal_time_target, Sensor_distr_B, ...
        N, GridMap, selection, sim_cfg_ns.sensor, M, sim_cfg.rng_seed, ...
        struct('Ps', Sensor_distr_B(1).Ps, 'control_interval', 5), struct('min_pool_size', 1));
    time_total_B = toc(ticB);
    result_B.name = ['算法B(', num2str(ns), '传感器)'];
    result_B.total_time = time_total_B;

    time_avg_A(ss) = result_A.Time_avg;
    time_avg_B(ss) = result_B.Time_avg;
    result_A_by_scale{ss} = result_A;
    result_B_by_scale{ss} = result_B;

    fprintf('规模 %d: A单步均时=%.4f s, B单步均时=%.4f s\n', ns, time_avg_A(ss), time_avg_B(ss));
end

% 绘图：单步平均决策时间随传感器数量变化
figure('Color', 'w', 'Name', 'Exp2_Scalability_Time');
plot(sensor_counts, time_avg_A, '-o', 'LineWidth', 1.8, 'MarkerSize', 6); hold on;
plot(sensor_counts, time_avg_B, '-s', 'LineWidth', 1.8, 'MarkerSize', 6);
grid on;
xlabel('传感器数量');
ylabel('单步平均决策时间 / s');
title('实验二：规模递增下的计算效率对比');
legend({'算法A', '算法B'}, 'Location', 'northwest');

% 保存结果
save_stamp = datestr(now, 'yyyymmdd_HHMMSS');
this_dir = fileparts(mfilename('fullpath'));
chat4_root = fileparts(this_dir);
data_dir = fullfile(chat4_root, '04_Data');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
save_file_name = fullfile(data_dir, sprintf('Exp2_Scalability_%s.mat', save_stamp));
save(save_file_name, 'sensor_counts', 'time_avg_A', 'time_avg_B', ...
    'result_A_by_scale', 'result_B_by_scale', 'M', 'N', 'sim_cfg');
disp(['实验二结果已保存: ', save_file_name]);

function Sensor_out = build_sensor_network(Sensor_template, ns)
    n0 = numel(Sensor_template);
    Sensor_out = repmat(Sensor_template(1), 1, ns);

    if ns <= n0
        for i = 1:ns
            Sensor_out(i) = Sensor_template(i);
            Sensor_out(i).serial = i;
        end
        return;
    end

    % 以圆周均匀布设新增传感器（前4个沿用原位置）
    base_locs = zeros(3, n0);
    for i = 1:n0
        base_locs(:, i) = Sensor_template(i).location(:);
    end
    center = mean(base_locs, 2);
    radius = max(vecnorm(base_locs(1:2, :) - center(1:2), 2, 1));
    if radius < 1e-6, radius = 1200; end

    for i = 1:ns
        if i <= n0
            Sensor_out(i) = Sensor_template(i);
        else
            Sensor_out(i) = Sensor_template(1);
            ang = 2*pi*(i-1)/ns;
            Sensor_out(i).location = [center(1) + radius*cos(ang); center(2) + radius*sin(ang); center(3)];
        end
        Sensor_out(i).serial = i;
    end
end

function groups = expand_fixed_groups(base_groups, ns)
    groups = cell(1, ns);
    nb = numel(base_groups);
    for i = 1:ns
        groups{i} = base_groups{1 + mod(i - 1, nb)};
    end
end
