clear;

%% 初始化（必须先运行以得到 N, M, N_sensor）
 config;
save('Xreal_target_time.mat','Xreal_target_time')

%% 多核并行：按 M 选择策略，避免嵌套 parfor
num_cores = feature('numcores');
pool_size = min(num_cores, max(M, sim_cfg.parallel.min_pool_size));
if isempty(gcp('nocreate'))
    parpool('local', pool_size);
end

%% 控制方案配置（统一来自 config.m）
base_seed = sim_cfg.rng_seed;  % 为了公平比较，所有方案使用相同随机种子
algo_cfgs = sim_cfg.algo_cfgs(sim_cfg.enabled_algo_indices);

%% 分别运行三种控制算法
empty_decision_log = struct('times', [], 'track_cost', [], 'search_gain', [], ...
    'cs_gain', [], 'info_gain', [], 'total_cost', [], 'selection', [], ...
    'objective_mode', '');
results = repmat(struct( ...
    'name', '', ...
    'tag', '', ...
    'control_opt', struct(), ...
    'OSPA_avg', zeros(1, N), ...
    'OSPA_avg_sensor', zeros(N_sensor, N), ...
    'Num_avg', zeros(1, N), ...
    'Time_avg', 0, ...
    'Sensor_traj_vis', [], ...
    'X_est_vis', [], ...
    'decision_log', empty_decision_log), 1, numel(algo_cfgs));
for a = 1:numel(algo_cfgs)
    results(a) = run_control_scheme(algo_cfgs(a), Xreal_time_target, Sensor_distr, ...
        N, GridMap, selection, sensor, M, base_seed, sim_cfg.process, sim_cfg.parallel);
    disp([results(a).name, ' 平均单次融合周期耗时: ', num2str(results(a).Time_avg), ' s']);
    if ~isempty(results(a).decision_log.times)
        mean_track = mean(results(a).decision_log.track_cost);
        mean_search = mean(results(a).decision_log.search_gain, 'omitnan');
        mean_cs = mean(results(a).decision_log.cs_gain, 'omitnan');
        mean_info = mean(results(a).decision_log.info_gain, 'omitnan');
        disp(['  决策统计: mean(J_track)=', num2str(mean_track), ...
              ', mean(J_search)=', num2str(mean_search), ...
              ', mean(D_CS)=', num2str(mean_cs), ...
              ', mean(info)=', num2str(mean_info)]);
    end
end



% ===== 结果保存：三套方案分别保存 + 汇总保存 =====
save_stamp = datestr(now, 'yyyymmdd_HHMMSS');
saved_files = cell(1, numel(results));
for a = 1:numel(results)
    algo_result = results(a); %#ok<NASGU>
    saved_files{a} = sprintf('result_%s_%s.mat', results(a).tag, save_stamp);
    save(saved_files{a}, 'algo_result');
    disp(['结果已保存: ', saved_files{a}]);
end
compare_file = sprintf('result_compare_%s.mat', save_stamp);
save(compare_file, 'results', 'algo_cfgs', 'saved_files');
disp(['对比汇总已保存: ', compare_file]);

%% ===== 可视化名称（按实际启用方案动态设置，避免索引越界） =====
for a = 1:numel(results)
    if strcmpi(results(a).tag, 'grid')
        results(a).name = '方案1：trace+网格';
    elseif strcmpi(results(a).tag, 'cs')
        results(a).name = '方案2：纯CS';
    elseif strcmpi(results(a).tag, 'cs_improved')
        results(a).name = '方案3：改进CS+网格';
    end
end

%% ===== 可视化1: 三种算法平均 OSPA 对比 =====
figure('Name', '三种算法 OSPA 对比', 'Color', 'w');
hold on;
style_list = {'-o', '-s', '-^'};
color_list = lines(numel(results));
for a = 1:numel(results)
    plot(1:N, results(a).OSPA_avg, style_list{a}, 'Color', color_list(a,:), ...
        'LineWidth', 1.4, 'MarkerSize', 4, 'MarkerFaceColor', color_list(a,:));
end
grid on;
xlabel('时刻 k');
ylabel('OSPA');
title('三种控制算法的平均 OSPA 对比');
legend({results.name}, 'Location', 'best');
hold off;

%% ===== 可视化2: 三种算法平均目标数量估计对比 =====
figure('Name', '三种算法目标数量估计对比', 'Color', 'w');
hold on;
num_real = zeros(1, N);
for k = 1:N
    Xk_real = Xreal_time_target{k,1};
    if isempty(Xk_real)
        num_real(k) = 0;
    else
        num_real(k) = sum(~isnan(Xk_real(1,:)));
    end
end
plot(1:N, num_real, 'k-', 'LineWidth', 1.5);
for a = 1:numel(results)
    plot(1:N, results(a).Num_avg, style_list{a}, 'Color', color_list(a,:), ...
        'LineWidth', 1.2, 'MarkerSize', 4);
end
grid on;
xlabel('时刻 k');
ylabel('目标数量');
title('三种控制算法的目标数量估计对比');
legend_entries = [{'真实目标数'}, {results.name}];
legend(legend_entries, 'Location', 'best');
hold off;

%% ===== 可视化3: 决策信息增益示意图 =====
figure('Name', '三种算法决策收益示意图', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
for a = 1:numel(results)
    log_data = results(a).decision_log;
    if strcmpi(log_data.objective_mode, 'grid')
        gain_curve = log_data.search_gain;
    elseif strcmpi(log_data.objective_mode, 'cs')
        gain_curve = log_data.cs_gain;
    else
        gain_curve = log_data.info_gain;
    end
    plot(log_data.times, gain_curve, style_list{a}, 'Color', color_list(a,:), ...
        'LineWidth', 1.4, 'MarkerSize', 5, 'MarkerFaceColor', color_list(a,:));
end
grid on;
xlabel('决策时刻 k');
ylabel('信息增益');
title('指标对比：grid画J_{search}，cs画D_{CS}，cs\_improved画\theta_{CS}');
legend({results.name}, 'Location', 'best');
hold off;

nexttile;
hold on;
for a = 1:numel(results)
    log_data = results(a).decision_log;
    plot(log_data.times, log_data.track_cost, style_list{a}, 'Color', color_list(a,:), ...
        'LineWidth', 1.4, 'MarkerSize', 5);
end
grid on;
xlabel('决策时刻 k');
ylabel('J_{track}');
title('三种算法对应的跟踪代价变化');
legend({results.name}, 'Location', 'best');
hold off;

%% ===== 可视化4: 决策目标函数值对比 =====
figure('Name', '三种算法决策目标函数值对比', 'Color', 'w');
hold on;
for a = 1:numel(results)
    log_data = results(a).decision_log;
    plot(log_data.times, log_data.total_cost, style_list{a}, 'Color', color_list(a,:), ...
        'LineWidth', 1.4, 'MarkerSize', 5, 'MarkerFaceColor', color_list(a,:));
end
grid on;
xlabel('决策时刻 k');
ylabel('目标函数值');
title('三种算法目标函数值（grid: J_{track}-\etaJ_{search}, cs: -D_{CS}, cs\_improved: -\theta_{CS}-\etaJ_{search}）');
legend({results.name}, 'Location', 'best');
hold off;

%% ===== 可视化5: 三种算法轨迹示意图 =====
figure('Name', '三种算法轨迹示意图', 'Color', 'w');
tiledlayout(1, numel(results), 'TileSpacing', 'compact', 'Padding', 'compact');
for a = 1:numel(results)
    nexttile;
    hold on;
    grid on;
    axis equal;

    % 真实目标轨迹
    h_true = [];
    h_true_start = [];
    h_true_end = [];
    n_target = length(Xreal_target_time);
    for j = 1:n_target
        traj = Xreal_target_time{j,1};
        h = plot(traj(1,:), traj(3,:), 'k-', 'LineWidth', 1.6);
        valid_idx = find(~isnan(traj(1,:)));
        if ~isempty(valid_idx)
            idx_start = valid_idx(1);
            idx_end = valid_idx(end);
            hs = plot(traj(1,idx_start), traj(3,idx_start), '^', 'Color', [0 0 0], ...
                'MarkerFaceColor', 'y', 'MarkerSize', 8);
            he = plot(traj(1,idx_end), traj(3,idx_end), 'o', 'Color', [0 0 0], ...
                'MarkerFaceColor', 'c', 'MarkerSize', 7);
            if isempty(h_true_start), h_true_start = hs; end
            if isempty(h_true_end), h_true_end = he; end
        end
        if isempty(h_true), h_true = h; end
    end

    % 传感器轨迹
    h_sensor = [];
    h_start = [];
    h_decision = [];
    h_end = [];
    Sensor_traj_vis = results(a).Sensor_traj_vis;
    decision_times = results(a).decision_log.times;
    for i = 1:size(Sensor_traj_vis, 3)
        xs = squeeze(Sensor_traj_vis(1,:,i));
        ys = squeeze(Sensor_traj_vis(2,:,i));
        hs = plot(xs, ys, 'LineWidth', 2);
        hst = plot(xs(1), ys(1), '^', 'Color', [0 0 0], 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        hse = plot(xs(end), ys(end), 'o', 'Color', [0 0 0], 'MarkerFaceColor', 'c', 'MarkerSize', 7);
        valid_decision_times = decision_times(decision_times >= 1 & decision_times <= N);
        hsd = plot(xs(valid_decision_times), ys(valid_decision_times), 'ks', ...
            'MarkerFaceColor', [0.95 0.6 0.2], 'MarkerSize', 5);
        if isempty(h_sensor), h_sensor = hs; end
        if isempty(h_start), h_start = hst; end
        if isempty(h_decision), h_decision = hsd; end
        if isempty(h_end), h_end = hse; end
    end

    xlabel('X / m');
    ylabel('Y / m');
    title(results(a).name);
    legend([h_true, h_true_start, h_true_end, h_sensor, h_start, h_decision, h_end], ...
        {'真实目标轨迹', '目标起点', '目标终点', '传感器轨迹', '传感器起点', '决策时刻位置', '传感器终点'}, ...
        'Location', 'best');
    hold off;
end

% %% ===== 单独为每种方法保存目标轨迹、传感器轨迹和 OSPA 指标 =====
% % 遍历每种方法，为它们各自生成一个独立的 .mat 文件
% for a = 1:numel(results)
%     % 提取当前方法的标签（例如 'grid' 或 'cs'）
%     method_tag = results(a).tag; 
%     
%     % 1. 提取目标真实轨迹 (两种方法对比时共用同一个真实环境)
%     True_Target_Traj = Xreal_target_time; 
%     True_Target_Time = Xreal_time_target; 
%     
%     % 2. 提取当前方法的传感器轨迹
%     Sensor_Traj = results(a).Sensor_traj_vis;
%     
%     % 3. 提取当前方法的 OSPA 指标
%     OSPA_Metric = results(a).OSPA_avg;
%     
%     % 构造当前方法的独立保存文件名
%     % 例如: CoreData_grid_20231024_120000.mat
%     individual_filename = sprintf('CoreData_%s_%s.mat', method_tag, save_stamp);
%     
%     % 将这三个关键变量保存到属于该方法的文件中
%     save(individual_filename, 'True_Target_Traj', 'True_Target_Time', 'Sensor_Traj', 'OSPA_Metric');
%     
%     disp(['方法 [', results(a).name, '] 的 目标轨迹、传感器轨迹、OSPA 已单独保存至: ', individual_filename]);
% end
% disp('===== 所有方法的核心数据已分别独立保存完毕 =====');
