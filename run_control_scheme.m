%{
    按指定控制方案运行完整仿真，并返回用于比较绘图的汇总结果。
%}
function result = run_control_scheme(algo_cfg, Xreal_time_target, Sensor_distr, N, GridMap, selection, sensor, M, base_seed, process_opt, parallel_opt)
    if nargin < 9
        base_seed = [];
    end
    if nargin < 10 || isempty(process_opt)
        process_opt = struct();
    end
    if nargin < 11 || isempty(parallel_opt)
        parallel_opt = struct();
    end
    if ~isempty(base_seed)
        rng(base_seed);
    end

    OSPA_total = zeros(1, N);
    OSPA_total_sensor = zeros(sensor.num, N);
    Num_total = zeros(1, N);
    Time_total = 0;
    Sensor_traj_vis = [];
    X_est_vis = [];
    decision_log_vis = struct('times', [], 'track_cost', [], 'search_gain', [], ...
        'cs_gain', [], 'info_gain', [], 'total_cost', [], 'selection', [], ...
        'objective_mode', '');

    if M > 1
        traj_store = cell(M, 1);
        log_store = cell(M, 1);
        parfor m = 1:M
            disp(['正在运行方案 ', algo_cfg.name, ' 的第 ', num2str(m), ' 次蒙特卡洛仿真...']);
            [OSPA, OSPA_sensor, Num_estimate, Time, Sensor_traj, X_est_series, decision_log] = ...
                PROCESS(m, Xreal_time_target, Sensor_distr, N, GridMap, selection, sensor, algo_cfg.control_opt, process_opt, parallel_opt);
            OSPA_total = OSPA_total + OSPA;
            OSPA_total_sensor = OSPA_total_sensor + OSPA_sensor;
            Num_total = Num_total + Num_estimate;
            Time_total = Time_total + Time;
            traj_store{m} = {Sensor_traj, X_est_series};
            log_store{m} = decision_log;
        end
        Sensor_traj_vis = traj_store{1}{1};
        X_est_vis = traj_store{1}{2};
        decision_log_vis = log_store{1};
    else
        for m = 1:M
            disp(['正在运行方案 ', algo_cfg.name, ' 的第 ', num2str(m), ' 次蒙特卡洛仿真...']);
            [OSPA, OSPA_sensor, Num_estimate, Time, Sensor_traj, X_est_series, decision_log] = ...
                PROCESS(m, Xreal_time_target, Sensor_distr, N, GridMap, selection, sensor, algo_cfg.control_opt, process_opt, parallel_opt);
            OSPA_total = OSPA_total + OSPA;
            OSPA_total_sensor = OSPA_total_sensor + OSPA_sensor;
            Num_total = Num_total + Num_estimate;
            Time_total = Time_total + Time;
            if m == 1
                Sensor_traj_vis = Sensor_traj;
                X_est_vis = X_est_series;
                decision_log_vis = decision_log;
            end
        end
    end

    result = struct();
    result.name = algo_cfg.name;
    result.tag = algo_cfg.tag;
    result.control_opt = algo_cfg.control_opt;
    result.OSPA_avg = OSPA_total / M;
    result.OSPA_avg_sensor = OSPA_total_sensor / M;
    result.Num_avg = Num_total / M;
    result.Time_avg = Time_total / M;
    result.Sensor_traj_vis = Sensor_traj_vis;
    result.X_est_vis = X_est_vis;
    result.decision_log = decision_log_vis;
end
