%{
    按指定蒙特卡洛次数运行第四章算法A（两阶段管控），
    并以与 run_control_scheme 类似的方式汇总平均结果。
%}
function result = run_control_twostage_scheme(Xreal_time_target, Sensor_distr_A, N, GridMap, sim_cfg, ch4_cfg, M, base_seed)
    if nargin < 7 || isempty(M)
        M = 1;
    end
    if nargin < 8
        base_seed = [];
    end
    if ~isempty(base_seed)
        rng(base_seed);
    end

    OSPA_total = zeros(1, N);
    Num_total = zeros(1, N);
    Time_total = 0;
    step_time_total = zeros(1, N);
    Sensor_traj_vis = [];
    X_est_vis = [];
    task_log_vis = [];
    mc_sensor_traj = cell(M, 1);

    if M > 1
        % 先确保有并行池（与B算法并行蒙特卡洛行为保持一致）
        if isempty(gcp('nocreate'))
            parpool('local', min(feature('numcores'), M));
        end

        OSPA_store = cell(M, 1);
        Num_store = cell(M, 1);
        Time_store = zeros(M, 1);
        step_time_store = cell(M, 1);
        traj_store = cell(M, 1);
        est_store = cell(M, 1);
        task_store = cell(M, 1);

        disp(['算法 A 蒙特卡洛并行运行中，总次数 M=', num2str(M), ' ...']);
        parfor m = 1:M
            Sensor_distr_A_m = Sensor_distr_A;
            runtime_opt = struct('verbose', false);
            res_m = run_control_twostage(Xreal_time_target, Sensor_distr_A_m, N, GridMap, sim_cfg, ch4_cfg, runtime_opt);

            OSPA_store{m} = res_m.OSPA_avg;
            Num_store{m} = res_m.Num_avg;
            Time_store(m) = res_m.Time_avg;

            if isfield(res_m, 'step_time_series') && ~isempty(res_m.step_time_series)
                step_time_store{m} = res_m.step_time_series;
            else
                tmp = zeros(1, N);
                tmp(3:end) = res_m.Time_avg;
                step_time_store{m} = tmp;
            end

            traj_store{m} = res_m.Sensor_traj_vis;
            mc_sensor_traj{m} = res_m.Sensor_traj_vis;
            est_store{m} = res_m.X_est_vis;
            if isfield(res_m, 'task_log')
                task_store{m} = res_m.task_log;
            else
                task_store{m} = [];
            end
        end

        for m = 1:M
            OSPA_total = OSPA_total + OSPA_store{m};
            Num_total = Num_total + Num_store{m};
            Time_total = Time_total + Time_store(m);
            step_time_total = step_time_total + step_time_store{m};
        end

        Sensor_traj_vis = traj_store{1};
        X_est_vis = est_store{1};
        task_log_vis = task_store{1};
        disp('算法 A 并行蒙特卡洛运行完成。');
    else
        for m = 1:M
            disp(['正在运行算法 A 的第 ', num2str(m), '/', num2str(M), ' 次蒙特卡洛仿真...']);
            Sensor_distr_A_m = Sensor_distr_A;
            runtime_opt = struct('verbose', true);
            res_m = run_control_twostage(Xreal_time_target, Sensor_distr_A_m, N, GridMap, sim_cfg, ch4_cfg, runtime_opt);

            OSPA_total = OSPA_total + res_m.OSPA_avg;
            Num_total = Num_total + res_m.Num_avg;
            Time_total = Time_total + res_m.Time_avg;

            if isfield(res_m, 'step_time_series') && ~isempty(res_m.step_time_series)
                step_time_total = step_time_total + res_m.step_time_series;
            else
                tmp = zeros(1, N);
                tmp(3:end) = res_m.Time_avg;
                step_time_total = step_time_total + tmp;
            end

            if m == 1
                Sensor_traj_vis = res_m.Sensor_traj_vis;
                X_est_vis = res_m.X_est_vis;
                if isfield(res_m, 'task_log')
                    task_log_vis = res_m.task_log;
                end
            end
            mc_sensor_traj{m} = res_m.Sensor_traj_vis;
        end
    end

    result = struct();
    result.name = '本文算法：任务分配两阶段管控';
    result.OSPA_avg = OSPA_total / M;
    result.Num_avg = Num_total / M;
    result.Time_avg = Time_total / M;
    result.step_time_series = step_time_total / M;
    result.cum_time_series = cumsum(result.step_time_series);
    result.Sensor_traj_vis = Sensor_traj_vis;
    result.X_est_vis = X_est_vis;
    result.task_log = task_log_vis;
    result.mc_sensor_traj = mc_sensor_traj;
end
