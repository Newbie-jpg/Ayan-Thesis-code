function result = run_control_twostage(Xreal_time_target, Sensor_distr, N, GridMap, sim_cfg, ch4_cfg)
    % RUN_CONTROL_TWOSTAGE 第四章：基于任务分配的两阶段多传感器管控方法核心主循环
    
    N_sensor = length(Sensor_distr);
    sensor_params = sim_cfg.sensor; 
    
    % --- 一些系统默认运行参数 ---
    Ps = 1; Vx_thre = 200; Vy_thre = 200; Vz_thre = 200;
    match_threshold = sim_cfg.algo_baseline.match_threshold;
    control_interval = 5; % 动作更新间隔（每 5 个时刻决策一次）
    
    % 初始化记录变量
    task_log = cell(N, 1); 
    OSPA = zeros(1, N);
    Num_estimate = zeros(1, N);
    Time_total = 0;
    step_time_series = zeros(1, N);
    X_est_global = cell(N, 1);
    Sensor_traj = zeros(3, N, N_sensor);
    
    Sensor = Sensor_distr;
    fixed_groups = ch4_cfg.fixed_search_target_groups;
    target_init = sim_cfg.target_init;
    oracle_cfg_A = ch4_cfg.oracle_cfg;
    use_oracle_guidance = true;
    if isfield(ch4_cfg, 'enable_oracle_guidance') && ~isempty(ch4_cfg.enable_oracle_guidance)
        use_oracle_guidance = logical(ch4_cfg.enable_oracle_guidance);
    end
    default_heading = 0;
    prev_heading_deg = default_heading * ones(1, N_sensor);
    last_action_cmd = [];
    last_roles = repmat({'S'}, 1, N_sensor);
    last_opt_actions = [];

    % 进度/剩余时间估计（算法A）
    tic_total = tic;
    total_steps = N - 2; % t 从 3 开始，共 N-2 次决策
    progress_refresh = max(1, floor(total_steps / 10)); % 约每10%刷新一次

    % 为了提高CPU利用率：若 Parallel Toolbox 可用且当前没有并行池，
    % 则在算法A开始时创建一个小型并行池，供 control_core_T 的 parfor 使用。
    if (exist('parpool', 'file') == 2) && (exist('gcp', 'file') == 2)
        if isempty(gcp('nocreate'))
            try
                pool_size = min(feature('numcores'), max(2, N_sensor));
                parpool('local', pool_size);
            catch
                % 若并行池创建失败，则继续使用串行计算
            end
        end
    end
    
    % === 预处理与 t=1, t=2 时刻的初始化 (直接复用 PROCESS.m 逻辑) ===
    ALG3_PHD_initial; 
    for i = 1:N_sensor
        Z_p1 = observe_FoV_3d_single(Xreal_time_target{1,1}, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
        Sensor(i).Z_dicaer_global{1,1} = polar2dicaer_3d_single(Z_p1, Sensor(i).location(1), Sensor(i).location(2), Sensor(i).location(3));
        
        Z_p2 = observe_FoV_3d_single(Xreal_time_target{2,1}, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
        Sensor(i).Z_dicaer_global{2,1} = polar2dicaer_3d_single(Z_p2, Sensor(i).location(1), Sensor(i).location(2), Sensor(i).location(3));
        
        % 记录轨迹初始点（t=1,2）
        Sensor_traj(:,1,i) = Sensor(i).location(:);
        if N >= 2
            Sensor_traj(:,2,i) = Sensor(i).location(:);
        end
    end
    X_est_global{1,1} = zeros(6,0); X_est_global{2,1} = zeros(6,0);
    
    % 初始化任务角色日志（t=1,2 时刻用于可视化脚本定位传感器数量）
    task_log{1} = {Sensor.task_type};
    task_log{2} = {Sensor.task_type};
    
    %% === 核心时间步循环 (从 t=3 开始) ===
    for t = 3:N
        % 避免高频 console 输出影响性能（只在刷新窗口内显示）
        if mod(t - 2, progress_refresh) == 0 || t == N
            disp(['--- 正在仿真 第 ', num2str(t), ' 步 ---']);
        end
        timing = tic;
        
        %% 【阶段 1】：环境更新与局部状态估计 (复用第三章底层)
        for i = 1:N_sensor 
            % 1. 观测
            Z_p_t = observe_FoV_3d_single(Xreal_time_target{t,1}, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
            Sensor(i).Z_polar_part{t,1} = Z_p_t;
            Sensor(i).Z_dicaer_global{t,1} = polar2dicaer_3d_single(Z_p_t, Sensor(i).location(1), Sensor(i).location(2), Sensor(i).location(3));
            
            % 2. 滤波
            [Sensor(i).state] = ALG3_PHD1time_3d_ukf_distr(Sensor(i).Z_dicaer_global{t-2,1},...
                Sensor(i).Z_dicaer_global{t-1,1}, Sensor(i).Z_polar_part{t,1},...
                Sensor(i).state, Sensor(i).Zr, Sensor(i).R_params, Ps, Sensor(i).Pd,...
                Vx_thre, Vy_thre, Vz_thre,...
                Sensor(i).location(1,1), Sensor(i).location(2,1), Sensor(i).location(3,1));   

            % 3. 网格更新
            Sensor(i).GridProb = Update_Grid_Density(GridMap, Sensor(i).GridProb, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd);
        end
        
        %% 【阶段 2】：多传感器信息融合 (复用第三章底层)
        Fusion_center = struct();
        for i=1:N_sensor 
            Fusion_center.sensor_inf(i).gm_particles = Sensor(i).state; 
            Fusion_center.sensor_inf(i).location = Sensor(i).location;  
            Fusion_center.sensor_inf(i).R_detect = Sensor(i).R_detect;  
            Fusion_center.sensor_inf(i).serial = Sensor(i).serial;      
            Fusion_center.sensor_inf(i).GridProb = Sensor(i).GridProb;  
            Fusion_center.sensor_inf(i).Pd = Sensor(i).Pd;              
            Fusion_center.sensor_inf(i).R_params = Sensor(i).R_params;    
        end
        
        % 运行融合算法
        [Fusion_center.results] = Centralized_Fusion_AGM(Fusion_center.sensor_inf, match_threshold);
        Fusion_center.GridProb_fused = Fuse_GridProb_GA(Fusion_center.sensor_inf);
        GridMap.GridProb = Fusion_center.GridProb_fused;
        
        % 提取全局估计用于计算指标
        [X_est_global{t,1},~] = statedraw_3d(Fusion_center.results);
        
        %% ====== 【阶段 3+4：按开关选择策略】 ======
        % 方案A-Oracle: 视域内带噪真值跟踪 + 视域外固定组合搜索
        % 方案A-Paper : 第四章论文原始两阶段逻辑
        if use_oracle_guidance
            if isempty(last_action_cmd) || mod(t - 3, control_interval) == 0
                [action_cmd, current_roles, heading_deg] = build_oracle_guidance_actions( ...
                    Sensor, Xreal_time_target{t,1}, target_init, fixed_groups, sensor_params, oracle_cfg_A, t, prev_heading_deg);
                prev_heading_deg = heading_deg;
                last_action_cmd = action_cmd;
                last_roles = current_roles;
            else
                action_cmd = last_action_cmd;
                current_roles = last_roles;
            end
            task_log{t} = current_roles;
        else
            if isempty(last_opt_actions) || mod(t - 3, control_interval) == 0
                v_fused = Fusion_center.results;
                Sensor = dynamic_task_allocation(Sensor, v_fused, ch4_cfg);
                current_roles = {Sensor.task_type};
                opt_actions = zeros(1, N_sensor);

                s_indices = find(strcmp(current_roles, 'S'));
                t_indices = find(strcmp(current_roles, 'T'));

                if ~isempty(s_indices)
                    S_sensors_array = Sensor(s_indices);
                    [s_opt_actions, ~] = greedy_path_planner(S_sensors_array, GridMap, sensor_params, ch4_cfg.search_threshold);
                    for idx = 1:length(s_indices)
                        opt_actions(s_indices(idx)) = s_opt_actions(idx);
                    end
                end

                if ~isempty(t_indices)
                    T_s = sensor_params.T;
                    F = [1 T_s 0 0 0 0;
                         0 1 0 0 0 0;
                         0 0 1 T_s 0 0;
                         0 0 0 1 0 0;
                         0 0 0 0 1 T_s;
                         0 0 0 0 0 1];
                    Q = diag([1, 0.1, 1, 0.1, 1, 0.1]);
                    v_fused_pred = PHD_predict_L_steps(v_fused, sensor_params.L, F, Q);

                    weight_thresh = 0.1;
                    X_prior = [];
                    if v_fused_pred{4,1} > 0
                        ww = v_fused_pred{1,1};
                        mm = v_fused_pred{2,1};
                        for jj = 1:v_fused_pred{4,1}
                            if ww(jj) >= weight_thresh
                                X_prior = [X_prior, mm(:, jj)]; %#ok<AGROW>
                            end
                        end
                    end

                    [PIMS_cell, loc_after, valid_idx] = PIMS_generate(X_prior, Sensor, sensor_params, sensor_params.L);
                    opt_actions_T = control_core_T(t_indices, Sensor, v_fused_pred, ...
                        PIMS_cell, loc_after, valid_idx, ch4_cfg, sensor_params, match_threshold);
                    for idx = 1:length(t_indices)
                        opt_actions(t_indices(idx)) = opt_actions_T(idx);
                    end
                end

                last_opt_actions = opt_actions;
                last_roles = current_roles;
            else
                opt_actions = last_opt_actions;
                current_roles = last_roles;
            end

            action_cmd = zeros(2, N_sensor);
            for i = 1:N_sensor
                idx_i = opt_actions(i);
                if idx_i <= 0 || idx_i > numel(sensor_params.C)
                    idx0 = find(sensor_params.C == 0, 1, 'first');
                    if isempty(idx0), idx0 = 1; end
                    idx_i = idx0;
                end
                action_cmd(1, i) = sensor_params.C(idx_i);
                action_cmd(2, i) = 0;
            end
            task_log{t} = current_roles;
        end
        %% 【阶段 5】：执行动作与状态推进
        for i = 1 : N_sensor
            act_beta = action_cmd(1, i);
            act_epsilon = 0;
            dt_sec = sensor_params.T;
            v_eff = sensor_params.v;
            if use_oracle_guidance && strcmp(current_roles{i}, 'S') && isfield(oracle_cfg_A, 'search_speed_scale')
                v_eff = sensor_params.v * oracle_cfg_A.search_speed_scale;
            end
            
            dx = v_eff * cos(deg2rad(act_epsilon)) * cos(deg2rad(act_beta)) * dt_sec;
            dy = v_eff * cos(deg2rad(act_epsilon)) * sin(deg2rad(act_beta)) * dt_sec;
            dz = v_eff * sin(deg2rad(act_epsilon)) * dt_sec;
            
            Sensor(i).location = Sensor(i).location + [dx; dy; dz];
            Sensor_traj(:, t, i) = Sensor(i).location(:);
            Sensor(i).task_type = current_roles{i};
        end
        
        %% 记录该步耗时与性能指标
        dt_step = toc(timing);
        Time_total = Time_total + dt_step;
        step_time_series(t) = dt_step;
        
        % 更新剩余时间（低频刷新，避免刷屏）
        steps_done = t - 2;
        if mod(steps_done, progress_refresh) == 0 || t == N
            avg_step = Time_total / steps_done;
            steps_left = (N - t); % 已完成 t 后，剩余决策步数
            eta_sec = avg_step * steps_left;
            fprintf('>>> 算法A进度：%d/%d 步，单步均耗时 %.2f s，预计剩余 %.1f s (%.1f min)\n', ...
                steps_done, total_steps, avg_step, eta_sec, eta_sec/60);
        end
        
        Xreal_t = Xreal_time_target{t,1};
        Xreal_t(:, isnan(Xreal_t(1,:))) = []; % 剔除无效目标
        OSPA(t) = ospa_dist(Xreal_t, X_est_global{t,1}, 120, 2);
        Num_estimate(t) = size(X_est_global{t,1}, 2);
    end
    
    % --- 打包结果返回 ---
    result = struct();
    result.task_log = task_log;
    result.OSPA_avg = OSPA;
    result.Num_avg = Num_estimate;
    result.Time_avg = Time_total / (N - 2); 
    result.step_time_series = step_time_series;
    result.cum_time_series = cumsum(step_time_series);
    result.X_est_vis = X_est_global;
    result.Sensor_traj_vis = Sensor_traj;
end