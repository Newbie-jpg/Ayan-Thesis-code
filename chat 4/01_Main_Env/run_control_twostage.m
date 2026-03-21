function result = run_control_twostage(Xreal_time_target, Sensor_distr, N, GridMap, sim_cfg, ch4_cfg)
    % RUN_CONTROL_TWOSTAGE 第四章：基于任务分配的两阶段多传感器管控方法核心主循环
    
    N_sensor = length(Sensor_distr);
    sensor_params = sim_cfg.sensor; 
    
    % --- 一些系统默认运行参数 ---
    Ps = 1; Vx_thre = 200; Vy_thre = 200; Vz_thre = 200;
    match_threshold = sim_cfg.algo_baseline.match_threshold;
    control_interval = 1; % 动作更新间隔
    
    % 初始化记录变量
    task_log = cell(N, 1); 
    OSPA = zeros(1, N);
    Num_estimate = zeros(1, N);
    Time_total = 0;
    X_est_global = cell(N, 1);
    Sensor_traj = zeros(3, N, N_sensor);
    
    Sensor = Sensor_distr;

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
        
        % 提取全局估计用于计算指标
        [X_est_global{t,1},~] = statedraw_3d(Fusion_center.results);
        
        %% ====== 【第四章核心阶段 3：任务分配】 ======
        % 将融合结果赋给 v_fused，供任务分配使用
        v_fused = Fusion_center.results; 
        
        % 更新全局网格概率到 GridMap
        GridMap.GridProb = Fusion_center.GridProb_fused;
        
        % 调用第四章的任务分配逻辑
        Sensor = dynamic_task_allocation(Sensor, v_fused, ch4_cfg);
        
        % 记录当前角色用于画图
        current_roles = {Sensor.task_type};
        task_log{t} = current_roles;
        
        %% ====== 【第四章核心阶段 4：分组决策优化】 ======
        s_indices = find(strcmp(current_roles, 'S'));
        t_indices = find(strcmp(current_roles, 'T'));
        opt_actions = zeros(1, N_sensor);
        
        % --- (1) 针对 S 传感器：贪心网格规划 ---
        if ~isempty(s_indices)
            S_sensors_array = Sensor(s_indices);
            [s_opt_actions, ~] = greedy_path_planner(S_sensors_array, GridMap, sensor_params, ch4_cfg.search_threshold);
            for idx = 1:length(s_indices)
                opt_actions(s_indices(idx)) = s_opt_actions(idx);
            end
        end
        
        % --- (2) 针对 T 传感器：跟踪精度优化（联合分组决策）---
        if ~isempty(t_indices)
            % 伪预测：为 k+L|k 生成先验 GM-PHD（CV 模型）
            T_s = sensor_params.T;
            F = [1 T_s 0 0 0 0;
                 0 1 0 0 0 0;
                 0 0 1 T_s 0 0;
                 0 0 0 1 0 0;
                 0 0 0 0 1 T_s;
                 0 0 0 0 0 1];
            Q = diag([1, 0.1, 1, 0.1, 1, 0.1]);
            
            v_fused_pred = PHD_predict_L_steps(v_fused, sensor_params.L, F, Q);
            
            % 提取用于生成伪量测的先验目标集合 X_prior
            weight_thresh = 0.1;
            X_prior = [];
            if v_fused_pred{4,1} > 0
                ww = v_fused_pred{1,1};
                mm = v_fused_pred{2,1};
                for jj = 1:v_fused_pred{4,1}
                    if ww(jj) >= weight_thresh
                        X_prior = [X_prior, mm(:, jj)];
                    end
                end
            end
            
            % 生成所有传感器在每个候选动作下的 PIMS（用于 T 分组联合跟踪决策）
            [PIMS_cell, loc_after, valid_idx] = PIMS_generate(X_prior, Sensor, sensor_params, sensor_params.L);
            
            % 对 T 传感器集合做联合枚举，最小化 J_track
            opt_actions_T = control_core_T(t_indices, Sensor, v_fused_pred, ...
                PIMS_cell, loc_after, valid_idx, ch4_cfg, sensor_params, match_threshold);
            
            for idx = 1:length(t_indices)
                opt_actions(t_indices(idx)) = opt_actions_T(idx);
            end
        end
        
        %% 【阶段 5】：执行动作与状态推进
        for i = 1 : N_sensor
            chosen_action_idx = opt_actions(i);
            if chosen_action_idx == 0 
                chosen_action_idx = find(sensor_params.C == 0); % 异常兜底：直飞
            end
            
            % 读取选中的偏航角并更新传感器位置
            act_beta = sensor_params.C(chosen_action_idx);
            act_epsilon = 0; % 假设只在XY平面转弯，Z固定
            dt_sec = sensor_params.T;
            
            dx = sensor_params.v * cos(deg2rad(act_epsilon)) * cos(deg2rad(act_beta)) * dt_sec;
            dy = sensor_params.v * cos(deg2rad(act_epsilon)) * sin(deg2rad(act_beta)) * dt_sec;
            dz = sensor_params.v * sin(deg2rad(act_epsilon)) * dt_sec;
            
            Sensor(i).location = Sensor(i).location + [dx; dy; dz];
            Sensor_traj(:, t, i) = Sensor(i).location(:);
        end
        
        %% 记录该步耗时与性能指标
        dt_step = toc(timing);
        Time_total = Time_total + dt_step;
        
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
    result.X_est_vis = X_est_global;
    result.Sensor_traj_vis = Sensor_traj;
end