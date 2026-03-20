function Sensor_distr = dynamic_task_allocation(Sensor_distr, v_fused, ch4_cfg)
    % DYNAMIC_TASK_ALLOCATION 动态任务分配机制 (对应论文第四章 4.2 节)
    % 输入:
    %   Sensor_distr: 包含传感器当前状态、位置及任务计数器的结构体数组
    %   v_fused:      当前时刻融合后的全局后验强度 (高斯混合形式)
    %   ch4_cfg:      包含 k_steps, theta_l, theta_h, W 的配置结构体
    % 输出:
    %   Sensor_distr: 更新了 task_type 和 计数器的传感器结构体
    
    N_sensor = length(Sensor_distr);
    
    %% 步骤 0: 提取当前已确认的目标状态
    % 从融合后的 GM-PHD 中提取权重大于阈值的有效目标
    weight_threshold = 0.1;
    % v_fused 来自 Centralized_Fusion_AGM，格式为 4x1 cell：{w, m, P, J}
    w_all = v_fused{1,1};
    m_all = v_fused{2,1};
    P_all = v_fused{3,1};
    J_all = v_fused{4,1};
    
    if isempty(w_all) || J_all == 0
        valid_idx = [];
    else
        valid_idx = find(w_all > weight_threshold);
    end
    N_targets = length(valid_idx);
    
    targets_m = [];
    if N_targets > 0
        targets_m = m_all(:, valid_idx); % 有效目标均值 (6*N_targets)
    end
    
    %% 步骤 1: 更新各传感器的连续观测计数器
    % 遍历传感器，严格使用第三章的 FoV_judge 判断视域内是否有目标
    for i = 1 : N_sensor
        has_target_in_fov = false;
        sensor_pos = Sensor_distr(i).location;
        r_detect = Sensor_distr(i).R_detect;
        
        for j = 1 : N_targets
            target_state = targets_m(:, j); % 提取目标的 6x1 状态向量
            % 调用第三章原有的视域判断函数
            if FoV_judge(sensor_pos, target_state, r_detect) == 1
                has_target_in_fov = true;
                break;
            end
        end
        
        if has_target_in_fov
            Sensor_distr(i).target_in_fov_count = Sensor_distr(i).target_in_fov_count + 1;
            Sensor_distr(i).no_target_in_fov_count = 0;
        else
            Sensor_distr(i).no_target_in_fov_count = Sensor_distr(i).no_target_in_fov_count + 1;
            Sensor_distr(i).target_in_fov_count = 0;
        end
    end
    
    %% 步骤 2: 响应目标出生与消亡的基于时间的任务转换
    for i = 1 : N_sensor
        if strcmp(Sensor_distr(i).task_type, 'S')
            % 若连续 k 个时刻视域内有目标：S -> T
            if Sensor_distr(i).target_in_fov_count >= ch4_cfg.k_steps
                Sensor_distr(i).task_type = 'T';
                Sensor_distr(i).target_in_fov_count = 0; % 重置计数器
            end
        elseif strcmp(Sensor_distr(i).task_type, 'T')
            % 若连续 k 个时刻视域内无目标：T -> S
            if Sensor_distr(i).no_target_in_fov_count >= ch4_cfg.k_steps
                Sensor_distr(i).task_type = 'S';
                Sensor_distr(i).no_target_in_fov_count = 0; % 重置计数器
            end
        end
    end
    
    %% 步骤 3: 基于当前跟踪性能的任务分配调节
    if N_targets > 0
        W = ch4_cfg.W;
        trace_P = zeros(1, N_targets);
        
        % 计算每个目标的位置误差协方差迹
        for j = 1 : N_targets
            comp_idx = valid_idx(j); % 在全局分量集合里的索引
            Pj = P_all(:, 6*(comp_idx-1)+1 : 6*comp_idx); % 6x6
            trace_P(j) = trace(W * Pj * W');
        end
        
        % 平均跟踪不确定度 \overline{\theta}^t
        theta_avg = sum(trace_P) / N_targets; 
        
        if theta_avg > ch4_cfg.theta_h
            % 跟踪性能较差：寻找最差目标，分配距离最近的 S 传感器转为 T
            [~, worst_idx] = max(trace_P);
            % 状态向量位置分量在 [1,3,5]（x,y,z）
            worst_target_pos = targets_m([1,3,5], worst_idx);
            
            s_idx = find(arrayfun(@(x) strcmp(x.task_type, 'S'), Sensor_distr));
            
            if ~isempty(s_idx)
                min_dist = inf;
                best_s_to_convert = -1;
                
                for s = s_idx
                    % 论文要求“选择距离该目标最近的S传感器”
                    dist = norm(Sensor_distr(s).location(1:3) - worst_target_pos);
                    if dist < min_dist
                        min_dist = dist;
                        best_s_to_convert = s;
                    end
                end
                
                if best_s_to_convert ~= -1
                    Sensor_distr(best_s_to_convert).task_type = 'T';
                end
            end
            
        elseif theta_avg < ch4_cfg.theta_l
            % 跟踪性能过剩：寻找最好目标，释放距离最远的 T 传感器转为 S
            [~, best_idx] = min(trace_P);
            best_target_state = targets_m(:, best_idx);
            best_target_pos = best_target_state([1,3,5]);
            
            t_idx = find(arrayfun(@(x) strcmp(x.task_type, 'T'), Sensor_distr));
            
            % 统计跟踪该最佳目标的 T 传感器数量 (严格依据 FoV_judge)
            tracking_T_sensors = [];
            for t = t_idx
                if FoV_judge(Sensor_distr(t).location, best_target_state, Sensor_distr(t).R_detect) == 1
                    tracking_T_sensors = [tracking_T_sensors, t];
                end
            end
            
            % 只有当跟踪该目标的传感器数量大于 1 时，才释放资源
            if length(tracking_T_sensors) > 1
                max_dist = -inf;
                best_t_to_convert = -1;
                
                for t = tracking_T_sensors
                    % 论文要求“选择距离该目标最远的一个T传感器”
                    dist = norm(Sensor_distr(t).location(1:3) - best_target_pos);
                    if dist > max_dist
                        max_dist = dist;
                        best_t_to_convert = t;
                    end
                end
                
                if best_t_to_convert ~= -1
                    Sensor_distr(best_t_to_convert).task_type = 'S';
                end
            end
        end
    end
end