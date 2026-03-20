function opt_action_idx = control_core_S(current_pos, next_target_v, sensor_params)
    % CONTROL_CORE_S 针对搜索(S)传感器的单节点动作决策
    % 对应论文第四章 4.4 节：针对搜索任务的控制动作目标函数
    % c_k^* = \arg\min_{c_k} \xi_{search}(c_k, v)
    %
    % 输入：
    %   current_pos:   传感器当前真实位置 (3x1)
    %   next_target_v: 分配到的下一个待搜索网格节点坐标 (3x1)
    %   sensor_params: 传感器动力学参数 (C, v, L, T)
    % 输出：
    %   opt_action_idx: 最优动作在可选集合 C 中的索引
    
    v_mag = sensor_params.v;
    dt = sensor_params.T;
    L_steps = sensor_params.L;
    action_set = sensor_params.C; 
    
    min_dist_to_v = inf;
    best_action_idx = -1;
    
    % 遍历允许动作空间 C_k
    for a_idx = 1 : length(action_set)
        yaw_angle = action_set(a_idx);
        
        % 预测执行候选动作 c_k 后的位置 (基于匀速直线模型预测 L 步)
        dx = v_mag * dt * L_steps * cosd(yaw_angle);
        dy = v_mag * dt * L_steps * sind(yaw_angle);
        pred_pos = current_pos + [dx; dy; 0];
        
        % 计算预测位置与目标节点 v 的欧氏距离作为代价 \xi_{search}
        dist = norm(pred_pos - next_target_v);
        
        if dist < min_dist_to_v
            min_dist_to_v = dist;
            best_action_idx = a_idx;
        end
    end
    
    opt_action_idx = best_action_idx;
end