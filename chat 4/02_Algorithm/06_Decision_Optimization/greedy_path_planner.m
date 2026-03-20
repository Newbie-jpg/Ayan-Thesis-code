function [opt_action_indices, planned_paths] = greedy_path_planner(S_sensors, GridMap, sensor_params, search_threshold)
    % GREEDY_PATH_PLANNER 针对搜索传感器的贪心路径规划与动作决策
    % 对应论文第四章 Algorithm 4.1 多传感器协同搜索算法
    %
    % 输入:
    %   S_sensors:        当前状态为 'S' 的传感器结构体数组
    %   GridMap:          全局网格地图结构体
    %   sensor_params:    传感器全局参数
    %   search_threshold: 搜索阈值
    % 输出:
    %   opt_action_indices: 每个 S 传感器最优控制动作在集合 C 中的索引
    %   planned_paths:      每个 S 传感器分配到的待搜索网格坐标序列
    
    num_S = length(S_sensors);
    opt_action_indices = zeros(1, num_S);
    planned_paths = cell(1, num_S);
    
    if num_S == 0
        return; 
    end
    
    %% 步骤 1: 确定待搜索区域集合 V_bar (U)
    valid_grid_idx = find(GridMap.GridProb > search_threshold);
    
    if isempty(valid_grid_idx)
        default_action_idx = find(sensor_params.C == 0); 
        opt_action_indices(:) = default_action_idx;
        return;
    end
    
    U_coords = GridMap.c(:, valid_grid_idx);
    num_U = size(U_coords, 2);
    
    %% 步骤 2: 初始化 (Algorithm 4.1 初始化阶段)
    last_nodes = zeros(3, num_S);
    for j = 1 : num_S
        last_nodes(:, j) = S_sensors(j).location(1:3);
        planned_paths{j} = []; 
    end
    
    delta_bar = 0; 
    unvisited_mask = true(1, num_U); 
    
    %% 步骤 3: 迭代分配节点 (Algorithm 4.1 循环)
    while any(unvisited_mask)
        delta_bar = delta_bar + 1;
        j = mod(delta_bar - 1, num_S) + 1; 
        
        current_node = last_nodes(:, j);
        min_cost = inf;
        best_u_idx = -1;
        
        for u = 1 : num_U
            if unvisited_mask(u)
                target_node = U_coords(:, u);
                cost = norm(target_node - current_node);
                if cost < min_cost
                    min_cost = cost;
                    best_u_idx = u;
                end
            end
        end
        
        if best_u_idx ~= -1
            allocated_node = U_coords(:, best_u_idx);
            planned_paths{j} = [planned_paths{j}, allocated_node];
            last_nodes(:, j) = allocated_node; 
            unvisited_mask(best_u_idx) = false; 
        end
    end
    
    %% 步骤 4: 基于分配序列的控制动作优化 (调用独立的 control_core_S)
    for j = 1 : num_S
        if isempty(planned_paths{j})
            % 如果某传感器未分到任何任务，保持直飞
            opt_action_indices(j) = find(sensor_params.C == 0);
            continue;
        end
        
        % 提取序列中的第一个目标节点 v
        next_target_v = planned_paths{j}(:, 1);
        current_pos = S_sensors(j).location(1:3);
        
        % 【修改处】直接调用独立的 S 传感器决策核心
        opt_action_indices(j) = control_core_S(current_pos, next_target_v, sensor_params);
    end
end