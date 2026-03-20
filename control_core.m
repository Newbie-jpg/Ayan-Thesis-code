%{
    核心控制函数（融合版本：支持两类控制模式）
    功能：接收 GM-PHD 融合后验 -> PIMS 伪预测/更新 -> 伪融合 -> 优化控制动作
    模式：
        1) grid: 联合目标 J_total = J_track - eta*J_search（现有方案）
        2) cs:   最大化 D_CS（关闭区域网格机制）
        3) cs_improved: 最大化 theta_CS = (n_post/n_pred)*D_CS，并结合 J_search
%}
function [selection, action, cost, detail] = control_core(fused_posterior, sensor_inf, GridMap, sensor, opt)
    if nargin < 4
        error('control_core:NotEnoughInputs', ...
            ['control_core 需要至少 4 个输入参数: ', ...
             'fused_posterior, sensor_inf, GridMap, sensor。', ...
             '请通过 PROCESS/main 调用，或手动传入完整参数。']);
    end

    if nargin < 5 || isempty(opt)
        opt = struct();
    end
    if ~isfield(opt, 'match_threshold'), opt.match_threshold = 200; end
    if ~isfield(opt, 'eta'),    opt.eta = 10; end
    if ~isfield(opt, 'beta'),   opt.beta = 100; end
    if ~isfield(opt, 'W'),      opt.W = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; end
    if ~isfield(opt, 'L')
        if isstruct(sensor) && isfield(sensor, 'L') && ~isempty(sensor.L)
            opt.L = sensor.L;
        else
            opt.L = 2;
        end
    end
    if ~isfield(opt, 'objective_mode'), opt.objective_mode = 'grid'; end
    if isempty(opt.L),          opt.L = 2; end
    L = opt.L;
    objective_mode = lower(string(opt.objective_mode));
    N_sensor = length(sensor_inf);
    C_list = sensor.C;

    % 1. 伪预测：v_{k+L|k}
    if isstruct(sensor) && isfield(sensor, 'T') && ~isempty(sensor.T)
        T_s = sensor.T;
    else
        T_s = 1;
    end
    F = [1 T_s 0 0 0 0; 0 1 0 0 0 0; 0 0 1 T_s 0 0; 0 0 0 1 0 0; 0 0 0 0 1 T_s; 0 0 0 0 0 1];
    Q = diag([1, 0.1, 1, 0.1, 1, 0.1]);
    state_pred = PHD_predict_L_steps(fused_posterior, L, F, Q);
    J_pred = state_pred{4,1};

    % 2. 提取先验目标状态 (用于 PIMS)
    weight_thresh = 0.1;
    X_prior = [];
    if J_pred > 0
        ww = state_pred{1,1};
        mm = state_pred{2,1};
        for j = 1:J_pred
            if ww(j) >= weight_thresh
                X_prior = [X_prior, mm(:, j)];
            end
        end
    end
    N_pred = size(X_prior, 2);

    % 3. PIMS 生成与动作剪枝
    [PIMS_cell, loc_after, valid_idx] = PIMS_generate(X_prior, sensor_inf, sensor, L);

    % 4. 仅 pure-cs 模式关闭网格机制；其余模式构建网格 L 步预测项
    omega_pred_L = cell(1, N_sensor);
    use_grid_objective = (objective_mode ~= "cs");
    if use_grid_objective
        for i = 1:N_sensor
            if isfield(sensor_inf(i), 'GridProb') && ~isempty(sensor_inf(i).GridProb)
                omega_i = sensor_inf(i).GridProb(:);
            else
                omega_i = GridMap.omega0 * ones(GridMap.M, 1);
            end
            for ll = 1:L
                omega_i = Predict_Grid_Density(GridMap, omega_i);
            end
            omega_pred_L{i} = omega_i;
        end
    end

    % 5. 位置约束 + 遍历联合动作（确定性枚举，不随机采样）
    x_min = -1750; x_max = 1750;
    y_min = -1750; y_max = 1750;
    valid_actions = cell(1, N_sensor);
    for i = 1:N_sensor
        cand_i = valid_idx{i};
        bounded_i = [];
        for kk = 1:length(cand_i)
            ci = cand_i(kk);
            loc_ci = loc_after{i, ci};
            if ~isempty(loc_ci) && ...
               loc_ci(1) >= x_min && loc_ci(1) <= x_max && ...
               loc_ci(2) >= y_min && loc_ci(2) <= y_max
                bounded_i = [bounded_i, ci];
            end
        end

        % 若剪枝后为空，则在全部动作中找边界内动作；再为空则保留首个候选避免崩溃
        if isempty(bounded_i)
            for ci = 1:length(C_list)
                loc_ci = loc_after{i, ci};
                if ~isempty(loc_ci) && ...
                   loc_ci(1) >= x_min && loc_ci(1) <= x_max && ...
                   loc_ci(2) >= y_min && loc_ci(2) <= y_max
                    bounded_i = [bounded_i, ci];
                end
            end
        end
        if isempty(bounded_i)
            if ~isempty(cand_i)
                bounded_i = cand_i(1);
            else
                bounded_i = 1;
            end
        end
        valid_actions{i} = bounded_i;
    end

    nv = cellfun(@length, valid_actions);
    n_comb = prod(nv);
    has_empty_action = any(nv == 0);
    
    if n_comb == 0
        n_comb = 1;
    end

    J_best = inf;
    best_sel = ones(1, N_sensor);
    best_pseudo_fused = cell(4, 1);
    best_pseudo_inf = sensor_inf;
    best_J_tr = inf;
    best_J_cs = NaN;
    best_J_cs_improved = NaN;

    for cc = 1:n_comb
        if has_empty_action
            sel = zeros(1, N_sensor);
            for i = 1:N_sensor
                if isempty(valid_actions{i})
                    sel(i) = 1;
                else
                    sel(i) = valid_actions{i}(1);
                end
            end
        else
            % 将组合编号 cc 映射到各传感器动作索引（混合进制展开）
            rem_idx = cc - 1;
            sel = zeros(1, N_sensor);
            for i = 1:N_sensor
                base_i = nv(i);
                local_id = mod(rem_idx, base_i) + 1;
                rem_idx = floor(rem_idx / base_i);
                sel(i) = valid_actions{i}(local_id);
            end
        end
        pseudo_inf = sensor_inf;
        for i = 1:N_sensor
            ci = sel(i);
            Zi = PIMS_cell{i, ci};
            loc_i = loc_after{i, ci};
            
            % --- 修改点：使用参数化结构体 R_params ---
            if isfield(sensor_inf(i), 'R_params') && ~isempty(sensor_inf(i).R_params)
                R_params_i = sensor_inf(i).R_params;
            else
                % 默认参数
                R_params_i = struct('r_0', 10, 'theta_0', 0.5, 'eta_r', 0.01, 'eta_theta', 0.0005);
            end
            
            % 调用伪更新，传入结构体而非固定矩阵
            post_i = PHD_pseudo_update(state_pred, Zi, loc_i, R_params_i, 1);
            pseudo_inf(i).gm_particles = post_i;
            pseudo_inf(i).location = loc_i;
        end

        % 伪融合与代价计算
        pseudo_fused = Centralized_Fusion_AGM(pseudo_inf, opt.match_threshold);
        N_post = pseudo_fused{4,1};

        % J_track 计算
        J_tr = 0;
        if N_post > 0
            for j = 1:N_post
                Pj = pseudo_fused{3,1}(:, 6*(j-1)+1 : 6*j);
                J_tr = J_tr + trace(opt.W * Pj * opt.W');
            end
            J_tr = J_tr / N_post + opt.beta * max(0, N_pred - N_post);
        else
            J_tr = opt.beta * max(0, N_pred);
        end

        if objective_mode == "grid"
            J_info = compute_grid_gain(omega_pred_L, loc_after, sel, sensor_inf, GridMap);
            J_total = J_tr - opt.eta * J_info;
            J_cs_cand = NaN;
            J_cs_improved_cand = NaN;
        elseif objective_mode == "cs"
            % CS方案：最大化 D_cs(伪预测, 伪后验)；在最小化框架下等价为最小化 -D_cs
            J_cs_cand = safe_cs_divergence(state_pred, pseudo_fused);
            J_total = -J_cs_cand;
            J_cs_improved_cand = NaN;
        elseif objective_mode == "cs_improved"
            % 改进 CS：theta = (n_post / n_pred) * D_cs；
            % 联合网格收益：在最小化框架下等价于最小化 -theta - eta*J_search
            J_cs_cand = safe_cs_divergence(state_pred, pseudo_fused);
            J_cs_improved_cand = improved_cs_objective(state_pred, pseudo_fused, J_cs_cand);
            J_info = compute_grid_gain(omega_pred_L, loc_after, sel, sensor_inf, GridMap);
            J_total = -J_cs_improved_cand - opt.eta * J_info;
        else
            error('control_core:UnknownObjectiveMode', ...
                '未知 objective_mode: %s，支持 grid / cs / cs_improved。', char(objective_mode));
        end

        if J_total < J_best
            J_best = J_total;
            best_sel = sel;
            best_pseudo_fused = pseudo_fused;
            best_pseudo_inf = pseudo_inf;
            best_J_tr = J_tr;
            best_J_cs = J_cs_cand;
            best_J_cs_improved = J_cs_improved_cand;
        end
    end

    selection = best_sel;
    cost = J_best;
    action = [C_list(selection); zeros(1, N_sensor)]; % 俯仰角固定 0

    % 对最优动作补算两类信息增益，便于主程序做算法比较图
    if use_grid_objective
        best_J_sr = compute_grid_gain(omega_pred_L, loc_after, best_sel, sensor_inf, GridMap);
    else
        best_J_sr = NaN;
    end
    if objective_mode == "grid"
        best_J_cs = safe_cs_divergence(state_pred, best_pseudo_fused);
        best_info = best_J_sr;
    elseif objective_mode == "cs"
        if isnan(best_J_cs)
            best_J_cs = safe_cs_divergence(state_pred, best_pseudo_fused);
        end
        best_info = best_J_cs;
    else % cs_improved
        if isnan(best_J_cs)
            best_J_cs = safe_cs_divergence(state_pred, best_pseudo_fused);
        end
        if isnan(best_J_cs_improved)
            best_J_cs_improved = improved_cs_objective(state_pred, best_pseudo_fused, best_J_cs);
        end
        best_info = best_J_cs_improved;
    end

    detail = struct();
    detail.objective_mode = char(objective_mode);
    detail.track_cost = best_J_tr;
    detail.search_gain = best_J_sr;
    detail.cs_gain = best_J_cs;
    detail.cs_improved_gain = best_J_cs_improved;
    detail.info_gain = best_info;
    detail.total_cost = J_best;
    detail.selection = selection;
    detail.pred_components = N_pred;
    detail.post_components = best_pseudo_fused{4,1};
    detail.best_pseudo_fused = best_pseudo_fused;
    detail.best_sensor_inf = best_pseudo_inf;
end

function J_sr = compute_grid_gain(omega_pred_L, loc_after, sel, sensor_inf, GridMap)
    N_sensor = length(sensor_inf);
    pseudo_sensor = sensor_inf;
    pred_sensor = sensor_inf;
    for i = 1:N_sensor
        pred_sensor(i).GridProb = omega_pred_L{i};
        omega_pred_i = omega_pred_L{i};
        if isfield(sensor_inf(i), 'Pd') && ~isempty(sensor_inf(i).Pd)
            Pd_i = sensor_inf(i).Pd;
        else
            Pd_i = 1;
        end
        omega_after_i = Update_Grid_Density( ...
            GridMap, omega_pred_i, loc_after{i, sel(i)}, sensor_inf(i).R_detect, Pd_i);
        pseudo_sensor(i).GridProb = omega_after_i;
    end
    % chapter3 融合式：p_fuse = prod_i p_i^{pi_i}
    % 在当前最小化框架 J_total = J_track - eta*J_search 下，
    % 令 J_search 为“不确定性下降量”，避免出现符号反向导致的策略退化。
    GridProb_pred_fused = Fuse_GridProb_GA(pred_sensor);
    GridProb_post_fused = Fuse_GridProb_GA(pseudo_sensor);
    delta = max(GridProb_pred_fused - GridProb_post_fused, 0);
    J_sr = mean(delta);
end

function D_cs = safe_cs_divergence(state_pred, state_post)
    try
        D_cs = gmphd_cs_divergence(state_pred, state_post);
        if ~isfinite(D_cs)
            D_cs = 0;
        end
    catch
        D_cs = 0;
    end
end

function theta_cs = improved_cs_objective(state_pred, state_post, D_cs)
    n_pred = estimate_cardinality(state_pred);
    n_post = estimate_cardinality(state_post);
    if n_pred <= eps
        theta_cs = 0;
        return;
    end

    % 理论上伪后验不会引入新生目标，数值上做截断增强鲁棒性
    n_post_eff = min(max(n_post, 0), n_pred);
    theta_cs = (n_post_eff / n_pred) * max(D_cs, 0);
end

function n_hat = estimate_cardinality(state)
    n_hat = 0;
    if isempty(state) || numel(state) < 1 || isempty(state{1,1})
        return;
    end
    w = state{1,1};
    n_hat = sum(max(real(w(:)), 0));
    if ~isfinite(n_hat)
        n_hat = 0;
    end
end