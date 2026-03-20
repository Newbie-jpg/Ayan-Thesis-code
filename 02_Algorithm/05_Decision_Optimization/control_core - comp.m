%{
    核心控制函数（chapter3.tex 决策优化）
    功能：接收 GM-PHD 融合后验与各传感器状态 -> PIMS 伪预测/伪更新 -> 伪融合
          -> 联合目标函数 J_total = J_track 求最优控制动作（无网格搜索项）
    控制动作：仅在 sensor.C（偏航角）范围内选择，俯仰角固定为 0，不融合俯仰与方向。
    输入：
        fused_posterior:  融合后验 (4*1 cell, Fusion_center.results)
        sensor_inf:       各传感器信息 (含 gm_particles, location, R_detect)
        GridMap:          保留接口兼容（未使用）
        sensor:           运动参数 (num, v, C, L, T)；C 为可选偏航角集合 (度)
        opt:               可选结构体 (match_threshold, eta, beta, W, L)
    输出：
        selection:        各传感器选中的动作下标 (1*N_sensor，对应 sensor.C 的索引)
        action:           控制动作 (2*N_sensor: 第1行=选中偏航角, 第2行=0 固定俯仰)
        cost:              最优 J_total 值
%}
function [selection, action, cost] = control_core(fused_posterior, sensor_inf, GridMap, sensor, opt)
    if nargin < 5 || isempty(opt)
        opt = struct();
    end
    if ~isfield(opt, 'match_threshold'), opt.match_threshold = 200; end
    if ~isfield(opt, 'eta'),    opt.eta = 0.5; end
    if ~isfield(opt, 'beta'),   opt.beta = 10; end
    if ~isfield(opt, 'W'),      opt.W = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; end  % 提取位置
    if ~isfield(opt, 'L'),      opt.L = sensor.L; end
    if isempty(opt.L),          opt.L = 2; end
    L = opt.L;
    N_sensor = length(sensor_inf);
    C_list = sensor.C;
    nC = length(C_list);

    % 运动模型 (与 ALG3 一致)
    T_s = 1;
    F = [1 T_s 0 0 0 0; 0 1 0 0 0 0; 0 0 1 T_s 0 0; 0 0 0 1 0 0; 0 0 0 0 1 T_s; 0 0 0 0 0 1];
    Q = diag([1, 0.1, 1, 0.1, 1, 0.1]);

    % ----- 1. 伪预测：v_{k+L|k} -----
    state_pred = PHD_predict_L_steps(fused_posterior, L, F, Q);
    J_pred = state_pred{4,1};

    % ----- 2. 提取先验目标状态 (用于 PIMS 与基数) -----
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
    if isempty(X_prior)
        N_pred = 0;
    end

    % ----- 3. PIMS 生成与动作剪枝 -----
    [PIMS_cell, loc_after, valid_idx] = PIMS_generate(X_prior, sensor_inf, sensor, L);

    % ----- 4. 遍历联合动作（在有效动作空间内）求 J_total -----
    valid_actions = valid_idx;
    nv = cellfun(@length, valid_actions);
    n_comb = prod(nv);
    if n_comb == 0
        n_comb = 1;
        indices_list = ones(N_sensor, 1);
    elseif n_comb > 5000
        indices_list = zeros(N_sensor, 500);
        for ss = 1:500
            for i = 1:N_sensor
                indices_list(i, ss) = valid_actions{i}(randi(nv(i)));
            end
        end
        n_comb = 500;
    else
        indices_list = zeros(N_sensor, n_comb);
        sub = cell(1, N_sensor);
        [sub{:}] = ndgrid(valid_actions{:});
        for i = 1:N_sensor
            indices_list(i, :) = sub{i}(:)';
        end
    end

    J_best = inf;
    best_sel = ones(1, N_sensor);
    for cc = 1:n_comb
        sel = indices_list(:, cc)';
        % 伪更新：每个传感器 i 用 PIMS_cell{i, sel(i)} 和 loc_after{i, sel(i)}
        pseudo_inf = sensor_inf;
        for i = 1:N_sensor
            ci = sel(i);
            Zi = PIMS_cell{i, ci};
            loc_i = loc_after{i, ci};
            R_params_i = [];
            if isfield(sensor_inf(i), 'R_params') && ~isempty(sensor_inf(i).R_params)
                R_params_i = sensor_inf(i).R_params;
            else
                R_params_i = struct('r_0', 10, 'theta_0', 0.5, 'eta_r', 0.01, 'eta_theta', 0.0005);
            end
            post_i = PHD_pseudo_update(state_pred, Zi, loc_i, R_params_i, 1);
            pseudo_inf(i).gm_particles = post_i;
            pseudo_inf(i).location = loc_i;
        end
        % 伪融合
        pseudo_fused = Centralized_Fusion_AGM(pseudo_inf, opt.match_threshold);
        N_post = pseudo_fused{4,1};

        % J_track (式 3.29)
        J_tr = 0;
        if N_post > 0
            for j = 1:N_post
                Pj = pseudo_fused{3,1}(:, 6*(j-1)+1 : 6*j);
                J_tr = J_tr + trace(opt.W * Pj * opt.W');
            end
            J_tr = J_tr + opt.beta * max(0, N_pred - N_post);
        else
            J_tr = opt.beta * max(0, N_pred - 0);
        end

        J_total = J_tr;
        if J_total < J_best
            J_best = J_total;
            best_sel = sel;
        end
    end

    selection = best_sel;
    cost = J_best;
    % 仅偏航角从 sensor.C 选择；俯仰角固定为 0，不参与选择
    action = zeros(2, N_sensor);
    action(1, :) = C_list(selection);   % 各传感器选中的偏航角 (度)
    action(2, :) = 0;                   % 俯仰角固定 0°
end
