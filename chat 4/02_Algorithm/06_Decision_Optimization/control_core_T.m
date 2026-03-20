function opt_actions_T = control_core_T(T_indices, Sensor_distr, v_fused_pred, ...
    PIMS_cell, loc_after, valid_idx, ch4_cfg, sensor_params, match_threshold)
% CONTROL_CORE_T 针对跟踪(T)传感器集合的联合决策优化
% 对应论文第四章 4.3：J_track(c_k)= sum_j tr(W P_{k+L}^j W^T) + beta*(N_prior-N_post)
%
% 输入：
%   T_indices:        参与跟踪任务的传感器 id 列表
%   Sensor_distr:     全体传感器状态结构体数组
%   v_fused_pred:     k+L|k 先验 GM-PHD（4x1 cell：w,m,P,J）
%   PIMS_cell:        由 PIMS_generate 生成的伪量测集 cell(N_sensor,nC)
%   loc_after:        由 PIMS_generate 生成的 L 步后传感器位置 cell(N_sensor,nC)
%   valid_idx:        每个传感器可用动作下标（来自 PIMS_generate）
%   ch4_cfg, sensor_params: 配置
%   match_threshold: 传感器间高斯匹配阈值
%
% 输出：
%   opt_actions_T: 对每个 T 传感器的最优动作索引（对应 sensor_params.C 的 1-based 下标）

    if nargin < 9 || isempty(match_threshold)
        match_threshold = 200;
    end

    numT = length(T_indices);
    opt_actions_T = zeros(1, numT);
    if numT == 0
        return;
    end

    W = ch4_cfg.W;
    beta = ch4_cfg.beta;

    % 先验预测基数 N_pred（与 chat3/control_core.m 保持一致）
    % chat3 逻辑：从 state_pred 中按 weight_thresh=0.1 选取有效分量得到 X_prior，
    % 然后 N_pred = size(X_prior, 2)。
    weight_thresh = 0.1;
    w_pred = v_fused_pred{1,1};
    m_pred = v_fused_pred{2,1};
    J_pred = v_fused_pred{4,1};
    X_prior = [];
    if ~isempty(J_pred) && J_pred > 0
        for jj = 1:J_pred
            if w_pred(jj) >= weight_thresh
                X_prior = [X_prior, m_pred(:, jj)];
            end
        end
    end
    N_pred = size(X_prior, 2);

    % 为每个 T 传感器准备候选动作集合（动作索引 c）
    C_list = sensor_params.C;
    nC = length(C_list);
    cand_actions = cell(1, numT);
    for ii = 1:numT
        sid = T_indices(ii);
        if iscell(valid_idx) && sid <= numel(valid_idx) && ~isempty(valid_idx{sid})
            cand_actions{ii} = valid_idx{sid};
        else
            cand_actions{ii} = 1:nC;
        end
        if isempty(cand_actions{ii})
            cand_actions{ii} = 1:nC;
        end
    end

    nv = cellfun(@length, cand_actions);
    n_comb = prod(nv);
    if n_comb <= 0 || ~isfinite(n_comb)
        % 兜底：选择直飞动作（如存在）
        zidx = find(C_list == 0, 1, 'first');
        if isempty(zidx), zidx = 1; end
        opt_actions_T(:) = zidx;
        return;
    end

    J_best = inf;
    best_sel = zeros(1, numT);

    % =========================
    % 加速：缓存伪更新结果
    % =========================
    % 伪更新 post_i 只依赖 (sensor_id, ci)，不依赖其它 T 传感器的动作组合。
    % 因此把它从“每个联合动作组合都重复算”变成“预先算一次并缓存”。
    if numT == 1
        % 特殊情况：只有一个 T 传感器，无需调用 Centralized_Fusion_AGM。
        sensor_id = T_indices(1);
        ci_list = cand_actions{1};
        J_list = inf(1, length(ci_list));
        use_parfor_numT1 = (exist('gcp', 'file') == 2) && ~isempty(gcp('nocreate'));
        if use_parfor_numT1
            parfor cidx = 1:length(ci_list)
                ci = ci_list(cidx);
                Zi = PIMS_cell{sensor_id, ci};
                loc_i = loc_after{sensor_id, ci};
                R_params_i = Sensor_distr(sensor_id).R_params;
                post_i = PHD_pseudo_update(v_fused_pred, Zi, loc_i, R_params_i, 1);
                N_post = post_i{4,1};
                if N_post > 0
                    trace_sum = 0;
                    P_all = post_i{3,1};
                    for j = 1:N_post
                        Pj = P_all(:, 6*(j-1)+1 : 6*j);
                        trace_sum = trace_sum + trace(W * Pj * W');
                    end
                    J_list(cidx) = trace_sum / N_post + beta * max(0, N_pred - N_post);
                else
                    J_list(cidx) = beta * max(0, N_pred);
                end
            end
        else
            for cidx = 1:length(ci_list)
                ci = ci_list(cidx);
                Zi = PIMS_cell{sensor_id, ci};
                loc_i = loc_after{sensor_id, ci};
                R_params_i = Sensor_distr(sensor_id).R_params;
                post_i = PHD_pseudo_update(v_fused_pred, Zi, loc_i, R_params_i, 1);
                N_post = post_i{4,1};
                if N_post > 0
                    trace_sum = 0;
                    P_all = post_i{3,1};
                    for j = 1:N_post
                        Pj = P_all(:, 6*(j-1)+1 : 6*j);
                        trace_sum = trace_sum + trace(W * Pj * W');
                    end
                    J_list(cidx) = trace_sum / N_post + beta * max(0, N_pred - N_post);
                else
                    J_list(cidx) = beta * max(0, N_pred);
                end
            end
        end
        [J_best, idx_best] = min(J_list);
        best_sel(1) = ci_list(idx_best);
        opt_actions_T = best_sel;
        return;
    end

    % 一般情况：numT>=2，缓存每个 T 传感器的 post_i(ci)
    post_cache = cell(1, numT); % post_cache{ii}{ci} = post_i
    for ii = 1:numT
        post_cache{ii} = cell(1, nC);
    end

    % 如已有并行池，可并行预计算（显著提高 CPU 利用率）
    use_parfor = (exist('gcp', 'file') == 2) && ~isempty(gcp('nocreate'));
    for ii = 1:numT
        sensor_id = T_indices(ii);
        ci_list = cand_actions{ii};
        if use_parfor
            post_list = cell(1, length(ci_list)); % parfor 内使用线性索引缓存
            parfor cidx = 1:length(ci_list)
                ci = ci_list(cidx);
                Zi = PIMS_cell{sensor_id, ci};
                loc_i = loc_after{sensor_id, ci};
                R_params_i = Sensor_distr(sensor_id).R_params;
                post_list{cidx} = PHD_pseudo_update(v_fused_pred, Zi, loc_i, R_params_i, 1);
            end
            tmp = cell(1, nC);
            for cidx = 1:length(ci_list)
                tmp{ci_list(cidx)} = post_list{cidx};
            end
            post_cache{ii} = tmp;
        else
            for cidx = 1:length(ci_list)
                ci = ci_list(cidx);
                Zi = PIMS_cell{sensor_id, ci};
                loc_i = loc_after{sensor_id, ci};
                R_params_i = Sensor_distr(sensor_id).R_params;
                post_cache{ii}{ci} = PHD_pseudo_update(v_fused_pred, Zi, loc_i, R_params_i, 1);
            end
        end
    end

    % 枚举 T 传感器的联合动作（混合进制展开）
    for cc = 1:n_comb
        rem_idx = cc - 1;
        sel = zeros(1, numT);
        for ii = 1:numT
            base_i = nv(ii);
            local_id = mod(rem_idx, base_i) + 1;
            rem_idx = floor(rem_idx / base_i);
            sel(ii) = cand_actions{ii}(local_id);
        end

        % 使用缓存的伪更新结果（避免在每个联合动作组合里重复 PHD_pseudo_update）
        pseudo_inf = repmat(struct(), 1, numT);
        for ii = 1:numT
            sensor_id = T_indices(ii);
            ci = sel(ii);
            pseudo_inf(ii).gm_particles = post_cache{ii}{ci};
            % Centralized_Fusion_AGM 内部会用 location + R_detect 做视域判断
            pseudo_inf(ii).location = loc_after{sensor_id, ci};
            pseudo_inf(ii).R_detect = Sensor_distr(sensor_id).R_detect;
            pseudo_inf(ii).serial = Sensor_distr(sensor_id).serial;
        end

        % 伪融合：只融合 T 传感器子群
        pseudo_fused = Centralized_Fusion_AGM(pseudo_inf, match_threshold);

        % 计算 J_track（与 chat3/control_core.m 一致）
        N_post = pseudo_fused{4,1};
        if N_post > 0
            trace_sum = 0;
            P_all = pseudo_fused{3,1};
            for j = 1:N_post
                Pj = P_all(:, 6*(j-1)+1 : 6*j);
                trace_sum = trace_sum + trace(W * Pj * W');
            end
            J_track = trace_sum / N_post + beta * max(0, N_pred - N_post);
        else
            J_track = beta * max(0, N_pred);
        end

        if J_track < J_best
            J_best = J_track;
            best_sel = sel;
        end
    end

    opt_actions_T = best_sel;
end