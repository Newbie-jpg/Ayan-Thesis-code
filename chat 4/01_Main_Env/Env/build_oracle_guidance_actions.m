function [action, sensor_roles, heading_deg, visible_flags] = build_oracle_guidance_actions( ...
    Sensor, Xreal_t, target_init, fixed_groups, sensor_params, oracle_cfg, t, prev_heading_deg, prev_visible_flags)
% BUILD_ORACLE_GUIDANCE_ACTIONS
% 统一生成“视域内带噪真值跟踪 + 视域外固定搜索/出生引导”的动作。

    N_sensor = numel(Sensor);
    action = zeros(2, N_sensor);
    sensor_roles = repmat({'S'}, 1, N_sensor);
    heading_deg = zeros(1, N_sensor);
    visible_flags = false(1, N_sensor);

    if nargin < 8 || isempty(prev_heading_deg)
        prev_heading_deg = nan(1, N_sensor);
    end
    if nargin < 9 || isempty(prev_visible_flags)
        prev_visible_flags = false(1, N_sensor);
    end
    if nargin < 7 || isempty(t)
        t = 0;
    end
    if nargin < 6 || isempty(oracle_cfg)
        oracle_cfg = struct('enable', false);
    end

    if isempty(Xreal_t)
        Xreal_t = zeros(6, 0);
    end
    if size(Xreal_t, 1) < 6
        Xreal_t(6, 1:size(Xreal_t, 2)) = 0;
    end
    n_target_total = size(Xreal_t, 2);

    if nargin < 4 || isempty(fixed_groups)
        fixed_groups = cell(1, N_sensor);
        for i = 1:N_sensor
            if n_target_total > 0
                fixed_groups{i} = 1 + mod(i - 1, n_target_total);
            else
                fixed_groups{i} = [];
            end
        end
    end

    active_ids = find(~isnan(Xreal_t(1, :)));
    Xreal_active = Xreal_t(:, active_ids);
    noisy_pts = compute_noisy_oracle_targets(Xreal_active, oracle_cfg, t);

    max_turn_deg = inf;
    if isfield(oracle_cfg, 'max_turn_deg') && ~isempty(oracle_cfg.max_turn_deg)
        max_turn_deg = oracle_cfg.max_turn_deg;
    end

    for i = 1:N_sensor
        loc_i = Sensor(i).location(:);
        assigned = safe_group(fixed_groups, i, n_target_total);
        visible_all = [];
        visible_assigned = [];

        for tid = 1:n_target_total
            st_all = Xreal_t(:, tid);
            if any(isnan(st_all(1)))
                continue;
            end
            if FoV_judge(loc_i, st_all, Sensor(i).R_detect) == 1
                visible_all(end + 1) = tid; %#ok<AGROW>
            end
        end

        for kk = 1:numel(assigned)
            tid = assigned(kk);
            if tid < 1 || tid > n_target_total
                continue;
            end
            st = Xreal_t(:, tid);
            if any(isnan(st(1)))
                continue;
            end
            if FoV_judge(loc_i, st, Sensor(i).R_detect) == 1
                visible_assigned(end + 1) = tid; %#ok<AGROW>
            end
        end

        if ~isempty(visible_assigned)
            sensor_roles{i} = 'T';
            waypoint = nearest_assigned_point(loc_i, visible_assigned, active_ids, noisy_pts, Xreal_t);
            visible_flags(i) = true;
        elseif ~isempty(visible_all)
            sensor_roles{i} = 'T';
            waypoint = nearest_assigned_point(loc_i, visible_all, active_ids, noisy_pts, Xreal_t);
            visible_flags(i) = true;
        else
            sensor_roles{i} = 'S';
            born_assigned = intersect(assigned, active_ids);
            if ~isempty(born_assigned)
                % 目标已出生但暂未进入 FoV：朝其当前（带噪）位置搜索，而非回出生点打转
                waypoint = nearest_assigned_point(loc_i, born_assigned, active_ids, noisy_pts, Xreal_t);
            else
                % 仅当目标尚未出生时，才朝出生点方向推进
                tid = choose_birth_anchor_target(assigned, target_init, t);
                if ~isempty(tid) && tid >= 1 && tid <= size(target_init, 2)
                    waypoint = target_init([1, 3, 5], tid);
                else
                    waypoint = loc_i;
                end
            end
        end

        idx = waypoint_to_heading_action(loc_i, waypoint, sensor_params);
        heading_i = sensor_params.C(idx);

        % 首次捕获豁免：上一时刻看不到、当前时刻看得到，则本步允许直接转向目标
        is_first_capture = false;
        if i <= numel(prev_visible_flags)
            is_first_capture = (~prev_visible_flags(i)) && visible_flags(i);
        end

        if (~is_first_capture) && isfinite(max_turn_deg) && i <= numel(prev_heading_deg) && ~isnan(prev_heading_deg(i))
            d = wrap180_local(heading_i - prev_heading_deg(i));
            d = min(max(d, -max_turn_deg), max_turn_deg);
            heading_i = prev_heading_deg(i) + d;
            [~, idx2] = min(abs(wrap180_local(sensor_params.C - heading_i)));
            heading_i = sensor_params.C(idx2);
        end

        heading_deg(i) = heading_i;
        action(1, i) = heading_i;
        action(2, i) = 0;
    end
end

function assigned = safe_group(fixed_groups, i, n_target_total)
    assigned = [];
    if i <= numel(fixed_groups)
        assigned = fixed_groups{i};
    end
    assigned = unique(assigned(:).');
    assigned = assigned(assigned >= 1 & assigned <= max(1, n_target_total));
    if isempty(assigned) && n_target_total > 0
        assigned = 1 + mod(i - 1, n_target_total);
    end
end

function tid = choose_birth_anchor_target(assigned, target_init, t)
    tid = [];
    if isempty(assigned) || isempty(target_init)
        return;
    end
    if size(target_init, 1) < 8
        tid = assigned(1);
        return;
    end

    % 优先选择“尚未出生”的目标作为出生引导锚点
    for k = 1:numel(assigned)
        j = assigned(k);
        if j > size(target_init, 2)
            continue;
        end
        begin_time = target_init(7, j);
        end_time = target_init(8, j);
        if t < begin_time && t <= end_time
            tid = j;
            return;
        end
    end

    % 次选：仍在存活窗口内的目标（用于无更优信息时的保守引导）
    for k = 1:numel(assigned)
        j = assigned(k);
        if j > size(target_init, 2)
            continue;
        end
        end_time = target_init(8, j);
        if t <= end_time
            tid = j;
            return;
        end
    end
    % 若均已死亡，则不再强制拉回出生点，避免原地打转
    tid = [];
end

function wp = nearest_assigned_point(loc_i, ids, active_ids, noisy_pts, Xreal_t)
    wp = loc_i;
    if isempty(ids)
        return;
    end
    cand = zeros(3, 0);
    for k = 1:numel(ids)
        tid = ids(k);
        c = find(active_ids == tid, 1, 'first');
        if ~isempty(c) && ~isempty(noisy_pts)
            cand(:, end + 1) = noisy_pts(:, c); %#ok<AGROW>
        else
            st = Xreal_t(:, tid);
            cand(:, end + 1) = [st(1); st(3); st(5)]; %#ok<AGROW>
        end
    end
    if isempty(cand)
        return;
    end
    [~, idx] = min(sum((cand - loc_i) .^ 2, 1));
    wp = cand(:, idx);
end

function a = wrap180_local(a)
    a = mod(a + 180, 360) - 180;
end
