function [OSPA, OSPA_sensor, Num_estimate, X_est_global] = evaluate_ospa_on_fixed_traj( ...
    Xreal_time_target, Sensor_init, Sensor_traj, N, GridMap, process_opt, control_opt)
% EVALUATE_OSPA_ON_FIXED_TRAJ
% 在固定传感器轨迹下重放滤波，并在评估层使用簇级存在概率提取目标集合。

    if nargin < 6 || isempty(process_opt), process_opt = struct(); end
    if nargin < 7 || isempty(control_opt), control_opt = struct(); end

    process_opt = set_default_fields_local(process_opt, struct( ...
        'Ps', 0.99, ...
        'Vx_thre', 200, ...
        'Vy_thre', 200, ...
        'Vz_thre', 200, ...
        'ospa_cluster_dist', 140, ...
        'ospa_assoc_dist', 220, ...
        'ospa_birth_min_sum', 0.01, ...
        'ospa_r_confirm', 0.45, ...
        'ospa_r_delete', 0.12, ...
        'ospa_r_hit_alpha', 0.8, ...
        'ospa_r_hit_gain', 0.35, ...
        'ospa_r_miss_decay', 0.55, ...
        'ospa_hold_steps', 2));
    control_opt = set_default_fields_local(control_opt, struct( ...
        'match_threshold', 200, ...
        'objective_mode', 'grid'));

    Ps = process_opt.Ps;
    Vx_thre = process_opt.Vx_thre;
    Vy_thre = process_opt.Vy_thre;
    Vz_thre = process_opt.Vz_thre;
    cluster_dist = process_opt.ospa_cluster_dist;
    assoc_dist = process_opt.ospa_assoc_dist;
    birth_min_sum = process_opt.ospa_birth_min_sum;
    r_confirm = process_opt.ospa_r_confirm;
    r_delete = process_opt.ospa_r_delete;
    r_hit_alpha = process_opt.ospa_r_hit_alpha;
    r_hit_gain = process_opt.ospa_r_hit_gain;
    r_miss_decay = process_opt.ospa_r_miss_decay;
    hold_steps = max(0, round(process_opt.ospa_hold_steps));
    match_threshold = control_opt.match_threshold;

    N_sensor = size(Sensor_traj, 3);
    Sensor = Sensor_init;
    X_est_global = cell(N, 1);
    OSPA_sensor = zeros(N_sensor, N);
    tracks = struct('x', {}, 'r', {}, 'misses', {});

    use_grid_mechanism = true;
    if isfield(control_opt, 'objective_mode') && strcmpi(control_opt.objective_mode, 'cs')
        use_grid_mechanism = false;
    end

    for i = 1:N_sensor
        Sensor(i).location = Sensor_traj(:, 1, i);
        Sensor(i).Z_polar_part = cell(N, 1);
        Sensor(i).Z_dicaer_global = cell(N, 1);
        Sensor(i).X_est_local = cell(N, 1);
        Sensor(i).X_est_fov = cell(N, 1);
    end

    ALG3_PHD_initial;

    for i = 1:N_sensor
        loc_1 = Sensor_traj(:, 1, i);
        loc_2 = Sensor_traj(:, min(2, size(Sensor_traj, 2)), i);
        Z_p1 = observe_FoV_3d_single(Xreal_time_target{1,1}, loc_1, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
        Sensor(i).Z_dicaer_global{1,1} = polar2dicaer_3d_single(Z_p1, loc_1(1), loc_1(2), loc_1(3));
        Z_p2 = observe_FoV_3d_single(Xreal_time_target{2,1}, loc_2, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
        Sensor(i).Z_dicaer_global{2,1} = polar2dicaer_3d_single(Z_p2, loc_2(1), loc_2(2), loc_2(3));
    end

    X_est_global{1,1} = zeros(6,0);
    X_est_global{2,1} = zeros(6,0);

    for t = 3:N
        for i = 1:N_sensor
            loc_obs = Sensor_traj(:, t-1, i);
            Sensor(i).location = loc_obs;
            Z_p_t = observe_FoV_3d_single(Xreal_time_target{t,1}, loc_obs, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
            Sensor(i).Z_polar_part{t,1} = Z_p_t;
            Sensor(i).Z_dicaer_global{t,1} = polar2dicaer_3d_single(Z_p_t, loc_obs(1), loc_obs(2), loc_obs(3));
            Sensor(i).state = ALG3_PHD1time_3d_ukf_distr( ...
                Sensor(i).Z_dicaer_global{t-2,1}, Sensor(i).Z_dicaer_global{t-1,1}, Sensor(i).Z_polar_part{t,1}, ...
                Sensor(i).state, Sensor(i).Zr, Sensor(i).R_params, get_sensor_ps_local(Sensor(i), Ps), Sensor(i).Pd, ...
                Vx_thre, Vy_thre, Vz_thre, loc_obs(1), loc_obs(2), loc_obs(3));
            Sensor(i).X_est_local{t,1} = statedraw_3d(Sensor(i).state);
            if use_grid_mechanism
                Sensor(i).GridProb = Update_Grid_Density(GridMap, Sensor(i).GridProb, loc_obs, Sensor(i).R_detect, Sensor(i).Pd);
            end
        end

        Fusion_center = struct();
        for i = 1:N_sensor
            loc_obs = Sensor_traj(:, t-1, i);
            Fusion_center.sensor_inf(i).gm_particles = Sensor(i).state;
            Fusion_center.sensor_inf(i).location = loc_obs;
            Fusion_center.sensor_inf(i).R_detect = Sensor(i).R_detect;
            Fusion_center.sensor_inf(i).serial = Sensor(i).serial;
            Fusion_center.sensor_inf(i).GridProb = Sensor(i).GridProb;
            Fusion_center.sensor_inf(i).Pd = Sensor(i).Pd;
            Fusion_center.sensor_inf(i).R_params = Sensor(i).R_params;
        end

        Fusion_center.results = Centralized_Fusion_AGM(Fusion_center.sensor_inf, match_threshold);
        clusters = build_fused_clusters(Fusion_center.results, cluster_dist);
        tracks = update_track_existence(tracks, clusters, assoc_dist, birth_min_sum, ...
            r_confirm, r_delete, r_hit_alpha, r_hit_gain, r_miss_decay, hold_steps);
        X_est_global{t,1} = tracks_to_state(tracks, r_confirm);

        for i = 1:N_sensor
            loc_obs = Sensor_traj(:, t-1, i);
            [state_in_fov, ~] = FoV_divide(Fusion_center.results, loc_obs, Sensor(i).R_detect);
            Sensor(i).X_est_fov{t,1} = statedraw_3d(state_in_fov);
        end
    end

    OSPA = zeros(1, N);
    Num_estimate = zeros(1, N);
    for t = 1:N
        Xreal_t = Xreal_time_target{t,1};
        Xreal_t(:, isnan(Xreal_t(1,:))) = [];
        X_est_t = X_est_global{t,1};
        if isempty(X_est_t), X_est_t = zeros(6,0); end
        OSPA(t) = ospa_dist(Xreal_t, X_est_t, 120, 2);
        Num_estimate(t) = size(X_est_t, 2);
        for i = 1:N_sensor
            X_est_local_t = Sensor(i).X_est_local{t,1};
            if isempty(X_est_local_t), X_est_local_t = zeros(6,0); end
            OSPA_sensor(i, t) = ospa_dist(Xreal_t, X_est_local_t, 120, 2);
        end
    end
end

function clusters = build_fused_clusters(fused_state, cluster_dist)
    clusters = struct('x', {}, 'sum_w', {});
    if isempty(fused_state) || numel(fused_state) < 4 || fused_state{4,1} <= 0, return; end
    w = fused_state{1,1};
    m = fused_state{2,1};
    J = min(fused_state{4,1}, min(numel(w), size(m,2)));
    if J <= 0, return; end
    w = w(1:J); m = m(:,1:J);
    valid = isfinite(w) & (w > 0) & all(isfinite(m([1 3 5], :)), 1);
    w = w(valid); m = m(:, valid);
    if isempty(w), return; end

    [w_sorted, idx] = sort(w, 'descend');
    m_sorted = m(:, idx);
    for k = 1:numel(w_sorted)
        wk = w_sorted(k);
        xk = m_sorted(:, k);
        pk = xk([1 3 5]);
        best = 0; d_best = inf;
        for c = 1:numel(clusters)
            d = norm(pk - clusters(c).x([1 3 5]));
            if d <= cluster_dist && d < d_best
                d_best = d; best = c;
            end
        end
        if best == 0
            clusters(end+1).x = xk; %#ok<AGROW>
            clusters(end).sum_w = wk;
        else
            sw = clusters(best).sum_w + wk;
            clusters(best).x = (clusters(best).x * clusters(best).sum_w + xk * wk) / sw;
            clusters(best).sum_w = sw;
        end
    end
end

function tracks = update_track_existence(tracks, clusters, assoc_dist, birth_min_sum, ...
    r_confirm, r_delete, r_hit_alpha, r_hit_gain, r_miss_decay, hold_steps)

    nT = numel(tracks); nC = numel(clusters);
    matched_t = false(1, nT); matched_c = false(1, nC);

    if nT > 0 && nC > 0
        D = inf(nT, nC);
        for i = 1:nT
            pi = tracks(i).x([1 3 5]);
            for j = 1:nC
                pj = clusters(j).x([1 3 5]);
                D(i,j) = norm(pi - pj);
            end
        end
        while true
            [dmin, idx] = min(D(:));
            if isempty(dmin) || ~isfinite(dmin) || dmin > assoc_dist, break; end
            [ii, jj] = ind2sub(size(D), idx);
            matched_t(ii) = true; matched_c(jj) = true;
            z = min(1, max(0, clusters(jj).sum_w));
            tracks(ii).r = min(1, r_hit_alpha * tracks(ii).r + r_hit_gain * z);
            tracks(ii).x = clusters(jj).x;
            tracks(ii).misses = 0;
            D(ii,:) = inf; D(:,jj) = inf;
        end
    end

    for i = 1:nT
        if ~matched_t(i)
            tracks(i).r = tracks(i).r * r_miss_decay;
            tracks(i).misses = tracks(i).misses + 1;
        end
    end

    for j = 1:nC
        if ~matched_c(j) && clusters(j).sum_w >= birth_min_sum
            z = min(1, max(0, clusters(j).sum_w));
            tr = struct('x', clusters(j).x, ...
                        'r', min(r_confirm, 0.5 * z + 0.1), ...
                        'misses', 0);
            tracks(end+1) = tr; %#ok<AGROW>
        end
    end

    keep = true(1, numel(tracks));
    for i = 1:numel(tracks)
        if tracks(i).r < r_delete || tracks(i).misses > hold_steps
            keep(i) = false;
        end
    end
    tracks = tracks(keep);
end

function X = tracks_to_state(tracks, r_confirm)
    X = zeros(6,0);
    for i = 1:numel(tracks)
        if tracks(i).r >= r_confirm
            X(:, end+1) = tracks(i).x; %#ok<AGROW>
        end
    end
end

function Ps_i = get_sensor_ps_local(sensor_i, default_ps)
    if isfield(sensor_i, 'Ps') && ~isempty(sensor_i.Ps)
        Ps_i = sensor_i.Ps;
    else
        Ps_i = default_ps;
    end
end

function out = set_default_fields_local(in, defaults)
    out = in;
    fns = fieldnames(defaults);
    for ii = 1:numel(fns)
        key = fns{ii};
        if ~isfield(out, key) || isempty(out.(key))
            out.(key) = defaults.(key);
        end
    end
end
