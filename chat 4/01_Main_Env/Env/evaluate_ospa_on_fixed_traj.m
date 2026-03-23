function [OSPA, OSPA_sensor, Num_estimate, X_est_global] = evaluate_ospa_on_fixed_traj( ...
    Xreal_time_target, Sensor_init, Sensor_traj, N, GridMap, process_opt, control_opt)
% EVALUATE_OSPA_ON_FIXED_TRAJ
% 在固定传感器轨迹下重放 chat3 滤波链路，并按 chat3 口径计算 OSPA。

    if nargin < 6 || isempty(process_opt), process_opt = struct(); end
    if nargin < 7 || isempty(control_opt), control_opt = struct(); end

    process_opt = set_default_fields_local(process_opt, struct( ...
        'Ps', 1, ...
        'Vx_thre', 200, ...
        'Vy_thre', 200, ...
        'Vz_thre', 200));
    control_opt = set_default_fields_local(control_opt, struct( ...
        'match_threshold', 200, ...
        'objective_mode', 'grid'));

    Ps = process_opt.Ps;
    Vx_thre = process_opt.Vx_thre;
    Vy_thre = process_opt.Vy_thre;
    Vz_thre = process_opt.Vz_thre;
    match_threshold = control_opt.match_threshold;

    N_sensor = size(Sensor_traj, 3);
    Sensor = Sensor_init;
    X_est_global = cell(N, 1);
    OSPA_sensor = zeros(N_sensor, N);

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

    % 与 chat3 一致：若检测概率足够高，则在滤波中按 Pd=1 处理
    for i = 1:N_sensor
        if isfield(Sensor(i), 'Pd') && Sensor(i).Pd >= 0.99
            Sensor(i).Pd = 1;
        end
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
                Sensor(i).state, Sensor(i).Zr, Sensor(i).R_params, Ps, Sensor(i).Pd, ...
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
        [X_est_global{t,1}, ~] = statedraw_3d(Fusion_center.results);

        for i = 1:N_sensor
            loc_obs = Sensor_traj(:, t-1, i);
            [state_in_fov, ~] = FoV_divide(Fusion_center.results, loc_obs, Sensor(i).R_detect);
            [Sensor(i).X_est_fov{t,1}, ~] = statedraw_3d(state_in_fov);
        end
    end

    OSPA = zeros(1, N);
    Num_estimate = zeros(1, N);
    for t = 1:N
        Xreal_t = Xreal_time_target{t,1};
        Xreal_t(:, isnan(Xreal_t(1,:))) = [];
        X_est_t = X_est_global{t,1};
        if isempty(X_est_t), X_est_t = zeros(6,0); end
        OSPA(t) = ospa_dist(Xreal_t, X_est_t, 120, 1);
        Num_estimate(t) = size(X_est_t, 2);
        for i = 1:N_sensor
            X_est_fov_t = Sensor(i).X_est_fov{t,1};
            if isempty(X_est_fov_t), X_est_fov_t = zeros(6,0); end
            OSPA_sensor(i, t) = ospa_dist(Xreal_t, X_est_fov_t, 120, 1);
        end
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
