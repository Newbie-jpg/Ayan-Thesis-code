%{
    预测理想量测集 PIMS 生成（chapter3.tex 式 3.26-3.27）
    对每个传感器、每个候选动作，根据 k+L|k 先验目标状态与视域指示函数生成理想量测集
    输入：
        X_prior:     先验目标状态 6*n (从 v_{k+L|k} 提取的均值)
        Sensor:      传感器结构体数组 (含 location, R_detect)
        sensor:      运动参数 (v, C, L, T)
        L:           预测步数
    输出：
        PIMS_cell:   cell(n_sensor, n_candidate)，PIMS_cell{i}(c) 为 3*m 极坐标量测
        loc_after:   cell(n_sensor, n_candidate)，执行动作后传感器位置 3*1
        valid_idx:   cell(n_sensor, 1)，每个传感器有效动作下标（非空 PIMS）
%}
function [PIMS_cell, loc_after, valid_idx] = PIMS_generate(X_prior, Sensor, sensor, L)
    N_sensor = length(Sensor);
    C_list = sensor.C;  % 仅在此偏航角集合中选择控制动作 (度)
    nC = length(C_list);
    % 采样周期：若 T<100 视为秒，否则视为毫秒
    if isfield(sensor, 'T') && sensor.T >= 100
        dt_sec = sensor.T / 1000;
    else
        dt_sec = 1;
        if isfield(sensor, 'T'), dt_sec = sensor.T; end
    end
    v = sensor.v;
    elev_deg = 0;  % 俯仰角固定 0°，不参与选择

    PIMS_cell = cell(N_sensor, nC);
    loc_after = cell(N_sensor, nC);
    valid_idx = cell(N_sensor, 1);

    for i = 1:N_sensor
        loc0 = Sensor(i).location;
        R_detect = Sensor(i).R_detect;
        for c = 1:nC
            beta_deg = C_list(c);
            beta_rad = deg2rad(beta_deg);
            elev_rad = deg2rad(elev_deg);
            % 执行 L 步后位置
            dx = v * cos(elev_rad) * cos(beta_rad) * dt_sec * L;
            dy = v * cos(elev_rad) * sin(beta_rad) * dt_sec * L;
            dz = v * sin(elev_rad) * dt_sec * L;
            loc_new = loc0 + [dx; dy; dz];
            loc_after{i, c} = loc_new;

            Z_pims = [];
            if ~isempty(X_prior)
                n_tar = size(X_prior, 2);
                for j = 1:n_tar
                    x_j = X_prior(:, j);
                    in_fov = FoV_judge(loc_new, x_j, R_detect);
                    if in_fov == 1
                        % 理想量测：无噪声极坐标 [r; azi; ele]
                        z_ideal = compute_R_theta_simple(loc_new, x_j(1), x_j(3), x_j(5));
                        Z_pims = [Z_pims, z_ideal];
                    end
                end
            end
            PIMS_cell{i, c} = Z_pims;
        end
        % 剪枝：仅保留 PIMS 非空的动作
        val = [];
        for c = 1:nC
            if ~isempty(PIMS_cell{i, c})
                val = [val, c];
            end
        end
        if isempty(val)
            val = 1:nC;
        end
        valid_idx{i} = val;
    end
end

function z_out = compute_R_theta_simple(loc_radar, x, y, z_t)
    x_radar = loc_radar(1); y_radar = loc_radar(2); z_radar = loc_radar(3);
    R_d = sqrt((x-x_radar)^2 + (y-y_radar)^2 + (z_t-z_radar)^2);
    d_xy = sqrt((x-x_radar)^2 + (y-y_radar)^2);
    if d_xy < 1e-6
        theta_tilt = 90 * (2*((z_t - z_radar) >= 0) - 1);
        theta_head = 0;
    else
        theta_tilt = rad2deg(atan((z_t-z_radar)/d_xy));
        theta_head = rad2deg(atan2(y-y_radar, x-x_radar));
        if theta_head < 0, theta_head = theta_head + 360; end
    end
    z_out = [R_d; theta_head; theta_tilt];
end
