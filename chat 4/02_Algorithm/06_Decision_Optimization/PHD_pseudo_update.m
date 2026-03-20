%{
    GM-PHD 伪更新：以 PIMS 为理想量测，Pd=1，无杂波（chapter3.tex 伪更新）
    输入：
        state_pred:  k+L|k 先验 (4*1 cell)
        Z_polar:     理想量测集 3*nz (极坐标 [r;azi;ele])
        location:    传感器位置 3*1
        R_params:    距离相关噪声参数 {r_0,theta_0,eta_r,eta_theta}，R 与传感器-目标距离相关
        Pd:          检测概率（伪更新取 1）
    输出：
        state_post:  伪后验 (4*1 cell，与 Fusion 格式一致)
%}
function state_post = PHD_pseudo_update(state_pred, Z_polar, location, R_params, Pd)
    J_pre = state_pred{4,1};
    if J_pre == 0
        state_post = state_pred;
        return;
    end
    w_pre = state_pred{1,1};
    m_pre = state_pred{2,1};
    P_pre = state_pred{3,1};
    x_radar = location(1); y_radar = location(2); z_radar = location(3);

    n_z = size(Z_polar, 2);
    if n_z == 0
        w_post = (1 - Pd) * w_pre;
        state_post = cell(4,1);
        state_post{1,1} = w_post;
        state_post{2,1} = m_pre;
        state_post{3,1} = P_pre;
        state_post{4,1} = J_pre;
        return;
    end

    eta = zeros(3, J_pre);
    S = zeros(3, 3*J_pre);
    K_all = zeros(6, 3*J_pre);
    P_upt = zeros(6, 6*J_pre);
    for j = 1:J_pre
        m_j = m_pre(:, j);
        P_j = P_pre(:, 6*(j-1)+1 : 6*j);
        [eta_j, H_j] = obs_model_polar(m_j, x_radar, y_radar, z_radar);
        eta(:, j) = eta_j;
        d_j = norm(m_j([1,3,5]) - location);
        R_j = compute_R_from_distance(d_j, R_params);
        S_j = H_j * P_j * H_j' + R_j;
        S(:, 3*(j-1)+1 : 3*j) = S_j;
        K_j = P_j * H_j' / S_j;
        K_all(:, 3*(j-1)+1 : 3*j) = K_j;
        P_upt(:, 6*(j-1)+1 : 6*j) = (eye(6) - K_j*H_j) * P_j;
    end

    % 漏检分量
    w_list = (1 - Pd) * w_pre;
    m_list = m_pre;
    P_list = P_pre;  % 6 x (6*J_pre)

    kappa = 0;
    for i = 1:n_z
        z_i = Z_polar(:, i);
        q = zeros(1, J_pre);
        for j = 1:J_pre
            S_j = S(:, 3*(j-1)+1 : 3*j);
            d = z_i - eta(:, j);
            d(2) = mod(d(2) + 180, 360) - 180;
            q(j) = w_pre(j) * (1/sqrt(det(2*pi*S_j))) * exp(-0.5 * (d' / S_j * d));
        end
        wsum = kappa + Pd * sum(q);
        if wsum < 1e-20
            continue;
        end
        for j = 1:J_pre
            w_ji = Pd * q(j) / wsum;
            K_j = K_all(:, 3*(j-1)+1 : 3*j);
            d = z_i - eta(:, j);
            d(2) = mod(d(2) + 180, 360) - 180;
            m_ji = m_pre(:, j) + K_j * d;
            w_list = [w_list, w_ji];
            m_list = [m_list, m_ji];
            P_list = [P_list, P_upt(:, 6*(j-1)+1 : 6*j)];
        end
    end

    % 剪枝
    T = 0.01;
    n_total = length(w_list);
    Jj = 0;
    for i = 1:n_total
        if w_list(i) >= T
            Jj = Jj + 1;
        end
    end
    if Jj == 0
        state_post = cell(4,1);
        state_post{1,1} = zeros(1,0);
        state_post{2,1} = zeros(6,0);
        state_post{3,1} = zeros(6,0);
        state_post{4,1} = 0;
        return;
    end
    w_out = zeros(1, Jj);
    m_out = zeros(6, Jj);
    P_out = zeros(6, 6*Jj);
    idx = 0;
    for i = 1:n_total
        if w_list(i) >= T
            idx = idx + 1;
            w_out(idx) = w_list(i);
            m_out(:, idx) = m_list(:, i);
            P_out(:, 6*(idx-1)+1 : 6*idx) = P_list(:, 6*(i-1)+1 : 6*i);
        end
    end
    state_post = cell(4,1);
    state_post{1,1} = w_out;
    state_post{2,1} = m_out;
    state_post{3,1} = P_out;
    state_post{4,1} = Jj;
end

function [eta, H] = obs_model_polar(m, xr, yr, zr)
    x = m(1); y = m(3); z = m(5);
    dx = x - xr; dy = y - yr; dz = z - zr;
    r = sqrt(dx^2 + dy^2 + dz^2);
    d_xy = sqrt(dx^2 + dy^2);
    if d_xy < 1e-6
        azi = 0;
        ele = 90 * sign(dz);
        if dz == 0, ele = 0; end
    else
        azi = rad2deg(atan2(dy, dx));
        if azi < 0, azi = azi + 360; end
        ele = rad2deg(atan(dz / d_xy));
    end
    eta = [r; azi; ele];
    H = zeros(3, 6);
    if r < 1e-6
        H(1,1) = 1; H(1,3) = 1; H(1,5) = 1;
    else
        H(1,1) = dx/r; H(1,3) = dy/r; H(1,5) = dz/r;
    end
    if d_xy < 1e-6
        H(3,5) = 1;
    else
        H(2,1) = -dy/(d_xy^2) * (180/pi);
        H(2,3) = dx/(d_xy^2) * (180/pi);
        H(3,1) = -dx*dz/(d_xy*r^2) * (180/pi);
        H(3,3) = -dy*dz/(d_xy*r^2) * (180/pi);
        H(3,5) = d_xy/r^2 * (180/pi);
    end
end
