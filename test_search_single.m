%{
    搜索+跟踪验证仿真
    场景：1 个目标 + 1 个传感器，初始时刻目标在传感器视域外
    主仿真循环：搜索（目标未发现）→ 发现 → 跟踪（目标在视域内）
    验证：1) 传感器能否在视域外找到目标  2) 发现后能否持续跟踪
%}
clear; clc; close all;

%% 1. 场景参数
% rng(2024);
N = 150;                    % 仿真步数（增加步数以验证搜索→发现→跟踪完整流程）
T = 1;                      % 采样周期 (s)
X_range = [-2000, 2000];    % X 范围 (m)
Y_range = [-2000, 2000];    % Y 范围 (m)
Z_range = [0, 0];           % Z 范围 (m)
step_size = 50;             % 网格步长

%% 2. 目标：初始在右上角，远离传感器，缓慢向左下移动（便于传感器搜索发现）
% targetset 格式：[x, vx, y, vy, z, vz, begin_time, end_time, motion_pattern]
Target_Init = [1000, -12, 1200, -14, 0, 0, 1, N, 1]';  % CV 模式，向西南（传感器方向）移动
[Xreal_target_time, Xreal_time_target] = targetset(N, Target_Init);

%% 3. 网格与传感器初始化
[X_grid, Y_grid, Z_grid] = meshgrid(X_range(1):step_size:X_range(2), ...
    Y_range(1):step_size:Y_range(2), Z_range(1):step_size:Z_range(2));
GridMap = struct;
GridMap.c = [X_grid(:)'; Y_grid(:)'; Z_grid(:)'];
GridMap.M = size(GridMap.c, 2);
GridMap.omega0 = 1;
GridMap.nx = numel(X_range(1):step_size:X_range(2));
GridMap.ny = numel(Y_range(1):step_size:Y_range(2));
GridMap.Ps_grid = 0.99;
GridMap.B_birth = 1e-3;
GridMap.T_kernel = [0.05, 0.10, 0.05; ...
                    0.10, 0.40, 0.10; ...
                    0.05, 0.10, 0.05];
GridMap.T_kernel = GridMap.T_kernel / sum(GridMap.T_kernel(:));

N_sensor = 1;
Sensor_distr = struct;
Sensor_distr(1).serial = 1;
Sensor_distr(1).R_detect = 500;       % 探测半径
Sensor_distr(1).location = [-1200; -1200; 0];  % 左下角，目标在右上角，初始距离 > R_detect
Sensor_distr(1).GridProb = GridMap.omega0 * ones(GridMap.M, 1);
Sensor_distr(1).R_params = struct('r_0', 10, 'theta_0', 0.5, 'eta_r', 0.01, 'eta_theta', 0.0005);

sensor = struct;
sensor.num = N_sensor;
sensor.v = 50;                         % 传感器速度 (m/s)
sensor.C = [-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180];
sensor.L = 2;
sensor.T = T;

%% 4. 验证初始条件：目标在传感器视域外
traj_t1 = Xreal_target_time{1,1};
x_t1 = traj_t1(1, 1); y_t1 = traj_t1(3, 1);
dist_init = sqrt((x_t1 - Sensor_distr(1).location(1))^2 + (y_t1 - Sensor_distr(1).location(2))^2);
in_str = '不在';
if dist_init <= Sensor_distr(1).R_detect, in_str = '在'; end
fprintf('初始距离: %.1f m, 探测半径: %d m => 目标%s视域内\n', dist_init, Sensor_distr(1).R_detect, in_str);

%% 5. 空后验及从检测状态构造后验的辅助参数
empty_posterior = cell(4, 1);
empty_posterior{1,1} = zeros(1, 0);
empty_posterior{2,1} = zeros(6, 0);
empty_posterior{3,1} = zeros(6, 0);
empty_posterior{4,1} = 0;

% 预测模型（与 control_core 一致，用于目标丢失时的状态外推）
T_s = 1;
F_pred = [1 T_s 0 0 0 0; 0 1 0 0 0 0; 0 0 1 T_s 0 0; 0 0 0 1 0 0; 0 0 0 0 1 T_s; 0 0 0 0 0 1];
Q_pred = diag([1, 0.1, 1, 0.1, 1, 0.1]);
P_init = diag([50, 5, 50, 5, 1, 1]);  % 检测到目标时的初始协方差

%% 6. 主仿真循环：搜索 + 跟踪完整逻辑
control_interval = 5;
last_action = [];
Sensor_traj = zeros(3, N, N_sensor);
Sensor_traj(:, 1, 1) = Sensor_distr(1).location;
if N >= 2
    Sensor_traj(:, 2, 1) = Sensor_distr(1).location;
end

first_detection_k = [];     % 首次发现目标的时刻
in_fov_log = false(1, N);   % 记录每步目标是否在视域内
current_posterior = empty_posterior;
max_lost_steps = 15;       % 连续丢失超过此步数则退回纯搜索
lost_count = 0;

for t = 3:N
    % 6.1 对上一时刻后验进行一步预测（用于未检测到时维持跟踪意图）
    if current_posterior{4,1} > 0
        pred_posterior = PHD_predict_L_steps(current_posterior, 1, F_pred, Q_pred);
    else
        pred_posterior = empty_posterior;
    end

    % 6.2 在当前传感器位置检测目标是否在视域内（在控制决策前完成）
    Xreal_t = Xreal_time_target{t, 1};
    Xreal_t(:, isnan(Xreal_t(1, :))) = [];
    target_in_fov = false;
    detected_state = zeros(6, 1);
    for j = 1:size(Xreal_t, 2)
        state_j = [Xreal_t(1,j); Xreal_t(2,j); Xreal_t(3,j); Xreal_t(4,j); Xreal_t(5,j); Xreal_t(6,j)];
        if FoV_judge(Sensor_distr(1).location, state_j, Sensor_distr(1).R_detect) == 1
            target_in_fov = true;
            detected_state = state_j;
            break;
        end
    end
    in_fov_log(t) = target_in_fov;

    % 6.3 根据检测结果更新当前后验：发现则用检测状态，未发现则用预测或清零
    if target_in_fov
        % 检测到目标：用检测状态构造后验，control_core 将以跟踪为主
        current_posterior = cell(4, 1);
        current_posterior{1,1} = 1;
        current_posterior{2,1} = detected_state;
        current_posterior{3,1} = P_init;
        current_posterior{4,1} = 1;
        lost_count = 0;
        if isempty(first_detection_k)
            first_detection_k = t;
            fprintf('>>> 时刻 k=%d: 目标首次进入传感器视域 [搜索→跟踪]\n', t);
        end
    else
        % 未检测到：若此前有目标，用预测后验以辅助重新捕获；连续丢失过久则退回搜索
        if pred_posterior{4,1} > 0
            lost_count = lost_count + 1;
            if lost_count <= max_lost_steps
                current_posterior = pred_posterior;
            else
                current_posterior = empty_posterior;
            end
        else
            current_posterior = empty_posterior;
        end
    end

    % 6.4 网格密度更新（根据当前视域）
    Sensor_distr(1).GridProb = Update_Grid_Density(GridMap, Sensor_distr(1).GridProb, ...
        Sensor_distr(1).location, Sensor_distr(1).R_detect);

    % 6.5 构建 sensor_inf 供 control_core（使用当前后验，实现搜索/跟踪模式切换）
    sensor_inf = struct;
    sensor_inf(1).gm_particles = current_posterior;
    sensor_inf(1).location = Sensor_distr(1).location;
    sensor_inf(1).R_detect = Sensor_distr(1).R_detect;
    sensor_inf(1).GridProb = Sensor_distr(1).GridProb;
    sensor_inf(1).R_params = Sensor_distr(1).R_params;

    % 6.6 控制决策：有目标时 eta 略降以兼顾跟踪，无目标时 eta 提高强化搜索
    if current_posterior{4,1} > 0
        opt = struct('match_threshold', 200, 'eta', 0.5, 'beta', 10);
    else
        opt = struct('match_threshold', 200, 'eta', 0.8, 'beta', 10);
    end
    if isempty(last_action) || mod(t - 3, control_interval) == 0
        [~, action, ~] = control_core(current_posterior, sensor_inf, GridMap, sensor, opt);
        last_action = action;
    else
        action = last_action;
    end

    % 6.7 传感器移动
    act_beta = action(1, 1);
    act_epsilon = action(2, 1);
    dt_sec = sensor.T;
    dx = sensor.v * cos(deg2rad(act_epsilon)) * cos(deg2rad(act_beta)) * dt_sec;
    dy = sensor.v * cos(deg2rad(act_epsilon)) * sin(deg2rad(act_beta)) * dt_sec;
    dz = sensor.v * sin(deg2rad(act_epsilon)) * dt_sec;
    Sensor_distr(1).location = Sensor_distr(1).location + [dx; dy; dz];
    Sensor_traj(:, t, 1) = Sensor_distr(1).location;
end

%% 7. 结果统计与可视化
if ~isempty(first_detection_k)
    steps_after_detection = N - first_detection_k + 1;
    track_steps = sum(in_fov_log(first_detection_k:end));
    track_ratio = track_steps / steps_after_detection * 100;
    fprintf('\n【搜索成功】首次发现目标时刻: k = %d\n', first_detection_k);
    fprintf('【跟踪连续性】发现后共 %d 步，其中 %d 步目标在视域内，跟踪率 %.1f%%\n', steps_after_detection, track_steps, track_ratio);
    if track_ratio >= 80
        fprintf('【结论】发现后能持续跟踪\n');
    else
        fprintf('【结论】发现后跟踪存在丢失（可调整 eta、控制间隔或目标/传感器速度比）\n');
    end
else
    fprintf('\n【搜索未完成】仿真结束前未发现目标\n');
end

% 目标轨迹
traj = Xreal_target_time{1, 1};
px = traj(1, :); py = traj(3, :);
valid = ~isnan(px);
px = px(valid); py = py(valid);

% 传感器轨迹
xs = squeeze(Sensor_traj(1, :, 1));
ys = squeeze(Sensor_traj(2, :, 1));

figure('Name', '搜索+跟踪验证：单目标单传感器', 'Color', 'w');
hold on; grid on; axis equal;
h1 = plot(px, py, 'b-', 'LineWidth', 1.8);           % 目标轨迹
h2 = plot(px(1), py(1), 'o', 'Color', [0 0.5 0], 'MarkerFaceColor', [0.5 1 0.5], 'MarkerSize', 10);
h3 = plot(px(end), py(end), 's', 'Color', [0.5 0 0], 'MarkerFaceColor', [1 0.5 0.5], 'MarkerSize', 9);
h4 = plot(xs, ys, 'r-', 'LineWidth', 1.5);           % 传感器轨迹
h5 = plot(xs(1), ys(1), '^', 'Color', [0.5 0 0], 'MarkerFaceColor', [1 0.7 0.7], 'MarkerSize', 10);
h6 = plot(xs(end), ys(end), 'd', 'Color', [0.5 0 0], 'MarkerFaceColor', [1 0.5 0.5], 'MarkerSize', 8);

% 首次发现时刻标记
h7 = [];
if ~isempty(first_detection_k) && first_detection_k <= size(traj, 2)
    kf = min(first_detection_k, size(traj, 2));
    if ~isnan(traj(1, kf))
        h7 = plot(traj(1, kf), traj(3, kf), 'p', 'Color', [1 0.5 0], 'MarkerFaceColor', [1 0.8 0], 'MarkerSize', 14);
    end
end

% 传感器初始视域（圆形，以起点为圆心）
theta_c = linspace(0, 2*pi, 100);
x0 = Sensor_traj(1, 1, 1); y0 = Sensor_traj(2, 1, 1);
x_c = x0 + Sensor_distr(1).R_detect * cos(theta_c);
y_c = y0 + Sensor_distr(1).R_detect * sin(theta_c);
h8 = plot(x_c, y_c, '--', 'Color', [0.7 0.7 0.9], 'LineWidth', 0.8);

xlabel('X / m'); ylabel('Y / m');
title('搜索+跟踪验证：目标初始在视域外 → 搜索发现 → 持续跟踪');
if ~isempty(h7)
    legend([h1, h2, h3, h4, h5, h6, h7, h8], ...
        '目标轨迹', '目标起点', '目标终点', '传感器轨迹', '传感器起点', '传感器终点', '首次发现', '初始视域', 'Location', 'best');
else
    legend([h1, h2, h3, h4, h5, h6, h8], ...
        '目标轨迹', '目标起点', '目标终点', '传感器轨迹', '传感器起点', '传感器终点', '初始视域', 'Location', 'best');
end
xlim(X_range); ylim(Y_range);
hold off;
