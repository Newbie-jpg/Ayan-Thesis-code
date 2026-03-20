%{
    绘制仿真场景示意图：
    - 传感器初始位置
    - 传感器视域半径（圆形）
    - 目标起点和终点
    - 目标轨迹
%}
clear;
config;

figure('Name', '仿真场景', 'Color', 'w');
hold on;
grid on;
axis equal;

%% 1. 传感器视域半径（圆形）
for i = 1:N_sensor
    cx = Sensor_distr(i).location(1);
    cy = Sensor_distr(i).location(2);
    R_detect = Sensor_distr(i).R_detect;
    theta_c = linspace(0, 2*pi, 100);
    x_c = cx + R_detect * cos(theta_c);
    y_c = cy + R_detect * sin(theta_c);
    plot(x_c, y_c, '--', 'Color', [0.5 0.5 0.9], 'LineWidth', 0.8);
end

%% 2. 传感器初始位置
h_sensor = [];
for i = 1:N_sensor
    xs = Sensor_distr(i).location(1);
    ys = Sensor_distr(i).location(2);
    h = plot(xs, ys, '^', 'Color', [0 0.5 0], 'MarkerFaceColor', [0.5 1 0.5], 'MarkerSize', 10);
    text(xs + 80, ys + 80, ['S', num2str(i)], 'FontSize', 10);
    if isempty(h_sensor), h_sensor = h; end
end

%% 3. 目标轨迹、起点、终点
n_target = size(Xreal_target_time, 1);
h_traj = [];
h_start = [];
h_end = [];
colors = lines(n_target);

for j = 1:n_target
    traj = Xreal_target_time{j, 1};  % 6 x N
    px = traj(1, :);  % x 位置
    py = traj(3, :);  % y 位置
    valid = ~isnan(px);
    if sum(valid) == 0, continue; end
    px = px(valid);
    py = py(valid);
    % 轨迹线
    h = plot(px, py, '-', 'Color', colors(j,:), 'LineWidth', 1.5);
    % 起点
    hs = plot(px(1), py(1), 'o', 'Color', colors(j,:), 'MarkerFaceColor', colors(j,:), 'MarkerSize', 8);
    % 终点
    he = plot(px(end), py(end), 's', 'Color', colors(j,:), 'MarkerFaceColor', colors(j,:), 'MarkerSize', 7);
    if isempty(h_traj), h_traj = h; end
    if isempty(h_start), h_start = hs; end
    if isempty(h_end), h_end = he; end
end

xlabel('X / m');
ylabel('Y / m');
title('仿真场景：传感器、视域与目标轨迹');
legend([h_sensor, h_traj, h_start, h_end], {'传感器', '目标轨迹', '目标起点', '目标终点'}, 'Location', 'best');
xlim(X_range);
ylim(Y_range);
