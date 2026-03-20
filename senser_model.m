%初始化
% sensor：传感器运动模型； sensor_track：传感器航迹；
sensor = struct();
sensor.num = N_sensor;             % 传感器总数
sensor.v = 100;

sensor.C = [-180, -135, -90, -45, 0, 45, 90, 135];
sensor.fixed_elevation = 0;        % 俯仰角固定为 0 度

sensor.T = dt * 1000;              % 单步采样时间 (毫秒，若 dt=1s，则为 1000)
sensor.L = 2 * 1000;               % 预测时域步长 (毫秒，此处设定往前看2秒)

sensor_track = repmat(struct('X', zeros(6, N_steps)), N_sensor, 1);

for i = 1 : N_sensor
    % 初始化每个传感器的起点坐标 (假设速度初始为0)
    % 这里的初始位置需要与你之前 Sensor_Init(i).location 保持一致
    start_x = 1000 * i;
    start_y = 0;
    start_z = 0;
    
    sensor_track(i).X(:, 1) = [start_x; 0; start_y; 0; start_z; 0]; 
end

filter_model = repmat(struct(), N_sensor, 1);

for i = 1 : N_sensor
    % 3.1 视场界限 (Area)
    % control.m 中要求 Area 为 3x2 矩阵，第一列为下界，第二列为上界
    % 对应: [距离下界, 距离上界; 方位角下界, 方位角上界; 俯仰角下界, 俯仰角上界]
    R_detect = 300; % 最大探测距离
    Azi_FoV = 120;   % 方位角视场宽度 (度)
    Ele_FoV = 90;    % 俯仰角视场宽度 (度)
    
    filter_model(i).Area = [
        0,      R_detect;       % 距离 (m)
        -Azi_FoV/2, Azi_FoV/2;  % 方位角范围 (度)
        -Ele_FoV/2, Ele_FoV/2   % 俯仰角范围 (度)
    ];

    % 3.2 理想量测生成函数 (H 矩阵/函数)
    % control.m 中通过 z = model_filter.H(ss, xx) 调用。
    % ss: 传感器3D坐标 [x;y;z], xx: 目标3D坐标 [x;y;z]
    % 输出 z: [距离; 方位角; 俯仰角]
    filter_model(i).H = @(ss, xx) [...
        norm(xx - ss); ...                                         % 距离 r
        rad2deg(atan2(xx(2) - ss(2), xx(1) - ss(1))); ...          % 方位角 azi
        rad2deg(asin((xx(3) - ss(3)) / max(norm(xx - ss), 1e-6)))  % 俯仰角 ele
    ];

    % 3.3 转移矩阵 F 与过程噪声 Q 
    % (此处给个默认占位符即可，因为 control.m 内部会在第 40-47 行根据 L/2 强制覆盖重写这两个矩阵)
    filter_model(i).T = sensor.L / 2;
    filter_model(i).F = eye(6); 
    filter_model(i).Q = eye(6); 
end