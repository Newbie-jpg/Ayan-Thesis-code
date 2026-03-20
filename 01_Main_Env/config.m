%{
    config.m - 多传感器多目标搜索与跟踪 仿真参数配置
%}
sim_cfg = struct();
sim_cfg.rng_seed = 10;
rng(sim_cfg.rng_seed);

%% 1. 全局仿真参数
N = 100;         % 总仿真时间步数
T = 1;               % 采样周期 (s)
M = 1;                % 蒙特卡洛次数；若希望提高 CPU 利用率，可设为 ≥ 并行 worker 数（如 8、12）
motion_pattern_default = 1;

X_range = [-2000, 2000]; % X轴范围 (米)
Y_range = [-2000, 2000]; % Y轴范围 (米)
Z_range = [0, 0];  % Z轴范围 (米)
step_size = 50;

%% 2. 真实目标轨迹生成参数 (严格对应 targetset.m 的输入格式)
% Target_Init=[
%          -1800   20   -100    20   0    0    1   80   motion_pattern_default; % 1
%         -1150   20  -1300    20    0    0     1    80   motion_pattern_default;%3
%         -800   20   -1500     20   0    0    1   80  motion_pattern_default; % 5
%         ]';
% Target_Init = [
%     -1400,   5,    -500,     10,  0,  0,  1,  80,  motion_pattern_default; % 目标 1: 左侧中间出发
%      1200,  -20,   1200,   -20,  0,  0,  1,  50,  motion_pattern_default; % 目标 2: 右上角出发 (早期消失)
%         0,    0,  -1600,    20,  0,  0, 10,  80,  motion_pattern_default; % 目标 3: 底部中间出发 (晚出生)
%      -500,  -15,   -500,    20,  0,  0, 20,  70,  motion_pattern_default; % 目标 4: 中心区域出发
%      1500,  -20,      0,   -10,  0,  0, 35,  80,  motion_pattern_default  % 目标 5: 右侧中间出发
% ]';

Target_Init = [
   -300, 10, 900, -5,  0,  0,  1,  100,  motion_pattern_default; % 目标 1: 
    1200, -15, 900, 5,  0,  0,  1,  60,  motion_pattern_default; % 目标 2: 
   -800, 14, -1200, 8,  0,  0, 1,  100,  motion_pattern_default; % 目标 3: 
   -1300, 15, 300, -5,  0,  0, 20,  90,  motion_pattern_default; % 目标 4: 
    1600, -16, -1000, 10,  0,  0, 10,  80,  motion_pattern_default  % 目标 5:
]';

% Target_Init = [
%    -300, 5, 1500, -10,  0,  0,  1,  100,  motion_pattern_default; % 目标 1: 
%     1200, -15, 900, 5,  0,  0,  1,  60,  motion_pattern_default; % 目标 2: 
%    -800, 15, -1200, 10,  0,  0, 1,  100,  motion_pattern_default; % 目标 3: 
%    -1600, 10, 900, -15,  0,  0, 20,  90,  motion_pattern_default; % 目标 4: 
%     900, -10, -1100, 10,  0,  0, 10,  100,  motion_pattern_default  % 目标 5:
% ]';

% Target_Init = [
%     0, 0, -1200, 30,  0,  0,  1,  80,  motion_pattern_default; % 目标 1: 
%     -1400, 30, -800, 20,  0,  0,  1,  100,  motion_pattern_default; % 目标 2: 
%     1500, -30, -850, 20,  0,  0, 1,  100,  motion_pattern_default; % 目标 3: 
% ]';

Target_Init = [
   -800, 30, 1000, 0,  0,  0,  40,  100,  motion_pattern_default; % 目标 1: 
    -1200, 30, -800, 0,  0,  0,  1,  80,  motion_pattern_default; % 目标 2: 
   -1200, 30, -1100, 0,  0,  0, 1,  80,  motion_pattern_default; % 目标 3: 
   -1200, 30, -900, 0,  0,  0, 1,  80,  motion_pattern_default; % 目标 3:
]';

% 调用你提供的 targetset.m 生成真实轨迹
[Xreal_target_time, Xreal_time_target] = targetset(N, Target_Init);

[X_grid, Y_grid, Z_grid] = meshgrid(X_range(1):step_size:X_range(2), ...
                                    Y_range(1):step_size:Y_range(2), ...
                                    Z_range(1):step_size:Z_range(2));

% 构建全局网格结构体 GridMap
GridMap = struct;
GridMap.c = [X_grid(:)'; Y_grid(:)'; Z_grid(:)']; % 网格中心坐标，尺寸为 3 x M
GridMap.M = size(GridMap.c, 2);                   % 网格单元的总数量 |\mathcal{G}|
GridMap.omega0 = 1;                               % 基础搜索需求值 \omega_0
GridMap.nx = numel(X_range(1):step_size:X_range(2));
GridMap.ny = numel(Y_range(1):step_size:Y_range(2));
GridMap.Ps_grid = 0.99;                           % 网格目标存活概率 P_S
GridMap.B_birth = 1e-3;                           % 新生目标概率 B（可按场景调节）
% 转移核 T（用于二维离散卷积），对应 chapter3 预测式 P_{k|k-1}=P_S*(P_{k-1|k-1}*T)+B
% GridMap.T_kernel = [0.05, 0.10, 0.05; ...
%                     0.10, 0.40, 0.10; ...
%                     0.05, 0.10, 0.05];
% GridMap.T_kernel = GridMap.T_kernel / sum(GridMap.T_kernel(:));

target_vmax_xy_physical = [];
target_vmax_xy = max(sqrt(Target_Init(2, :).^2 + Target_Init(4, :).^2)); % 从初始速度估计
if ~isempty(target_vmax_xy_physical)
    target_vmax_xy = target_vmax_xy_physical;
end
[GridMap.T_kernel, GridMap.sigma_grid, GridMap.kernel_radius] = ...
    Build_Grid_Transition_Kernel(target_vmax_xy, T, step_size);
GridMap.B_birth = 0.001;                        % chapter3: 新生目标概率矩阵 B（标量时等价全图同值）

disp(['网格初始化完成，共生成 ', num2str(GridMap.M), ' 个网格单元。']);

%% 3. 传感器参数初始化
N_sensor = 3; 
Sensor_distr=struct;

for i = 1 : N_sensor
    % 物理参数
    Sensor_distr(i).serial = i;
    Sensor_distr(i).R_detect = 500;       % 探测半径
    Sensor_distr(i).Pd = 0.98;             % 检测概率
    Sensor_distr(i).Ps = 0.99;             % 存活概率
    Sensor_distr(i).Zr = 2; % 杂波期望

     
    % 滤波与杂波参数（chapter3.tex 式3.34：噪声与距离相关 σ_r=r_0+η_r*d, σ_θ=θ_0+η_θ*d）
%     r_0 = 10;           % 距离基准噪声标准差 (m)，文档 r_0=10m
%     theta_0 = 0.5;      % 方位角基准噪声标准差 (度)，文档 θ_0=0.5°
%     eta_r = 0.01;       % 距离对σ_r的缩放系数 (m/m)
%     eta_theta = 0.0005; % 距离对σ_θ的缩放系数 (度/m)
%     Sensor_distr(i).R_params = struct('r_0', r_0, 'theta_0', theta_0, 'eta_r', eta_r, 'eta_theta', eta_theta);
%     Sensor_distr(i).R = []; % 改为由 compute_R_from_distance 按距离动态计算
    r_0 = 1;           % 距离基准噪声标准差 (m)，文档 r_0=10m
    theta_0 = pi / 180;      % 方位角基准噪声标准差 (度)，文档 θ_0=0.5°
    eta_r = 0.00005;       % 距离对σ_r的缩放系数 (m/m)
    eta_theta = 0.00005; % 距离对σ_θ的缩放系数 (度/m)
    Sensor_distr(i).R_params = struct('r_0', r_0, 'theta_0', theta_0, 'eta_r', eta_r, 'eta_theta', eta_theta);
    Sensor_distr(i).R = []; % 改为由 compute_R_from_distance 按距离动态计算

    % 观测相关变量
    Sensor_distr(i).Z_polar_part = cell(N,1);    % 极坐标系下的局部观测(以平台为中心)
    Sensor_distr(i).Z_dicaer_global = cell(N,1); % 笛卡尔坐标系下的全局观测

    Sensor_distr(i).track = zeros(N, 3);
    Sensor_distr(i).GridProb = GridMap.omega0 * ones(GridMap.M, 1);
end

% Sensor_distr(1).location = [1200; -1200; 0]; % 初始位置
% Sensor_distr(2).location = [0; -1200; 0]; % 初始位置
% Sensor_distr(3).location = [-1200; -1200; 0]; % 初始位置
% Sensor_distr(4).location = [-1000; 1000; 0]; % 初始位置

% Sensor_distr(1).location = [1000; -1000; 0]; % 初始位置
% Sensor_distr(2).location = [1000; 1000; 0]; % 初始位置
% Sensor_distr(3).location = [-1000; -1000; 0]; % 初始位置
% Sensor_distr(4).location = [-1000; 1000; 0]; % 初始位置

% Sensor_distr(1).location = [-400; -400; 0]; % 初始位置
% Sensor_distr(2).location = [400; -400; 0]; % 初始位置
% Sensor_distr(3).location = [400; 400; 0]; % 初始位置
% Sensor_distr(4).location = [-400; 400; 0]; % 初始位置

Sensor_distr(1).location = [-1200; -1000; 0]; % 初始位置
Sensor_distr(2).location = [-1200; -500; 0]; % 初始位置
Sensor_distr(3).location = [-1200; -800; 0]; % 初始位置

sensor = struct;
sensor.num = N_sensor;
sensor.v = 30;               % 传感器飞行速度
% 控制动作仅在此偏航角集合中选择，俯仰角固定为 0，不单独选择
sensor.C = [-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180];  % 可选偏航角 (度)
sensor.L = 2;                 % 预测步长
sensor.T = T;
selection = ones(1, N_sensor) * 7;  % 默认选第 7 个 (0°)

%% 4. 并行与流程参数（统一入口）
sim_cfg.parallel = struct();
sim_cfg.parallel.min_pool_size = 8;                 % 与 M 联动：pool_size = min(num_cores, max(M, min_pool_size))
sim_cfg.parallel.process_inner_pool_max = 8;        % PROCESS 内部兜底 parpool 上限

sim_cfg.process = struct();
sim_cfg.process.Ps = 1;
sim_cfg.process.Vx_thre = 200;
sim_cfg.process.Vy_thre = 200;
sim_cfg.process.Vz_thre = 200;
sim_cfg.process.control_interval = 5;

%% 5. 控制器参数（统一入口）
sim_cfg.control = struct();
sim_cfg.control.match_threshold = 200;
sim_cfg.control.eta = 10;
sim_cfg.control.beta = 100;
sim_cfg.control.W = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
sim_cfg.control.L = sensor.L;
sim_cfg.control.objective_mode = 'grid';

%% 6. 算法方案参数（统一入口）
% 每个方案在 base_control_opt 基础上做覆盖，避免多处硬编码
sim_cfg.base_control_opt = sim_cfg.control;
sim_cfg.algo_cfgs = repmat(struct('name', '', 'tag', '', 'control_opt', struct()), 1, 3);
sim_cfg.algo_cfgs(1).name = '现有方案：J_{track}-\etaJ_{search}';
sim_cfg.algo_cfgs(1).tag = 'grid';
sim_cfg.algo_cfgs(1).control_opt = merge_struct(sim_cfg.base_control_opt, struct( ...
    'objective_mode', 'grid', 'eta', 10, 'beta', 100));

sim_cfg.algo_cfgs(2).name = '方案2：纯CS框架(最大化D_{CS})';
sim_cfg.algo_cfgs(2).tag = 'cs';
sim_cfg.algo_cfgs(2).control_opt = merge_struct(sim_cfg.base_control_opt, struct( ...
    'objective_mode', 'cs', 'beta', 1));

sim_cfg.algo_cfgs(3).name = '方案3：改进CS+网格(最大化\theta_{CS})';
sim_cfg.algo_cfgs(3).tag = 'cs_improved';
sim_cfg.algo_cfgs(3).control_opt = merge_struct(sim_cfg.base_control_opt, struct( ...
    'objective_mode', 'cs_improved', 'eta', 10, 'beta', 100));

% 启用的方案索引（例如 [1,3] 表示仅运行 grid 和 cs_improved）
sim_cfg.enabled_algo_indices = [1, 3];

function out = merge_struct(base, override)
out = base;
if nargin < 2 || isempty(override)
    return;
end
fns = fieldnames(override);
for ii = 1:numel(fns)
    out.(fns{ii}) = override.(fns{ii});
end
end
