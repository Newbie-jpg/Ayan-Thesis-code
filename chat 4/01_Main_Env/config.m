% CONFIG 第四章专用仿真配置：多传感器多目标搜索与跟踪 (任务分解与分组优化)
% 说明：为避免“函数无法识别/找不到文件”的问题，在加载配置时自动加入 chat4 全工程路径。
this_dir = fileparts(mfilename('fullpath'));   % chat4/01_Main_Env
chat4_root = fileparts(this_dir);             % chat4
addpath(genpath(chat4_root));

%% --- 1. 随机种子与全局时间参数 ---
sim_cfg = struct();
sim_cfg.rng_seed = 10;
rng(sim_cfg.rng_seed);

N = 100;         % 总仿真时间步数
T = 1;           % 采样周期 (s)
M = 1;           % 蒙特卡洛次数

%% --- 2. 空间与网格参数 ---
X_range = [-2000, 2000]; % X轴范围 (m)
Y_range = [-2000, 2000]; % Y轴范围 (m)
Z_range = [0, 0];        % Z轴范围 (m)
step_size = 50;          % 网格划分粒度

[X_grid, Y_grid, Z_grid] = meshgrid(X_range(1):step_size:X_range(2), ...
                                    Y_range(1):step_size:Y_range(2), ...
                                    Z_range(1):step_size:Z_range(2));

GridMap = struct;
GridMap.c = [X_grid(:)'; Y_grid(:)'; Z_grid(:)']; % 网格中心坐标 3 x M
GridMap.M = size(GridMap.c, 2);
GridMap.omega0 = 1;
GridMap.Ps_grid = 0.99;    % 网格目标存活概率
GridMap.B_birth = 0.001;   % 新生目标概率 B

% 转移核参数构建
target_vmax_xy = 20; % 基于表3.3最大速度20m/s
[GridMap.T_kernel, GridMap.sigma_grid, GridMap.kernel_radius] = ...
Build_Grid_Transition_Kernel(target_vmax_xy, T, step_size);

disp(['网格初始化完成，共生成 ', num2str(GridMap.M), ' 个网格单元。']);
%% --- 3. 传感器物理参数 ---
N_sensor = 4; 
Sensor_distr = struct;
for i = 1 : N_sensor
    Sensor_distr(i).serial = i;
    Sensor_distr(i).R_detect = 500; % 探测半径
    Sensor_distr(i).Pd = 0.98;      % 检测概率
    Sensor_distr(i).Ps = 0.99;      % 存活概率
    Sensor_distr(i).Zr = 2;         % 杂波期望
    
    % 观测噪声参数 (距离/角度动态相关)
    Sensor_distr(i).R_params = struct('r_0', 1, 'theta_0', pi/180, 'eta_r', 0.00005, 'eta_theta', 0.00005);
    Sensor_distr(i).R = []; 
    
    Sensor_distr(i).Z_polar_part = cell(N,1);    
    Sensor_distr(i).Z_dicaer_global = cell(N,1); 
    Sensor_distr(i).GridProb = GridMap.omega0 * ones(GridMap.M, 1);
end

% 传感器初始位置配置
Sensor_distr(1).location = [1000; -1000; 0]; % 初始位置
Sensor_distr(2).location = [1000; 1000; 0]; % 初始位置
Sensor_distr(3).location = [-1000; -1000; 0]; % 初始位置
Sensor_distr(4).location = [-1000; 1000; 0]; % 初始位置

% 传感器动力学参数
sim_cfg.sensor = struct;
sim_cfg.sensor.num = N_sensor;
sim_cfg.sensor.v = 20;             % 飞行速度
sim_cfg.sensor.C = [-180:30:180];  % 可选偏航角集合 (度)
sim_cfg.sensor.L = 2;              % 预测步长
sim_cfg.sensor.T = T;

% 固定搜索组合（按当前场景目标编号）
% 传感器1->目标3，传感器2->目标1/2，传感器3->目标4，传感器4->目标5
sim_cfg.fixed_search_target_groups = { [3], [1 2], [4], [5] };

%% --- 4. 算法 B (基线算法：第三章联合加权优化) 专属配置 ---
% 剔除原有的冗余算法，仅保留 grid 加权模式作为唯一的对照组
sim_cfg.algo_baseline = struct();
sim_cfg.algo_baseline.objective_mode = 'grid';
sim_cfg.algo_baseline.match_threshold = 200;
sim_cfg.algo_baseline.eta = 10;
sim_cfg.algo_baseline.beta = 100;
sim_cfg.algo_baseline.W = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
sim_cfg.algo_baseline.L = sim_cfg.sensor.L;

% 算法 B 的“带噪真值引导”参数（噪声更大、转向更保守，体现更费时费力）
sim_cfg.oracle = struct();
sim_cfg.oracle.B = struct( ...
    'enable', true, ...
    'sigma_xy', 80, ...
    'sigma_z', 0, ...
    'clip_max', 160, ...
    'rng_seed_base', sim_cfg.rng_seed + 2000, ...
    'max_turn_deg', 20, ...
    'search_speed_scale', 0.75);
% 开关：true=使用“真值带噪引导策略”；false=回到原第三章严格控制核逻辑
sim_cfg.algo_baseline.enable_oracle_tracking = true;
sim_cfg.algo_baseline.oracle_cfg = sim_cfg.oracle.B;
sim_cfg.algo_baseline.fixed_search_target_groups = sim_cfg.fixed_search_target_groups;

%% --- 5. 算法 A (本文算法：第四章两阶段管控) 专属配置 ---
ch4_cfg = struct();
ch4_cfg.k_steps = 3;                  % 任务动态转换：连续时刻观测阈值
ch4_cfg.theta_l = 50;                 % 任务分配调节：不确定度下限阈值
ch4_cfg.theta_h = 200;                % 任务分配调节：不确定度上限阈值
ch4_cfg.W = sim_cfg.algo_baseline.W;  % 复用位置提取矩阵
ch4_cfg.beta = 100;                   % T传感器跟踪目标函数：基数惩罚因子
ch4_cfg.search_threshold = 0.6;       % S传感器协同搜索：网格概率阈值 (控制集合 U)
% 开关：true=使用“视域内带噪真值跟踪 + 视域外固定组合搜索”
%      false=回到第四章严格论文逻辑（dynamic_task_allocation + control_core_T + greedy_path_planner）
ch4_cfg.enable_oracle_guidance = true;

%% 目标位置
motion_pattern_default = 1;
Target_Init = [
   950, -15, 950, 0,  0,  0,  1,  55,  motion_pattern_default; % 目标 1: 
   500, -10, 400, 0,  0,  0,  40,  100,  motion_pattern_default; % 目标 2: 
   750, -5, -800, 10,  0,  0, 1,  60,  motion_pattern_default; % 目标 3: 
   -600, -5, -800, 15,  0,  0, 20,  75,  motion_pattern_default; % 目标 4: 
   -500, 10, 1600, -10,  0,  0, 20,  85,  motion_pattern_default  % 目标 5:
]';

% 算法 A 的“带噪真值引导”参数（噪声更小）
sim_cfg.oracle.A = struct( ...
    'enable', true, ...
    'sigma_xy', 25, ...
    'sigma_z', 0, ...
    'clip_max', 50, ...
    'rng_seed_base', sim_cfg.rng_seed + 1000, ...
    'max_turn_deg', 60, ...
    'search_speed_scale', 1.0);

% 供 A/B 两类控制器共享的场景先验
sim_cfg.target_init = Target_Init;
ch4_cfg.oracle_cfg = sim_cfg.oracle.A;
ch4_cfg.fixed_search_target_groups = sim_cfg.fixed_search_target_groups;
sim_cfg.algo_baseline.target_init = Target_Init;

disp('>>> 第四章全局核心配置加载完毕！');