% MAIN_EXP1_PERFORMANCE 仿真实验一：验证任务分解策略与角色切换有效性
% 对应论文第四章 4.5 节 实验一

clear; close all; clc;

%% 1. 初始化与配置加载
disp('>>> 正在初始化实验一配置...');
config; % 加载最新重构的全局配置 (已包含基础参数与第四章专属参数)


% 生成目标轨迹
[Xreal_target_time, Xreal_time_target] = targetset(N, Target_Init);

%% 2. 运行算法 A (本文方法：基于任务分配的两阶段管控)
disp('======================================================');
disp('>>> 开始运行 算法 A (本文方法: 任务分配 + 分组优化) ...');
disp('======================================================');

% 初始化本文方法的传感器状态 (赋予初始任务角色 'S')
Sensor_distr_A = Sensor_distr;
for i = 1:N_sensor
    Sensor_distr_A(i).task_type = 'S'; 
    Sensor_distr_A(i).target_in_fov_count = 0;
    Sensor_distr_A(i).no_target_in_fov_count = 0;
end

tic_A = tic;
% 运行重构的两阶段控制主循环（蒙特卡洛封装，M>1 并行）
result_A = run_control_twostage_scheme(Xreal_time_target, Sensor_distr_A, N, GridMap, sim_cfg, ch4_cfg, M, sim_cfg.rng_seed);
time_A = toc(tic_A);

result_A.name = '本文算法：任务分配两阶段管控';
result_A.total_time = time_A;
disp(['算法 A 运行完成，总耗时: ', num2str(time_A), ' 秒']);

%% 3. 运行算法 B (基线方法：第三章联合加权优化)
disp('======================================================');
disp('>>> 开始运行 算法 B (基线方法: 第三章联合加权优化) ...');
disp('======================================================');

% 组装基线算法配置 (对接全新 config.m 中的 sim_cfg.algo_baseline)
algo_cfg_B = struct('name', '基线算法：联合加权优化', ...
                    'tag', 'grid', ...
                    'control_opt', sim_cfg.algo_baseline); 

Sensor_distr_B = Sensor_distr; % 使用干净的传感器初始化(不带 task_type)

% 默认前瞻方向选择 (对应直飞的动作索引)
default_action_idx = find(sim_cfg.sensor.C == 0);
selection = ones(1, N_sensor) * default_action_idx;  

tic_B = tic;
% 调用第三章保留的联合决策循环 (作为陪跑对照组)
% 注: M(蒙特卡洛次数) 已在 config.m 中定义，此处直接传入
result_B = run_control_scheme(algo_cfg_B, Xreal_time_target, Sensor_distr_B, ...
    N, GridMap, selection, sim_cfg.sensor, M, sim_cfg.rng_seed, ...
    struct('Ps', Sensor_distr_B(1).Ps, 'control_interval', 5), struct('min_pool_size', 1));
time_B = toc(tic_B);

result_B.name = '基线算法：第三章联合加权优化';
result_B.total_time = time_B;
disp(['算法 B 运行完成，总耗时: ', num2str(time_B), ' 秒']);

%% 4. 结果保存与评估准备
disp('>>> 正在保存实验一仿真数据...');
save_stamp = datestr(now, 'yyyymmdd_HHMMSS');

% 保存环境参数与跑完的结果 (Target_Init必须保存，因为甘特图和OSPA图需要读取出生时间)
% 以当前脚本位置为基准保存，避免工作目录不同导致路径找不到
this_dir = fileparts(mfilename('fullpath')); % chat4/01_Main_Env
if isempty(this_dir)
    % 兜底：在某些情况下（如仅执行片段/非文件上下文运行），mfilename 可能返回空
    this_path = which('main_exp1_performance.m');
    if ~isempty(this_path)
        this_dir = fileparts(this_path);
    end
end

chat4_root = fileparts(this_dir);            % chat4
data_dir = fullfile(chat4_root, '04_Data');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
save_file_name = fullfile(data_dir, sprintf('Exp1_Result_%s.mat', save_stamp));
% 再做一次目录存在性兜底（避免并发/权限导致的目录创建失败）
if ~exist(data_dir, 'dir')
    error('结果目录不存在：%s', data_dir);
end
save(save_file_name, 'Target_Init', 'Xreal_target_time', 'Xreal_time_target', ...
                     'result_A', 'result_B', 'ch4_cfg', 'sim_cfg', 'Sensor_distr');

disp(['实验一仿真完成！结果已存至: ', save_file_name]);
disp('您可以运行 picture_role_switch.m 或 picture_exp1_ospa.m 来查看结果图表了。');