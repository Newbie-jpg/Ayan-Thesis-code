%{
                                算法3子函数
           输入: 1.真实数据
                 2.观测数据
                 3.传感器属性
                 4.总跟踪时长

           输出: 1.跟踪结果的OSPA (包含每个时刻的OSPA数值)
                 2.跟踪结果的数量 (包含每个时刻的数量)
                 3.算法跟踪总耗时 

          
%}
function [OSPA,OSPA_sensor,Num_estimate,Time,Sensor_traj,X_est_series,decision_log] = PROCESS(m, Xreal_time_target, Sensor, N, GridMap, ~, sensor, control_opt, process_opt, parallel_opt)

%===一些默认的参数===
if nargin < 7
    error(['PROCESS 输入参数不足。正确调用示例：', newline, ...
        'PROCESS(m, Xreal_time_target, Sensor_distr, N, GridMap, selection, sensor, control_opt, process_opt, parallel_opt)']);
end

if nargin < 8 || isempty(control_opt)
    control_opt = struct();
end
if nargin < 9 || isempty(process_opt)
    process_opt = struct();
end
if nargin < 10 || isempty(parallel_opt)
    parallel_opt = struct();
end

process_opt = set_default_fields(process_opt, struct( ...
    'Ps', 1, ...
    'Vx_thre', 200, ...
    'Vy_thre', 200, ...
    'Vz_thre', 200, ...
    'control_interval', 5));
parallel_opt = set_default_fields(parallel_opt, struct( ...
    'process_inner_pool_max', 8));
control_opt = set_default_fields(control_opt, struct( ...
    'match_threshold', 200, ...
    'eta', 50000, ...
    'beta', 500));

Ps = process_opt.Ps; % 存活概率
Vx_thre = process_opt.Vx_thre;
Vy_thre = process_opt.Vy_thre;
Vz_thre = process_opt.Vz_thre;
match_threshold = control_opt.match_threshold;
N_sensor = size(Sensor,2);
use_grid_mechanism = true;
if isfield(control_opt, 'objective_mode') && strcmpi(control_opt.objective_mode, 'cs')
    use_grid_mechanism = false;
end
Sensor_traj = zeros(3, N, N_sensor);
X_est_global = cell(N, 1);
for i = 1:N_sensor
    loc_i = Sensor(i).location;
    if isempty(loc_i) || numel(loc_i) ~= 3
        error('PROCESS:SensorLocationMissing', ...
            '传感器 %d 的 location 为空或非 3×1 向量。请检查 config.m 中是否为所有 %d 个传感器设置了初始位置。', i, N_sensor);
    end
    Sensor_traj(:,1,i) = loc_i(:);
    if N >= 2
        Sensor_traj(:,2,i) = loc_i(:);
    end
end

%===预处理===
% 如果检测概率很高，内部当成1来进行滤波
for i = 1:N_sensor
    if Sensor(i).Pd >= 0.99
        Sensor(i).Pd = 1;
    end
end

%======PHD最初估计值初始化======
ALG3_PHD_initial;

%====== 仅当在客户端且无池时启动并行池（供滤波内 parfor 使用；worker 内不启动）======
if isempty(getCurrentTask()) && isempty(gcp('nocreate'))
    parpool('local', min(parallel_opt.process_inner_pool_max, feature('numcores')));
end

for i = 1:N_sensor
    % 生成 t=1 观测
    Z_p1 = observe_FoV_3d_single(Xreal_time_target{1,1}, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
    Sensor(i).Z_dicaer_global{1,1} = polar2dicaer_3d_single(Z_p1, Sensor(i).location(1), Sensor(i).location(2), Sensor(i).location(3));
    
    % 生成 t=2 观测
    Z_p2 = observe_FoV_3d_single(Xreal_time_target{2,1}, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
    Sensor(i).Z_dicaer_global{2,1} = polar2dicaer_3d_single(Z_p2, Sensor(i).location(1), Sensor(i).location(2), Sensor(i).location(3));
    
    % 初始化单传感器局部估计（融合前），用于 per-sensor OSPA
    Sensor(i).X_est_local{1,1} = zeros(6,0);
    Sensor(i).X_est_local{2,1} = zeros(6,0);
    % 初始化融合后按视域划分的每传感器估计
    Sensor(i).X_est_fov{1,1} = zeros(6,0);
    Sensor(i).X_est_fov{2,1} = zeros(6,0);
end
X_est_global{1,1} = zeros(6,0);
X_est_global{2,1} = zeros(6,0);

% 时间消耗
Time=0;
control_interval = process_opt.control_interval;  % 每隔 control_interval 个时刻更新一次控制动作
last_action = [];      % 缓存上一次控制动作
decision_log = struct('times', [], 'track_cost', [], 'search_gain', [], ...
    'cs_gain', [], 'info_gain', [], 'total_cost', [], 'selection', [], ...
    'objective_mode', '');
total_steps = N - 2; % t 从 3 开始到 N 共 N-2 次
progress_refresh = max(1, floor(total_steps / 10)); % 约每10%刷新一次
display_time_total = 0; % 用于 ETA 显示（只做展示，不影响原始 Time 统计）

for t = 3:N
    timing_total = tic;
    timing = tic;
    %==================
    % 单传感器滤波过程（for 循环；多核并行在 ALG3_PHD1time_3d_ukf_distr 内按高斯分量 parfor）
    %==================
    for i = 1:N_sensor 
        % 1. 动态生成当前 t 时刻的观测 (基于移动后的最新 location)
        Z_p_t = observe_FoV_3d_single(Xreal_time_target{t,1}, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd, Sensor(i).Zr, Sensor(i).R_params);
        Sensor(i).Z_polar_part{t,1} = Z_p_t;
        
        % 2. 转换为全局笛卡尔坐标用于新生逻辑
        Sensor(i).Z_dicaer_global{t,1} = polar2dicaer_3d_single(Z_p_t, Sensor(i).location(1), Sensor(i).location(2), Sensor(i).location(3));
        
        % 3. 执行滤波（内部对高斯分量做 parfor，充分利用多核）
        [Sensor(i).state] = ALG3_PHD1time_3d_ukf_distr(Sensor(i).Z_dicaer_global{t-2,1},...
            Sensor(i).Z_dicaer_global{t-1,1}, Sensor(i).Z_polar_part{t,1},...
            Sensor(i).state, Sensor(i).Zr, Sensor(i).R_params, Ps, Sensor(i).Pd,...
            Vx_thre, Vy_thre, Vz_thre,...
            Sensor(i).location(1,1), Sensor(i).location(2,1), Sensor(i).location(3,1));   

        % 4. 搜索网格预测更新（仅 pure-cs 模式关闭区域网格机制）
        if use_grid_mechanism
            Sensor(i).GridProb = Update_Grid_Density(GridMap, Sensor(i).GridProb, Sensor(i).location, Sensor(i).R_detect, Sensor(i).Pd);
        end
        
        % 5. 提取融合前局部估计，用于单传感器 OSPA
        [Sensor(i).X_est_local{t,1}, ~] = statedraw_3d(Sensor(i).state);
    end
    t_sus = toc(timing);
    Time = Time + t_sus/N_sensor;
    timing = tic;
    
    %===融合中心接收信息===
    for i=1:N_sensor 
        Fusion_center.sensor_inf(i).gm_particles = Sensor(i).state; % 传感器估计的状态信息
        Fusion_center.sensor_inf(i).location = Sensor(i).location;  % 传感器的位置信息
        Fusion_center.sensor_inf(i).R_detect = Sensor(i).R_detect;  % 传感器的观测半径
        Fusion_center.sensor_inf(i).serial = Sensor(i).serial;      % 传感器的编号
        Fusion_center.sensor_inf(i).GridProb = Sensor(i).GridProb;  % 搜索网格密度
        Fusion_center.sensor_inf(i).Pd = Sensor(i).Pd;              % 检测概率（供网格更新收益计算）
        Fusion_center.sensor_inf(i).R_params = Sensor(i).R_params;    % 距离相关噪声参数（供 control_core 伪更新用）
    end
    
    %===AGM融合方法===
    [Fusion_center.results] = Centralized_Fusion_AGM(Fusion_center.sensor_inf,match_threshold);
    % 添加搜索网格信息融合（chapter3：网格概率采用 GA 融合）
    if use_grid_mechanism
        Fusion_center.GridProb_fused = Fuse_GridProb_GA(Fusion_center.sensor_inf);
    else
        Fusion_center.GridProb_fused = [];
    end

    %===状态提取：全局 + 融合后按各传感器视域划分的估计（用于 per-sensor OSPA）===
    [X_est_global{t,1},~] = statedraw_3d(Fusion_center.results);
    for i=1:N_sensor  
        [Sensor(i).X_est{t,1},~] = statedraw_3d( Fusion_center.results);       
        [state_in_fov, ~] = FoV_divide(Fusion_center.results, Sensor(i).location, Sensor(i).R_detect);
        [Sensor(i).X_est_fov{t,1}, ~] = statedraw_3d(state_in_fov);
    end

    %=== 核心控制：每隔固定时刻更新一次动作，其余时刻沿用上一时刻动作 ===
    if isempty(last_action) || mod(t - 3, control_interval) == 0
        control_opt_run = control_opt;
        [~, action, ~, control_detail] = control_core( ...
            Fusion_center.results, Fusion_center.sensor_inf, GridMap, sensor, control_opt_run);
        last_action = action;

        decision_log.times(end+1) = t; %#ok<AGROW>
        decision_log.track_cost(end+1) = control_detail.track_cost; %#ok<AGROW>
        decision_log.search_gain(end+1) = control_detail.search_gain; %#ok<AGROW>
        decision_log.cs_gain(end+1) = control_detail.cs_gain; %#ok<AGROW>
        decision_log.info_gain(end+1) = control_detail.info_gain; %#ok<AGROW>
        decision_log.total_cost(end+1) = control_detail.total_cost; %#ok<AGROW>
        decision_log.selection(:, end+1) = control_detail.selection(:); %#ok<AGROW>
        decision_log.objective_mode = control_detail.objective_mode;
    else
        action = last_action;
    end

%     action

    for i = 1:N_sensor
        act_beta = action(1, i);     % 偏航角
        act_epsilon = action(2, i);  % 俯仰角
        % config 中 T 的单位为秒，位移应按 v*T 计算
        dt_sec = sensor.T; 
        
        dx = sensor.v * cos(deg2rad(act_epsilon)) * cos(deg2rad(act_beta)) * dt_sec;
        dy = sensor.v * cos(deg2rad(act_epsilon)) * sin(deg2rad(act_beta)) * dt_sec;
        dz = sensor.v * sin(deg2rad(act_epsilon)) * dt_sec;
        
        % 移动传感器到新位置
        Sensor(i).location = Sensor(i).location + [dx; dy; dz];
        Sensor_traj(:,t,i) = Sensor(i).location;
        
        % 剔除视野外分量
        [Sensor(i).state, ~] = FoV_divide(Fusion_center.results, Sensor(i).location, Sensor(i).R_detect);
    end


    t_sus = toc(timing);
    Time = Time + t_sus;
    
    % 计算该步总耗时（用于 ETA）
    dt_total = toc(timing_total);
    display_time_total = display_time_total + dt_total;
    steps_done = t - 2;
    if m == 1 && isempty(getCurrentTask()) && (mod(steps_done, progress_refresh) == 0 || t == N)
        avg_step = display_time_total / steps_done;
        steps_left = total_steps - steps_done;
        eta_sec = avg_step * steps_left;
        fprintf('>>> 算法B进度：%d/%d 步，单步均耗时 %.2f s，预计剩余 %.1f s (%.1f min)\n', ...
            steps_done, total_steps, avg_step, eta_sec, eta_sec/60);
    end
  
end
Time = Time / ( N - 2 ); % 单个融合周期时间计算

%===分布式 OSPA 计算===
OSPA = zeros(1,N);
OSPA(1:2) = 0;
OSPA_sensor = zeros(N_sensor, N);  % 各传感器局部估计的 OSPA
OSPA_sensor(:, 1:2) = 0;
for t=3:N
    Xreal_t = Xreal_time_target{t,1};
    Xreal_t(:, isnan(Xreal_t(1,:))) = [];

    % 全局 OSPA：直接使用融合中心结果提取出的全局估计
    X_est_global_t = X_est_global{t,1};
    OSPA(t) = ospa_dist(Xreal_t, X_est_global_t, 120, 2);

    for i = 1:N_sensor
        OSPA_sensor(i, t) = ospa_dist(Xreal_t, Sensor(i).X_est_fov{t,1}, 120, 2);
    end
end

%===分布式数量计算===
Num_estimate = zeros(1,N);
for t=1:N
    Num_estimate(t) = size(X_est_global{t,1},2);
end
X_est_series = Sensor(1).X_est;

%{
% =======计算每个时刻真实目标数目=======
num_real = zeros(1,N); % 全局真实目标数目
for t=1:N
    num_real(1,t) = size(Xreal_time_target{t,1},2);
end

figure
hold on;
plot(OSPA,'-ko','Markerface','b');
legend('分布式GA','FontSize',14);
t=title('OSPA');
t.FontSize = 14;
xlabel('时刻 t/s','FontSize',14)
ylabel('OSPA距离','FontSize',14)
disp(num2str(sum(OSPA(3:end))/98));

figure
hold on;
plot(num_real,'-k');
plot(Num_estimate,'-ko','Markerface','b');
legend('真实值','分布式GA','FontSize',14);
t=title('数量跟踪对比');
t.FontSize = 14;
xlabel('时刻 t/s','FontSize',14)
ylabel('估计数量','FontSize',14)

figure
hold on;
flag_realtarget=0;
flag_ob=0;
flag_estimate=0;

% 画出平台节点以及其检测范围
for i=1:N_sensor
    text(Sensor(i).location(1,1),Sensor(i).location(2,1),num2str(i),'Fontsize',20,'Position',...
        [Sensor(i).location(1,1),Sensor(i).location(2,1)]);
    h_sensor=plot(Sensor(i).location(1,1),Sensor(i).location(2,1),'k^','Markerface','y','MarkerSize',10);
    draw_circle(Sensor(i).location(1,1),Sensor(i).location(2,1),Sensor(i).R_detect);
end

for t=1:N
    
    if t>2
        %===估计值===
        h_estimate=plot(Sensor(1).X_est{t,1}(1,:),Sensor(1).X_est{t,1}(3,:),'bo');
        hold on;
        if ~isempty(h_estimate) && flag_estimate==0
            leg_estimate=h_estimate;
            flag_estimate=1;
        end
    end
    
end

for t=1:N
    %===真实值===
    h_realtarget=plot(Xreal_time_target{t,1}(1,:),Xreal_time_target{t,1}(3,:),'r.');
    hold on;
end

% 获取legend素材
if ~isempty(h_realtarget) && flag_realtarget==0
    leg_realtarget=h_realtarget;
    flag_realtarget=1;
end
if isempty(h_estimate)
    h_estimate=plot(0,0,'bo');
    leg_estimate=h_estimate;
    hold on;
end

xlabel('X轴坐标/m','FontSize',20);
ylabel('Y轴坐标/m','FontSize',20);
set(gca,'FontSize',20);
t=title('xxx','FontSize',20);
t.FontSize = 20;
grid on;
axis equal
legend([h_sensor,leg_realtarget,leg_estimate],...
    {'传感器坐标','真实目标点迹','传感器估计点迹'},'Fontsize',20);
%}
end

function out = set_default_fields(in, defaults)
out = in;
fns = fieldnames(defaults);
for ii = 1:numel(fns)
    key = fns{ii};
    if ~isfield(out, key) || isempty(out.(key))
        out.(key) = defaults.(key);
    end
end
end
