# `config.m` 参数说明（当前版本）

本文档对应文件：`chat 4/01_Main_Env/config.m`。  
目标是说明**当前代码框架下真实生效的参数**，避免旧文档字段与实现不一致。

---

## 1. 基础仿真参数

### `sim_cfg.rng_seed`
- 随机种子，影响目标轨迹扰动与 Oracle 带噪点生成的可复现性。

### `N`, `T`, `M`
- `N`：总步数（默认 100）
- `T`：采样周期（默认 1s）
- `M`：蒙特卡洛次数（默认 1）

---

## 2. 网格地图参数 `GridMap`

### 空间范围与分辨率
- `X_range`, `Y_range`, `Z_range`
- `step_size`

### 网格状态参数
- `GridMap.omega0`：初始网格概率
- `GridMap.Ps_grid`：网格存活概率
- `GridMap.B_birth`：网格新生概率

### 网格转移核
- `target_vmax_xy` 用于 `Build_Grid_Transition_Kernel(...)`

---

## 3. 传感器参数

### 每个传感器（`Sensor_distr(i)`）
- `serial`：编号
- `location`：初始位置
- `R_detect`：探测半径
- `Pd`：探测概率
- `Ps`：存活概率
- `Zr`：杂波强度
- `R_params`：观测噪声模型参数

### 全局机动参数（`sim_cfg.sensor`）
- `num`：传感器数量
- `v`：速度
- `C`：离散偏航角集合（度）
- `L`：预测步长
- `T`：采样时间（与全局 `T` 对齐）

---

## 4. 固定搜索组合

`sim_cfg.fixed_search_target_groups` 定义各传感器默认搜索目标组：

- 传感器1 -> 目标3
- 传感器2 -> 目标1、2
- 传感器3 -> 目标4
- 传感器4 -> 目标5

该配置用于 Oracle 引导下的“视域外固定组合搜索 + 出生点引导”。

---

## 5. 算法 B（第三章基线）参数

配置对象：`sim_cfg.algo_baseline`

### 核心项
- `objective_mode`（当前为 `grid`）
- `match_threshold`
- `eta`
- `beta`
- `W`
- `L`

### Oracle 开关与参数
- `enable_oracle_tracking`
  - `true`：启用 Oracle 基线逻辑
  - `false`：严格第三章 `control_core` 逻辑
- `oracle_cfg`：绑定 `sim_cfg.oracle.B`
- `fixed_search_target_groups`
- `target_init`

### B 的 Oracle 噪声与机动约束（`sim_cfg.oracle.B`）
- `sigma_xy`, `sigma_z`, `clip_max`
- `rng_seed_base`
- `max_turn_deg`（转向限制，体现更“费时费力”）
- `search_speed_scale`（搜索状态速度比例）

---

## 6. 算法 A（第四章）参数

配置对象：`ch4_cfg`

### 论文原始任务分配参数
- `k_steps`
- `theta_l`
- `theta_h`
- `W`
- `beta`
- `search_threshold`

### Oracle 开关与参数
- `enable_oracle_guidance`
  - `true`：启用“视域内带噪真值跟踪 + 视域外固定组合搜索”
  - `false`：回到第四章原始两阶段（`dynamic_task_allocation + control_core_T + greedy_path_planner`）
- `oracle_cfg`：绑定 `sim_cfg.oracle.A`
- `fixed_search_target_groups`

### A 的 Oracle 噪声（`sim_cfg.oracle.A`）
- `sigma_xy`, `sigma_z`, `clip_max`
- `rng_seed_base`
- `max_turn_deg`
- `search_speed_scale`

---

## 7. 目标场景参数

### `Target_Init`
- 每列一个目标，包含初始状态、出生时刻、消亡时刻、运动模式。
- 当前配置包含 5 个目标。

### 场景共享字段
- `sim_cfg.target_init = Target_Init`
- 同时传给 A/B 逻辑，支持出生前引导。

---

## 8. 与主程序的关系

### 主入口
- `main_exp1_performance.m`
  - 运行算法A：`run_control_twostage.m`
  - 运行算法B：`run_control_scheme -> PROCESS.m`

### 决策频率（当前实现）
- A：每 5 步更新动作
- B：每 5 步更新动作（包含 Oracle 基线）

---

## 9. 常见使用场景

### 想回到论文原始策略
- `ch4_cfg.enable_oracle_guidance = false`
- `sim_cfg.algo_baseline.enable_oracle_tracking = false`

### 想保留 Oracle 但调弱其影响
- 增大 `sigma_xy`
- 减小 `clip_max`
- 减小 `search_speed_scale`
- 减小 `max_turn_deg`

---

## 10. 备注

- `config.m` 会自动加入 `chat 4` 全路径：`addpath(genpath(chat4_root))`。
- 若修改了字段名或默认值，请同步更新本文件。
