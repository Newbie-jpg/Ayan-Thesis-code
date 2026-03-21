# 第四章代码说明（`chat 4`）

本目录为第四章“多传感器多目标搜索与跟踪”实验代码，当前支持两种可切换策略：

- **论文原始逻辑**
  - 算法A：任务分配（S/T）+ 分组优化（`dynamic_task_allocation` + `greedy_path_planner` + `control_core_T`）
  - 算法B：第三章联合优化基线（`control_core`）
- **Oracle 引导逻辑（当前默认可开）**
  - 视域内：带噪真值引导跟踪
  - 视域外：固定目标组合搜索 + 目标出生前方向引导
  - A/B 噪声与运动约束可不同（A更准，B更“费时费力”）

---

## 目录结构

- `01_Main_Env/`：主程序、全局配置、核心主循环
- `02_Algorithm/03_Local_Estimation/`：局部 GM-PHD 与网格转移
- `02_Algorithm/04_Information_Fusion/`：AGM 融合与网格 GA 融合
- `02_Algorithm/05_Task_Allocation/`：第四章任务分配模块
- `02_Algorithm/06_Decision_Optimization/`：决策优化模块（A/B）
- `03_Compare_Eval/`：绘图与对比评估脚本
- `04_Data/`：实验输出数据（`.mat`）

---

## 运行实验（实验一）

运行：

- `chat 4/01_Main_Env/main_exp1_performance.m`

该脚本会依次执行：

- **算法A**：`run_control_twostage.m`
- **算法B**：`run_control_scheme.m -> PROCESS.m`

并保存结果到：

- `chat 4/04_Data/Exp1_Result_YYYYmmdd_HHMMSS.mat`

---

## 当前决策节奏

- 算法A：每 **5** 步更新一次动作（中间沿用上次动作）
- 算法B：每 **5** 步更新一次动作  
  - 即使启用 Oracle 基线，也已改为每 5 步更新，并只在更新时记录决策时刻

---

## 开关说明（是否启用 Oracle 引导）

配置文件：`chat 4/01_Main_Env/config.m`

- 算法A开关：`ch4_cfg.enable_oracle_guidance`
  - `true`：Oracle 引导（视域内带噪真值 + 视域外固定组合搜索）
  - `false`：回到论文原始两阶段逻辑

- 算法B开关：`sim_cfg.algo_baseline.enable_oracle_tracking`
  - `true`：Oracle 基线
  - `false`：回到第三章严格基线 `control_core`

---

## 绘图脚本

推荐使用新版评估脚本：

- `chat 4/03_Compare_Eval/picture_exp1_metrics_rewrite.m`

输出内容：

- 目标与传感器轨迹图（每个算法单独窗口）
- OSPA 对比曲线
- 累计计算时间阶梯曲线（每步耗时累加）
- 平均发现延迟柱状图（支持“连续 k 步覆盖才算发现”）
- 每 10 步在控制台打印传感器 S/T 状态

其它脚本（按需）：

- `picture_role_switch.m`：角色切换可视化
- `picture_exp1_ospa_and_discovery.m`：旧版指标图

---

## 关键入口文件

- 全局配置：`01_Main_Env/config.m`
- 主实验脚本：`01_Main_Env/main_exp1_performance.m`
- 算法A主循环：`01_Main_Env/run_control_twostage.m`
- 算法B主循环：`01_Main_Env/PROCESS.m`
- Oracle 动作生成：`01_Main_Env/Env/build_oracle_guidance_actions.m`

---

## 说明

- `config.m` 会自动 `addpath(genpath(chat4_root))`，通常无需手动加路径。
- 并行工具箱可用时会自动启用并行；不可用时可串行运行。

