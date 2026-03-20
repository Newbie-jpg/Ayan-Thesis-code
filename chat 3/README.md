有限视域下多传感器多目标搜索与跟踪系统 (Chapter 3)
📖 项目简介
本项目为硕士毕业论文第三章的仿真代码实现。针对传感器视域受限条件下的多目标搜索与跟踪冲突问题，构建了一个包含本地状态估计、多传感器信息融合与决策优化三个核心模块的闭环管控框架。

系统旨在通过动态调整传感器的运行状态，最大化对未知区域的搜索探索收益，同时维持对已知目标的高精度跟踪。

📂 代码架构与目录说明
为了保持代码逻辑清晰，整个工程重构为 4 个核心文件夹：

01_Main_Env/ (主控程序与环境模型)
负责控制全局仿真流转、参数配置以及真实物理环境的模拟。

main.m / PROCESS.m: 程序的总入口与单次蒙特卡洛仿真的主循环控制脚本。

config.m: 全局参数配置文件（滤波参数、网格大小、传感器动力学约束等）。

Env/: 包含目标真实轨迹生成 (targetset.m)、传感器观测生成 (observe_FoV_3d_single.m)、视域判断剥除 (FoV_judge.m) 以及自适应观测噪声计算等环境交互底层代码。

02_Algorithm/ (核心算法模块)
实现了论文第三章的核心理论机制，按逻辑流分为三个子目录：

03_Local_Estimation/ (本地状态估计)

采用基于 UKF 的 GM-PHD 滤波器 (ALG3_PHD1time_3d_ukf_distr.m) 处理视域内的目标跟踪。

采用区域网格机制 (Predict/Update_Grid_Density.m) 对视域外潜在目标的出现概率进行递归更新。

04_Information_Fusion/ (多传感器融合)

利用欧式距离与匈牙利算法 (ALG3_Match.m, Hungarian.m) 完成高斯分量匹配。

采用算术-几何平均 (AGM)准则 (Centralized_Fusion_AGM.m) 融合公共视域内的目标后验状态。

采用几何平均 (GA)准则 (Fuse_GridProb_GA.m) 融合视域外的网格存在概率。

05_Decision_Optimization/ (协同决策管控)

基于预测理想量测集 PIMS (PIMS_generate.m) 执行虚拟预测与伪更新 (PHD_pseudo_update.m)。

核心决策函数 (control_core.m)：通过线性加权组合基于网格概率的搜索目标函数与基于协方差/基数补偿的跟踪目标函数，寻优输出下一步多传感器联合动作。

03_Compare_Eval/ (对比算法与性能评估)
用于验证本章所提算法有效性的对比实验与指标评测模块。

gmphd_cs_divergence.m: 传统基于 Cauchy-Schwarz (CS) 散度的管控算法（Baseline）。

ospa_dist.m: 核心评价指标，用于计算多目标跟踪的 OSPA 距离（综合评估位置误差与势估计误差）。

picture.m / statedraw_3d.m: 结果可视化脚本，用于绘制传感器/目标运动轨迹对比图及 OSPA 指标曲线。

04_Data/ (数据缓存)
用于存储预设的目标轨迹文件 (Xreal_target_time.mat) 或是仿真运行中生成的临时结果 (CoreData...mat)，避免绘图时重复进行高耗时的蒙特卡洛仿真。

🚀 运行指南
环境要求:

MATLAB (建议 R2021a 及以上版本)。

需确保安装了统计与机器学习工具箱 (Statistics and Machine Learning Toolbox) 等基础工具箱。

启动仿真:

打开 MATLAB，将当前工作目录切换至项目根目录。

在命令行运行代码自动添加所有子文件路径：addpath(genpath(pwd));

运行 01_Main_Env/main.m 即可开始执行整个时序仿真。

查看结果:

仿真结束后，运行 03_Compare_Eval/picture.m 即可生成论文中展示的轨迹跟踪图与 OSPA 性能对比图。

💡 核心创新点回顾
视域受限下的双重建模：利用 GM-PHD 处理可视域目标，概率栅格处理盲区探索。

分层融合机制：创新性地将 AGM 融合（针对已知高斯分量）与 GA 融合（针对未知网格）结合。

搜索与跟踪联合寻优：引入基数惩罚因子与 PIMS，在统一框架下量化了长期跟踪收益与广域搜索收益的冲突。