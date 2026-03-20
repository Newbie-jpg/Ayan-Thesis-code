# Chapter 4 Code (chat 4)

This folder contains the MATLAB implementation for Chapter 4: a two-stage multi-sensor search-and-tracking control framework.

## Output folders
- Simulation results (.mat): `chat 4/04_Data/`
- Figures (OSPA, discovery stats, role-switch diagram): `chat 4/03_Compare_Eval/Result_Fig_Ch4/`

## Folder structure (high level)
- `01_Main_Env/`: experiment entry points, top-level control loops, global configuration
- `02_Algorithm/03_Local_Estimation/`: local GM-PHD and grid transition utilities (kept from Chapter 3)
- `02_Algorithm/04_Information_Fusion/`: centralized AGM fusion and grid probability GA fusion (kept from Chapter 3)
- `02_Algorithm/05_Task_Allocation/`: Chapter 4 task decomposition (dynamic assignment to S/T sensors)
- `02_Algorithm/06_Decision_Optimization/`: Chapter 4 decision optimization (T-group tracking decision, S-group search decision)
- `03_Compare_Eval/`: plotting scripts for Chapter 4 experiments
- `04_Data/`: cached experiment outputs

## How to run Experiment 1
1. Run:
   - `chat 4/01_Main_Env/main_exp1_performance.m`
2. It will run:
   - Algorithm A (proposed): `run_control_twostage.m`
   - Algorithm B (baseline): `run_control_scheme.m` (calls Chapter 3 `control_core.m`)
3. It saves a timestamped result file to:
   - `chat 4/04_Data/Exp1_Result_YYYYmmdd_HHMMSS.mat`

## Generate figures for Experiment 1
Run:
- `chat 4/03_Compare_Eval/picture_exp1_ospa_and_discovery.m`

This produces (in `Result_Fig_Ch4/`):
- OSPA distance comparison curve (Algorithm A vs B)
- Discovery statistics bar charts:
  - cumulative discovered target count
  - average discovery delay (first-discovery time minus birth time)

Role-switch (gantt-style) diagram:
- `chat 4/03_Compare_Eval/picture_role_switch.m`

## Key runtime components
- Configuration / paths: `01_Main_Env/config.m`
- Proposed two-stage controller: `01_Main_Env/run_control_twostage.m`
  - Stage 1: local estimation + centralized AGM fusion
  - Stage 2: dynamic task allocation (S/T) + grouped decision:
    - S-group: `02_Algorithm/06_Decision_Optimization/greedy_path_planner.m` + `control_core_S.m`
    - T-group: `02_Algorithm/06_Decision_Optimization/control_core_T.m`
- Baseline controller (Chapter 3 joint optimization):
  - `01_Main_Env/run_control_scheme.m` -> `02_Algorithm/06_Decision_Optimization/control_core.m`

## Notes
- These scripts assume you run them from MATLAB with the folder paths handled by `config.m`.
- If you use parallel computing, ensure Parallel Toolbox is available; otherwise the code falls back to serial execution where possible.

