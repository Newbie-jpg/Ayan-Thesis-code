%{
    区域网格搜索密度递归执行程序
    功能：根据传感器当前位置和视域覆盖情况，预测并更新全域网格的搜索密度
    对应文稿：第三章 公式 (3.16) 和 (3.17)
%}
function [GridProb_new] = Update_Grid_Density(GridMap, GridProb_old, location, r_detect, Pd)
    % 获取参数
    if isfield(GridMap, 'Ps_grid') && ~isempty(GridMap.Ps_grid)
        Ps_grid = GridMap.Ps_grid;
    else
        Ps_grid = 0.99;
    end
    if isfield(GridMap, 'B_birth') && ~isempty(GridMap.B_birth)
        B_birth = GridMap.B_birth;
    else
        B_birth = 1e-3;
    end
    if ~isfield(GridMap, 'T_kernel') || isempty(GridMap.T_kernel)
        T_kernel = [0.05, 0.10, 0.05; 0.10, 0.40, 0.10; 0.05, 0.10, 0.05];
        T_kernel = T_kernel / sum(T_kernel(:));
    else
        T_kernel = GridMap.T_kernel;
    end
    if isfield(GridMap, 'nx') && isfield(GridMap, 'ny') && ~isempty(GridMap.nx) && ~isempty(GridMap.ny)
        nx = GridMap.nx;
        ny = GridMap.ny;
    else
        nx = numel(unique(GridMap.c(1, :)));
        ny = numel(unique(GridMap.c(2, :)));
    end
    if nargin < 5 || isempty(Pd)
        Pd = 1;
    end
    Pd = min(max(Pd, 0), 1);
    
    % 1) 预测：P_{k|k-1} = P_S * (P_{k-1|k-1} * T) + B
    P_prev = reshape(GridProb_old(:), [ny, nx]);
    if isscalar(B_birth)
        B_mat = B_birth * ones(ny, nx);
    else
        B_mat = reshape(B_birth(:), [ny, nx]);
    end
    P_pred = Ps_grid * conv2(P_prev, T_kernel, 'same') + B_mat;
    P_pred = min(max(P_pred, 0), 1);

    % 2) 构建覆盖指示：计算所有网格中心到传感器位置的三维距离
    diff_x = GridMap.c(1, :) - location(1, 1);
    diff_y = GridMap.c(2, :) - location(2, 1);
    diff_z = GridMap.c(3, :) - location(3, 1);
    distances = sqrt(diff_x.^2 + diff_y.^2 + diff_z.^2);
    I_cov = reshape((distances <= r_detect), [ny, nx]);

    % 3) 更新：P_{k|k} = P_{k|k-1} ⊙ (1 - P_D)
    % 仅覆盖区域使用真实检测概率，其余区域 P_D=0
    Pd_mat = zeros(ny, nx);
    Pd_mat(I_cov) = Pd;
    P_post = P_pred .* (1 - Pd_mat);
    P_post = min(max(P_post, 0), 1);
    GridProb_new = P_post(:);

end