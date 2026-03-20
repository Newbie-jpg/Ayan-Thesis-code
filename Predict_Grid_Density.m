%{
    区域网格仅预测步骤（对应 chapter3 式 P_{k|k-1}=P_S*(P_{k-1|k-1}*T)+B）
%}
function GridProb_pred = Predict_Grid_Density(GridMap, GridProb_old)
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

    P_prev = reshape(GridProb_old(:), [ny, nx]);
    if isscalar(B_birth)
        B_mat = B_birth * ones(ny, nx);
    else
        B_mat = reshape(B_birth(:), [ny, nx]);
    end

    P_pred = Ps_grid * conv2(P_prev, T_kernel, 'same') + B_mat;
    P_pred = min(max(P_pred, 0), 1);
    GridProb_pred = P_pred(:);
end
