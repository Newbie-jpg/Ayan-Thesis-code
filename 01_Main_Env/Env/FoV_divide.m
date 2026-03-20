%{
    按视域划分：从融合后验中保留落在传感器 i 视域内的高斯分量
    输入：
        fusion_results:  融合后验 (4*1 cell: 权重, 均值, 协方差, 分量数)
        location:        传感器位置 3*1
        r_detect:        探测半径
    输出：
        state_in_fov:    视域内的后验 (4*1 cell)
        state_out_fov:   视域外的后验 (4*1 cell)，可选
%}
function [state_in_fov, state_out_fov] = FoV_divide(fusion_results, location, r_detect)
    J = fusion_results{4,1};
    if J == 0
        state_in_fov = fusion_results;
        state_out_fov = fusion_results;
        return;
    end
    w = fusion_results{1,1};
    m = fusion_results{2,1};
    P = fusion_results{3,1};
    in_mask = false(1, J);
    for j = 1:J
        in_mask(j) = (FoV_judge(location, m(:, j), r_detect) == 1);
    end
    J_in = sum(in_mask);
    J_out = J - J_in;
    state_in_fov = cell(4, 1);
    state_out_fov = cell(4, 1);
    idx_in = find(in_mask);
    idx_out = find(~in_mask);
    P_in = zeros(6, 6*J_in);
    for ii = 1:J_in
        P_in(:, 6*(ii-1)+1 : 6*ii) = P(:, 6*(idx_in(ii)-1)+1 : 6*idx_in(ii));
    end
    P_out = zeros(6, 6*J_out);
    for ii = 1:J_out
        P_out(:, 6*(ii-1)+1 : 6*ii) = P(:, 6*(idx_out(ii)-1)+1 : 6*idx_out(ii));
    end
    state_in_fov{1,1} = w(in_mask);
    state_in_fov{2,1} = m(:, in_mask);
    state_in_fov{3,1} = P_in;
    state_in_fov{4,1} = J_in;
    state_out_fov{1,1} = w(~in_mask);
    state_out_fov{2,1} = m(:, ~in_mask);
    state_out_fov{3,1} = P_out;
    state_out_fov{4,1} = J_out;
end
