function fused_pos = extract_fused_positions_for_search(v_fused, weight_thresh)
% EXTRACT_FUSED_POSITIONS_FOR_SEARCH 从融合 GM-PHD 中提取多目标实时位置 (x,y,z)，供 S 组搜索指向
%
% 与 statedraw_3d 不同：不按显示规则合并，而是保留权重大于阈值的**所有**分量均值，
% 使贪心规划可对多个“实时估计目标点”做分档/分散，减轻终点挤在同一片区域。
%
% 输入:
%   v_fused:     4×1 cell，{w, m, P, J}
%   weight_thresh: 权重阈值（可低于控制核 PIMS 的 0.1，仅用于搜索层）
%
% 输出:
%   fused_pos:   3×K，每列为一个目标的位置 [x;y;z]

    fused_pos = zeros(3, 0);
    if nargin < 2 || isempty(weight_thresh)
        weight_thresh = 0.05;
    end
    if isempty(v_fused) || numel(v_fused) < 4
        return;
    end

    w = v_fused{1, 1};
    m = v_fused{2, 1};
    J = v_fused{4, 1};
    if isempty(w) || isempty(m) || isempty(J) || J <= 0
        return;
    end

    for j = 1:J
        if j > numel(w)
            break;
        end
        if w(j) >= weight_thresh
            mj = m(:, j);
            if numel(mj) >= 5
                fused_pos = [fused_pos, [mj(1); mj(3); mj(5)]]; %#ok<AGROW>
            end
        end
    end
end
