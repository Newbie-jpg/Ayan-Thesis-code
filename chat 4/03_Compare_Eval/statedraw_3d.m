%% 三维状态提取模块
%% 输入: 粒子状态集4*1的cell，分别代表1.权值，2.均值，3.协方差，4.粒子个数
%% 输出: 当前时刻状态6*?的数组
%% 说明:
%% 1) 先用总强度 sum(w) 估计目标数 N_hat=round(sum(w))
%% 2) 再按权重从大到小选取前 N_hat 个分量
%% 3) 最后对空间位置很近的分量做一次近邻合并，减少同一目标被重复提取
function [state_draw, num_draw] = statedraw_3d(state)
    merge_distance = 135;
    % 针对目标消失后 1~2 步的弱残留提取门限
    % 当 round(sum(w))=0 但仍存在可解释的残留强度时，保留 1 个目标。
    min_single_keep_weight = 0.015;
    min_single_keep_sum = 0.02;

    state_draw = zeros(6, 0);
    num_draw = 0;
    if isempty(state) || numel(state) < 4 || isempty(state{4,1}) || state{4,1} <= 0
        return;
    end

    w = state{1,1};
    m = state{2,1};
    J = min(state{4,1}, min(numel(w), size(m, 2)));
    if J <= 0
        return;
    end

    w = w(1:J);
    m = m(:, 1:J);

    valid_mask = isfinite(w) & (w > 0);
    if size(m, 1) >= 5
        valid_mask = valid_mask & all(isfinite(m([1 3 5], :)), 1);
    else
        valid_mask = valid_mask & all(isfinite(m), 1);
    end
    w = w(valid_mask);
    m = m(:, valid_mask);

    if isempty(w)
        return;
    end

    sum_w = sum(w);
    N_hat = round(sum_w);
    if N_hat <= 0
        if max(w) >= min_single_keep_weight && sum_w >= min_single_keep_sum
            N_hat = 1;
        end
    end
    if N_hat <= 0
        return;
    end

    [w_sorted, idx_sorted] = sort(w, 'descend');
    keep_cnt = min(N_hat, numel(idx_sorted));
    cand_idx = idx_sorted(1:keep_cnt);
    cand_w = w_sorted(1:keep_cnt);
    cand_m = m(:, cand_idx);

    % 对相邻很近的分量做加权合并，避免同一目标被重复提取
    merged_states = zeros(size(cand_m, 1), 0);
    merged_weights = zeros(1, 0);
    used = false(1, size(cand_m, 2));
    for i = 1:size(cand_m, 2)
        if used(i)
            continue;
        end

        pos_i = cand_m([1 3 5], i);
        group = i;
        used(i) = true;
        for j = i+1:size(cand_m, 2)
            if used(j)
                continue;
            end
            pos_j = cand_m([1 3 5], j);
            if norm(pos_i - pos_j) <= merge_distance
                group(end+1) = j; %#ok<AGROW>
                used(j) = true;
            end
        end

        w_group = cand_w(group);
        m_group = cand_m(:, group);
        w_sum = sum(w_group);
        if w_sum <= 0
            continue;
        end

        merged_weights(end+1) = w_sum; %#ok<AGROW>
        merged_states(:, end+1) = (m_group * w_group(:)) / w_sum; %#ok<AGROW>
    end

    if isempty(merged_weights)
        return;
    end

    [~, idx_final] = sort(merged_weights, 'descend');
    state_draw = merged_states(:, idx_final);
    num_draw = size(state_draw, 2);
end