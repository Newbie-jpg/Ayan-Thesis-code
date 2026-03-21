function J = oracle_guidance_cost(loc_after, sel, oracle_targets, sensor_ids, use_diverse_assign)
% ORACLE_GUIDANCE_COST 控制层伪目标引导代价
%
%   use_diverse_assign (可选): 为 true 时，传感器 i 绑定第 mod(i-1,Npt)+1 列伪目标（分散多平台，避免全员追最近点）；
%                               为 false 时，各传感器到「最近」伪目标的 min 距离平方和（原行为）。
%
%   loc_after:   cell，loc_after{i,ci} 为 3×1
%   sel:         长度 = numel(sensor_ids)，sel(k) 为第 sensor_ids(k) 个传感器的动作下标
%   oracle_targets: 3×N_pt，列为带噪伪目标位置；空则返回 0
%   sensor_ids:  参与累加的传感器编号（1..N_sensor）

    J = 0;
    if isempty(oracle_targets) || size(oracle_targets, 2) < 1
        return;
    end
    if nargin < 4 || isempty(sensor_ids)
        return;
    end
    if nargin < 5 || isempty(use_diverse_assign)
        use_diverse_assign = false;
    end

    nt = size(oracle_targets, 2);
    for k = 1:numel(sensor_ids)
        i = sensor_ids(k);
        if k > numel(sel)
            break;
        end
        ci = sel(k);
        if i > numel(loc_after) || isempty(loc_after{i, ci})
            continue;
        end
        loc = loc_after{i, ci}(:);
        if numel(loc) < 3
            continue;
        end
        loc = loc(1:3);

        if use_diverse_assign
            col = 1 + mod(i - 1, nt);
            anchor = oracle_targets(:, col);
            J = J + sum((loc - anchor) .^ 2);
        else
            d2 = sum((oracle_targets - loc) .^ 2, 1);
            J = J + min(d2);
        end
    end
end
