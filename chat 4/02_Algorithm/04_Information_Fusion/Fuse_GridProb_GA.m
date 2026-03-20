%{
    多传感器网格概率 GA 融合（chapter3 式 p_fuse = prod_i p_i^{pi_i}）
%}
function GridProb_fused = Fuse_GridProb_GA(sensor_inf, fusion_weights)
    N_sensor = numel(sensor_inf);
    if N_sensor == 0
        GridProb_fused = [];
        return;
    end

    if nargin < 2 || isempty(fusion_weights)
        fusion_weights = ones(1, N_sensor) / N_sensor;
    else
        fusion_weights = fusion_weights(:).';
        if numel(fusion_weights) ~= N_sensor
            error('Fuse_GridProb_GA:BadWeights', ...
                'fusion_weights 长度(%d)与传感器数量(%d)不一致。', numel(fusion_weights), N_sensor);
        end
        s = sum(fusion_weights);
        if s <= 0
            fusion_weights = ones(1, N_sensor) / N_sensor;
        else
            fusion_weights = fusion_weights / s;
        end
    end

    base = sensor_inf(1).GridProb(:);
    M = numel(base);
    log_fused = zeros(M, 1);
    for i = 1:N_sensor
        p_i = sensor_inf(i).GridProb(:);
        if numel(p_i) ~= M
            error('Fuse_GridProb_GA:SizeMismatch', ...
                '第 %d 个传感器 GridProb 长度与第1个传感器不一致。', i);
        end
        p_i = min(max(p_i, 1e-12), 1); % 防止 log(0)
        log_fused = log_fused + fusion_weights(i) * log(p_i);
    end
    GridProb_fused = exp(log_fused);
    GridProb_fused = min(max(GridProb_fused, 0), 1);
end
