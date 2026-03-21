function pts = compute_noisy_oracle_targets(Xreal_t, og, t)
% COMPUTE_NOISY_ORACLE_TARGETS 由真值位置生成带噪“伪目标点”（仅用于控制层，不注入滤波器）
%
% 输入:
%   Xreal_t: 6×N_tar，状态 [x vx y vy z vz]'（与 targetset 一致）
%   og:      结构体，字段见下
%   t:       当前时刻（用于可复现的 rng(t)）
%
% 输出:
%   pts:     3×N_tar，每列为带噪位置 [x;y;z]；若无有效目标则为 0×0 空矩阵
%
% og 字段:
%   enable, sigma_xy, sigma_z, clip_max (可选), rng_seed_base (与 sim_cfg.rng_seed 对齐则 A/B 每步伪目标一致)

    if nargin < 3
        t = 0;
    end
    pts = zeros(3, 0);
    if nargin < 2 || isempty(og) || ~isfield(og, 'enable') || ~og.enable
        return;
    end

    if isempty(Xreal_t) || size(Xreal_t, 1) < 5
        return;
    end

    Xt = Xreal_t;
    Xt(:, isnan(Xt(1, :))) = [];
    if isempty(Xt)
        return;
    end

    pos = [Xt(1, :); Xt(3, :); Xt(5, :)]; % x,y,z
    n = size(pos, 2);

    sigma_xy = 100;
    if isfield(og, 'sigma_xy') && ~isempty(og.sigma_xy)
        sigma_xy = og.sigma_xy;
    end
    sigma_z = 0;
    if isfield(og, 'sigma_z') && ~isempty(og.sigma_z)
        sigma_z = og.sigma_z;
    end

    rng_seed_base = 0;
    if isfield(og, 'rng_seed_base') && ~isempty(og.rng_seed_base)
        rng_seed_base = og.rng_seed_base;
    end

    s = rng;
    rng(rng_seed_base + 7919 * double(t)); %#ok<RAND>
    noise = [randn(1, n) * sigma_xy; randn(1, n) * sigma_xy; randn(1, n) * sigma_z];
    rng(s); %#ok<RAND>

    pts = pos + noise;

    if isfield(og, 'clip_max') && ~isempty(og.clip_max) && isfinite(og.clip_max)
        err = pts - pos;
        d = sqrt(sum(err .^ 2, 1));
        mask = d > og.clip_max & d > 0;
        if any(mask)
            pts(:, mask) = pos(:, mask) + bsxfun(@times, err(:, mask), og.clip_max ./ d(mask));
        end
    end
end
