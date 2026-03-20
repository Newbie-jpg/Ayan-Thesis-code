%{
    GM-PHD 强度函数之间的 CS 散度解析计算
    使用用户给出的高斯混合解析式：
        D_cs(f,g) = 0.5 * <f,f> + 0.5 * <g,g> - <f,g>
    其中每个内积项都由高斯混合两两卷积的解析结果给出。

    为了与当前控制目标中的位置不确定性度量保持一致，这里在位置子空间
    [x, y, z] = [1, 3, 5] 上计算散度。
%}
function D_cs = gmphd_cs_divergence(state_f, state_g)
    if nargin < 2 || isempty(state_f) || isempty(state_g)
        D_cs = 0;
        return;
    end

    [w_f, m_f, P_f, J_f] = unpack_gmphd_state(state_f);
    [w_g, m_g, P_g, J_g] = unpack_gmphd_state(state_g);

    if J_f == 0 && J_g == 0
        D_cs = 0;
        return;
    end

    ff = gm_overlap_sum(w_f, m_f, P_f, J_f, w_f, m_f, P_f, J_f);
    gg = gm_overlap_sum(w_g, m_g, P_g, J_g, w_g, m_g, P_g, J_g);
    fg = gm_overlap_sum(w_f, m_f, P_f, J_f, w_g, m_g, P_g, J_g);

    D_cs = max(real(0.5 * ff + 0.5 * gg - fg), 0);
end

function [w, m, P, J] = unpack_gmphd_state(state)
    if isempty(state) || numel(state) < 4 || isempty(state{4,1}) || state{4,1} == 0
        w = zeros(1, 0);
        m = zeros(0, 0);
        P = cell(1, 0);
        J = 0;
        return;
    end

    J = state{4,1};
    w = state{1,1};
    m = state{2,1};
    P_raw = state{3,1};
    pos_idx = [1 3 5];
    P = cell(1, J);
    for j = 1:J
        P_full = P_raw(:, size(state{2,1}, 1)*(j-1)+1 : size(state{2,1}, 1)*j);
        P{j} = P_full(pos_idx, pos_idx);
    end
    m = m(pos_idx, :);
end

function total = gm_overlap_sum(w_a, m_a, P_a, J_a, w_b, m_b, P_b, J_b)
    total = 0;
    if J_a == 0 || J_b == 0
        return;
    end

    for i = 1:J_a
        for j = 1:J_b
            S_ij = P_a{i} + P_b{j};
            total = total + w_a(i) * w_b(j) * gaussian_overlap_density(m_a(:, i), m_b(:, j), S_ij);
        end
    end
end

function val = gaussian_overlap_density(mu_a, mu_b, Sigma)
    dim = length(mu_a);
    d = mu_a - mu_b;
    Sigma = regularize_covariance(Sigma);
    Sigma = force_positive_definite(Sigma);
    [R, p] = chol(Sigma);
    if p ~= 0
        % 兜底：极端数值情况下，直接返回极小重叠密度，避免中断主流程
        val = realmin;
        return;
    end

    y = R' \ d;
    quad = y' * y;
    log_det = 2 * sum(log(diag(R)));
    log_val = -0.5 * (dim * log(2 * pi) + log_det + quad);
    val = exp(log_val);
end

function P_reg = regularize_covariance(P)
    P_reg = (P + P') / 2;
    dim = size(P_reg, 1);
    scale = max(trace(P_reg) / max(dim, 1), 1);
    eps_reg = 1e-9 * scale;
    P_reg = P_reg + eps_reg * eye(dim);
end

function P_pd = force_positive_definite(P)
    P_sym = (P + P') / 2;
    [V, D] = eig(P_sym);
    evals = real(diag(D));
    dim = size(P_sym, 1);
    scale = max(trace(P_sym) / max(dim, 1), 1);
    min_eval = 1e-8 * scale;
    evals(evals < min_eval) = min_eval;
    P_pd = V * diag(evals) * V';
    P_pd = (P_pd + P_pd') / 2;
end
