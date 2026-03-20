%{
    PHD L 步纯预测（无新生目标）
    对应 chapter3.tex 中 GM-PHD 预测：存活目标强度 v_{S,k|k-1}，此处做 L 步递推
    输入：
        state_k:  k 时刻后验强度 (4*1 cell: 权重, 均值6*n, 协方差6*6*n, 分量数)
        L:        预测步数
        F:        状态转移矩阵 6*6 (CV 模型)
        Q:        过程噪声协方差 6*6
    输出：
        state_pred:  k+L|k 先验强度 (4*1 cell)
%}
function state_pred = PHD_predict_L_steps(state_k, L, F, Q)
    J = state_k{4,1};
    if J == 0
        state_pred = state_k;
        return;
    end
    w = state_k{1,1};
    m = state_k{2,1};
    P = state_k{3,1};
    % L 步预测：m_{k+L|k} = F^L * m_k, P = F*P*F' + Q (递推 L 次)
    F_L = F^L;
    % P_{k+L|k} = F^L * P_k * (F^L)' + sum_{i=0}^{L-1} F^i * Q * (F^i)'
    Q_cum = Q;
    F_pow = F;
    for ell = 2:L
        Q_cum = F_pow * Q * F_pow' + Q_cum;
        F_pow = F_pow * F;
    end
    w_pre = w;
    m_pre = zeros(6, J);
    P_pre = zeros(6, 6*J);
    for j = 1:J
        m_pre(:, j) = F_L * m(:, j);
        P_pre(:, 6*(j-1)+1 : 6*j) = F_L * P(:, 6*(j-1)+1 : 6*j) * F_L' + Q_cum;
    end
    state_pred = cell(4,1);
    state_pred{1,1} = w_pre;
    state_pred{2,1} = m_pre;
    state_pred{3,1} = P_pre;
    state_pred{4,1} = J;
end
