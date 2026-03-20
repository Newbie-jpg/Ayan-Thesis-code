%{
    根据传感器与目标之间的距离计算观测噪声协方差矩阵 R_k (chapter3.tex 式3.34)
    σ_r = r_0 + η_r * ||s_k - t_k||
    σ_θ = θ_0 + η_θ * ||s_k - t_k||
    输入：
        d:            传感器与目标之间的距离 (标量，米)
        R_params:     结构体，含 r_0, theta_0, eta_r, eta_theta
    输出：
        R:            3x3 观测噪声协方差矩阵 diag(σ_r^2, σ_θ^2, σ_θ^2)
%}
function R = compute_R_from_distance(d, R_params)
    sigma_r = R_params.r_0 + R_params.eta_r * d;
    sigma_theta = R_params.theta_0 + R_params.eta_theta * d;  % 度
    R = diag([sigma_r^2, sigma_theta^2, sigma_theta^2]);
end
