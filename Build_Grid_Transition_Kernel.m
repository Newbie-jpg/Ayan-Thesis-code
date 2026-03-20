%{
    基于最大运动能力构建二维离散高斯转移核 T
    公式：
        sigma_grid = Vmax * dt / (3 * L0)
        T(dm,dn) = exp(-(dm^2 + dn^2)/(2*sigma_grid^2)) / Z
%}
function [T_kernel, sigma_grid, kernel_radius] = Build_Grid_Transition_Kernel(vmax, dt, L0, kernel_radius)
    if nargin < 4 || isempty(kernel_radius)
        kernel_radius = [];
    end

    if L0 <= 0 || dt <= 0 || vmax < 0
        error('Build_Grid_Transition_Kernel:InvalidInput', 'vmax>=0, dt>0, L0>0 required.');
    end

    sigma_grid = vmax * dt / (3 * L0);

    % 极端情形：sigma 近似为 0 时，退化为单位脉冲核
    if sigma_grid < 1e-9
        T_kernel = 1;
        kernel_radius = 0;
        return;
    end

    % 默认按 3sigma 截断离散支持域
    if isempty(kernel_radius)
        kernel_radius = max(1, ceil(3 * sigma_grid));
    else
        kernel_radius = max(0, round(kernel_radius));
    end

    d = -kernel_radius:kernel_radius;
    [DM, DN] = meshgrid(d, d);
    T_kernel = exp(-(DM.^2 + DN.^2) ./ (2 * sigma_grid^2));
    Z = sum(T_kernel(:));
    T_kernel = T_kernel / Z;
end
