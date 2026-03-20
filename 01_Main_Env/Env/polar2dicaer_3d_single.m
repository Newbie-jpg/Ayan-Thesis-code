%{
    单步极坐标转笛卡尔坐标函数
%}
function [Z_dicaer_t] = polar2dicaer_3d_single(Z_polar_t, location_x, location_y, location_z)

    Z_dicaer_t = [];
    if isempty(Z_polar_t)
        return;
    end
    
    n_ob = size(Z_polar_t, 2);
    for i = 1:n_ob
        x = abs(Z_polar_t(1,i)*cos(deg2rad(Z_polar_t(3,i)))) * cos(deg2rad(Z_polar_t(2,i)));
        y = abs(Z_polar_t(1,i)*cos(deg2rad(Z_polar_t(3,i)))) * sin(deg2rad(Z_polar_t(2,i)));
        z = Z_polar_t(1,i)*sin(deg2rad(Z_polar_t(3,i)));
        
        Z_dicaer_t(1,i) = x + location_x;
        Z_dicaer_t(2,i) = y + location_y;
        Z_dicaer_t(3,i) = z + location_z;
    end
end