%{
    单步极坐标观测生成函数 (适配移动传感器)
    chapter3.tex：观测噪声协方差 R_k 与传感器-目标距离相关，σ_r=r_0+η_r*d, σ_θ=θ_0+η_θ*d
    输入：
        Xre_target_t -- 当前单一时刻的真实目标状态 (6*n_target)
        location     -- 传感器当前时刻的坐标 (3*1)
        r_detect, Pd, Zr -- 传感器属性参数
        R_params     -- 结构体 {r_0, theta_0, eta_r, eta_theta}，用于按距离计算 R
    输出：
        Z_polar_t    -- 当前时刻局部极坐标观测矩阵 (3*cnt)
%}
function [Z_polar_t] = observe_FoV_3d_single(Xre_target_t, location, r_detect, Pd, Zr, R_params)

    % 保持传入检测概率 Pd，不做硬化到 1。
    % 对应 GM-PHD 更新中的 (1-p_D) 未检项。
    
    x_radar = location(1); y_radar = location(2); z_radar = location(3);
    Z_polar_t = [];
    cnt_watch = 0;
    
    % 如果当前时刻有真实目标存在
    if ~isempty(Xre_target_t)
        N_target = size(Xre_target_t, 2);
        for i = 1:N_target 
            r = rand;
            d = sqrt((Xre_target_t(1,i)-x_radar)^2 + (Xre_target_t(3,i)-y_radar)^2 + (Xre_target_t(5,i)-z_radar)^2);
            
            % 真实目标被观测判断
            if d <= r_detect && Pd > r 
                cnt_watch = cnt_watch + 1;
                % 计算非线性极坐标并加入噪声（R与距离d相关，chapter3.tex 式3.34）
                Z_polar_t(:,cnt_watch) = compute_R_theta(Xre_target_t(1,i), Xre_target_t(3,i), Xre_target_t(5,i), x_radar, y_radar, z_radar);
                R_k = compute_R_from_distance(d, R_params);
                Z_polar_t(:,cnt_watch) = Z_polar_t(:,cnt_watch) + sqrtm(R_k)*randn(3,1);
            end
        end
    end
    
    % 边界值处理
    if ~isempty(Z_polar_t)
        Z_polar_t(2,(Z_polar_t(2,:)<0)) = Z_polar_t(2,(Z_polar_t(2,:)<0)) + 360;
        Z_polar_t(2,(Z_polar_t(2,:)>360)) = Z_polar_t(2,(Z_polar_t(2,:)>360)) - 360;
        Z_polar_t(3,(Z_polar_t(3,:)>=90)) = 89.99;
        Z_polar_t(3,(Z_polar_t(3,:)<=-90)) = -89.99;
    end
    
    % 生成杂波点
    for i = cnt_watch+1 : Zr+cnt_watch
        angle1 = rand*2*pi;             
        angle2 = acos(rand*2-1);        
        r_d = r_detect*power(rand,1/3); 
        
        loc_noise(1) = x_radar + r_d*cos(angle1)*sin(angle2);
        loc_noise(2) = y_radar + r_d*sin(angle1)*sin(angle2);
        loc_noise(3) = z_radar + r_d*cos(angle2);
        
        Z_polar_t(:,i) = compute_R_theta(loc_noise(1), loc_noise(2), loc_noise(3), x_radar, y_radar, z_radar);
    end
end

% 附带原有的角度计算子函数 (直接拷入此文件末尾)
function location_3d=compute_R_theta(x,y,z,x_radar,y_radar,z_radar)
    R_d=sqrt((x-x_radar)^2+(y-y_radar)^2+(z-z_radar)^2);
    
    if x-x_radar==0
        if y-y_radar>0%%y轴坐标为正，则为90°
            theta_head=90;
        elseif y-y_radar<0%%y轴坐标为付，则为279°
            theta_head=270;
        elseif y-y_radar==0
            theta_head=nan;%%在 x y 平面的原点
        end
    else
        theta_sus_head=rad2deg(atan((y-y_radar)/(x-x_radar)));
        if x-x_radar>=0&&y-y_radar>=0 %%第1象限
            theta_head=theta_sus_head;
        elseif x-x_radar<0&&y-y_radar>=0 %%第2象限
            theta_head=theta_sus_head+180;
        elseif x-x_radar<0&&y-y_radar<0 %%第3象限
            theta_head=theta_sus_head+180;
        elseif x-x_radar>=0&&y-y_radar<0 %%第4象限
            theta_head=theta_sus_head+360;
        end
    end
    
    d_xy=sqrt((x-x_radar)^2+(y-y_radar)^2);
    if d_xy==0 %%目标与雷达xy平面投影点重合
        if z-z_radar>0
            theta_tilt=90;
        elseif z-z_radar<0
            theta_tilt=-90;
        elseif z-z_radar==0 %%目标和雷达重合
            theta_tilt=nan;
        end
    else
        theta_tilt=rad2deg(atan((z-z_radar)/(d_xy)));
    end
    
    location_3d=[R_d;theta_head;theta_tilt];

end