%判断程序：判断粒子是否在传感器视域内
%输入为传感器的坐标(3*1)，需要判断的粒子状态（6*1的数组），传感器的检测半径
%输出为flag_mon，当其为1代表在防区内，当期为0代表在防区外
function flag_FoV=FoV_judge(location,state,r_detect)
distance=sqrt((location(1,1)-state(1,1))^2+(location(2,1)-state(3,1))^2+(location(3,1)-state(5,1))^2);

% 若检测半径+一定的余量大于观测距离，则标志位置1，否则置0
if r_detect>=distance
    flag_FoV=1;
else
    flag_FoV=0;
end

end