%{
                       跟踪目标运动参数设置

         功能：根据设置仿真出目标所有时刻的运动状态

         输入：跟踪周期N，目标运动模式编号

         输出：1.Xreal_target_time -- 单个目标所有时刻的运动状态  
               2.Xreal_time_target -- 单个时刻所有目标的运动状态   

%}
function [Xreal_target_time,Xreal_time_target]=targetset(N,Target)

n_target=size(Target,2); % 目标数量
Xreal_target_time=cell(n_target,1); % 存放每个目标所有时刻的状态

fun_motion_pattern={@m1_CV,@m2_CA,@m3_CT}; 

%=====模拟真实值======
for i=1:n_target
    Xreal_target_time{i,1}=fun_motion_pattern{Target(9,i)}(Target(:,i),N);
end

%====真实值格式转化，转化为单个时间所有真实目标状态的形式====
Xreal_time_target=cell(N,1);
for i=1:N
    for k=1:n_target
        Xreal_time_target{i,1}(:,k)=Xreal_target_time{k,1}(:,i);
    end
end

end

%*********************************************
%说明:模式1:匀速直线运动CV+扰动
%输入:1.目标的初始状态 2.仿真周期数 
%输出:1.所有成员所有时刻的运动状态，num_member*1的cell数组，其中每个cell为6*N的double数组
function [state]=m1_CV(initial_state,N)

%=====运动模型参数=====
T=1; % 周期
A=[1 T 0 0 0 0;0 1 0 0 0 0;0 0 1 T 0 0;0 0 0 1 0 0;0 0 0 0 1 T;0 0 0 0 0 1]; % 运动模型
Q=diag([4/3,1,1,2,1,0.01]); % 扰动
% Q=diag([1,0.01,1,0.01,1,0.01]);


[dim_state,num_member]=size(initial_state); % 状态维数,成员数量
dim_state=dim_state-3; % 状态维数需要减去开始时间、终止时间、运动模式

member_state=cell(num_member,1); % 所有成员所有时刻的运动状态
for i=1:num_member
    member_state{i,1}=zeros(dim_state,N); % 预先声明内存
end

% 目标的真实轨迹生成
for i=1:num_member
    
    begin_time=initial_state(7,i); % 目标运动开始时间
    end_time=initial_state(8,i);   % 目标运动结束时间
    member_state{i,1}(:,1:begin_time-1)=nan(dim_state,begin_time-1); % 目标出现前时刻赋值
    member_state{i,1}(:,begin_time)=initial_state(1:6,i);            % 出现时刻初始值赋值
    for j=begin_time+1:end_time
        member_state{i,1}(:,j)=A*member_state{i,1}(:,j-1)+sqrtm(Q)*randn(6,1); % 运动仿真
    end
    member_state{i,1}(:,end_time+1:N)=nan(dim_state,N-end_time); % 目标消失后的时刻赋值
    
end

% 当前模式下每次只对一个目标进行仿真
state=member_state{1,1};

end

%*********************************************
%说明:模式2:匀加速运动CA+扰动
%输入:1.所有成员的初始状态 2.总的跟踪时间数
%输出:1.所有成员所有时刻的运动状态，num_member*1的cell数组，其中每个cell为6*N的doubel数组
function [state]=m2_CA(initial_state,N)

%=====运动模型参数=====
V_max=250; % 三个坐标轴方向的最大速度
a=[1,-1,1]; % 三维加速度
T=1;
A_part=[1 T 0.5*T^2;
        0 1 T;
        0 0 1]; % 单维的运动矩阵块
A=blkdiag(A_part,A_part,A_part); % 三维的运动模型
Q=diag([1,0.1,0.1,1,0.1,0.1,1,0.1,0.1]); % 扰动
[dim_state,num_member]=size(initial_state); % 状态维数,成员数量
dim_state=dim_state-3; % 状态维数需要减去开始时间、终止时间、运动模式

% 预先声明内存
member_state=cell(num_member,1); % 所有成员所有时刻的运动状态
for i=1:num_member
    member_state{i,1}=zeros(dim_state,N);%%预先声明内存
end

% 注:由于需要加入加速度这个一维，故比原先的维数要多3维(x,y,z的加速度)
dim_state=dim_state+3;
member_state_exten=cell(num_member,1);%%所有成员所有时刻的运动状态
for i=1:num_member
    member_state_exten{i,1}=zeros(dim_state,N);%%预先声明内存
end

%%目标的真实轨迹生成
for i=1:num_member
    begin_time=initial_state(7,i); % 目标运动开始时间
    end_time=initial_state(8,i);   % 目标运动结束时间
    member_state_exten{i,1}(:,1:begin_time-1)=nan(dim_state,begin_time-1);%%目标出现前时刻赋值
    %%出现时刻初始值赋值
    %%说明：由于外部是6维数据，而内部是9维(算上了加速度),故需要重新进行状态赋值
    %x轴
    member_state_exten{i,1}(1:2,begin_time)=initial_state(1:2,i);
    member_state_exten{i,1}(3,begin_time)=a(1);
    %y轴
    member_state_exten{i,1}(4:5,begin_time)=initial_state(3:4,i);
    member_state_exten{i,1}(6,begin_time)=a(2);
    %z轴
    member_state_exten{i,1}(7:8,begin_time)=initial_state(5:6,i);
    member_state_exten{i,1}(9,begin_time)=a(3);
    %运动仿真
    for j=begin_time+1:end_time
        member_state_exten{i,1}(:,j)=A*member_state_exten{i,1}(:,j-1)+sqrtm(Q)*randn(9,1);
        % 判断速度绝对值是否达到最大，超出设定最大速度则加速度归零
        if abs( member_state_exten{i,1}(2,j) )>V_max
           member_state_exten{i,1}(2,j)=V_max*member_state_exten{i,1}(2,j)/abs(member_state_exten{i,1}(2,j)); % X轴速度
           member_state_exten{i,1}(3,j)=0;%%X轴加速度
        elseif abs(member_state_exten{i,1}(5,j))>V_max
           member_state_exten{i,1}(5,j)=V_max*member_state_exten{i,1}(5,j)/abs(member_state_exten{i,1}(5,j)); % Y轴速度
           member_state_exten{i,1}(6,j)=0;%%Y轴加速度
        elseif abs(member_state_exten{i,1}(8,j))>V_max
           member_state_exten{i,1}(8,j)=V_max*member_state_exten{i,1}(8,j)/abs(member_state_exten{i,1}(8,j)); % Z轴速度
           member_state_exten{i,1}(9,j)=0;%%Z轴加速度
        end
    end
    member_state_exten{i,1}(:,end_time+1:N)=nan(dim_state,N-end_time);%%目标消失后的时刻赋值
end
%格式转换：将9维的状态数据变成6维的状态数据
for i=1:num_member
    for j=1:N
    %x轴
    member_state{i,1}(1:2,j)=member_state_exten{i,1}(1:2,j);
    %y轴
    member_state{i,1}(3:4,j)=member_state_exten{i,1}(4:5,j);
    %z轴
    member_state{i,1}(5:6,j)=member_state_exten{i,1}(7:8,j);
    end
end

% 当前模式下每次只对一个目标进行仿真
state=member_state{1,1};

end


%*********************************************
%说明:模式3:匀速转弯CT+扰动
%输入:1.所有成员的初始状态 2.总的跟踪时间数
%输出:1.所有成员所有时刻的运动状态，num_member*1的cell数组，其中每个cell为6*N的doubel数组
function [state]=m3_CT(initial_state,N)
%运动模型参数
a=[1,1,1]'; % 三维加速度
T=1;        % 仿真时间
[dim_state,num_member]=size(initial_state); % 状态维数,成员数量
dim_state=dim_state-3;    % 状态维数需要减去开始时间、终止时间、运动模式
Q=diag([1,1,1]); % 三个维度加速度的噪声干扰
%%预先声明内存
member_state=cell(num_member,1);%%所有成员所有时刻的运动状态
for i=1:num_member
    member_state{i,1}=zeros(dim_state,N);%%预先声明内存
end
%%注:由于需要加入加速度这个一维，故比原先的维数要多3维(x,y,z的加速度)
dim_state=dim_state+3;
member_state_exten=cell(num_member,1);%%所有成员所有时刻的运动状态
for i=1:num_member
    member_state_exten{i,1}=zeros(dim_state,N);%%预先声明内存
end
%%目标的真实轨迹生成
for i=1:num_member
    begin_time=initial_state(7,i); % 目标运动开始时间
    end_time=initial_state(8,i);   % 目标运动结束时间
    %%角速度的确定
    a=a+sqrtm(Q)*randn(3,1);%%加速度添加杂波干扰
    omiga=sqrt(a(1)^2+a(2)^2+a(3)^2)/sqrt(initial_state(2,i)^2+initial_state(4,i)^2+...
        initial_state(6,i)^2);%%角速度
    A_part=[1 sin(omiga*T)/omiga (1-cos(omiga*T))/(omiga^2);
        0 cos(omiga*T) sin(omiga*T)/omiga;
        0 -omiga*sin(omiga*T) cos(omiga*T)];%%单维的运动矩阵块
    A=blkdiag(A_part,A_part,A_part);%%三维的运动模型
    member_state_exten{i,1}(:,1:begin_time-1)=nan(dim_state,begin_time-1);%%目标出现前时刻赋值
    %%出现时刻初始值赋值
    %%说明：由于外部是6维数据，而内部是9维(算上了加速度),故需要重新进行状态赋值
    %x轴
    member_state_exten{i,1}(1:2,begin_time)=initial_state(1:2,i);
    member_state_exten{i,1}(3,begin_time)=a(1);
    %y轴
    member_state_exten{i,1}(4:5,begin_time)=initial_state(3:4,i);
    member_state_exten{i,1}(6,begin_time)=a(2);
    %z轴
    member_state_exten{i,1}(7:8,begin_time)=initial_state(5:6,i);
    member_state_exten{i,1}(9,begin_time)=a(3);
    %%运动仿真
    for j=begin_time+1:end_time
        member_state_exten{i,1}(:,j)=A*member_state_exten{i,1}(:,j-1);
    end
    member_state_exten{i,1}(:,end_time+1:N)=nan(dim_state,N-end_time);%%目标消失后的时刻赋值
end
%格式转换：将9维的状态数据变成6维的状态数据
for i=1:num_member
    for j=1:N
    %x轴
    member_state{i,1}(1:2,j)=member_state_exten{i,1}(1:2,j);
    %y轴
    member_state{i,1}(3:4,j)=member_state_exten{i,1}(4:5,j);
    %z轴
    member_state{i,1}(5:6,j)=member_state_exten{i,1}(7:8,j);
    end
end

% 当前模式下每次只对一个目标进行仿真
state=member_state{1,1};

end



