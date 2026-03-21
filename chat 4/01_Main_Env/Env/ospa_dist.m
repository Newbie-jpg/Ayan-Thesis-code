%**************************************************************
%
%    fun:ospa_dist ：计算OSPA距离
%
%    涉及函数：Hungarian.m
%==============================================================
function [dist,varargout]= ospa_dist(X,Y,c,p)

%Compute Schumacher distance between two finite sets X and Y
%as described in the reference
%[1] D. Schuhmacher, B.-T. Vo, and B.-N. Vo, "A consistent metric for performance evaluation in multi-object 
%filtering," IEEE Trans. Signal Processing, Vol. 56, No. 8 Part 1, pp. 3447?3457, 2008.
%
%输入 : X,Y-   matrices of column vectors  矩阵列向量
%        c  -   cut-off parameter (see [1] for details) 截断参数
%        p  -   p-parameter for the metric (see [1] for details)
%输出: scalar distance between X and Y   X Y之间的标量距离
%Note: the Euclidean 2-norm is used as the "base" distance on the region
%

if nargout ~=1 && nargout ~=3
   error('Incorrect number of outputs'); 
end

if isempty(X) && isempty(Y)   %如果X,Y同时为空时，dist才为0.
    dist = 0;

    if nargout == 3
        varargout(1)= {0};
        varargout(2)= {0};
    end
    
    return;
end

if isempty(X) || isempty(Y)  %X,Y任一集合为空时，dist才为c.
    dist = c;
    if nargout == 3
        varargout(1)= {0};
        varargout(2)= {c};
    end
    
    return;
end


%计算 输入点模式的size
n = size(X,2);
m = size(Y,2);

%计算配对儿权重 - fast method with vectorization
XX= repmat(X,[1 m]);
YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
D = reshape(sqrt(sum((XX-YY).^2)),[n m]);
D = min(c,D).^p;

% %计算配对儿权重 - slow method with for loop
% D= zeros(n,m);
% for j=1:m
%     D(:,j)= sqrt(sum( ( repmat(Y(:,j),[1 n])- X ).^2 )');
% end
% D= min(c,D).^p;

%Compute optimal assignment and cost using the Hungarian algorithm
[assignment,cost]= Hungarian(D);

%计算距离
dist= ( 1/max(m,n)*( c^p*abs(m-n)+ cost ) ) ^(1/p);

%输出 components if called for in varargout
if nargout == 3
    varargout(1)= {(1/max(m,n)*cost)^(1/p)};
    varargout(2)= {(1/max(m,n)*c^p*abs(m-n))^(1/p)};
end

end