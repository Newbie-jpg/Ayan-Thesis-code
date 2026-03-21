function assign = assign_sensors_targets_oracle(sensor_locs, pts)
% ASSIGN_SENSORS_TARGETS_ORACLE 将传感器分配到目标（带噪伪点），避免全部挤向同一目标
%
% 使用“每目标多槽位”的代价矩阵 + assignmentoptimal：每个传感器占一个槽位，
% 每目标最多 ceil(Ns/Nt) 个传感器，使分配在总距离意义下较优且自动分散。
%
% sensor_locs: 3 x Ns
% pts:         3 x Nt（Nt>=1）
% assign:      1 x Ns，第 i 个传感器对应的目标列索引 1..Nt

    assign = ones(1, 0);
    if nargin < 2 || isempty(sensor_locs) || isempty(pts)
        return;
    end

    Ns = size(sensor_locs, 2);
    Nt = size(pts, 2);
    if Nt == 0
        assign = zeros(1, Ns);
        return;
    end

    max_per = max(1, ceil(Ns / Nt));
    ncol = Nt * max_per;
    C = zeros(Ns, ncol);

    for j = 1:Nt
        dcol = vecnorm(sensor_locs - pts(:, j), 2, 1).'; % Ns x 1
        for s = 1:max_per
            C(:, (j - 1) * max_per + s) = dcol;
        end
    end

    [ass, ~] = assignmentoptimal(C);
    assign = zeros(1, Ns);
    for i = 1:Ns
        col = ass(i);
        if col <= 0 || col > ncol
            [~, assign(i)] = min(vecnorm(pts - sensor_locs(:, i), 2, 1));
        else
            assign(i) = floor((double(col) - 1) / double(max_per)) + 1;
        end
    end
end
