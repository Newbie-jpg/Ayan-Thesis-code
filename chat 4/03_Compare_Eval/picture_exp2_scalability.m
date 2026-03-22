clear; clc; close all;

% 可选：手动指定结果文件；留空则自动读取最新 Exp2_Scalability_*.mat
result_file = '';

this_dir = fileparts(mfilename('fullpath'));
chat4_root = fileparts(this_dir);
data_dir = fullfile(chat4_root, '04_Data');

if isempty(result_file)
    files = dir(fullfile(data_dir, 'Exp2_Scalability_*.mat'));
    if isempty(files)
        error('未找到 Exp2_Scalability_*.mat，请先运行 main_exp2_scalability.m');
    end
    [~, idx] = max([files.datenum]);
    result_file = fullfile(files(idx).folder, files(idx).name);
end

S = load(result_file);
required = {'sensor_counts', 'time_avg_A', 'time_avg_B'};
for i = 1:numel(required)
    if ~isfield(S, required{i})
        error('结果文件缺少字段: %s', required{i});
    end
end

sensor_counts = S.sensor_counts;
time_avg_A = S.time_avg_A;
time_avg_B = S.time_avg_B;

figure('Color', 'w', 'Name', 'Exp2_Scalability_Time');
plot(sensor_counts, time_avg_A, '-o', 'LineWidth', 1.8, 'MarkerSize', 6); hold on;
plot(sensor_counts, time_avg_B, '-s', 'LineWidth', 1.8, 'MarkerSize', 6);
grid on;
xlabel('传感器数量');
ylabel('单步平均决策时间 / s');
legend({'算法A', '算法B'}, 'Location', 'northwest');

fprintf('已加载: %s\n', result_file);
for i = 1:numel(sensor_counts)
    fprintf('Ns=%d: A=%.4f s, B=%.4f s\n', sensor_counts(i), time_avg_A(i), time_avg_B(i));
end
