function persistent_series = persistent_track_extract_for_ospa(raw_series, opt)
% PERSISTENT_TRACK_EXTRACT_FOR_OSPA
% 仅用于 OSPA 评估层的轻量持久化轨迹提取：
% - 连续命中 confirm_hits 次后确认
% - 连续丢失 delete_misses 次后删除
% - 删除前保留上一时刻状态，使目标消失时 OSPA 出现更真实的峰值

    if nargin < 2 || isempty(opt)
        opt = struct();
    end
    if ~isfield(opt, 'confirm_hits') || isempty(opt.confirm_hits), opt.confirm_hits = 2; end
    if ~isfield(opt, 'delete_misses') || isempty(opt.delete_misses), opt.delete_misses = 3; end
    if ~isfield(opt, 'match_distance') || isempty(opt.match_distance), opt.match_distance = 180; end
    if ~isfield(opt, 'pos_idx') || isempty(opt.pos_idx), opt.pos_idx = [1 3 5]; end

    T = numel(raw_series);
    persistent_series = cell(T, 1);
    tracks = struct('state', {}, 'hits', {}, 'misses', {}, 'confirmed', {});

    for t = 1:T
        Z = raw_series{t};
        if isempty(Z)
            Z = zeros(6, 0);
        end
        Z(:, any(isnan(Z), 1)) = [];

        nTr = numel(tracks);
        nZ = size(Z, 2);
        matched_tracks = false(1, nTr);
        matched_meas = false(1, nZ);

        % 贪心最近邻匹配（目标数较少时足够稳定）
        if nTr > 0 && nZ > 0
            D = inf(nTr, nZ);
            for i = 1:nTr
                p_i = tracks(i).state(opt.pos_idx, 1);
                for j = 1:nZ
                    p_j = Z(opt.pos_idx, j);
                    D(i, j) = norm(p_i - p_j);
                end
            end

            while true
                [dmin, idx] = min(D(:));
                if isempty(dmin) || ~isfinite(dmin) || dmin > opt.match_distance
                    break;
                end
                [ii, jj] = ind2sub(size(D), idx);
                matched_tracks(ii) = true;
                matched_meas(jj) = true;

                tracks(ii).state = Z(:, jj);
                tracks(ii).hits = tracks(ii).hits + 1;
                tracks(ii).misses = 0;
                if tracks(ii).hits >= opt.confirm_hits
                    tracks(ii).confirmed = true;
                end

                D(ii, :) = inf;
                D(:, jj) = inf;
            end
        end

        % 未匹配轨迹：累积丢失，但保留上一时刻状态
        for i = 1:nTr
            if ~matched_tracks(i)
                tracks(i).misses = tracks(i).misses + 1;
            end
        end

        % 未匹配量测：新建候选轨迹
        for j = 1:nZ
            if ~matched_meas(j)
                tr = struct();
                tr.state = Z(:, j);
                tr.hits = 1;
                tr.misses = 0;
                tr.confirmed = (tr.hits >= opt.confirm_hits);
                tracks(end+1) = tr; %#ok<AGROW>
            end
        end

        % 删除丢失过久的轨迹
        keep_mask = true(1, numel(tracks));
        for i = 1:numel(tracks)
            if tracks(i).misses > opt.delete_misses
                keep_mask(i) = false;
            end
        end
        tracks = tracks(keep_mask);

        % 输出当前确认轨迹
        Xt = zeros(6, 0);
        for i = 1:numel(tracks)
            if tracks(i).confirmed
                Xt(:, end+1) = tracks(i).state; %#ok<AGROW>
            end
        end
        persistent_series{t} = Xt;
    end
end
