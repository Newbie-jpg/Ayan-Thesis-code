function idx = waypoint_to_heading_action(p, wp, sensor_params)
% WAYPOINT_TO_HEADING_ACTION 在离散偏航集合 C 中选最接近指向航路点的航向（XY 平面）

    idx = find(sensor_params.C == 0);
    if isempty(idx)
        idx = 1;
    end
    if nargin < 3 || isempty(sensor_params) || ~isfield(sensor_params, 'C')
        return;
    end

    Cdeg = sensor_params.C(:).';
    dx = wp(1) - p(1);
    dy = wp(2) - p(2);
    if abs(dx) < 1e-9 && abs(dy) < 1e-9
        return;
    end

    ang_des = atan2(dy, dx) * 180 / pi; % [-180,180]

    if exist('wrapTo180', 'file') == 2
        diff_ = wrapTo180(Cdeg - ang_des);
    else
        diff_ = mod(Cdeg - ang_des + 180, 360) - 180;
    end

    [~, k] = min(abs(diff_));
    idx = k;
end
