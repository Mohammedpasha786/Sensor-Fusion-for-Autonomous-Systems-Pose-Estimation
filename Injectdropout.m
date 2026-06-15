function data = injectDropout(data, sensorName, varargin)
% INJECTDROPOUT  Simulate sensor dropout, outages, or spike corruption
%   on a sensor data stream for robustness testing.
%
%   data = injectDropout(data, sensorName, ...)
%
%   Inputs:
%     data       - sensor data struct (output of loadEuRoC/simulateTrajectory/etc.)
%     sensorName - name of sensor field to corrupt, e.g. 'gps', 'imu',
%                  'lidar_odometry', 'visual_odometry', 'wheel_encoder'
%
%   Name-Value options:
%     'type'      - 'outage' (default) | 'spike' | 'noise_increase' | 'bias'
%     'startTime' - start time of the dropout window (s)  [default: 1/3 of duration]
%     'duration'  - duration of the dropout window (s)    [default: 10]
%     'magnitude' - severity multiplier for 'spike'/'noise_increase'/'bias' [default: 5]
%
%   Output:
%     data       - modified data struct with corruption applied
%
%   Examples:
%     data = injectDropout(data, 'gps', 'startTime', 60, 'duration', 30);
%     data = injectDropout(data, 'imu', 'type', 'spike', 'magnitude', 5);
%     data = injectDropout(data, 'lidar_odometry', 'type', 'noise_increase', 'magnitude', 10);

    p = inputParser;
    addParameter(p, 'type', 'outage', @ischar);
    addParameter(p, 'startTime', [], @isnumeric);
    addParameter(p, 'duration', 10, @isnumeric);
    addParameter(p, 'magnitude', 5, @isnumeric);
    parse(p, varargin{:});
    opts = p.Results;

    assert(isfield(data, sensorName), 'Sensor "%s" not found in data struct.', sensorName);
    sensorData = data.(sensorName);
    assert(isfield(sensorData, 'time'), 'Sensor "%s" has no .time field.', sensorName);

    t = sensorData.time;

    if isempty(opts.startTime)
        opts.startTime = t(1) + (t(end) - t(1)) / 3;
    end
    endTime = opts.startTime + opts.duration;

    mask = t >= opts.startTime & t <= endTime;
    nAffected = sum(mask);

    switch lower(opts.type)

        case 'outage'
            % Remove samples entirely within the dropout window
            data.(sensorName) = removeFields(sensorData, mask);
            fprintf('  [Dropout] %s OUTAGE: %d samples removed (t=%.1f–%.1f s)\n', ...
                    sensorName, nAffected, opts.startTime, endTime);

        case 'spike'
            % Add large impulsive noise to all numeric measurement fields
            fields = setdiff(fieldnames(sensorData), {'time'});
            for f = 1:numel(fields)
                fname = fields{f};
                val = sensorData.(fname);
                if isnumeric(val) && size(val,1) == numel(t)
                    spikeNoise = opts.magnitude * randn(nAffected, size(val,2));
                    val(mask, :) = val(mask, :) + spikeNoise;
                    sensorData.(fname) = val;
                end
            end
            data.(sensorName) = sensorData;
            fprintf('  [Dropout] %s SPIKE injected: %d samples, magnitude=%.1fx (t=%.1f–%.1f s)\n', ...
                    sensorName, nAffected, opts.magnitude, opts.startTime, endTime);

        case 'noise_increase'
            % Multiply noise on all numeric measurement fields by magnitude
            fields = setdiff(fieldnames(sensorData), {'time'});
            for f = 1:numel(fields)
                fname = fields{f};
                val = sensorData.(fname);
                if isnumeric(val) && size(val,1) == numel(t)
                    extraNoise = (opts.magnitude - 1) * std(val(:), 'omitnan') * randn(nAffected, size(val,2));
                    val(mask, :) = val(mask, :) + extraNoise;
                    sensorData.(fname) = val;
                end
            end
            data.(sensorName) = sensorData;
            fprintf('  [Dropout] %s NOISE INCREASE: %dx for %d samples (t=%.1f–%.1f s)\n', ...
                    sensorName, opts.magnitude, nAffected, opts.startTime, endTime);

        case 'bias'
            % Add a constant offset bias to all numeric measurement fields
            fields = setdiff(fieldnames(sensorData), {'time'});
            for f = 1:numel(fields)
                fname = fields{f};
                val = sensorData.(fname);
                if isnumeric(val) && size(val,1) == numel(t)
                    bias = opts.magnitude * ones(1, size(val,2));
                    val(mask, :) = val(mask, :) + bias;
                    sensorData.(fname) = val;
                end
            end
            data.(sensorName) = sensorData;
            fprintf('  [Dropout] %s BIAS injected: +%.2f for %d samples (t=%.1f–%.1f s)\n', ...
                    sensorName, opts.magnitude, nAffected, opts.startTime, endTime);

        otherwise
            error('Unknown dropout type: %s', opts.type);
    end
end


function s = removeFields(s, mask)
% Remove rows indicated by mask from all numeric fields with matching length.
    n = numel(s.time);
    fields = fieldnames(s);
    for f = 1:numel(fields)
        fname = fields{f};
        val = s.(fname);
        if isnumeric(val) && size(val,1) == n
            s.(fname) = val(~mask, :);
        end
    end
end
