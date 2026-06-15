function data = simulateSensors(groundTruth, sensorParams, sensorList)
% SIMULATESENSORS  Generate simulated sensor measurements from an
%   arbitrary ground-truth trajectory.
%
%   data = simulateSensors(groundTruth, sensorParams, sensorList)
%
%   Inputs:
%     groundTruth - struct with .time, .pos [Nx3], .vel [Nx3], .quat [Nx4],
%                   and optionally .accel [Nx3], .angVel [Nx3]
%                   (if accel/angVel not provided, they are computed by
%                    numerical differentiation)
%     sensorParams - struct from configs/sensor_params.yaml
%     sensorList   - cell array of sensor names to simulate, e.g.
%                    {'imu','gps','lidar_odometry','wheel_encoder'}
%                    (default: {'imu','gps'})
%
%   Output:
%     data - struct with .time and one field per requested sensor

    if nargin < 3
        sensorList = {'imu', 'gps'};
    end

    t = groundTruth.time;
    dt = median(diff(t));
    n = numel(t);

    %% ── Derive accel / angular velocity if missing ──────────────────────
    if ~isfield(groundTruth, 'vel')
        groundTruth.vel = gradient2(groundTruth.pos, dt);
    end
    if ~isfield(groundTruth, 'accel')
        groundTruth.accel = gradient2(groundTruth.vel, dt);
    end
    if ~isfield(groundTruth, 'angVel')
        eul = quat2eul(groundTruth.quat, 'ZYX');
        groundTruth.angVel = gradient2(eul, dt);
    end

    data.time = t;

    %% ── IMU ──────────────────────────────────────────────────────────────
    if any(strcmpi(sensorList, 'imu'))
        imu = imuModel(sensorParams);
        gravityNED = [0 0 9.81];
        specificForce = groundTruth.accel + gravityNED;
        quatObj = quaternion(groundTruth.quat);
        [accelMeas, gyroMeas] = imu(specificForce, groundTruth.angVel, quatObj);
        data.imu.time  = t;
        data.imu.accel = accelMeas;
        data.imu.gyro  = gyroMeas;
    end

    %% ── GPS ──────────────────────────────────────────────────────────────
    if any(strcmpi(sensorList, 'gps'))
        gps = gpsModel(sensorParams);
        gpsRate = sensorParams.gps.sample_rate;
        decim = max(1, round(1/dt / gpsRate));
        idx = 1:decim:n;
        [lla, gpsVel] = gps(groundTruth.pos(idx,:), groundTruth.vel(idx,:));

        refLoc = sensorParams.gps.reference_location;
        R_earth = 6378137.0;
        north = (lla(:,1) - refLoc(1)) * pi/180 * R_earth;
        east  = (lla(:,2) - refLoc(2)) * pi/180 * R_earth .* cos(refLoc(1)*pi/180);
        down  = -(lla(:,3) - refLoc(3));

        data.gps.time = t(idx);
        data.gps.pos  = [north, east, down];
        data.gps.vel  = gpsVel;
    end

    %% ── LiDAR odometry ───────────────────────────────────────────────────
    if any(strcmpi(sensorList, 'lidar_odometry'))
        lp = sensorParams.lidar_odometry;
        decim = max(1, round(1/dt / lp.sample_rate));
        idx = 1:decim:n;
        data.lidar_odometry = buildOdometry(groundTruth, t, idx, lp);
    end

    %% ── Visual odometry ──────────────────────────────────────────────────
    if any(strcmpi(sensorList, 'visual_odometry'))
        vp = sensorParams.visual_odometry;
        decim = max(1, round(1/dt / vp.sample_rate));
        idx = 1:decim:n;
        data.visual_odometry = buildOdometry(groundTruth, t, idx, vp);
    end

    %% ── Wheel encoder ────────────────────────────────────────────────────
    if any(strcmpi(sensorList, 'wheel_encoder'))
        ep = sensorParams.wheel_encoder;
        decim = max(1, round(1/dt / ep.sample_rate));
        idx = 1:decim:n;
        speed = sqrt(sum(groundTruth.vel(idx,:).^2, 2));
        data.wheel_encoder.time = t(idx);
        data.wheel_encoder.velocity = speed + ep.velocity_std * randn(size(speed));
        data.wheel_encoder.headingRate = groundTruth.angVel(idx,3) + ep.heading_rate_std * randn(numel(idx),1);
    end

    fprintf('  Sensors simulated: %s\n', strjoin(sensorList, ', '));
end


%% ── Helper: numerical gradient over time ────────────────────────────────
function g = gradient2(x, dt)
    g = gradient(x, dt);
end


%% ── Helper: relative odometry stream ─────────────────────────────────────
function odom = buildOdometry(groundTruth, t, idx, params)
    odom.time = t(idx(2:end));
    n = numel(idx) - 1;
    odom.relPos  = zeros(n, 3);
    odom.relQuat = zeros(n, 4);
    for k = 1:n
        i0 = idx(k); i1 = idx(k+1);
        [relPos, relQuat, ~] = lidarOdometryModel( ...
            groundTruth.pos(i0,:),  groundTruth.pos(i1,:), ...
            groundTruth.quat(i0,:), groundTruth.quat(i1,:), params);
        odom.relPos(k,:)  = relPos;
        odom.relQuat(k,:) = relQuat;
    end
end
