function [data, groundTruth] = simulateTrajectory(platform, sensorParams)
% SIMULATETRAJECTORY  Generate a ground-truth trajectory and simulate
%   sensor data using UAV Toolbox / Automated Driving Toolbox scenarios.
%
%   [data, groundTruth] = simulateTrajectory(platform, sensorParams)
%
%   Inputs:
%     platform     - 'quadcopter' or 'ground'
%     sensorParams - struct from configs/sensor_params.yaml
%
%   Outputs:
%     data        - struct with .time, .imu, .gps (and optionally
%                   .lidar_odometry, .visual_odometry, .wheel_encoder)
%     groundTruth - struct with .time, .pos, .quat, .vel
%
%   Trajectory generation:
%     - Quadcopter: waypointTrajectory through a 3D figure-eight pattern
%     - Ground: waypointTrajectory along a 2D loop (constant altitude)

    rng(2024);

    switch lower(platform)
        case 'quadcopter'
            [waypoints, toa, vel, orient] = quadcopterWaypoints();
        case 'ground'
            [waypoints, toa, vel, orient] = groundWaypoints();
        otherwise
            error('Unknown platform: %s (use ''quadcopter'' or ''ground'')', platform);
    end

    %% ── Build waypoint trajectory ────────────────────────────────────────
    traj = waypointTrajectory(waypoints, 'TimeOfArrival', toa, ...
                               'Velocities', vel, ...
                               'Orientation', orient, ...
                               'SampleRate', sensorParams.imu.sample_rate);

    duration = toa(end);
    nSamples = floor(duration * sensorParams.imu.sample_rate);

    fprintf('  Simulating %s trajectory: %.1f s @ %.0f Hz (%d samples)\n', ...
            platform, duration, sensorParams.imu.sample_rate, nSamples);

    %% ── Ground truth arrays ──────────────────────────────────────────────
    gtPos  = zeros(nSamples, 3);
    gtVel  = zeros(nSamples, 3);
    gtAcc  = zeros(nSamples, 3);
    gtAngVel = zeros(nSamples, 3);
    gtQuat = zeros(nSamples, 4);
    gtTime = zeros(nSamples, 1);

    for i = 1:nSamples
        [pos, q, v, a, w] = traj();
        gtPos(i,:)    = pos;
        gtVel(i,:)    = v;
        gtAcc(i,:)    = a;
        gtAngVel(i,:) = w;
        gtQuat(i,:)   = compact(q);
        gtTime(i)     = (i-1) / sensorParams.imu.sample_rate;
        if ~traj.isDone(), continue; else, nSamples = i; break; end
    end
    gtPos  = gtPos(1:nSamples,:);
    gtVel  = gtVel(1:nSamples,:);
    gtAcc  = gtAcc(1:nSamples,:);
    gtAngVel = gtAngVel(1:nSamples,:);
    gtQuat = gtQuat(1:nSamples,:);
    gtTime = gtTime(1:nSamples);

    groundTruth.time = gtTime;
    groundTruth.pos  = gtPos;
    groundTruth.vel  = gtVel;
    groundTruth.quat = gtQuat;

    %% ── Simulate IMU ─────────────────────────────────────────────────────
    imu = imuModel(sensorParams);
    gravityNED = [0 0 9.81];   % gravity in NED (down positive)

    % Specific force = true accel - gravity (sensed by accelerometer)
    specificForce = gtAcc + gravityNED;
    quatObj = quaternion(gtQuat);

    [accelMeas, gyroMeas] = imu(specificForce, gtAngVel, quatObj);

    data.imu.time  = gtTime;
    data.imu.accel = accelMeas;
    data.imu.gyro  = gyroMeas;
    data.time      = gtTime;

    %% ── Simulate GPS ─────────────────────────────────────────────────────
    gps = gpsModel(sensorParams);
    gpsRate = sensorParams.gps.sample_rate;
    gpsDecim = round(sensorParams.imu.sample_rate / gpsRate);
    gpsIdx = 1:gpsDecim:nSamples;

    [lla, gpsVel] = gps(gtPos(gpsIdx,:), gtVel(gpsIdx,:));

    % Convert lla back to local NED for consistency with EKF state
    refLoc = sensorParams.gps.reference_location;
    [gpsNorth, gpsEast, gpsDown] = lla2nedLocal(lla, refLoc);

    data.gps.time = gtTime(gpsIdx);
    data.gps.pos  = [gpsNorth, gpsEast, gpsDown];
    data.gps.vel  = gpsVel;

    %% ── Platform-specific extra sensors ──────────────────────────────────
    if strcmpi(platform, 'quadcopter')
        % Visual odometry
        voParams = sensorParams.visual_odometry;
        voDecim = round(sensorParams.imu.sample_rate / voParams.sample_rate);
        voIdx = 1:voDecim:nSamples;
        data.visual_odometry = computeOdometryStream(gtPos, gtQuat, gtTime, voIdx, voParams);
    else
        % LiDAR odometry + wheel encoder
        lidarParams = sensorParams.lidar_odometry;
        lidarDecim = round(sensorParams.imu.sample_rate / lidarParams.sample_rate);
        lidarIdx = 1:lidarDecim:nSamples;
        data.lidar_odometry = computeOdometryStream(gtPos, gtQuat, gtTime, lidarIdx, lidarParams);

        encParams = sensorParams.wheel_encoder;
        encDecim = round(sensorParams.imu.sample_rate / encParams.sample_rate);
        encIdx = 1:encDecim:nSamples;
        data.wheel_encoder.time = gtTime(encIdx);
        speed = sqrt(sum(gtVel(encIdx,:).^2, 2));
        data.wheel_encoder.velocity = speed + encParams.velocity_std * randn(size(speed));
        yawRate = gtAngVel(encIdx, 3);
        data.wheel_encoder.headingRate = yawRate + encParams.heading_rate_std * randn(size(yawRate));
    end

    fprintf('  ✓ Simulation complete: %d IMU, %d GPS samples\n', nSamples, numel(gpsIdx));
end


%% ── Helper: quadcopter figure-eight trajectory ─────────────────────────
function [waypoints, toa, vel, orient] = quadcopterWaypoints()
    t = linspace(0, 2*pi, 9)';
    radius = 20;  % m
    altitude = -15;  % NED: negative = up

    north = radius * sin(t);
    east  = radius * sin(2*t) / 2;
    down  = altitude * ones(size(t));

    waypoints = [north, east, down];
    toa = linspace(0, 60, numel(t))';   % 60-second trajectory

    vel = gradient(waypoints) ./ gradient(toa);
    vel(1,:) = 0; vel(end,:) = 0;

    yaw = atan2(gradient(east), gradient(north));
    orient = quaternion([yaw, zeros(size(yaw)), zeros(size(yaw))], 'eulerd', 'ZYX', 'frame');
end


%% ── Helper: ground vehicle loop trajectory ───────────────────────────────
function [waypoints, toa, vel, orient] = groundWaypoints()
    t = linspace(0, 2*pi, 13)';
    radius = 30;  % m

    north = radius * cos(t);
    east  = radius * sin(t);
    down  = zeros(size(t));

    waypoints = [north, east, down];
    toa = linspace(0, 90, numel(t))';   % 90-second loop

    vel = gradient(waypoints) ./ gradient(toa);
    vel(1,:) = 0; vel(end,:) = 0;

    yaw = atan2(gradient(east), gradient(north));
    orient = quaternion([yaw, zeros(size(yaw)), zeros(size(yaw))], 'eulerd', 'ZYX', 'frame');
end


%% ── Helper: generic relative-pose odometry stream ────────────────────────
function odom = computeOdometryStream(gtPos, gtQuat, gtTime, idx, params)
    odom.time = gtTime(idx(2:end));
    n = numel(idx) - 1;
    odom.relPos  = zeros(n, 3);
    odom.relQuat = zeros(n, 4);

    for k = 1:n
        i0 = idx(k); i1 = idx(k+1);
        [relPos, relQuat, ~] = lidarOdometryModel(gtPos(i0,:), gtPos(i1,:), ...
                                                    gtQuat(i0,:), gtQuat(i1,:), params);
        odom.relPos(k,:)  = relPos;
        odom.relQuat(k,:) = relQuat;
    end
end


%% ── Helper: LLA to local NED (flat-Earth) ────────────────────────────────
function [north, east, down] = lla2nedLocal(lla, refLoc)
    R_earth = 6378137.0;
    lat0 = refLoc(1); lon0 = refLoc(2); alt0 = refLoc(3);

    north = (lla(:,1) - lat0) * pi/180 * R_earth;
    east  = (lla(:,2) - lon0) * pi/180 * R_earth .* cos(lat0*pi/180);
    down  = -(lla(:,3) - alt0);
end
