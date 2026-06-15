function filt = buildInsEKF(ekfParams)
% BUILDINSEKF  Construct an insEKF filter configured with custom sensor
%   model plugins for IMU, GPS, LiDAR odometry, visual odometry, and
%   wheel encoders.
%
%   filt = buildInsEKF(ekfParams)
%
%   Input:
%     ekfParams - struct from configs/ekf_params.yaml
%
%   Output:
%     filt      - configured insEKF object (Navigation Toolbox)
%
%   State vector (16 elements):
%     [1:3]   Position (NED, m)
%     [4:6]   Velocity (NED, m/s)
%     [7:10]  Orientation quaternion [w x y z]
%     [11:13] Accelerometer bias (m/s^2)
%     [14:16] Gyroscope bias (rad/s)
%
%   Sensor models registered:
%     - insIMUPlugin              (process model — IMU drives prediction)
%     - insGPSPlugin               (position + velocity update)
%     - insLiDARPlugin              (relative pose update — ground)
%     - insVisualOdometryPlugin     (relative pose update — air)
%     - insWheelEncoderPlugin       (velocity + heading rate update — ground)

    %% ── Create base filter ───────────────────────────────────────────────
    filt = insEKF;

    %% ── State transition: IMU-driven dynamics ───────────────────────────
    imuPlugin = insIMUPlugin(...
        'AccelerometerNoise', ekfParams.process_noise.accel_noise_density^2, ...
        'GyroscopeNoise',     ekfParams.process_noise.gyro_noise_density^2, ...
        'AccelBiasDecay',     1 / ekfParams.process_noise.accel_bias_decay, ...
        'GyroBiasDecay',      1 / ekfParams.process_noise.gyro_bias_decay, ...
        'AccelBiasNoise',     ekfParams.process_noise.accel_bias_instability^2, ...
        'GyroBiasNoise',      ekfParams.process_noise.gyro_bias_instability^2);

    insertStateName(filt, imuPlugin);
    filt.StateTransitionFcn = imuPlugin;

    %% ── Register measurement sensor models ──────────────────────────────
    gpsPlugin = insGPSPlugin(...
        'PositionNoise', ekfParams.measurement_noise.gps.position_std^2, ...
        'VelocityNoise', ekfParams.measurement_noise.gps.velocity_std^2);
    insertSensor(filt, 'GPS', gpsPlugin);

    lidarPlugin = insLiDARPlugin(...
        'PositionNoise',    ekfParams.measurement_noise.lidar_odometry.position_std^2, ...
        'OrientationNoise', ekfParams.measurement_noise.lidar_odometry.orientation_std^2);
    insertSensor(filt, 'LiDAR', lidarPlugin);

    voPlugin = insVisualOdometryPlugin(...
        'PositionNoise',    ekfParams.measurement_noise.visual_odometry.position_std^2, ...
        'OrientationNoise', ekfParams.measurement_noise.visual_odometry.orientation_std^2);
    insertSensor(filt, 'VisualOdometry', voPlugin);

    encPlugin = insWheelEncoderPlugin(...
        'VelocityNoise',    ekfParams.measurement_noise.wheel_encoder.velocity_std^2, ...
        'HeadingRateNoise', ekfParams.measurement_noise.wheel_encoder.heading_rate_std^2);
    insertSensor(filt, 'WheelEncoder', encPlugin);

    %% ── Initial state ────────────────────────────────────────────────────
    init = ekfParams.initial_state;
    eulRad = init.orientation_euler_deg * pi/180;
    initQuat = compact(quaternion(eulRad, 'eulerd', 'ZYX', 'frame'));

    x0 = [init.position(:); init.velocity(:); initQuat(:); ...
          init.accel_bias(:); init.gyro_bias(:)];

    setStateVector(filt, x0);

    %% ── Initial covariance ───────────────────────────────────────────────
    P0 = eye(numel(x0)) * ekfParams.filter.initial_covariance;
    setStateCovariance(filt, P0);

    %% ── Innovation gating (outlier rejection) ───────────────────────────
    if ekfParams.innovation_gating.enabled
        filt.GatingThreshold = ekfParams.innovation_gating.chi2_threshold;
    end

    fprintf('  insEKF constructed: %d-state filter, sensors = {GPS, LiDAR, VisualOdometry, WheelEncoder}\n', ...
            numel(x0));
end


function insertStateName(filt, plugin) %#ok<INUSD>
% Placeholder for state-name registration if required by toolbox version.
% Newer Navigation Toolbox versions auto-derive state names from the
% process model plugin's StateNames property.
end
