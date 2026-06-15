function data = loadEuRoC(dataPath)
% LOADEUROC  Load the EuRoC MAV dataset (e.g. MH_01_easy) into a common
%   data structure for the sensor fusion pipeline.
%
%   data = loadEuRoC(dataPath)
%
%   Input:
%     dataPath - path to the sequence's "mav0" folder, e.g.
%                './data/raw/euroc/MH_01_easy/mav0/'
%
%   Output:
%     data     - struct with fields:
%                  .time          [N x 1]  master timeline (s)
%                  .imu.time      [Ni x 1]
%                  .imu.accel     [Ni x 3]  m/s^2
%                  .imu.gyro      [Ni x 3]  rad/s
%                  .gps.time      [Ng x 1]  (EuRoC has no native GPS —
%                                            synthesized from ground truth
%                                            + noise, see note below)
%                  .gps.pos       [Ng x 3]  m (NED)
%                  .groundTruth.time [Nt x 1]
%                  .groundTruth.pos  [Nt x 3]
%                  .groundTruth.quat [Nt x 4]  (w,x,y,z)
%                  .groundTruth.vel  [Nt x 3]
%
%   EuRoC directory structure expected:
%     mav0/imu0/data.csv
%     mav0/state_groundtruth_estimate0/data.csv
%
%   NOTE: EuRoC does not include GPS. A synthetic GPS stream is generated
%   from ground truth + Gaussian noise (see configs/sensor_params.yaml)
%   to enable GPS-aided EKF demonstrations. Set 'synthesizeGPS' = false
%   to skip this.

    assert(isfolder(dataPath), 'EuRoC sequence folder not found: %s', dataPath);

    %% ── IMU data ─────────────────────────────────────────────────────────
    imuFile = fullfile(dataPath, 'imu0', 'data.csv');
    assert(isfile(imuFile), 'IMU data not found: %s', imuFile);

    imuRaw = readmatrix(imuFile);
    % Columns: timestamp[ns], gyro_x,y,z [rad/s], accel_x,y,z [m/s^2]
    data.imu.time  = imuRaw(:,1) * 1e-9;            % ns -> s
    data.imu.gyro  = imuRaw(:, 2:4);
    data.imu.accel = imuRaw(:, 5:7);

    %% ── Ground truth ─────────────────────────────────────────────────────
    gtFile = fullfile(dataPath, 'state_groundtruth_estimate0', 'data.csv');
    assert(isfile(gtFile), 'Ground truth not found: %s', gtFile);

    gtRaw = readmatrix(gtFile);
    % Columns: timestamp[ns], p_x,p_y,p_z, q_w,q_x,q_y,q_z, v_x,v_y,v_z, b_w(3), b_a(3)
    data.groundTruth.time = gtRaw(:,1) * 1e-9;
    data.groundTruth.pos  = gtRaw(:, 2:4);
    data.groundTruth.quat = gtRaw(:, 5:8);   % [w x y z]
    data.groundTruth.vel  = gtRaw(:, 9:11);

    %% ── Master timeline ──────────────────────────────────────────────────
    t0 = min(data.imu.time(1), data.groundTruth.time(1));
    data.imu.time         = data.imu.time         - t0;
    data.groundTruth.time = data.groundTruth.time - t0;
    data.time = data.imu.time;

    %% ── Synthesize GPS from ground truth (EuRoC has no GPS) ─────────────
    gpsRate = 5;  % Hz
    gtInterp_t = data.groundTruth.time;
    gpsTimes = (gtInterp_t(1):1/gpsRate:gtInterp_t(end))';

    gpsPosTrue = interp1(gtInterp_t, data.groundTruth.pos, gpsTimes, 'linear', 'extrap');

    rng(1);
    gpsNoiseStd = 1.5;  % metres, see sensor_params.yaml
    data.gps.time = gpsTimes;
    data.gps.pos  = gpsPosTrue + gpsNoiseStd * randn(size(gpsPosTrue));

    %% ── Summary ──────────────────────────────────────────────────────────
    fprintf('  EuRoC sequence loaded: %s\n', dataPath);
    fprintf('    IMU samples         : %d (%.1f Hz)\n', ...
            numel(data.imu.time), 1/median(diff(data.imu.time)));
    fprintf('    Ground truth samples: %d\n', numel(data.groundTruth.time));
    fprintf('    Synthetic GPS samples (%.0f Hz): %d  [NOTE: synthesized, EuRoC has no native GPS]\n', ...
            gpsRate, numel(data.gps.time));
end
