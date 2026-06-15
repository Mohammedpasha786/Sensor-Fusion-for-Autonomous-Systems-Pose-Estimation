function data = loadANSFL(dataPath)
% LOADANSFL  Load the ANSFL (Autonomous Platforms Inertial Dataset) into
%   the common data structure for the sensor fusion pipeline.
%
%   data = loadANSFL(dataPath)
%
%   Input:
%     dataPath - path to a sequence folder containing:
%                  imu.csv   : [time, ax, ay, az, wx, wy, wz]
%                  gps.csv   : [time, x, y, z] or [time, lat, lon, alt]
%                  groundtruth.csv : [time, x, y, z, qw, qx, qy, qz, vx, vy, vz]
%
%   Output:
%     data     - struct (same format as loadEuRoC):
%                  .time, .imu.{time,accel,gyro}, .gps.{time,pos}
%                  .groundTruth.{time,pos,quat,vel}
%
%   Reference:
%     Shurin, A., et al. (2022). The Autonomous Platforms Inertial Dataset.
%     IEEE Access, 10, 10191-10201.

    imuFile = fullfile(dataPath, 'imu.csv');
    gpsFile = fullfile(dataPath, 'gps.csv');
    gtFile  = fullfile(dataPath, 'groundtruth.csv');

    assert(isfile(imuFile), 'IMU file not found: %s', imuFile);
    assert(isfile(gtFile),  'Ground truth file not found: %s', gtFile);

    %% ── IMU ───────────────────────────────────────────────────────────────
    imuTable = readtable(imuFile);
    data.imu.time  = imuTable.time - imuTable.time(1);
    data.imu.accel = [imuTable.ax, imuTable.ay, imuTable.az];
    data.imu.gyro  = [imuTable.wx, imuTable.wy, imuTable.wz];

    %% ── Ground truth ─────────────────────────────────────────────────────
    gtTable = readtable(gtFile);
    data.groundTruth.time = gtTable.time - gtTable.time(1);
    data.groundTruth.pos  = [gtTable.x, gtTable.y, gtTable.z];

    if all(ismember({'qw','qx','qy','qz'}, gtTable.Properties.VariableNames))
        data.groundTruth.quat = [gtTable.qw, gtTable.qx, gtTable.qy, gtTable.qz];
    else
        % If only Euler angles provided, convert
        if all(ismember({'roll','pitch','yaw'}, gtTable.Properties.VariableNames))
            eul = [gtTable.yaw, gtTable.pitch, gtTable.roll];
            data.groundTruth.quat = eul2quat(eul, 'ZYX');
        else
            warning('No orientation found in ground truth; using identity quaternions.');
            data.groundTruth.quat = repmat([1 0 0 0], height(gtTable), 1);
        end
    end

    if all(ismember({'vx','vy','vz'}, gtTable.Properties.VariableNames))
        data.groundTruth.vel = [gtTable.vx, gtTable.vy, gtTable.vz];
    else
        % Estimate velocity by differentiation
        dt = diff(data.groundTruth.time);
        dt(dt == 0) = eps;
        vel = diff(data.groundTruth.pos) ./ dt;
        data.groundTruth.vel = [vel; vel(end,:)];
    end

    %% ── GPS (optional) ───────────────────────────────────────────────────
    if isfile(gpsFile)
        gpsTable = readtable(gpsFile);
        data.gps.time = gpsTable.time - imuTable.time(1);

        if all(ismember({'x','y','z'}, gpsTable.Properties.VariableNames))
            data.gps.pos = [gpsTable.x, gpsTable.y, gpsTable.z];
        elseif all(ismember({'lat','lon','alt'}, gpsTable.Properties.VariableNames))
            % Convert geodetic to local NED using flat-Earth approx
            R_earth = 6378137.0;
            lat0 = gpsTable.lat(1); lon0 = gpsTable.lon(1); alt0 = gpsTable.alt(1);
            north = (gpsTable.lat - lat0) * pi/180 * R_earth;
            east  = (gpsTable.lon - lon0) * pi/180 * R_earth .* cos(lat0*pi/180);
            down  = -(gpsTable.alt - alt0);
            data.gps.pos = [north, east, down];
        end
    else
        warning('No GPS file found; synthesizing GPS from ground truth.');
        gpsRate = 5;
        gpsTimes = (data.groundTruth.time(1):1/gpsRate:data.groundTruth.time(end))';
        gpsPosTrue = interp1(data.groundTruth.time, data.groundTruth.pos, gpsTimes, 'linear', 'extrap');
        rng(1);
        data.gps.time = gpsTimes;
        data.gps.pos  = gpsPosTrue + 1.5 * randn(size(gpsPosTrue));
    end

    %% ── Master timeline ──────────────────────────────────────────────────
    data.time = data.imu.time;

    fprintf('  ANSFL sequence loaded: %s\n', dataPath);
    fprintf('    IMU samples : %d (%.1f Hz)\n', numel(data.imu.time), 1/median(diff(data.imu.time)));
    fprintf('    GPS samples : %d\n', numel(data.gps.time));
    fprintf('    GT samples  : %d\n', numel(data.groundTruth.time));
end
