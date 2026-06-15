function data = loadKITTI(dataPath)
% LOADKITTI  Load a KITTI raw dataset sequence (OXTS GPS/IMU) into the
%   common data structure for the sensor fusion pipeline.
%
%   data = loadKITTI(dataPath)
%
%   Input:
%     dataPath - path to sequence folder, e.g.
%                './data/raw/kitti/2011_09_26/2011_09_26_drive_0001_sync/'
%                Expects subfolders: oxts/data/*.txt, oxts/timestamps.txt
%
%   Output:
%     data     - struct (same format as loadEuRoC):
%                  .time, .imu.{time,accel,gyro}, .gps.{time,pos,vel}
%                  .groundTruth.{time,pos,quat,vel}
%
%   KITTI OXTS format (each line in oxts/data/XXXXXXXXXX.txt):
%     lat lon alt roll pitch yaw vn ve vf vl vu ax ay az af al au wx wy wz
%     wf wl wu pos_accuracy vel_accuracy navstat numsats posmode velmode orimode
%
%   GPS (lat/lon/alt) is converted to local NED metres using the first
%   sample as the origin (flat-Earth approximation, valid for short
%   sequences).

    oxtsDir = fullfile(dataPath, 'oxts', 'data');
    tsFile  = fullfile(dataPath, 'oxts', 'timestamps.txt');

    assert(isfolder(oxtsDir), 'OXTS data folder not found: %s', oxtsDir);
    assert(isfile(tsFile),    'OXTS timestamps not found: %s', tsFile);

    %% ── Timestamps ───────────────────────────────────────────────────────
    tsTable = readlines(tsFile);
    tsTable = tsTable(strlength(tsTable) > 0);
    timestamps = datetime(tsTable, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
    t = seconds(timestamps - timestamps(1));

    %% ── OXTS data files ──────────────────────────────────────────────────
    files = dir(fullfile(oxtsDir, '*.txt'));
    files = sort_nat({files.name});
    n = numel(files);
    assert(n == numel(t), 'Mismatch: %d OXTS files vs %d timestamps', n, numel(t));

    oxts = zeros(n, 30);
    for i = 1:n
        vals = readmatrix(fullfile(oxtsDir, files{i}), 'Delimiter', ' ', ...
                           'FileType', 'text');
        oxts(i, 1:numel(vals)) = vals;
    end

    lat = oxts(:,1); lon = oxts(:,2); alt = oxts(:,3);
    roll = oxts(:,4); pitch = oxts(:,5); yaw = oxts(:,6);
    vn = oxts(:,7); ve = oxts(:,8); vf = oxts(:,9); vl = oxts(:,10); vu = oxts(:,11);
    ax = oxts(:,12); ay = oxts(:,13); az = oxts(:,14);
    wx = oxts(:,18); wy = oxts(:,19); wz = oxts(:,20);

    %% ── Convert lat/lon/alt to local NED metres (flat-Earth) ─────────────
    R_earth = 6378137.0;  % WGS84 semi-major axis
    lat0 = lat(1); lon0 = lon(1); alt0 = alt(1);

    north = (lat - lat0) * pi/180 * R_earth;
    east  = (lon - lon0) * pi/180 * R_earth .* cos(lat0 * pi/180);
    down  = -(alt - alt0);

    %% ── Assemble output struct ────────────────────────────────────────────
    data.time = t;

    % IMU (high rate ~100 Hz in KITTI OXTS)
    data.imu.time  = t;
    data.imu.accel = [ax, ay, az];
    data.imu.gyro  = [wx, wy, wz];

    % GPS (same rate as OXTS, but treat as GPS-rate measurement)
    gpsDecim = 10;  % simulate ~10 Hz GPS from 100 Hz OXTS
    gpsIdx = 1:gpsDecim:n;
    data.gps.time = t(gpsIdx);
    data.gps.pos  = [north(gpsIdx), east(gpsIdx), down(gpsIdx)];
    data.gps.vel  = [vn(gpsIdx), ve(gpsIdx), -vu(gpsIdx)];

    % Ground truth (full OXTS trajectory)
    data.groundTruth.time = t;
    data.groundTruth.pos  = [north, east, down];
    eul = [yaw, pitch, roll];  % [Z Y X] order for eul2quat
    data.groundTruth.quat = eul2quat(eul, 'ZYX');  % [w x y z]
    data.groundTruth.vel  = [vn, ve, -vu];

    fprintf('  KITTI sequence loaded: %s\n', dataPath);
    fprintf('    OXTS samples: %d (%.1f Hz)\n', n, 1/median(diff(t)));
    fprintf('    GPS decimated to: %d samples (%.1f Hz)\n', ...
            numel(gpsIdx), 1/median(diff(data.gps.time)));
end


function sorted = sort_nat(names)
% Natural sort of filenames (0000000000.txt, 0000000001.txt, ...)
    [~, idx] = sort(names);
    sorted = names(idx);
end
