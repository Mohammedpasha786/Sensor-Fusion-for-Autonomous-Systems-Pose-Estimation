function metrics = computeMetrics(estPoses, groundTruth)
% COMPUTEMETRICS  Compute pose estimation accuracy metrics by comparing
%   estimated poses against ground truth.
%
%   metrics = computeMetrics(estPoses, groundTruth)
%
%   Inputs:
%     estPoses    - struct with fields:
%                     .time  [N x 1]
%                     .pos   [N x 3]  (NED, metres)
%                     .quat  [N x 4]  ([w x y z])
%     groundTruth - struct with fields:
%                     .time  [M x 1]
%                     .pos   [M x 3]
%                     .quat  [M x 4]
%
%   Output:
%     metrics     - struct with fields:
%                     .posRMSE   - position RMSE (m)
%                     .yawRMSE   - yaw RMSE (degrees)
%                     .pitchRMSE - pitch RMSE (degrees)
%                     .rollRMSE  - roll RMSE (degrees)
%                     .ATE       - Absolute Trajectory Error (m, RMS)
%                     .RPE       - Relative Pose Error (m/s, RMS over 1s windows)
%                     .maxError  - maximum position error (m)
%                     .errorOverTime - [N x 1] position error per timestep

    %% ── Interpolate ground truth onto estimate timeline ─────────────────
    gtPosInterp  = interp1(groundTruth.time, groundTruth.pos,  estPoses.time, 'linear', 'extrap');
    gtQuatInterp = interpolateQuaternion(groundTruth.time, groundTruth.quat, estPoses.time);

    %% ── Position error ───────────────────────────────────────────────────
    posError = estPoses.pos - gtPosInterp;          % [N x 3]
    errNorm  = sqrt(sum(posError.^2, 2));            % [N x 1]

    metrics.posRMSE  = sqrt(mean(errNorm.^2));
    metrics.maxError = max(errNorm);
    metrics.errorOverTime = errNorm;

    %% ── Orientation error (Euler angles) ─────────────────────────────────
    estEul = quat2eul(estPoses.quat,  'ZYX') * 180/pi;   % [yaw pitch roll]
    gtEul  = quat2eul(gtQuatInterp,   'ZYX') * 180/pi;

    yawErr   = wrapTo180(estEul(:,1) - gtEul(:,1));
    pitchErr = wrapTo180(estEul(:,2) - gtEul(:,2));
    rollErr  = wrapTo180(estEul(:,3) - gtEul(:,3));

    metrics.yawRMSE   = sqrt(mean(yawErr.^2));
    metrics.pitchRMSE = sqrt(mean(pitchErr.^2));
    metrics.rollRMSE  = sqrt(mean(rollErr.^2));

    %% ── Absolute Trajectory Error (ATE) ──────────────────────────────────
    % ATE = RMS of full-trajectory position error (after optional alignment)
    % Here we report direct RMS error (assumes common reference frame)
    metrics.ATE = sqrt(mean(sum(posError.^2, 2)));

    %% ── Relative Pose Error (RPE) ────────────────────────────────────────
    % RPE measures drift over a fixed time interval (1 second)
    dt = median(diff(estPoses.time));
    windowSize = max(1, round(1 / dt));   % ~1 second

    if numel(estPoses.time) > windowSize
        relErrors = zeros(numel(estPoses.time) - windowSize, 1);
        for i = 1:numel(relErrors)
            estDelta = estPoses.pos(i+windowSize,:) - estPoses.pos(i,:);
            gtDelta  = gtPosInterp(i+windowSize,:)   - gtPosInterp(i,:);
            relErrors(i) = norm(estDelta - gtDelta) / (windowSize * dt);
        end
        metrics.RPE = sqrt(mean(relErrors.^2));
    else
        metrics.RPE = NaN;
    end
end


function quatInterp = interpolateQuaternion(t, quat, tQuery)
% Spherical linear interpolation (SLERP) of quaternion array onto new timeline.
    quatObj = quaternion(quat);
    quatInterp = zeros(numel(tQuery), 4);

    for i = 1:numel(tQuery)
        tq = tQuery(i);
        if tq <= t(1)
            quatInterp(i,:) = compact(quatObj(1));
            continue;
        elseif tq >= t(end)
            quatInterp(i,:) = compact(quatObj(end));
            continue;
        end
        idx = find(t <= tq, 1, 'last');
        idx = min(idx, numel(t)-1);
        frac = (tq - t(idx)) / (t(idx+1) - t(idx) + eps);
        q_slerp = slerp(quatObj(idx), quatObj(idx+1), frac);
        quatInterp(i,:) = compact(q_slerp);
    end
end


function wrapped = wrapTo180(angleDeg)
% Wrap angle (degrees) to [-180, 180]
    wrapped = mod(angleDeg + 180, 360) - 180;
end
