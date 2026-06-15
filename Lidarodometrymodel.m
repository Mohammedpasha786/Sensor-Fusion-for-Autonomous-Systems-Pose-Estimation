function [relPos, relQuat, covOut] = lidarOdometryModel(truePosPrev, truePosCurr, ...
                                                          trueQuatPrev, trueQuatCurr, params, dropoutFlag)
% LIDARODOMETRYMODEL  Simulate relative-pose odometry measurements
%   (e.g. from LiDAR ICP or visual odometry) with realistic noise.
%
%   [relPos, relQuat, covOut] = lidarOdometryModel(truePosPrev, truePosCurr, ...
%                                    trueQuatPrev, trueQuatCurr, params, dropoutFlag)
%
%   Inputs:
%     truePosPrev, truePosCurr   - [1x3] true positions at t-1 and t (NED, m)
%     trueQuatPrev, trueQuatCurr - [1x4] true quaternions ([w x y z]) at t-1, t
%     params       - struct with .position_std, .orientation_std,
%                     .dropout_probability, .scale_drift (optional)
%     dropoutFlag  - (optional) force dropout (true/false). If omitted,
%                     dropout is sampled using params.dropout_probability.
%
%   Outputs:
%     relPos   - [1x3] noisy relative position (body frame), or NaN if dropout
%     relQuat  - [1x4] noisy relative orientation quaternion, or NaN if dropout
%     covOut   - [6x6] measurement covariance [pos(3); rot(3)]
%
%   Model:
%     True relative pose = inv(q_prev) * (p_curr - p_prev), inv(q_prev)*q_curr
%     Noise added: Gaussian position noise + small-angle rotation noise
%     Optional scale drift simulates accumulated odometry drift.

    if nargin < 6
        dropoutFlag = rand() < params.dropout_probability;
    end

    if dropoutFlag
        relPos  = [NaN NaN NaN];
        relQuat = [NaN NaN NaN NaN];
        covOut  = NaN(6,6);
        return;
    end

    %% ── True relative pose ───────────────────────────────────────────────
    qPrev = quaternion(trueQuatPrev);
    qCurr = quaternion(trueQuatCurr);

    deltaPosWorld = truePosCurr - truePosPrev;
    relPosTrue = rotateframe(conj(qPrev), deltaPosWorld);   % world -> body

    relQuatTrue = qPrev' * qCurr;   % relative rotation (quaternion product)

    %% ── Add scale drift (optional) ──────────────────────────────────────
    if isfield(params, 'scale_drift') && params.scale_drift > 0
        scaleFactor = 1 + params.scale_drift * (rand() - 0.5) * 2;
        relPosTrue = relPosTrue * scaleFactor;
    end

    %% ── Add Gaussian noise ────────────────────────────────────────────────
    posNoise = params.position_std * randn(1,3);
    relPos = relPosTrue + posNoise;

    % Small-angle rotation noise
    rotNoiseAngle = params.orientation_std * randn(1,3);
    qNoise = quaternion(rotNoiseAngle, 'rotvec');
    relQuatNoisy = relQuatTrue * qNoise;
    relQuat = compact(relQuatNoisy);

    %% ── Covariance ───────────────────────────────────────────────────────
    covOut = diag([repmat(params.position_std^2, 1, 3), ...
                    repmat(params.orientation_std^2, 1, 3)]);
end
