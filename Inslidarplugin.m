classdef insLiDARPlugin < positioning.INSSensorModel
% INSLIDARPLUGIN  Custom LiDAR (scan-matching / ICP) relative-pose
%   odometry measurement model for insEKF.
%
%   Measures the relative pose (position delta in body frame + relative
%   orientation) between two consecutive keyframes. Requires the filter
%   to retain the previous-keyframe state internally via PreviousState.
%
%   Measurement vector: z = [relPos(3); relRotVec(3)]  (6 elements,
%     relative orientation expressed as a rotation vector for linearization)
%
%   Usage:
%     lidar = insLiDARPlugin('PositionNoise', 0.05^2, 'OrientationNoise', 0.01^2);
%     insertSensor(filt, 'LiDAR', lidar);
%     correct(filt, 'LiDAR', [relPos, relRotVec], 'PreviousState', xPrevKeyframe);
%
%   See: MathWorks "Design Fusion Filter for Custom Sensors" [Ref 5]

    properties
        PositionNoise    (1,1) double = 0.05^2   % m^2
        OrientationNoise (1,1) double = 0.01^2   % rad^2
    end

    properties (Constant)
        IDX_POS  = 1:3;
        IDX_QUAT = 7:10;
        MeasurementNames = ["dx","dy","dz","drx","dry","drz"];
    end

    methods
        function obj = insLiDARPlugin(varargin)
            obj = obj@positioning.INSSensorModel();
            obj = setProperties(obj, nargin, varargin{:});
        end

        %% ── Measurement function ─────────────────────────────────────────
        function z = measurement(obj, filt, x, varargin)
        % Predicted relative pose between PreviousState (xPrev) and current
        % state x, expressed in xPrev's body frame.
        %
        %   varargin{1} (or Name-Value 'PreviousState') = xPrev [16x1]

            xPrev = parsePreviousState(varargin{:});

            posPrev = xPrev(obj.IDX_POS);
            posCurr = x(obj.IDX_POS);
            qPrev   = quaternion(xPrev(obj.IDX_QUAT)');
            qCurr   = quaternion(x(obj.IDX_QUAT)');

            % Relative position in previous body frame
            deltaPosWorld = (posCurr - posPrev)';
            relPos = rotateframe(conj(qPrev), deltaPosWorld)';

            % Relative rotation as rotation vector
            relQuat = conj(qPrev) * qCurr;
            relRotVec = rotvec(relQuat)';

            z = [relPos; relRotVec];
        end

        %% ── Measurement Jacobian ──────────────────────────────────────────
        function H = measurementJacobian(obj, filt, x, varargin)
        % Numerical Jacobian (relative-pose measurement is nonlinear in
        % the quaternion components).
            xPrev = parsePreviousState(varargin{:});

            n = numel(x);
            H = zeros(6, n);
            epsVal = 1e-6;

            z0 = obj.measurement(filt, x, xPrev);
            for i = 1:n
                dx = zeros(n,1);
                dx(i) = epsVal;
                z1 = obj.measurement(filt, x + dx, xPrev);
                H(:,i) = (z1 - z0) / epsVal;
            end
        end

        %% ── Measurement noise covariance ──────────────────────────────────
        function R = measurementNoise(obj, filt, x, varargin)
            R = diag([repmat(obj.PositionNoise, 1, 3), ...
                      repmat(obj.OrientationNoise, 1, 3)]);
        end
    end
end


function xPrev = parsePreviousState(varargin)
% Extract PreviousState argument from varargin (positional or Name-Value).
    xPrev = [];
    for i = 1:numel(varargin)
        if isnumeric(varargin{i}) && numel(varargin{i}) == 16
            xPrev = varargin{i}(:);
            return;
        end
        if ischar(varargin{i}) && strcmpi(varargin{i}, 'PreviousState') && i < numel(varargin)
            xPrev = varargin{i+1}(:);
            return;
        end
    end
    if isempty(xPrev)
        error('insLiDARPlugin:measurement requires a ''PreviousState'' argument.');
    end
end
