classdef insVisualOdometryPlugin < positioning.INSSensorModel
% INSVISUALODOMETRYPLUGIN  Custom visual odometry (VO) relative-pose
%   measurement model for insEKF.
%
%   Functionally similar to insLiDARPlugin, but tuned for camera-based
%   feature-tracking odometry, which typically exhibits scale drift and
%   higher-frequency but noisier relative-pose estimates than LiDAR ICP.
%
%   Measurement vector: z = [relPos(3); relRotVec(3)]  (6 elements)
%
%   Usage:
%     vo = insVisualOdometryPlugin('PositionNoise', 0.03^2, 'OrientationNoise', 0.008^2);
%     insertSensor(filt, 'VisualOdometry', vo);
%     correct(filt, 'VisualOdometry', [relPos, relRotVec], 'PreviousState', xPrevKeyframe);
%
%   See: MathWorks "Design Fusion Filter for Custom Sensors" [Ref 5]

    properties
        PositionNoise    (1,1) double = 0.03^2   % m^2
        OrientationNoise (1,1) double = 0.008^2  % rad^2
        ScaleUncertainty (1,1) double = 0.001    % fractional, inflates PositionNoise with distance
    end

    properties (Constant)
        IDX_POS  = 1:3;
        IDX_QUAT = 7:10;
        MeasurementNames = ["dx","dy","dz","drx","dry","drz"];
    end

    methods
        function obj = insVisualOdometryPlugin(varargin)
            obj = obj@positioning.INSSensorModel();
            obj = setProperties(obj, nargin, varargin{:});
        end

        %% ── Measurement function ─────────────────────────────────────────
        function z = measurement(obj, filt, x, varargin)
            xPrev = parsePreviousState(varargin{:});

            posPrev = xPrev(obj.IDX_POS);
            posCurr = x(obj.IDX_POS);
            qPrev   = quaternion(xPrev(obj.IDX_QUAT)');
            qCurr   = quaternion(x(obj.IDX_QUAT)');

            deltaPosWorld = (posCurr - posPrev)';
            relPos = rotateframe(conj(qPrev), deltaPosWorld)';

            relQuat = conj(qPrev) * qCurr;
            relRotVec = rotvec(relQuat)';

            z = [relPos; relRotVec];
        end

        %% ── Measurement Jacobian ──────────────────────────────────────────
        function H = measurementJacobian(obj, filt, x, varargin)
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

        %% ── Measurement noise covariance (with scale-drift inflation) ─────
        function R = measurementNoise(obj, filt, x, varargin)
            xPrev = parsePreviousState(varargin{:});
            travelDist = norm(x(obj.IDX_POS) - xPrev(obj.IDX_POS));

            posVar = obj.PositionNoise * (1 + obj.ScaleUncertainty * travelDist)^2;

            R = diag([repmat(posVar, 1, 3), ...
                      repmat(obj.OrientationNoise, 1, 3)]);
        end
    end
end


function xPrev = parsePreviousState(varargin)
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
        error('insVisualOdometryPlugin:measurement requires a ''PreviousState'' argument.');
    end
end
