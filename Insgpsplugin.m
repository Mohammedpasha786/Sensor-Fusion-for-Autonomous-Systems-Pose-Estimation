classdef insGPSPlugin < positioning.INSSensorModel
% INSGPSPLUGIN  Custom GPS measurement model for insEKF.
%
%   Measures absolute position (NED) and velocity (NED) directly from
%   the state vector. Suitable for GPS receivers that report local
%   tangent-plane coordinates (after lla2ned conversion) and velocity.
%
%   Measurement vector: z = [pos(3); vel(3)]  (6 elements)
%
%   Usage:
%     gps = insGPSPlugin('PositionNoise', 1.5^2, 'VelocityNoise', 0.2^2);
%     insertSensor(filt, 'GPS', gps);
%     correct(filt, 'GPS', [posMeas, velMeas]);
%
%   See: MathWorks "Design Fusion Filter for Custom Sensors" [Ref 5]

    properties
        PositionNoise (1,1) double = 1.5^2   % m^2
        VelocityNoise (1,1) double = 0.2^2   % (m/s)^2
    end

    properties (Constant)
        IDX_POS = 1:3;
        IDX_VEL = 4:6;
        MeasurementNames = ["px","py","pz","vx","vy","vz"];
    end

    methods
        function obj = insGPSPlugin(varargin)
            obj = obj@positioning.INSSensorModel();
            obj = setProperties(obj, nargin, varargin{:});
        end

        %% ── Measurement function ─────────────────────────────────────────
        function z = measurement(obj, filt, x, varargin)
        % Predicted GPS measurement given current state estimate.
            z = [x(obj.IDX_POS); x(obj.IDX_VEL)];
        end

        %% ── Measurement Jacobian ──────────────────────────────────────────
        function H = measurementJacobian(obj, filt, x, varargin)
        % Jacobian of measurement() w.r.t. state (linear -> constant).
            n = numel(x);
            H = zeros(6, n);
            H(1:3, obj.IDX_POS) = eye(3);
            H(4:6, obj.IDX_VEL) = eye(3);
        end

        %% ── Measurement noise covariance ──────────────────────────────────
        function R = measurementNoise(obj, filt, x, varargin)
            R = diag([repmat(obj.PositionNoise, 1, 3), ...
                      repmat(obj.VelocityNoise, 1, 3)]);
        end
    end
end
