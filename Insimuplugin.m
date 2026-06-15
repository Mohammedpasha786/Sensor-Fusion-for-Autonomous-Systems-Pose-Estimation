classdef insIMUPlugin < positioning.INSMotionModel
% INSIMUPLUGIN  Custom IMU-driven process (state transition) model for insEKF.
%
%   This plugin implements the strapdown inertial navigation equations,
%   propagating the 16-element state vector:
%     [pos(3), vel(3), quat(4), accelBias(3), gyroBias(3)]
%   using IMU acceleration and angular rate measurements as control inputs.
%
%   Usage:
%     imuPlugin = insIMUPlugin('AccelerometerNoise', 0.02^2, ...
%                               'GyroscopeNoise', 0.002^2, ...
%                               'AccelBiasDecay', 0.01, ...
%                               'GyroBiasDecay', 0.01, ...
%                               'AccelBiasNoise', 0.0005^2, ...
%                               'GyroBiasNoise', 0.00005^2);
%     filt.StateTransitionFcn = imuPlugin;
%
%   See: MathWorks "Design Fusion Filter for Custom Sensors" [Ref 5]

    properties
        AccelerometerNoise (1,1) double = 0.02^2     % (m/s^2)^2
        GyroscopeNoise     (1,1) double = 0.002^2    % (rad/s)^2
        AccelBiasDecay     (1,1) double = 0.01       % 1/tau (1/s)
        GyroBiasDecay      (1,1) double = 0.01       % 1/tau (1/s)
        AccelBiasNoise     (1,1) double = 0.0005^2   % (m/s^3)^2
        GyroBiasNoise      (1,1) double = 0.00005^2  % (rad/s^2)^2
        GravityNED         (1,3) double = [0 0 9.81] % m/s^2
    end

    properties (Constant)
        % State indices for readability
        IDX_POS   = 1:3;
        IDX_VEL   = 4:6;
        IDX_QUAT  = 7:10;
        IDX_ABIAS = 11:13;
        IDX_GBIAS = 14:16;
        StateNames = ["px","py","pz","vx","vy","vz", ...
                       "qw","qx","qy","qz","abx","aby","abz","gbx","gby","gbz"];
    end

    methods
        function obj = insIMUPlugin(varargin)
            obj = obj@positioning.INSMotionModel();
            obj = setProperties(obj, nargin, varargin{:});
        end

        %% ── State transition (predict) ───────────────────────────────────
        function xNext = stateTransition(obj, filt, x, dt, accelMeas, gyroMeas)
        % Propagate state forward by dt using IMU measurements.
        %
        %   xNext = stateTransition(obj, filt, x, dt, accelMeas, gyroMeas)
        %
        %   accelMeas - [1x3] specific force measurement (m/s^2), body frame
        %   gyroMeas  - [1x3] angular rate measurement (rad/s), body frame

            pos   = x(obj.IDX_POS);
            vel   = x(obj.IDX_VEL);
            q     = quaternion(x(obj.IDX_QUAT)');
            aBias = x(obj.IDX_ABIAS);
            gBias = x(obj.IDX_GBIAS);

            % Bias-corrected measurements
            accelCorr = accelMeas(:) - aBias(:);
            gyroCorr  = gyroMeas(:)  - gBias(:);

            % Rotate specific force into NED frame and remove gravity
            accelNED = rotateframe(q, accelCorr')' - obj.GravityNED(:);

            % ── Integrate position & velocity (simple Euler integration) ──
            posNext = pos + vel * dt + 0.5 * accelNED * dt^2;
            velNext = vel + accelNED * dt;

            % ── Integrate orientation quaternion ──────────────────────────
            % q_dot = 0.5 * q * [0, gyro]
            omega = quaternion([0, gyroCorr']);
            qDot  = q * omega * 0.5;
            qNext = q + qDot * dt;
            qNext = normalize(qNext);

            % ── Bias random-walk (first-order Gauss-Markov decay) ─────────
            aBiasNext = aBias * (1 - obj.AccelBiasDecay * dt);
            gBiasNext = gBias * (1 - obj.GyroBiasDecay * dt);

            xNext = x;
            xNext(obj.IDX_POS)   = posNext;
            xNext(obj.IDX_VEL)   = velNext;
            xNext(obj.IDX_QUAT)  = compact(qNext)';
            xNext(obj.IDX_ABIAS) = aBiasNext;
            xNext(obj.IDX_GBIAS) = gBiasNext;
        end

        %% ── State transition Jacobian ────────────────────────────────────
        function F = stateTransitionJacobian(obj, filt, x, dt, accelMeas, gyroMeas)
        % Compute Jacobian of stateTransition w.r.t. state (for covariance prop).
        % Uses numerical differentiation for clarity and robustness.

            n = numel(x);
            F = eye(n);
            epsVal = 1e-6;

            f0 = obj.stateTransition(filt, x, dt, accelMeas, gyroMeas);

            for i = 1:n
                dx = zeros(n,1);
                dx(i) = epsVal;
                f1 = obj.stateTransition(filt, x + dx, dt, accelMeas, gyroMeas);
                F(:,i) = (f1 - f0) / epsVal;
            end
        end

        %% ── Process noise covariance ─────────────────────────────────────
        function Q = processNoise(obj, filt, x, dt)
        % Construct discrete-time process noise covariance matrix.

            Q = zeros(16);

            % Velocity noise driven by accelerometer noise
            Q(obj.IDX_VEL, obj.IDX_VEL) = eye(3) * obj.AccelerometerNoise * dt;

            % Orientation noise driven by gyroscope noise
            Q(obj.IDX_QUAT(2:4), obj.IDX_QUAT(2:4)) = eye(3) * obj.GyroscopeNoise * dt * 0.25;

            % Bias random-walk noise
            Q(obj.IDX_ABIAS, obj.IDX_ABIAS) = eye(3) * obj.AccelBiasNoise * dt;
            Q(obj.IDX_GBIAS, obj.IDX_GBIAS) = eye(3) * obj.GyroBiasNoise * dt;

            % Small position noise (propagated from velocity)
            Q(obj.IDX_POS, obj.IDX_POS) = eye(3) * obj.AccelerometerNoise * dt^3 / 3;
        end
    end
end
