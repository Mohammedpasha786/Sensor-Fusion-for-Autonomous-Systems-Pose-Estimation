function imu = imuModel(params)
% IMUMODEL  Configure an imuSensor object (accelerometer + gyroscope)
%   from the sensor_params.yaml configuration.
%
%   imu = imuModel(params)
%
%   Input:
%     params - sensor parameters struct (from loadConfig), expects
%              params.imu.sample_rate, .accelerometer, .gyroscope
%
%   Output:
%     imu    - configured imuSensor object (Navigation/Sensor Fusion Toolbox)
%
%   Usage:
%     imu = imuModel(sensorParams);
%     [accelReading, gyroReading] = imu(trueAccel, trueAngVel, trueOrientation);

    imuParams = params.imu;

    imu = imuSensor('accel-gyro', 'SampleRate', imuParams.sample_rate);

    %% ── Accelerometer ────────────────────────────────────────────────────
    accelParams = imuParams.accelerometer;
    imu.Accelerometer.MeasurementRange   = 19.62;     % m/s^2 (~2g)
    imu.Accelerometer.Resolution         = 0.0006;     % m/s^2 / LSB
    imu.Accelerometer.NoiseDensity       = accelParams.noise_density;
    imu.Accelerometer.BiasInstability    = accelParams.bias_instability;
    imu.Accelerometer.ConstantBias       = accelParams.constant_bias;

    %% ── Gyroscope ────────────────────────────────────────────────────────
    gyroParams = imuParams.gyroscope;
    imu.Gyroscope.MeasurementRange   = 4.36;       % rad/s (~250 deg/s)
    imu.Gyroscope.Resolution         = 0.00013;     % rad/s / LSB
    imu.Gyroscope.NoiseDensity       = gyroParams.noise_density;
    imu.Gyroscope.BiasInstability    = gyroParams.bias_instability;
    imu.Gyroscope.ConstantBias       = gyroParams.constant_bias;

    fprintf('  IMU model configured: %.0f Hz, accel noise=%.4f, gyro noise=%.4f\n', ...
            imuParams.sample_rate, accelParams.noise_density, gyroParams.noise_density);
end
