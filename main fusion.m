%  MAIN_FUSION.M — Sensor Fusion for Autonomous Systems
%  Vehicle Pose Estimation via insEKF or LSTM Fusion
%
%  Usage:
%    >> main_fusion                                          % defaults
%    >> main_fusion('dataset','euroc','dataPath','./data/raw/euroc/MH_01_easy/')
%    >> main_fusion('dataset','simulate','platform','quadcopter')
%    >> main_fusion('method','lstm')
%
%  Required Toolboxes:
%    Navigation Toolbox, Sensor Fusion and Tracking Toolbox,
%    UAV Toolbox (for simulation), Deep Learning Toolbox (for LSTM)
% =========================================================================

function main_fusion(varargin)

clear; clc; close all;
addpath(genpath(fileparts(mfilename('fullpath'))));

fprintf('=======================================================\n');
fprintf('  Sensor Fusion — Autonomous Vehicle Pose Estimation\n');
fprintf('  Extended Kalman Filter (insEKF) + LSTM Comparison\n');
fprintf('=======================================================\n\n');

%% ── Parse arguments ──────────────────────────────────────────────────────
p = inputParser;
addParameter(p, 'dataset',   'simulate',     @ischar);  % 'euroc','kitti','ansfl','simulate'
addParameter(p, 'dataPath',  './data/raw/',  @ischar);
addParameter(p, 'platform',  'quadcopter',   @ischar);  % 'quadcopter','ground'
addParameter(p, 'method',    'ekf',          @ischar);  % 'ekf','lstm','both'
addParameter(p, 'dropout',   false,          @islogical);
addParameter(p, 'configDir', './configs/',   @ischar);
parse(p, varargin{:});
cfg = p.Results;

%% ── Load configuration ───────────────────────────────────────────────────
ekfParams    = loadConfig(fullfile(cfg.configDir, 'ekf_params.yaml'));
sensorParams = loadConfig(fullfile(cfg.configDir, 'sensor_params.yaml'));

%% ── Stage 1: Data loading / simulation ──────────────────────────────────
fprintf('[Stage 1] Preparing sensor data (%s)...\n', cfg.dataset);

switch lower(cfg.dataset)
    case 'euroc'
        data = loadEuRoC(cfg.dataPath);
    case 'kitti'
        data = loadKITTI(cfg.dataPath);
    case 'ansfl'
        data = loadANSFL(cfg.dataPath);
    case 'simulate'
        [data, groundTruth] = simulateTrajectory(cfg.platform, sensorParams);
    otherwise
        error('Unknown dataset: %s', cfg.dataset);
end

if ~strcmp(cfg.dataset, 'simulate')
    groundTruth = data.groundTruth;
end

fprintf('  Duration    : %.1f s\n', data.time(end) - data.time(1));
fprintf('  IMU rate    : %.0f Hz\n', 1 / median(diff(data.imu.time)));
fprintf('  GPS rate    : %.1f Hz\n', 1 / median(diff(data.gps.time)));
fprintf('  Samples     : %d IMU, %d GPS\n', size(data.imu.accel,1), size(data.gps.pos,1));

%% ── Stage 2: Inject sensor dropout (optional) ────────────────────────────
if cfg.dropout
    fprintf('[Stage 2] Injecting sensor dropout scenarios...\n');
    data = injectDropout(data, 'gps', 'duration', 30, 'startTime', 60);
    data = injectDropout(data, 'imu', 'type', 'spike', 'magnitude', 5);
    fprintf('  GPS outage: 60–90 s | IMU spike injected\n');
else
    fprintf('[Stage 2] Sensor dropout: disabled\n');
end

%% ── Stage 3: Run fusion filter(s) ───────────────────────────────────────
results = struct();

if ismember(lower(cfg.method), {'ekf', 'both'})
    fprintf('[Stage 3a] Running insEKF...\n');
    tic;
    results.ekf = runEKF(data, ekfParams);
    results.ekf.runtime = toc;
    fprintf('  ✓ EKF complete in %.2f s\n', results.ekf.runtime);
end

if ismember(lower(cfg.method), {'lstm', 'both'})
    fprintf('[Stage 3b] Running LSTM fusion...\n');
    lstmParams = loadConfig(fullfile(cfg.configDir, 'lstm_params.yaml'));
    tic;
    results.lstm = runLSTMFusion(data, lstmParams);
    results.lstm.runtime = toc;
    fprintf('  ✓ LSTM complete in %.2f s\n', results.lstm.runtime);
end

%% ── Stage 4: Evaluate ────────────────────────────────────────────────────
fprintf('[Stage 4] Computing performance metrics...\n');

if isfield(results, 'ekf')
    results.ekf.metrics  = computeMetrics(results.ekf.poses,  groundTruth);
    printMetrics('insEKF', results.ekf.metrics);
end
if isfield(results, 'lstm')
    results.lstm.metrics = computeMetrics(results.lstm.poses, groundTruth);
    printMetrics('LSTM',   results.lstm.metrics);
end

%% ── Stage 5: Save & visualize ────────────────────────────────────────────
fprintf('[Stage 5] Saving results...\n');
outDir = './results';

% 3D trajectory plot
plotTrajectory(results, groundTruth, outDir);

% Error-over-time analysis
plotErrorAnalysis(results, groundTruth, outDir);

% Save metrics CSV
saveResultsCSV(results, fullfile(outDir, 'metrics', 'performance.csv'));

fprintf('\n✓ Pipeline complete. Results saved to: %s\n', outDir);
end

% ── Helper ────────────────────────────────────────────────────────────────
function printMetrics(label, m)
    fprintf('\n  ── %s Metrics ───────────────────────────\n', label);
    fprintf('  Pos. RMSE  : %.4f m\n',   m.posRMSE);
    fprintf('  Yaw RMSE   : %.4f deg\n', m.yawRMSE);
    fprintf('  ATE        : %.4f m\n',   m.ATE);
    fprintf('  RPE        : %.4f m/s\n', m.RPE);
    fprintf('  ─────────────────────────────────────────\n');
end
