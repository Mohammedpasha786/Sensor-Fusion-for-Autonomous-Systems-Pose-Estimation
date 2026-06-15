function saveResultsCSV(results, outFile)
% SAVERESULTSCSV  Write performance metrics for all fusion methods to CSV.
%
%   saveResultsCSV(results, outFile)
%
%   Inputs:
%     results - struct with .ekf and/or .lstm, each containing .metrics
%               (from computeMetrics) and .runtime
%     outFile - full path for output CSV file

    [outDir, ~, ~] = fileparts(outFile);
    if ~isfolder(outDir), mkdir(outDir); end

    methods = fieldnames(results);
    metricFields = {'posRMSE', 'rollRMSE', 'pitchRMSE', 'yawRMSE', 'ATE', 'RPE', 'maxError', 'runtime'};
    metricLabels = {'Position RMSE (m)', 'Roll RMSE (deg)', 'Pitch RMSE (deg)', ...
                    'Yaw RMSE (deg)', 'ATE (m)', 'RPE (m/s)', 'Max Position Error (m)', ...
                    'Runtime (s)'};

    fid = fopen(outFile, 'w');

    % Header
    fprintf(fid, 'Metric');
    for m = 1:numel(methods)
        fprintf(fid, ',%s', upper(methods{m}));
    end
    fprintf(fid, '\n');

    % Rows
    for f = 1:numel(metricFields)
        fprintf(fid, '%s', metricLabels{f});
        for m = 1:numel(methods)
            method = methods{m};
            val = NaN;
            if strcmp(metricFields{f}, 'runtime') && isfield(results.(method), 'runtime')
                val = results.(method).runtime;
            elseif isfield(results.(method), 'metrics') && isfield(results.(method).metrics, metricFields{f})
                val = results.(method).metrics.(metricFields{f});
            end
            fprintf(fid, ',%.6f', val);
        end
        fprintf(fid, '\n');
    end

    fclose(fid);
    fprintf('  Performance metrics CSV saved: %s\n', outFile);
end
