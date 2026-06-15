function plotErrorAnalysis(results, groundTruth, outDir)
% PLOTERRORANALYSIS  Plot position and orientation error over time for
%   each fusion method, plus a summary bar chart of RMSE metrics.
%
%   plotErrorAnalysis(results, groundTruth, outDir)
%
%   Inputs:
%     results     - struct with .ekf and/or .lstm, each containing
%                   .poses and .metrics (from computeMetrics)
%     groundTruth - struct with .time, .pos, .quat
%     outDir      - output directory for saved figure

    fig = figure('Name', 'Error Analysis', 'Position', [150 150 1100 800], 'Visible', 'off');

    colors = struct('ekf', [0.13 0.45 0.85], 'lstm', [0.85 0.33 0.10]);
    labels = struct('ekf', 'insEKF', 'lstm', 'LSTM Fusion');
    methods = fieldnames(results);

    %% ── Panel 1: Position error over time ───────────────────────────────
    ax1 = subplot(2, 2, 1);
    hold on;
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'metrics')
            plot(results.(m).poses.time, results.(m).metrics.errorOverTime, ...
                 'Color', colors.(m), 'LineWidth', 1.3, 'DisplayName', labels.(m));
        end
    end
    xlabel('Time (s)'); ylabel('Position error (m)');
    title('Position Error over Time', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best'); grid on;

    %% ── Panel 2: Position error histogram ────────────────────────────────
    ax2 = subplot(2, 2, 2);
    hold on;
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'metrics')
            histogram(results.(m).metrics.errorOverTime, 30, ...
                      'FaceColor', colors.(m), 'FaceAlpha', 0.5, ...
                      'DisplayName', labels.(m));
        end
    end
    xlabel('Position error (m)'); ylabel('Count');
    title('Position Error Distribution', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best'); grid on;

    %% ── Panel 3: RMSE summary bar chart ───────────────────────────────────
    ax3 = subplot(2, 2, 3);
    metricNames = {'posRMSE', 'yawRMSE', 'ATE', 'RPE'};
    metricLabels = {'Pos. RMSE (m)', 'Yaw RMSE (°)', 'ATE (m)', 'RPE (m/s)'};

    barData = [];
    legendNames = {};
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'metrics')
            row = zeros(1, numel(metricNames));
            for j = 1:numel(metricNames)
                row(j) = results.(m).metrics.(metricNames{j});
            end
            barData = [barData; row]; %#ok<AGROW>
            legendNames{end+1} = labels.(m); %#ok<AGROW>
        end
    end

    if ~isempty(barData)
        b = bar(ax3, barData');
        for i = 1:numel(legendNames)
            if isfield(colors, methods{i})
                b(i).FaceColor = colors.(methods{i});
            end
        end
        set(ax3, 'XTickLabel', metricLabels);
        ylabel('Value');
        title('Performance Summary', 'FontSize', 11, 'FontWeight', 'bold');
        legend(legendNames, 'Location', 'best');
        grid on;
    end

    %% ── Panel 4: Orientation (yaw) comparison ──────────────────────────────
    ax4 = subplot(2, 2, 4);
    hold on;
    gtEul = quat2eul(groundTruth.quat, 'ZYX') * 180/pi;
    plot(groundTruth.time, gtEul(:,1), 'k-', 'LineWidth', 1.8, 'DisplayName', 'Ground Truth Yaw');
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'poses')
            estEul = quat2eul(results.(m).poses.quat, 'ZYX') * 180/pi;
            plot(results.(m).poses.time, estEul(:,1), '--', 'Color', colors.(m), ...
                 'LineWidth', 1.3, 'DisplayName', [labels.(m) ' Yaw']);
        end
    end
    xlabel('Time (s)'); ylabel('Yaw (degrees)');
    title('Yaw Angle over Time', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best'); grid on;

    sgtitle('Sensor Fusion — Error Analysis', 'FontSize', 13, 'FontWeight', 'bold');

    %% ── Save ─────────────────────────────────────────────────────────────
    if ~isfolder(fullfile(outDir, 'plots')), mkdir(fullfile(outDir, 'plots')); end
    outFile = fullfile(outDir, 'plots', 'error_analysis.png');
    saveas(fig, outFile);
    fprintf('  Error analysis plot saved: %s\n', outFile);
    close(fig);
end
