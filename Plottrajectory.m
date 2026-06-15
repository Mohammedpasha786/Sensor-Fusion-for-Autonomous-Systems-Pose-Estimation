function plotTrajectory(results, groundTruth, outDir)
% PLOTTRAJECTORY  Plot 3D trajectory comparison: estimate(s) vs ground truth.
%
%   plotTrajectory(results, groundTruth, outDir)
%
%   Inputs:
%     results     - struct, may contain .ekf.poses and/or .lstm.poses,
%                   each with fields .time, .pos [N x 3]
%     groundTruth - struct with .time, .pos [M x 3]
%     outDir      - output directory for saved figure

    fig = figure('Name', '3D Trajectory', 'Position', [100 100 1100 850], 'Visible', 'off');

    %% ── 3D plot ──────────────────────────────────────────────────────────
    ax1 = subplot(2, 2, [1 3]);
    hold on;

    % Ground truth (NED -> plot as North=X, East=Y, -Down=Z(up))
    plot3(groundTruth.pos(:,1), groundTruth.pos(:,2), -groundTruth.pos(:,3), ...
          'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');

    colors = struct('ekf', [0.13 0.45 0.85], 'lstm', [0.85 0.33 0.10]);
    labels = struct('ekf', 'insEKF', 'lstm', 'LSTM Fusion');

    methods = fieldnames(results);
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'poses')
            p = results.(m).poses.pos;
            plot3(p(:,1), p(:,2), -p(:,3), '--', 'LineWidth', 1.5, ...
                  'Color', colors.(m), 'DisplayName', labels.(m));
        end
    end

    % Start/end markers
    scatter3(groundTruth.pos(1,1), groundTruth.pos(1,2), -groundTruth.pos(1,3), ...
             80, 'g', 'filled', 'DisplayName', 'Start');
    scatter3(groundTruth.pos(end,1), groundTruth.pos(end,2), -groundTruth.pos(end,3), ...
             80, 'r', 'filled', 'DisplayName', 'End');

    xlabel('North (m)'); ylabel('East (m)'); zlabel('Up (m)');
    title('3D Trajectory: Estimated vs Ground Truth', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on; axis equal; view(45, 30);

    %% ── Top-down (North-East) view ──────────────────────────────────────
    ax2 = subplot(2, 2, 2);
    hold on;
    plot(groundTruth.pos(:,2), groundTruth.pos(:,1), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'poses')
            p = results.(m).poses.pos;
            plot(p(:,2), p(:,1), '--', 'LineWidth', 1.5, 'Color', colors.(m), ...
                 'DisplayName', labels.(m));
        end
    end
    xlabel('East (m)'); ylabel('North (m)');
    title('Top-Down View (North-East Plane)', 'FontSize', 11);
    legend('Location', 'best'); grid on; axis equal;

    %% ── Altitude over time ──────────────────────────────────────────────
    ax3 = subplot(2, 2, 4);
    hold on;
    plot(groundTruth.time, -groundTruth.pos(:,3), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    for i = 1:numel(methods)
        m = methods{i};
        if isfield(results.(m), 'poses')
            p = results.(m).poses;
            plot(p.time, -p.pos(:,3), '--', 'LineWidth', 1.5, 'Color', colors.(m), ...
                 'DisplayName', labels.(m));
        end
    end
    xlabel('Time (s)'); ylabel('Altitude (m)');
    title('Altitude over Time', 'FontSize', 11);
    legend('Location', 'best'); grid on;

    sgtitle('Sensor Fusion — Trajectory Comparison', 'FontSize', 13, 'FontWeight', 'bold');

    %% ── Save ─────────────────────────────────────────────────────────────
    if ~isfolder(fullfile(outDir, 'plots')), mkdir(fullfile(outDir, 'plots')); end
    outFile = fullfile(outDir, 'plots', 'trajectory_3d.png');
    saveas(fig, outFile);
    fprintf('  Trajectory plot saved: %s\n', outFile);
    close(fig);
end
