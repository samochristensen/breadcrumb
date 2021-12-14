function [ hfigs ] = plotMonteCarlo(errors, traj_ref, traj, simpar)
%PLOTMONTECARLO_GPSINS creates hair plots of the estimation error for each
%state.  I left example code so you can see how I normally make these
%plots.
[n, ~, ~] = size(errors);
hfigs = [];
%% Plot estimation errors
ylabels = {'E Position Est Err (m)',...
    'N Position Est Err (m)',...
    'U Position Est Err (m)',...
    'E Velocity Est Err (m/s)',...
    'N Velocity Est Err (m/s)',...
    'U Velocity Est Err (m/s)',...
    'E Attitude Est Err (rad)',...
    'N Attitude Est Err (rad)',...
    'U Attitude Est Err (rad)',...
    'X_b Accel Bias Est Err (m/s^2)',...
    'Y_b Accel Bias Est Err (m/s^2)',...
    'Z_b Accel Bias Est Err (m/s^2)',...
    'X_b Gyro Bias Est Err (rad/s)',...
    'Y_b Gyro Bias Est Err (rad/s)',...
    'Z_b Gyro Bias Est Err (rad/s)',...
    'E IBC Position Est Err (m)',...
    'N IBC Position Est Err (m)',...
    'U IBC Position Est Err (m)'};
for i=1:n
    
    hfigs(end + 1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    hold on;
    grid on;
    ensemble = squeeze(errors(i,:,:));
    filter_cov = squeeze(traj_ref.navCov(i,i,:));
    h_hair = stairs(traj_ref.time_nav, ensemble,'Color',[0.8 0.8 0.8]);
    h_filter_cov = stairs(traj_ref.time_nav, ...
        [3*sqrt(filter_cov) -3*sqrt(filter_cov)],'--r');
    legend([h_hair(1), h_filter_cov(1)],'MC run','EKF cov')
    xlabel('time(s)')
    ylabel(ylabels{i})
end
end
