function h_figs = plotNavPropErrors(traj)
% In this function, you should plot everything you want to see for a
% single simulation when there are no errors present.  I like to see things
% like the 3D trajectory, altitude, angular rates, etc.  Two sets of plots
% that are required are the estimation errors for every state and the
% residuals for any measurements.  I left some of my code for a lunar
% lander application, so you can see how I do it.  Feel free to use or
% remove whatever applies to your problem.
%% Prelims
h_figs = [];
simpar = traj.simpar;
m2km = 1/1000;
%% Plot trajectory
h_figs(end+1) = figure;
x = traj.truthState(simpar.states.ix.position(2),:)';
y = traj.truthState(simpar.states.ix.position(1),:)';

htraj = plot(x, y, 'LineWidth',2);
hold all
xlabel('East (m)');
ylabel('North (m)');

hstart = scatter(traj.truthState(simpar.states.ix.position(2),1)',...
                 traj.truthState(simpar.states.ix.position(1),1)',...
                 'filled','g');
hstop =  scatter(traj.truthState(simpar.states.ix.position(2),end)',...
                 traj.truthState(simpar.states.ix.position(1),end)',...
                 'filled','r');
x_estimate = traj.navState(simpar.states.ixf.position(2),:)';
y_estimate = traj.navState(simpar.states.ixf.position(1),:)';
z_estimate = traj.navState(simpar.states.ixf.position(3),:)';

htraj = plot3(x_estimate,y_estimate,z_estimate,'-.');
hold off;
grid on;

%% Plot Velocity vs. Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.truthState(simpar.states.ix.velocity,:));
title("Vehicle Velocity vs. Time");
xlabel('Time(s)');
ylabel('Velocity(m/s)');
legend('x','y');
grid on;

%% Heading and Steering Angle vs. Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.truthState(simpar.states.ix.heading));
hold all;
plot(traj.time_nav,traj.truthState(simpar.states.ix.steer_ang,:));
title('Heading and Steering Angle vs. Time');
xlabel('time(s)');
ylabel('(deg)');
legend('psi','phi');
grid on;

%% IBC Position vs. Time
h_figs(end+1) = figure;
plot(traj.time_nav,traj.navState(simpar.states.ixf.crumb_pos,:));
title('IBC Position vs Time');
xlabel('Time(s)');
ylabel('Position(m)');
legend('x','y','z');
grid on;

%% Accelerometer Measurements vs Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.navState(simpar.states.ixf.accl_bias,:));
title('Accelerometer Measurements vs. Time');
xlabel('Time(s)');
ylabel('(m/s^2)');
legend('x', 'y', 'z');
grid on;

%% Gyro Measurement vs Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.navState(simpar.states.ixf.accl_bias,:));
title('Gyro Measurement vs time');
xlabel('Time(s)');
ylabel('(rad/s)');
legend('x', 'y', 'z');
grid on;

% %% Example residuals
% h_figs(end+1) = figure;
% stairs(traj.time_kalman,traj.navRes.example'); hold on
% xlabel('Time(s)')
% ylabel('Star Tracker Residuals(rad)')
% legend('x_{st}','y_{st}','z_{st}')
% grid on;
% %% Calculate estimation errors
% dele = calcErrors(traj.navState, traj.truthState, simpar);
% %% Plot position estimation error
% h_figs(end+1) = figure;
% stairs(traj.time_nav, dele(simpar.states.ixfe.pos,:)');
% title('Position Error');
% xlabel('time(s)');
% ylabel('m');
% legend('$x_i$','$y_i$','$z_i$')
% grid on;
% %% Plot velocity error
% h_figs(end+1) = figure;
% stairs(traj.time_nav, dele(simpar.states.ixfe.vel,:)');
% title('Velocity Error');
% xlabel('time(s)');
% ylabel('m/s');
% legend('x_i','y_i','z_i')
% grid on;
% %% Add the remaining estimation error plots
end