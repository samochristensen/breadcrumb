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

%% Plot 3x1 trajectory
x = traj.truthState(simpar.states.ix.position(2),:)';
y = traj.truthState(simpar.states.ix.position(1),:)';
z = zeros(length(x));
x_estimate = traj.navState(simpar.states.ixf.position(2),:)';
y_estimate = traj.navState(simpar.states.ixf.position(1),:)';
z_estimate = traj.navState(simpar.states.ixf.position(3),:)';

h_figs(end+1) = figure; hold on;
subplot(311);
x_true = plot(x, 'LineWidth', 2); hold on;
x_est = plot(x_estimate, 'LineWidth', 2);
legend('x_{true}', 'x_{est}'); ylabel('X'); hold off;
subplot(312);
y_true = plot(y, 'LineWidth', 2); hold on;
y_est = plot(y_estimate, 'LineWidth', 2);
legend('y_{true}', 'y_{est}'); ylabel('Y'); hold off;
subplot(313);
z_true = plot(z, 'LineWidth', 2); hold on;
z_est = plot(z_estimate, 'LineWidth', 2);
legend('z_{true}', 'z_{est}'); ylabel('Z'); hold off;
hold off;
grid on;

return;

%% Plot Velocity vs. Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.truthState(simpar.states.ix.velocity,:));
title("Vehicle Velocity vs. Time");
xlabel('Time(s)');
ylabel('Velocity(m/s)');
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

%% Orientation, True
h_figs(end+1) = figure;
nav_state_orient = quat2eul(traj.navState(simpar.states.ixf.attitude,:)')';
tru_state_headin = traj.truthState(simpar.states.ix.heading,:);

subplot(411); plot(traj.time_nav, nav_state_orient(1,:)); ylabel('roll?');
subplot(412); plot(traj.time_nav, nav_state_orient(2,:)); ylabel('pitch?');
subplot(413); plot(traj.time_nav, nav_state_orient(3,:)); ylabel('yaw?'); 
subplot(414); plot(traj.time_nav, tru_state_headin); ylabel('tru heading');

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
plot(traj.time_nav, traj.continuous_measurements(1:3,:));
title('Accelerometer Measurements vs. Time');
xlabel('Time(s)');
ylabel('(m/s^2)');
legend('x', 'y', 'z');
grid on;
%% Accelerometer Bias vs Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.navState(simpar.states.ixf.accl_bias,:));
title('Accelerometer Bias vs. Time');
xlabel('Time(s)');
ylabel('(m/s^2)');
legend('x', 'y', 'z');
grid on;

%% Gyro Measurement vs Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.continuous_measurements(4:6,:));
title('Gyro Measurement vs time');
xlabel('Time(s)');
ylabel('(rad/s)');
legend('x', 'y', 'z');
grid on;

%% Gyro Bias vs Time
h_figs(end+1) = figure;
plot(traj.time_nav, traj.navState(simpar.states.ixf.accl_bias,:));
title('Gyro Bias vs time');
xlabel('Time(s)');
ylabel('(rad/s)');
legend('x', 'y', 'z');
grid on;
end