% data from https://www.sciencedirect.com/science/article/pii/S2667379721000334#appA
%clc,clear

% Parameters
n = 5;                  % Interpolation points per gait
tt = 1;                   % Total time for the motion (seconds)
t_stationary = 0;          % Time at the start where the robot is stationary (seconds)
steps = 1;                % Total number of steps
t_gait = (tt - t_stationary) / steps; % Period for each gait cycle (seconds)
phase_offset=0.5;
% Fore leg joint angles (19 moments)
fore_leg_data = [
    -135.15, 27.49, -11.49;
    -154.67, 27.08, -31.33;
    -169.29, 45.25, -51.57;
    -178.41, 67.56, -72.61;
    -186.48, 91.50, -89.01;
    -190.26, 109.99, -100.40;
    -186.05, 123.83, -112.25;
    -176.81, 132.31, -119.91;
    -165.18, 135.66, -123.72;
    -151.81, 136.11, -123.09;
    -137.32, 133.38, -116.44;
    -119.06, 124.79, -99.55;
    -100.48, 110.35, -75.98;
    -82.99, 88.28, -35.06;
    -65.69, 58.51, 1.27;
    -50.75, 20.04, 15.72;
    -60.38, 25.94, 7.80;
    -81.39, 25.95, 4.50;
    -110.00, 30.51, -2.64
];

% Hind leg joint angles (19 moments)
hind_leg_data = [
    -110.07, -43.59, 39.35;
    -125.11, -48.07, 36.26;
    -130.46, -53.90, 29.53;
    -116.37, -88.06, 31.88;
    -101.57, -112.11, 38.27;
    -85.76, -128.43, 67.40;
    -71.20, -134.32, 85.82;
    -55.72, -136.46, 99.24;
    -42.22, -135.97, 113.11;
    -28.40, -134.84, 126.60;
    -13.99, -135.23, 127.85;
    -1.54, -134.67, 118.17;
    7.99, -130.12, 106.86;
    13.94, -118.94, 92.70;
    10.52, -102.23, 80.91;
    1.70, -84.00, 68.78;
    -10.28, -64.01, 52.58;
    -39.60, -33.34, 35.47;
    -73.93, -45.34, 49.04
];

% Interpolation for trajectories
original_points = linspace(0, 1, 19); % Original moments
interpolated_points = linspace(0, 1, n);

% Fore leg interpolation
fore_hip = interp1(original_points, fore_leg_data(:, 1), interpolated_points);
fore_knee = interp1(original_points, fore_leg_data(:, 2), interpolated_points);
fore_ankle = interp1(original_points, fore_leg_data(:, 3), interpolated_points);

% Hind leg interpolation
hind_hip = interp1(original_points, hind_leg_data(:, 1), interpolated_points);
hind_knee = interp1(original_points, hind_leg_data(:, 2), interpolated_points);
hind_ankle = interp1(original_points, hind_leg_data(:, 3), interpolated_points);

% Time vectors
t_gait_points = linspace(0, t_gait, n); % Time for one gait cycle
t_total = linspace(0, tt, n * steps + n * (t_stationary / t_gait)); % Total time with stationary period

% Fore and Hind leg trajectories (per gait cycle)
fore_leg_traj = [t_gait_points', fore_hip'+90, -fore_knee'];
hind_leg_traj = [t_gait_points', hind_hip'+90, -hind_knee'];

%phase offset
if steps ~= 1
    hind_leg_traj(:,2:3)=circshift(hind_leg_traj(:,2:3), phase_offset*n,1);
end

% Initialize arrays for entire sequence
jAngsF_cheetah = zeros(length(t_total), 3);
jAngsH_cheetah = zeros(length(t_total), 3);

% Assign time vectors
jAngsF_cheetah(:, 1) = t_total;
jAngsH_cheetah(:, 1) = t_total;

% Stationary phase
stationary_frames = n * (t_stationary / t_gait);
jAngsF_cheetah(1:stationary_frames, 2:3) = repmat(fore_leg_traj(1, 2:3), stationary_frames, 1);
jAngsH_cheetah(1:stationary_frames, 2:3) = repmat(hind_leg_traj(1, 2:3), stationary_frames, 1);

% Repeated gait cycles
for step = 1:steps
    start_idx = stationary_frames + (step - 1) * n + 1;
    end_idx = stationary_frames + step * n;
    jAngsF_cheetah(start_idx:end_idx, 2:3) = fore_leg_traj(:, 2:3);
    jAngsH_cheetah(start_idx:end_idx, 2:3) = hind_leg_traj(:, 2:3);
end

% Convert to radians
jAngsF_cheetah_rad = jAngsF_cheetah;
jAngsH_cheetah_rad = jAngsH_cheetah;
jAngsF_cheetah_rad(:, 2:3) = deg2rad(jAngsF_cheetah_rad(:, 2:3));
jAngsH_cheetah_rad(:, 2:3) = deg2rad(jAngsH_cheetah_rad(:, 2:3));



%% Save data
save('cheetah_traj_haha.mat', 'jAngsF_cheetah', 'jAngsH_cheetah', 'jAngsF_cheetah_rad', 'jAngsH_cheetah_rad');

if steps==1 %for single gait cycle trajectories (making waypoints for optimization code)
    load('cheetah_traj_haha.mat')
    hip_motion_F=jAngsF_cheetah_rad(:,2)';
    knee_motion_F=jAngsF_cheetah_rad(:,3)'; %ignore left as left is just right traj with phase offset
    hip_motion_H=jAngsH_cheetah_rad(:,2)';
    knee_motion_H=jAngsH_cheetah_rad(:,3)'; %ignore left as left is just right traj with phase offset
    traj_times=jAngsH_cheetah_rad(:,1)';
    save('cheetah_waypoints.mat','hip_motion_F','knee_motion_F','hip_motion_H','knee_motion_H', 'traj_times')
end
%% plot
figure('Name','waypoints')
subplot(2,1,1);
plot(jAngsF_cheetah_rad(:,1), jAngsF_cheetah_rad(:,2), 'LineWidth', 2); 
hold on;
plot(jAngsH_cheetah_rad(:,1), jAngsH_cheetah_rad(:,2), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Fore', 'Hind');
title(['Hip Angles: Running (', num2str(n), ' waypoints)']);
grid on;
subplot(2,1,2);
plot(jAngsF_cheetah_rad(:,1), jAngsF_cheetah_rad(:,3), 'LineWidth', 2); 
hold on;
plot(jAngsH_cheetah_rad(:,1), jAngsH_cheetah_rad(:,3), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Fore', 'Hind');
title(['Knee Angles: Running (', num2str(n), ' waypoints)']);
grid on;