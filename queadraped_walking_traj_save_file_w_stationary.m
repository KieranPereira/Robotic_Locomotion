%Equation from https://pmc.ncbi.nlm.nih.gov/articles/PMC8512058/pdf/sensors-21-06366.pdf

clc,clear
% Parameters
t_total = 5;              % Total time for the cycle (seconds)
num_steps = 5;             % Number of steps in 10 seconds
samples_per_leg = 500;    % Number of time steps
qH = 0.3;                  % Scaling factor for hind leg
qF = 0.0;                  % Scaling factor for fore leg
v = 1;                     % Constant gait speed in m/s
stationary_time = 0;       % Time for stationary phase (seconds), can be set to 0

% Time sequence offsets for each leg [fore left, hind left, fore right, hind right]
time_offsets = [0, 0.75, 0.5, 0.25];
leg_order = {'fore_left', 'hind_left', 'fore_right', 'hind_right'};

% Prepare data structure
gait_data = struct();

% Generate time vectors
if stationary_time > 0
    t_stationary = linspace(0, stationary_time, samples_per_leg * stationary_time / t_total)';
else
    t_stationary = []; % No stationary phase if stationary_time is zero
end
t_motion = linspace(0, t_total - stationary_time, samples_per_leg)';  % Adjust motion time if stationary time is zero

% Calculate phase angle based on number of steps
phi = (2 * pi * num_steps / t_total) * t_motion;

% Scaling factors
sF = qF * v + 1;
sH = qH * v + 1;

% Define the joint angle functions for the fore and hind legs
f1 = @(phi) -0.3 + sF * (0.4 * cos(phi) + 0.1 * cos(2*phi + 0.6));
f2 = @(phi) 0.45 + sF * (0.3 * cos(phi + 1.1) + 0.2 * cos(2*phi + 2.9));
f4 = @(phi) -0.2 + sH * (0.24 * cos(phi - 0.2) + 0.03 * cos(2*phi + 0.9));
f5 = @(phi) 0.4 + sH * (0.1 * cos(phi + 1.7) + 0.1 * cos(2*phi - 2.9));


% Loop over each leg and generate trajectories
for i = 1:length(leg_order)
    % Apply phase offset for each leg
    phi_offset = phi + time_offsets(i) * 2 * pi * num_steps;
    
    % Compute hip and knee angles during motion
    if contains(leg_order{i}, 'fore')
        hip_angle_motion = f1(phi_offset); 
        knee_angle_motion = f2(phi_offset);
        hip_angle_stationary = f1(0+time_offsets(i) * 2 * pi * num_steps) * ones(size(t_stationary));
        knee_angle_stationary = f2(0+time_offsets(i) * 2 * pi * num_steps) * ones(size(t_stationary));
    else
        hip_angle_motion = f4(phi_offset); 
        knee_angle_motion = f5(phi_offset);
        hip_angle_stationary = f4(0+time_offsets(i) * 2 * pi * num_steps)  * ones(size(t_stationary));
        knee_angle_stationary = f5(0+time_offsets(i) * 2 * pi * num_steps) * ones(size(t_stationary));
    end
    
    % Combine stationary and motion phases
    if isempty(t_stationary)
        % No stationary phase, only motion
        t_combined = t_motion;
        hip_angle_combined = hip_angle_motion;
        knee_angle_combined = knee_angle_motion;
    else
        % Stationary phase followed by motion phase
        t_combined = [t_stationary; t_motion + t_stationary(end)];
        hip_angle_combined = [hip_angle_stationary; hip_angle_motion];
        knee_angle_combined = [knee_angle_stationary; knee_angle_motion];
    end
    
    % Store time, hip, and knee angles in radians for each leg
    gait_data.(leg_order{i}) = [t_combined, hip_angle_combined, knee_angle_combined];
end

%% Save to .mat file
 save('walking_traj_CF.mat', '-struct', 'gait_data');

if num_steps==1 %for single gait cycle trajectories (making waypoints for optimization code)
    load('quadruped_gait_waypoints.mat')
    hip_motion_F=fore_right(:,2);
    knee_motion_F=fore_right(:,3); %ignore left as leftis just right traj with phase offset
    hip_motion_H=hind_right(:,2);
    knee_motion_H=hind_right(:,3);
    save('walking_waypoints.mat','hip_motion_F','knee_motion_F','hip_motion_H','knee_motion_H')
end
%% plot
figure('Name','waypoints')
subplot(2,1,1);
plot(gait_data.fore_left(:,1), gait_data.fore_left(:,2), 'LineWidth', 2); 
hold on;
plot(gait_data.hind_left(:,1), gait_data.hind_left(:,2), 'LineWidth', 2);
plot(gait_data.fore_right(:,1), gait_data.fore_right(:,2), 'LineWidth', 2);
plot(gait_data.hind_right(:,1), gait_data.hind_right(:,2), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('fore left', 'hind left', 'fore right', 'hind right');
title(['Hip Angles: Trotting (', num2str(samples_per_leg), ' waypoints)']);
grid on;
subplot(2,1,2);
plot(gait_data.fore_left(:,1), gait_data.fore_left(:,3), 'LineWidth', 2); 
hold on;
plot(gait_data.hind_left(:,1), gait_data.hind_left(:,3), 'LineWidth', 2);
plot(gait_data.fore_right(:,1), gait_data.fore_right(:,3), 'LineWidth', 2);
plot(gait_data.hind_right(:,1), gait_data.hind_right(:,3), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('fore left', 'hind left', 'fore right', 'hind right');
title(['Knee Angles: Trotting (', num2str(samples_per_leg), ' waypoints)']);
grid on;