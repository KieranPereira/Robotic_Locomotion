% Fore leg joint angles (19 moments)
% Columns: [Hip joint angle (degrees), Knee joint angle (degrees), Ankle joint angle (degrees)]
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
% Columns: [Hip joint angle (degrees), Knee joint angle (degrees), Ankle joint angle (degrees)]
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

% Time vectors for interpolation
original_points = linspace(0, 1, 19); % Original 19 moments
interpolated_points = linspace(0, 1, 100); % 100 points for interpolation

% Interpolation for fore leg joints
fore_hip = interp1(original_points, fore_leg_data(:, 1), interpolated_points);
fore_knee = interp1(original_points, fore_leg_data(:, 2), interpolated_points);
fore_ankle = interp1(original_points, fore_leg_data(:, 3), interpolated_points);

% Interpolation for hind leg joints
hind_hip = interp1(original_points, hind_leg_data(:, 1), interpolated_points);
hind_knee = interp1(original_points, hind_leg_data(:, 2), interpolated_points);
hind_ankle = interp1(original_points, hind_leg_data(:, 3), interpolated_points);

% Plotting the interpolated data
figure;
tiledlayout(2, 3); % 2 rows, 3 columns of subplots

% Fore leg plots
nexttile;
plot(interpolated_points, fore_hip, 'r', 'LineWidth', 1.5);
title('Fore Leg Hip');
xlabel('Normalized Time');
ylabel('Angle (degrees)');

nexttile;
plot(interpolated_points, fore_knee, 'g', 'LineWidth', 1.5);
title('Fore Leg Knee');
xlabel('Normalized Time');
ylabel('Angle (degrees)');

nexttile;
plot(interpolated_points, fore_ankle, 'b', 'LineWidth', 1.5);
title('Fore Leg Ankle');
xlabel('Normalized Time');
ylabel('Angle (degrees)');

% Hind leg plots
nexttile;
plot(interpolated_points, hind_hip, 'r', 'LineWidth', 1.5);
title('Hind Leg Hip');
xlabel('Normalized Time');
ylabel('Angle (degrees)');

nexttile;
plot(interpolated_points, hind_knee, 'g', 'LineWidth', 1.5);
title('Hind Leg Knee');
xlabel('Normalized Time');
ylabel('Angle (degrees)');

nexttile;
plot(interpolated_points, hind_ankle, 'b', 'LineWidth', 1.5);
title('Hind Leg Ankle');
xlabel('Normalized Time');
ylabel('Angle (degrees)');

% Optional: Display interpolated arrays in the console
disp('Interpolated Fore Leg Hip:');
disp(fore_hip);

disp('Interpolated Hind Leg Ankle:');
disp(hind_ankle);
