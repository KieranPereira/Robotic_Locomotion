%% Quadruped robot parameters

% Copyright 2019-2022 The MathWorks Inc.

% Body and leg geometry
L = 1;        % Distance b/w front and rear hip joints (m)
L_back = 1.5; % Length of the torso (m)
l1 = 0.5517;  % Link #1 length (m)
l2 = 0.5517;  % Link #2 length (m)

% Robot mass
M = 20;      % Torso mass (kg)
m1 = 2;   % Leg link #1 mass (kg)
m2 = 2;   % Leg link #2 mass (kg)

% Moment of inertia (kg-m^2)
Ixx = 1/12 * M * ((0.1*L_back)^2 + (0.1*L_back)^2);
Iyy = 1/12 * M * (L_back^2 + (0.1*L_back)^2);
Izz = 1/12 * M * (L_back^2 + (0.1*L_back)^2);
torso_MOI = [Ixx, Iyy, Izz];

% Gravity (m/s^2)
g = -9.81;

% Sample Time (s)
Ts = 0.025;

% Simulation Time (s)
Tf = 25;

% Desired height of the torso (m)
h_final = 0.75;

% Contact bias
contact_bias = 5;

% Initial body height and foot displacement (m)
init_foot_disp_x = 0;
init_body_height = h_final;

% Initial joint angles (rad) and velocities (rad/s)
d2r = pi/180;
init_ang_FL = d2r * quadrupedInverseKinematics(init_foot_disp_x,-init_body_height,l1,l2);
init_ang_FR = init_ang_FL;
init_ang_RL = -init_ang_FL;
init_ang_RR = -init_ang_FL;
init_whip_FL = 0;
init_whip_FR = 0;
init_whip_RL = 0;
init_whip_RR = 0;

% initial height (m)
foot_height = 0.05*l2*(1-sin(2*pi-(3*pi/2+init_ang_FL(1)+init_ang_FL(2))));
y_init = init_body_height + foot_height;

% Initial body speeds in x,y (m/s)
vx_init = 0;
vy_init = 0;

% Contact friction properties
mu_kinetic = 0.93;
mu_static = 0.95;
v_thres = 0.001;

% Ground properties
ground.stiffness = 12500;
ground.damping = 1e2;
ground.length = 100;
ground.width = 1;
ground.depth = 0.05;

% Hip and Knee Joint properties
joint.stiffness = 0;
joint.damping = 8;
joint.limitStiffness = 500;
joint.limitDamping = 50;
joint.transitionWidth = 2 * d2r;
hip_eq_angle = 0;
knee_eq_angle = 0;

% Define limits on variables
u_max = 50;                        % max joint torque = +/- u_max
y_min = 0.5;                       % min height of body from ground
z_max = 0.5;                       % max translation in z
vx_max = 50;                      % max horizontal speed of body
vy_max = 10;                      % max vertical speed of body
vz_max = 2.5;                      % max lateral speed of body
roll_max = 10 * d2r;               % max roll angle of body
pitch_max = 40 * d2r;              % max pitch angle of body
yaw_max = 20 * d2r;                % max yaw angle of body
omega_x_max = pi/2;                % max angular speed about x
omega_y_max = pi/2;                % max angular speed about x
omega_z_max = pi/2;                % max angular speed about x

q_hip_min_F = -120 * d2r;            % hip and knee joint angle limit      
q_hip_min_R = -120 * d2r;            

q_hip_max_F = 45 * d2r;
q_hip_max_R = 120 * d2r;

q_knee_min_F = 0 * d2r;
q_knee_min_R = -60 * d2r;

q_knee_max_F = 180 * d2r;
q_knee_max_R = 0 * d2r;

w_max = 2*pi*120/60;                % hip and knee joint angular speed limit
y_max = l1 + l2+ l1;                    % max height of body from ground
normal_force_max = ((M+4*m1+4*m2)*abs(g))/4;
friction_force_max = mu_static * normal_force_max;