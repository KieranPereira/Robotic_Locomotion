% Walking Robot Startup Script
%
% Copyright 2017-2019 The MathWorks, Inc.

%% Clear everything
clc
clear
close all

%% Add folders to the path
addpath(genpath('LIPM'), ...                    % Linear inverted pendulum model (LIPM) files
        genpath('ModelingSimulation'), ...      % Modeling and simulation files
        genpath('Optimization'), ...            % Optimization files
        genpath('ControlDesign'), ...           % Control design files
        genpath('ReinforcementLearning'), ...   % Reinforcement learning files
        genpath('OnshapeTest'), ... 
        genpath('Libraries'), ...               % Other dependencies
        genpath('CAD Files'), ... 
        genpath('DDPG Neural Net'), ...
        genpath('Gait Cycles'));

%% Load basic robot parameters from modeling and simulation example
robotParameters
run('CAD Files\Importing_CAD.m')

% Only run if CAD model has changed on OnShape
%run("OnshapeTest\Onshape_Test.m")
run("OnshapeTest\Assembly1_DataFile.m");



%% Loading custom data
custom_joint_data = readmatrix('Gait Cycles\JointAnglesOnShape.csv');
default_model_joint_data = readmatrix('Gait Cycles\JointAnglesDefault_Left.csv');


%% Gait Cycle Loading
% Loading Cheetah Gait Cycle
data = load('jointAngs.mat','jAngsL');
writematrix(data.jAngsL, 'Gait Cycles\DefaultLeft.csv');
data = load('jointAngs.mat','jAngsR');
writematrix(data.jAngsR, 'Gait Cycles\DefaultRight.csv');

load("jointAngs.mat");

Default_left_data = jAngsL(:, [1, 4, 5]);
Default_right_data = jAngsR(:, [1, 4, 5]);

default_left_data_time = Default_left_data(:,1);
default_left_data_hip = Default_left_data(:,2);
default_left_data_ankle = Default_left_data(:,3);

default_right_data_time = Default_right_data(:,1);
default_right_data_hip = Default_right_data(:,2);
default_right_data_ankle = Default_right_data(:,3);




%%
% Define leg weights
%% Open the Simscape Multibody Assembly
blade_stiffness = 21500;
half_leg_weight = 3.5;
body_weight = 25;
friction_coefficient = 0.9;
back_leg_weight = 3.5;
gravity = 9.81;

%loading Cheetah Gait Cycles (will need to replace with your path):
%load('jAngs_cheetah_10_rad.mat');
%load('jAngs_cheetah_55_rad.mat');
load('walking_traj_stationary2.mat');

%For Optimization
load('walking_waypoints.mat');
%load('cheetah_waypoints.mat');

% Define blade stiffness
%blade_stiffness = 10;    % Example stiffness value in N/m

% Define gravity (assuming Earth's gravity)
gravity = 9.81;    % Gravity vector in m/s^2 (pointing downward)

% Optionally, define other parameters as needed for the model

