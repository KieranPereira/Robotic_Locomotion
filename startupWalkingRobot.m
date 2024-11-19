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
        genpath('Libraries'));                  % Other dependencies
        genpath('CAD Files')

%% Load basic robot parameters from modeling and simulation example
robotParameters
run('CAD Files\Importing_CAD.m')

% Only run if CAD model has changed on OnShape
%run("OnshapeTest\Onshape_Test.m")
run("OnshapeTest\Assembly1_DataFile.m")
load("Gait Cycles\cheetah_trajectory.mat")

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

%loading Cheetah Gait Cycles (will need to replace with your path):
%load('jAngs_cheetah_10_rad.mat');
load('jAngs_cheetah_55_rad.mat');
load('quadruped_walking_gait_data.mat');
