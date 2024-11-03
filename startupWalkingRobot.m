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

%% Loading custom data
custom_joint_data = readmatrix('ModelingSimulation\JointAnglesOnShape.csv');
%load('C:\Users\kiera\OneDrive\Documents\Berkeley\ME 239- Robotic Locomotion\Project\Current\OnshapeTest\cheetah_trajectory.mat')
