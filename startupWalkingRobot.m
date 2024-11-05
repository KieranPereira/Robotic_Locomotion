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
load("OnshapeTest\cheetah_trajectory.mat")

%% Loading custom data
custom_joint_data = readmatrix('ModelingSimulation\JointAnglesOnShape.csv');
default_model_joint_data = readmatrix('ModelingSimulation\JointAnglesDefault.csv');