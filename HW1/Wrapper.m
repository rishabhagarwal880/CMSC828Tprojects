%% Wrapper for HW1 for CMSC828T Course at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
addpath('gtsam_toolbox');

%% Load Data
load('HW1.mat');

%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(Odom, ObservedLandMarks, StartingPose);