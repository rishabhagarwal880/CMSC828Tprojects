%% Wrapper for P2Ph1 for CMSC828T Course at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox'et
addpath('gtsam_toolbox');

%% Load DatagGTSAM
% Download data from the following link: 
% https://drive.google.com/drive/folders/0B6NXn7BBGQf5MS0tUGR0Nno0Nk0
load('DataMapping.mat','DetAll','Pose');
load('CalibParams.mat');

%% SLAM Using GTSAM
[LandMarksComputed,AllPosesComputed] = SLAMusingGTSAM_temp(DetAll, K, TagSize, qIMUToC, TIMUToC);
                                            
                                            
%% Localization usin iSAM2
AllPosesComputed = LocalizationUsingiSAM2_ri(DetAll, K, TagSize, LandMarksComputed);