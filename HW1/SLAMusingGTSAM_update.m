function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(Odom, ObservedLandMarks, StartingPose)
% INPUTS:
% Odom: Each column has the values of odometry between two time steps,
% eg.,  Odom(:,1) has the odometry between t=0 (starting position) and t=1
% ObservedLandMarks is a cell array of structures where
% each cell has a structure.
% Here each structure has Locations array of size
% (NumberOfObservedLocations x 2) and a vector Idx of size (1 x
% NumberOfObservedLocations)
% Each row of ObservedLandMarks{count}.Locations is x and y observed
% co-ordinates
% StartingPose is a 3 x 1 array of starting pose in [x; y; theta]
% OUTPUTS:
% AllPosesComputed: Array of size 3 x NumberOfSteps+1, the first column
% will be the pose at t=0
% LandMarksComputed: Each row has [ID, LocX, LocY], Note that LandMarkIDs
% have to be sorted in ascending order

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples
end
