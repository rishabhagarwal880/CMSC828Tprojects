function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Data generation
dat.K = Cal3_S2(K(1,1), K(2,2), 0, K(1,3), K(2,3));
truth.K = dat.K;
totalLandmarks = DetAll{1,1}(:,1);
for i = 1:size(DetAll, 2)
   truth.cameras{i} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([0,0,1]'), truth.K);
   for j = 1:size(DetAll{1,i}, 1)
       % Only one of the corners of the Tags for now
       dat.Z{i}{j} = Point2(DetAll{1,i}(j,2), DetAll{1,i}(j,3));
       dat.J{i}{j} = DetAll{1,i}(j,1);
       totalLandmarks = union(totalLandmarks, DetAll{1,i}(j,1));
   end
end
for i = 1:size(totalLandmarks, 1)
    truth.landmarks{i} = Point3([1, 1, 1]');
end

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%% Create graph container and add factors to it from GTSAM library
graph_container = NonlinearFactorGraph;

%% Add factors for all measurements
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
for i = 1:length(dat.Z)
   for k = 1:length(dat.Z{i})
       j = dat.J{i}{k};
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',j), dat.K));
   end
end

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
graph_container.add(PriorFactorPose3(symbol('x',1), truth.cameras{1}.pose, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
graph_container.add(PriorFactorPoint3(symbol('p',10), Point3([0,0,0]'), pointPriorNoise));

%% Initialize cameras and points close to ground truth
initialEstimate = Values;
% Camera Poses
for i = 1:length(truth.cameras)
    estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    initialEstimate.insert(symbol('x',i), estimatePose);
end
% Landmarks
for i = 1:size(totalLandmarks, 1)
    estimateLandmark = truth.landmarks{i}.retract(0.1*randn(3,1));
    initialEstimate.insert(symbol('p',totalLandmarks(i,1)), estimateLandmark);
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Fine grain optimization, allowing user to iterate step by step
parameters = LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

optimizer = LevenbergMarquardtOptimizer(graph_container, initialEstimate, parameters);

for i=1:5
    optimizer.iterate();
end

result = optimizer.values();
%% Plot results with covariance ellipses
marginals = Marginals(graph_container, result);
cla
hold on;
plot3DPoints(result, [], marginals);
plot3DTrajectory(result, '*', 1, 8, marginals);

view(3)
colormap('hot')
end

