function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC)
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Data generation
dat.K = Cal3_S2(K(1,1), K(2,2), 0, K(1,3), K(2,3));
truth.K = dat.K;
totalLandmarks = DetAll{1}(:,1);
for i = 1:size(DetAll, 2)
   truth.cameras{i} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([0,0,1]'), truth.K);
   for j = 1:size(DetAll{i}, 1)
       % Only one of the corners of the Tags for now
       dat.Z{i}{j,1} = Point2([DetAll{i}(j,2) DetAll{i}(j,3)]');
       dat.Z{i}{j,2} = Point2([DetAll{i}(j,4), DetAll{i}(j,5)]');
       dat.Z{i}{j,3} = Point2([DetAll{i}(j,6), DetAll{i}(j,7)]');
       dat.Z{i}{j,4} = Point2([DetAll{i}(j,8), DetAll{i}(j,9)]');
       dat.J{i}{j} = DetAll{i}(j,1);
       truth.landmarks{i,1} = Point3([DetAll{i}(j,2) DetAll{i}(j,3) 0]');
       truth.landmarks{i,2} = Point3([DetAll{i}(j,4) DetAll{i}(j,5) 0]');
       truth.landmarks{i,3} = Point3([DetAll{i}(j,6) DetAll{i}(j,7) 0]');
       truth.landmarks{i,4} = Point3([DetAll{i}(j,8) DetAll{i}(j,9) 0]');
       totalLandmarks = union(totalLandmarks, DetAll{i}(j,1));
   end
end

% for i = 1:size(DetAll, 2)
%    for j = 1:size(DetAll{i}, 1)
%        dat.Z{i}{j,1} = truth.cameras{i}.project(Point3([DetAll{i}(j,2) DetAll{i}(j,3) 0]'));
%        dat.Z{i}{j,2} = truth.cameras{i}.project(Point3([DetAll{i}(j,4) DetAll{i}(j,5) 0]'));  
%        dat.Z{i}{j,3} = truth.cameras{i}.project(Point3([DetAll{i}(j,6) DetAll{i}(j,7) 0]'));     
%        dat.Z{i}{j,4} = truth.cameras{i}.project(Point3([DetAll{i}(j,8) DetAll{i}(j,9) 0]'));            
% 
%    end
% end
measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%% Create graph container and add factors to it from GTSAM library
graph_container = NonlinearFactorGraph;

for i=1:1:length(dat.Z)
        x(i)=symbol('x',i);  
end

for j=1:length(totalLandmarks)
    p(totalLandmarks(j))=symbol('p',totalLandmarks(j));
    q(totalLandmarks(j))=symbol('q',totalLandmarks(j));
    r(totalLandmarks(j))=symbol('r',totalLandmarks(j));
    s(totalLandmarks(j))=symbol('s',totalLandmarks(j));
end

%% Add factors for all measurements
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
for i = 1:length(dat.Z)
   for k = 1:length(dat.Z{i})
       j = dat.J{i}{k};
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,1}, measurementNoise, symbol('x',i), symbol('p',j), dat.K));
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,2}, measurementNoise, symbol('x',i), symbol('q',j), dat.K));
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,3}, measurementNoise, symbol('x',i), symbol('r',j), dat.K));
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,4}, measurementNoise, symbol('x',i), symbol('s',j), dat.K));

   end
end
%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
graph_container.add(PriorFactorPose3(symbol('x',1), truth.cameras{1}.pose, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
graph_container.add(PriorFactorPoint3(symbol('p',10), Point3([0 0 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('q',10), Point3([TagSize 0 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('r',10), Point3([TagSize TagSize 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('s',10), Point3([TagSize 0 0]'), pointPriorNoise));

% Add Gaussian priors for a pose and a landmark to constrain the system
% cameraPriorNoise  = noiseModel.Diagonal.Sigmas(cameraNoiseSigmas);
% firstCamera = SimpleCamera(truth.cameras{1}.pose, truth.K);
% graph.add(PriorFactorSimpleCamera(symbol('c',1), firstCamera, cameraPriorNoise));
% 
% pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
% graph.add(PriorFactorPoint3(symbol('p',1), truth.points{1}, pointPriorNoise));
%% Constraints
% PoseNoise = noiseModel.Diagonal.Sigmas([2*0.1; 2*0.1]);
tag(1) = Point3([TagSize 0 0]');
tag(2) = Point3([0 TagSize 0]');
tag(3) = Point3([-TagSize 0 0]');
tag(4) = Point3([0 -TagSize 0]');
tag(5) = Point3([TagSize TagSize 0]');
tag(6) = Point3([-TagSize TagSize 0]');

for i = 1:length(totalLandmarks)
        graph_container.add(BetweenFactorPoint3(p(totalLandmarks(i)), q(totalLandmarks(i)), tag(1), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(q(totalLandmarks(i)), r(totalLandmarks(i)), tag(2), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(r(totalLandmarks(i)), s(totalLandmarks(i)), tag(3), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(s(totalLandmarks(i)), p(totalLandmarks(i)), tag(4), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(p(totalLandmarks(i)), r(totalLandmarks(i)), tag(5), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(q(totalLandmarks(i)), s(totalLandmarks(i)), tag(6), pointPriorNoise));
end
graph_container.print(sprintf('\nFactor graph:\n'));

%% Initialize cameras and points close to ground truth
initialEstimate = Values;
% Camera Poses
for i = 1:length(truth.cameras)
    estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    initialEstimate.insert(symbol('x',i), estimatePose);
end
% Landmarks
for i = 1:length(totalLandmarks)
    estimateLandmark(1) = truth.landmarks{i,1}.retract(0.1*[randn(2,1)' 0]');
    estimateLandmark(2) = truth.landmarks{i,2}.retract(0.1*[randn(2,1)' 0]');
    estimateLandmark(3) = truth.landmarks{i,3}.retract(0.1*[randn(2,1)' 0]');
    estimateLandmark(4) = truth.landmarks{i,4}.retract(0.1*[randn(2,1)' 0]');
    initialEstimate.insert(p(totalLandmarks(i)), estimateLandmark(1));
    initialEstimate.insert(q(totalLandmarks(i)), estimateLandmark(2));
    initialEstimate.insert(r(totalLandmarks(i)), estimateLandmark(3));
    initialEstimate.insert(s(totalLandmarks(i)), estimateLandmark(4));
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
disp(result);
for i = 1:size(totalLandmarks, 1)
       scatter3(result.at(symbol('x',totalLandmarks(i))).x, result.at(symbol('p',totalLandmarks(i))).y, result.at(symbol('p',totalLandmarks(i))).z)
       hold on;
end
graph_container.print(sprintf('\nFactor graph:\n'));

% Plot results with covariance ellipses
% marginals = Marginals(graph_container, result);
% cla
% hold on;
% plot3DPoints(result, [], marginals);
% plot3DTrajectory(result, '*', 1, 81, marginals);
% 
% view(3)
% colormap('hot')
end
