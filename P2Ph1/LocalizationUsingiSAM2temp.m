function AllPosesComputed = LocalizationUsingiSAM2temp(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, TLeftImgs, LandMarksComputed);
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Initialize iSAM
params = gtsam.ISAM2Params;
% if options.alwaysRelinearize
%     params.setRelinearizeSkip(1);
% end
isam = ISAM2(params);

for j = 1:size(LandMarksComputed, 1)
   truth.points1{j} = Point3([LandMarksComputed(j,2), LandMarksComputed(j,3), 0]');
   truth.points2{j} = Point3([LandMarksComputed(j,4), LandMarksComputed(j,5), 0]');
   truth.points3{j} = Point3([LandMarksComputed(j,6), LandMarksComputed(j,7), 0]');
   truth.points4{j} = Point3([LandMarksComputed(j,8), LandMarksComputed(j,9), 0]');
   truth.pointID{j} = LandMarksComputed(j,1);
end

dat.K = Cal3_S2(K(1,1), K(2,2), 0, K(1,3), K(2,3));
truth.K = dat.K;
totalLandmarks = LandMarksComputed(:,1);
for i = 1:size(DetAll, 2)
   truth.cameras{i} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([0,0,1]'), truth.K);
   for j = 1:size(DetAll{i}, 1)
       % Only one of the corners of the Tags for now
       dat.Z{i}{j,1} = Point2([DetAll{i}(j,2), DetAll{i}(j,3)]');
       dat.Z{i}{j,2} = Point2([DetAll{i}(j,4), DetAll{i}(j,5)]');
       dat.Z{i}{j,3} = Point2([DetAll{i}(j,6), DetAll{i}(j,7)]');
       dat.Z{i}{j,4} = Point2([DetAll{i}(j,8), DetAll{i}(j,9)]');
       dat.J{i}{j} = DetAll{i}(j,1);
%        truth.landmarks{i,1} = Point3([DetAll{i}(j,2) DetAll{i}(j,3) 0]');
%        truth.landmarks{i,2} = Point3([DetAll{i}(j,4) DetAll{i}(j,5) 0]');
%        truth.landmarks{i,3} = Point3([DetAll{i}(j,6) DetAll{i}(j,7) 0]');
%        truth.landmarks{i,4} = Point3([DetAll{i}(j,8) DetAll{i}(j,9) 0]');
%        totalLandmarks = union(totalLandmarks, DetAll{i}(j,1));
   end
end
%% Set Noise parameters
noiseModels.pose = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
%noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.05 0.05 0.2 0.2 0.2]');
noiseModels.point = noiseModel.Isotropic.Sigma(3, 0.1);
noiseModels.measurement = noiseModel.Isotropic.Sigma(2, 1.0);
%% Add constraints/priors
% TODO: should not be from ground truth!
newFactors = NonlinearFactorGraph;
initialEstimates = Values;

for i=1:1:length(dat.Z)
        x(i)=symbol('x',i);  
end

for j=1:length(totalLandmarks)
    p(totalLandmarks(j))=symbol('p',totalLandmarks(j));
    q(totalLandmarks(j))=symbol('q',totalLandmarks(j));
    r(totalLandmarks(j))=symbol('r',totalLandmarks(j));
    s(totalLandmarks(j))=symbol('s',totalLandmarks(j));
end


newFactors.add(PriorFactorPose3(symbol('x',1), truth.cameras{1}.pose, noiseModels.pose));
newFactors.add(PriorFactorPoint3(symbol('p',10), Point3([0 0 0]'), noiseModels.point));
newFactors.add(PriorFactorPoint3(symbol('q',10), Point3([TagSize 0 0]'), noiseModels.point));
newFactors.add(PriorFactorPoint3(symbol('r',10), Point3([TagSize TagSize 0]'), noiseModels.point));
newFactors.add(PriorFactorPoint3(symbol('s',10), Point3([TagSize 0 0]'), noiseModels.point));

% for i=1:2
%     ii = symbol('x',i);
%     if i==1
%         if options.hardConstraint % add hard constraint
%             newFactors.add(NonlinearEqualityPose3(ii,truth.cameras{1}.pose));
%         else
%             newFactors.add(PriorFactorPose3(ii,truth.cameras{i}.pose, noiseModels.pose));
%         end
%     end
%     initialEstimates.insert(ii,truth.cameras{i}.pose);
% end
% 
% nextPoseIndex = 3;

%% Add visual measurement factors from two first poses and initialize observed landmarks

for i = 1:length(dat.Z)
   for k = 1:length(dat.Z{i})
       j = dat.J{i}{k};
       newFactors.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,1}, noiseModels.measurement, symbol('x',i), symbol('p',j), dat.K));
       newFactors.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,2}, noiseModels.measurement, symbol('x',i), symbol('q',j), dat.K));
       newFactors.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,3}, noiseModels.measurement, symbol('x',i), symbol('r',j), dat.K));
       newFactors.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,4}, noiseModels.measurement, symbol('x',i), symbol('s',j), dat.K));
   end
end
initialEstimates = Values;
for a = 1:length(dat.Z)
     estimatePose = truth.cameras{a}.pose.retract(0.1*randn(6,1));
    initialEstimates.insert(x(a),estimatePose);
end

for i=1:length(totalLandmarks)
    initialEstimates.insert(p(totalLandmarks(i)), Point3([LandMarksComputed(1,2) LandMarksComputed(1,3)  0]'));
    initialEstimates.insert(q(totalLandmarks(i)), Point3([LandMarksComputed(1,4) LandMarksComputed(1,5)  0]'));
    initialEstimates.insert(r(totalLandmarks(i)), Point3([LandMarksComputed(1,6) LandMarksComputed(1,7)  0]'));
    initialEstimates.insert(s(totalLandmarks(i)), Point3([LandMarksComputed(1,8) LandMarksComputed(1,9)  0]'));
end

% for i=1:2
%     ii = symbol('x',i);
%     for k=1:length(data.Z{i})
%         j = data.J{i}{k};
%         jj = symbol('l',data.J{i}{k});
%         newFactors.add(GenericProjectionFactorCal3_S2(data.Z{i}{k}, noiseModels.measurement, ii, jj, data.K));
%         % TODO: initial estimates should not be from ground truth!
%         if ~initialEstimates.exists(jj)
%             initialEstimates.insert(jj, truth.points{j});
%         end
%         if options.pointPriors % add point priors
%             newFactors.add(PriorFactorPoint3(jj, truth.points{j}, noiseModels.point));
%         end
%     end
% end

%% Add odometry between frames 1 and 2
%newFactors.add(BetweenFactorPose3(symbol('x',1), symbol('x',2), data.odometry{1}, noiseModels.odometry));
tag(1) = Point3([TagSize 0 0]');
tag(2) = Point3([0 TagSize 0]');
tag(3) = Point3([-TagSize 0 0]');
tag(4) = Point3([0 -TagSize 0]');
tag(5) = Point3([TagSize TagSize 0]');
tag(6) = Point3([-TagSize TagSize 0]');

for i = 1:length(totalLandmarks)
        newFactors.add(BetweenFactorPoint3(p(totalLandmarks(i)), q(totalLandmarks(i)), tag(1), noiseModels.point));
        newFactors.add(BetweenFactorPoint3(q(totalLandmarks(i)), r(totalLandmarks(i)), tag(2), noiseModels.point));
        newFactors.add(BetweenFactorPoint3(r(totalLandmarks(i)), s(totalLandmarks(i)), tag(3), noiseModels.point));
        newFactors.add(BetweenFactorPoint3(s(totalLandmarks(i)), p(totalLandmarks(i)), tag(4), noiseModels.point));
        newFactors.add(BetweenFactorPoint3(p(totalLandmarks(i)), r(totalLandmarks(i)), tag(5), noiseModels.point));
        newFactors.add(BetweenFactorPoint3(q(totalLandmarks(i)), s(totalLandmarks(i)), tag(6), noiseModels.point));
end
%% Update ISAM
% if options.batchInitialization % Do a full optimize for first two poses
    batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initialEstimates);
    fullyOptimized = batchOptimizer.optimize();
    isam.update(newFactors, fullyOptimized);
% else
%     isam.update(newFactors, initialEstimates);
% end
% figure(1);tic;
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.calculateEstimate();
% t=toc; plot(frame_i,t,'g.');


end
