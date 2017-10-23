function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, qIMUToC, TIMUToC, LandMarksComputed)
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Truth Points
for j = 1:size(LandMarksComputed, 1)
   truth.points1{j} = Point3([LandMarksComputed(j,2), LandMarksComputed(j,3), 0]');
   truth.points2{j} = Point3([LandMarksComputed(j,4), LandMarksComputed(j,5), 0]');
   truth.points3{j} = Point3([LandMarksComputed(j,6), LandMarksComputed(j,7), 0]');
   truth.points4{j} = Point3([LandMarksComputed(j,8), LandMarksComputed(j,9), 0]');
   truth.pointID{j} = LandMarksComputed(j,1);
end

truth.K = Cal3_S2(K(1,1), K(2,2), 0, K(1,3), K(2,3));
data.K = truth.K;
cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Truth poses

for i = 1:size(DetAll, 2)
    c = 1;
    cc = 1;
    imagePoints = [];
    worldPoints = [];
    for j = 1:size(DetAll{1,i}, 1)
        IDPose2D = DetAll{1,i}(j,1);
        IDPose3D = find(LandMarksComputed(:,1)==IDPose2D);      
        if ~isempty(IDPose3D)
            worldPoints(cc,:) = [LandMarksComputed(IDPose3D(1),2), LandMarksComputed(IDPose3D(1),3), 0, IDPose2D];
            cc = cc + 1;
            worldPoints(cc,:) = [LandMarksComputed(IDPose3D(1),4), LandMarksComputed(IDPose3D(1),5), 0, IDPose2D];
            cc = cc + 1;
            worldPoints(cc,:) = [LandMarksComputed(IDPose3D(1),6), LandMarksComputed(IDPose3D(1),7), 0, IDPose2D];
            cc = cc + 1;
            worldPoints(cc,:) = [LandMarksComputed(IDPose3D(1),8), LandMarksComputed(IDPose3D(1),9), 0, IDPose2D];
            cc = cc + 1;
            for l = 2:2:8
                imagePoints(c,:) = [DetAll{1,i}(j,l), DetAll{1,i}(j,l+1), IDPose2D];
                c = c + 1;
            end
        end
    end
    [rot, loc] = estimateWorldCameraPose(imagePoints(:,1:2),worldPoints(:,1:3),cameraParams, 'MaxNumTrials' , 700, 'Confidence', 80,...
                        'MaxReprojectionError' , 3);
    
%     scatter3(loc(1,1), loc(1,2), loc(1,3));
%     hold on
    location(i,:) = loc;  
    rotation(i,:) = rotm2eul(rot);
end

%% Data images points
for i = 1:size(location, 1)
    truth.cameras{i} = SimpleCamera.Lookat(Point3(location(i,:)'), Point3, Point3(rotation(i,:)'), truth.K);
    c = 1;
    for j = 1:size(LandMarksComputed, 1)
        idx = find(DetAll{1,i}(:,1) == LandMarksComputed(j,1));
        if ~isempty(idx)
            data.Z1{i}{c} = Point2([DetAll{1,i}(idx,2), DetAll{1,i}(idx,3)]');
            data.Z2{i}{c} = Point2([DetAll{1,i}(idx,4), DetAll{1,i}(idx,5)]');
            data.Z3{i}{c} = Point2([DetAll{1,i}(idx,6), DetAll{1,i}(idx,7)]');
            data.Z4{i}{c} = Point2([DetAll{1,i}(idx,8), DetAll{1,i}(idx,9)]');
            data.J{i}{c} = DetAll{1,i}(idx,1);
            c = c + 1;
        end
    end
end

%% Odometry of data
for i = 1:size(location, 1) - 1
   odometry = truth.cameras{i}.pose.between(truth.cameras{i+1}.pose);
   data.odometry{i} = odometry;
end



%% Initialize iSAM with the first pose and points
% iSAM Options
options.hardConstraint = false;
options.pointPriors = false;
options.batchInitialization = true;
options.reorderInterval = 10;
options.alwaysRelinearize = false;

% Initialize iSAM
params = gtsam.ISAM2Params;
if options.alwaysRelinearize
    params.setRelinearizeSkip(1);
end
isam = ISAM2(params);

% Set Noise parameters
noiseModels.pose = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
%noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.05 0.05 0.2 0.2 0.2]');
noiseModels.point = noiseModel.Isotropic.Sigma(3, 0.1);
noiseModels.measurement = noiseModel.Isotropic.Sigma(2, 1.0);

%  Add constraints/priors
% TODO: should not be from ground truth!
newFactors = NonlinearFactorGraph;
initialEstimates = Values;
for i=1:2
    ii = symbol('x',i);
    if i==1
        if options.hardConstraint % add hard constraint
            newFactors.add(NonlinearEqualityPose3(ii,truth.cameras{1}.pose));
        else
            newFactors.add(PriorFactorPose3(ii,truth.cameras{i}.pose, noiseModels.pose));
        end
    end
    initialEstimates.insert(ii,truth.cameras{i}.pose);
end
nextPoseIndex = 3;

% Add visual measurement factors from two first poses and initialize observed landmarks
pointNoiseSigma = 0.00001;
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
for i=1:2
    ii = symbol('x',i);
    for k=1:length(data.Z1{i})
        j = data.J{i}{k};
        
        jj1 = symbol('p',data.J{i}{k});
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z1{i}{k}, noiseModels.measurement, ii, jj1, data.K));
        % TODO: initial estimates should not be from ground truth!
        idx = find(LandMarksComputed(:,1) == j);
        if ~initialEstimates.exists(jj1)
            initialEstimates.insert(jj1, truth.points1{idx});
        end
        if options.pointPriors % add point priors
            newFactors.add(PriorFactorPoint3(jj1, truth.points1{idx}, noiseModels.point));
        end
        
        jj2 = symbol('q',data.J{i}{k});
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z2{i}{k}, noiseModels.measurement, ii, jj2, data.K));
        % TODO: initial estimates should not be from ground truth!
        idx = find(LandMarksComputed(:,1) == j);
        if ~initialEstimates.exists(jj2)
            initialEstimates.insert(jj2, truth.points2{idx});
        end
        if options.pointPriors % add point priors
            newFactors.add(PriorFactorPoint3(jj2, truth.points2{idx}, noiseModels.point));
        end
        
        jj3 = symbol('r',data.J{i}{k});
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z3{i}{k}, noiseModels.measurement, ii, jj3, data.K));
        % TODO: initial estimates should not be from ground truth!
        idx = find(LandMarksComputed(:,1) == j);
        if ~initialEstimates.exists(jj3)
            initialEstimates.insert(jj3, truth.points3{idx});
        end
        if options.pointPriors % add point priors
            newFactors.add(PriorFactorPoint3(jj3, truth.points3{idx}, noiseModels.point));
        end
        
        jj4 = symbol('s',data.J{i}{k});
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z4{i}{k}, noiseModels.measurement, ii, jj4, data.K));
        % TODO: initial estimates should not be from ground truth!
        idx = find(LandMarksComputed(:,1) == j);
        if ~initialEstimates.exists(jj4)
            initialEstimates.insert(jj4, truth.points4{idx});
        end
        if options.pointPriors % add point priors
            newFactors.add(PriorFactorPoint3(jj4, truth.points4{idx}, noiseModels.point));
        end
        
        newFactors.add(BetweenFactorPoint3(jj1, jj2, Point3([TagSize 0 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj2, jj3, Point3([0 TagSize 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj3, jj4, Point3([-TagSize 0 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj4, jj1, Point3([0 -TagSize 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj1, jj3, Point3([TagSize TagSize 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj2, jj4, Point3([-TagSize TagSize 0]'),pointPriorNoise));
    end
end

% Add odometry between frames 1 and 2
newFactors.add(BetweenFactorPose3(symbol('x',1), symbol('x',2), data.odometry{1}, noiseModels.odometry));
% Update ISAM
if options.batchInitialization % Do a full optimize for first two poses
    batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initialEstimates);
    fullyOptimized = batchOptimizer.optimize();
    isam.update(newFactors, fullyOptimized);
else
    isam.update(newFactors, initialEstimates);
end

result = isam.calculateEstimate();
AllPosesComputed=[];
for i= 1:2
    AllPosesComputed=[AllPosesComputed;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z];
end
scatter3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
hold on


%% Main loop for iSAM: stepping through all poses
for i = 3:size(location, 1)
    i
    % iSAM expects us to give it a new set of factors 
    % along with initial estimates for any new variables introduced.
    newFactors = NonlinearFactorGraph;
    initialEstimates = Values;
    
    % Add odometry
    odometry = data.odometry{nextPoseIndex-1};
    newFactors.add(BetweenFactorPose3(symbol('x',nextPoseIndex-1), symbol('x',nextPoseIndex), odometry, noiseModels.odometry));

    %% Add visual measurement factors and initializations as necessary
    for k=1:length(data.Z1{nextPoseIndex})
        j = data.J{nextPoseIndex}{k};
        
        jj1 = symbol('p', j);
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z1{nextPoseIndex}{k}, noiseModels.measurement, symbol('x',nextPoseIndex), jj1, data.K));
        % TODO: initialize with something other than truth
        idx = find(LandMarksComputed(:,1) == j);
        if ~result.exists(jj1) && ~initialEstimates.exists(jj1)
            lmInit = truth.points1{idx};
            initialEstimates.insert(jj1, lmInit);
        end
        
        jj2 = symbol('q', j);
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z2{nextPoseIndex}{k}, noiseModels.measurement, symbol('x',nextPoseIndex), jj2, data.K));
        % TODO: initialize with something other than truth
        idx = find(LandMarksComputed(:,1) == j);
        if ~result.exists(jj2) && ~initialEstimates.exists(jj2)
            lmInit = truth.points2{idx};
            initialEstimates.insert(jj2, lmInit);
        end
        
        jj3 = symbol('r', j);
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z3{nextPoseIndex}{k}, noiseModels.measurement, symbol('x',nextPoseIndex), jj3, data.K));
        % TODO: initialize with something other than truth
        idx = find(LandMarksComputed(:,1) == j);
        if ~result.exists(jj3) && ~initialEstimates.exists(jj3)
            lmInit = truth.points3{idx};
            initialEstimates.insert(jj3, lmInit);
        end
        
        jj4 = symbol('s', j);
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z4{nextPoseIndex}{k}, noiseModels.measurement, symbol('x',nextPoseIndex), jj4, data.K));
        % TODO: initialize with something other than truth
        idx = find(LandMarksComputed(:,1) == j);
        if ~result.exists(jj4) && ~initialEstimates.exists(jj4)
            lmInit = truth.points4{idx};
            initialEstimates.insert(jj4, lmInit);
        end
        
        newFactors.add(BetweenFactorPoint3(jj1, jj2, Point3([TagSize 0 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj2, jj3, Point3([0 TagSize 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj3, jj4, Point3([-TagSize 0 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj4, jj1, Point3([0 -TagSize 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj1, jj3, Point3([TagSize TagSize 0]'), pointPriorNoise));
        newFactors.add(BetweenFactorPoint3(jj2, jj4, Point3([-TagSize TagSize 0]'),pointPriorNoise));
    end
    %% Initial estimates for the new pose.
    prevPose = result.at(symbol('x',nextPoseIndex-1));
    initialEstimates.insert(symbol('x',nextPoseIndex), prevPose.compose(odometry));

    %% Update ISAM

    isam.update(newFactors, initialEstimates);
    result = isam.calculateEstimate();
    
    AllPosesComputed=[AllPosesComputed;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z];
    scatter3(result.at(symbol('x',i)).x,result.at(symbol('x',i)).y,result.at(symbol('x',i)).z);

    nextPoseIndex = nextPoseIndex + 1;
end
% scatter3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));

end
