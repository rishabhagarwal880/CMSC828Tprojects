function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, LandMarksComputed, qIMUToC, TIMUToC, IMU, LeftImgs, TLeftImgs)
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Data generation
tic
dat.K = Cal3_S2(K(1,1), K(2,2), 0, K(1,3), K(2,3));
truth.K = dat.K;
m=1; Pose_lock=[]; Pose_rota=[]; eang=[];
cameraParams = cameraParameters('IntrinsicMatrix', K');
totalLandmarks = DetAll{1}(:,1);

for i = 1:size(DetAll, 2)
    truth.cameras{i} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([0,0,1]'), truth.K);
    for j = 1:size(DetAll{i}, 1)
        % Only one of the corners of the Tags for now
        dat.Z{i}{j,1} = Point2(DetAll{i}(j,2), DetAll{i}(j,3));
        dat.Z{i}{j,2} = Point2(DetAll{i}(j,4), DetAll{i}(j,5));
        dat.Z{i}{j,3} = Point2(DetAll{i}(j,6), DetAll{i}(j,7));
        dat.Z{i}{j,4} = Point2(DetAll{i}(j,8), DetAll{i}(j,9));
        dat.J{i}{j} = DetAll{i}(j,1);
        totalLandmarks = union(totalLandmarks, DetAll{i}(j,1));
    end
end

for i = 1:size(totalLandmarks, 1)
    truth.landmarks{i,1} = Point3([0, 0, 0]');
    truth.landmarks{i,2} = Point3([1, 0, 0]');
    truth.landmarks{i,3} = Point3([1, 1, 0]');
    truth.landmarks{i,4} = Point3([0, 1, 0]');
end

for j = 1:size(LandMarksComputed, 1)
   truth.points1{j} = Point3([LandMarksComputed(j,2), LandMarksComputed(j,3), 0]');
   truth.points2{j} = Point3([LandMarksComputed(j,4), LandMarksComputed(j,5), 0]');
   truth.points3{j} = Point3([LandMarksComputed(j,6), LandMarksComputed(j,7), 0]');
   truth.points4{j} = Point3([LandMarksComputed(j,8), LandMarksComputed(j,9), 0]');
   truth.pointID{j} = LandMarksComputed(j,1);
end
 %% New vision
% for a = 1:size(DetAll, 2)
%     for b=1:1%size(DetAll{a}, 1)
%         tagPoints = [DetAll{a}(b,2) DetAll{a}(b,3); DetAll{a}(b,4) DetAll{a}(b,5); ... % pixel points of global Id
%             DetAll{a}(b,6) DetAll{a}(b,7); DetAll{a}(b,8) DetAll{a}(b,9)];
%         refID=find(DetAll{a}(b,1)==LandMarksComputed(:,1));
%         homo1=[LandMarksComputed(refID,2),LandMarksComputed(refID,3),1;
%             LandMarksComputed(refID,4),LandMarksComputed(refID,5),1;
%             LandMarksComputed(refID,6),LandMarksComputed(refID,7),1;
%             LandMarksComputed(refID,8),LandMarksComputed(refID,9),1];
%         [crot,cloc]=estimateWorldCameraPose(tagPoints,homo1, cameraParams , 'MaxNumTrials',600,'Confidence',75,'MaxReprojectionError',7);
%     end
%     Pose_lock=[Pose_lock;cloc];
%     Pose_rota=[Pose_rota;crot];
%     eang=[eang;rotm2eul(crot)];
% end
% 
% % %% Odometry of data
% % for i = 1:size(location, 1) - 1
% %    odometry = truth.cameras{i}.pose.between(truth.cameras{i+1}.pose);
% %    dat.odometry{i} = odometry;
% % end

%% Create graph container and add factors to it from GTSAM library
graph_container = NonlinearFactorGraph;

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.01;
poseNoiseSigmas = [0.1 0.1 0.1 0.1 0.1 0.1]';

posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);

[M,N]=size(dat.Z);


X=uint64([]);
for i=1:N
        X(i)=symbol('x',i);  
end

ar_tag = uint64([]);
count=1;
for j=1:length(totalLandmarks)
    for a = 1:4
        ar_tag(totalLandmarks(j),a) = uint64(symbol('p',count)); 
        count = count + 1;          
     end
end
                                            % %% Create graph container and add factors to it from GTSAM library
                                            % graph_container = NonlinearFactorGraph;
                                            % % iSAM Options
                                            % options.hardConstraint = false;
                                            % options.pointPriors = false;
                                            % options.batchInitialization = true;
                                            % options.reorderInterval = 10;
                                            % options.alwaysRelinearize = false;
                                            % 
                                            % % Initialize iSAM
                                            % params = gtsam.ISAM2Params;
                                            % if options.alwaysRelinearize
                                            %     params.setRelinearizeSkip(1);
                                            % end
                                            % isam = ISAM2(params);
                                            % 
                                            % 
                                            % for i=1:1:length(dat.Z)
                                            %     Camera_positions(i)=symbol('x',i);
                                            % end
                                            % 
                                            % for j=1:length(totalLandmarks)
                                            %     p(totalLandmarks(j))=symbol('p',totalLandmarks(j));
                                            %     q(totalLandmarks(j))=symbol('q',totalLandmarks(j));
                                            %     r(totalLandmarks(j))=symbol('r',totalLandmarks(j));
                                            %     s(totalLandmarks(j))=symbol('s',totalLandmarks(j));
                                            % end

%% Add Gaussian priors for a pose and a landmark to constrain the system
graph_container.add(PriorFactorPose3(X(1), truth.cameras{1}.pose, posePriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,1), Point3([0 0 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,2), Point3([TagSize 0 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,3), Point3([TagSize TagSize 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,4), Point3([0 TagSize 0]'), pointPriorNoise));

% for i=1:length(LandMarksComputed)
%         graph_container.add(PriorFactorPoint3(ar_tag(LandMarksComputed(i,1),1), Point3([LandMarksComputed(i,2) LandMarksComputed(i,3) 0]'), pointPriorNoise));
%         graph_container.add(PriorFactorPoint3(ar_tag(LandMarksComputed(i,1),2), Point3([LandMarksComputed(i,4) LandMarksComputed(i,5) 0]'), pointPriorNoise));
%         graph_container.add(PriorFactorPoint3(ar_tag(LandMarksComputed(i,1),3), Point3([LandMarksComputed(i,6) LandMarksComputed(i,7) 0]'), pointPriorNoise));
%         graph_container.add(PriorFactorPoint3(ar_tag(LandMarksComputed(i,1),4), Point3([LandMarksComputed(i,8) LandMarksComputed(i,9) 0]'), pointPriorNoise));
% end

%% Initialize cameras and points close to ground truth
initialEstimate = Values;

                        % Camera Poses
                        % for i = 1:size(Pose_lock,1)
                        %    %eimatePose = Pose3([eang(i,1) ,eang(i,2) ,eang(i,3), Pose_lock(i,1), Pose_lock(i,2), Pose_lock(i,3)]);
                        %      estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
                        %     % estimatePose = Pose3([eang(i,1),eang(i,2),eang(i,3),Pose_loc(i,1),Pose_loc(i,2),Pose_loc(i,3)]);
                        %     initialEstimate.insert(X(i), estimatePose);
                        % end
                        % for i = 1:size(LandMarksComputed)
                        %     estimateLandmark1 = Point3([LandMarksComputed(i,2),LandMarksComputed(i,3),0]');%0.1*randn(3,1));
                        %     estimateLandmark2 = Point3([LandMarksComputed(i,4),LandMarksComputed(i,5),0]');%0.1*randn(3,1));
                        %     estimateLandmark3 = Point3([LandMarksComputed(i,6),LandMarksComputed(i,7),0]');%0.1*randn(3,1));
                        %     estimateLandmark4 = Point3([LandMarksComputed(i,8),LandMarksComputed(i,9),0]');%0.1*randn(3,1));
                        %     
                        % end

%% Add factors for all measurements

                        % %% Add Gaussian priors for a pose and a landmark to constrain the system
                        % graph_container.add(PriorFactorPose3(X(1), truth.cameras{1}.pose, posePriorNoise));
                        % graph_container.add(PriorFactorPoint3(ar_tag(10,1), Point3([0 0 0]'), pointPriorNoise));
                        % graph_container.add(PriorFactorPoint3(ar_tag(10,2), Point3([TagSize 0 0]'), pointPriorNoise));
                        % graph_container.add(PriorFactorPoint3(ar_tag(10,3), Point3([TagSize TagSize 0]'), pointPriorNoise));
                        % graph_container.add(PriorFactorPoint3(ar_tag(10,4), Point3([0 TagSize 0]'), pointPriorNoise));
                        % 
tag(1) = Point3([0, TagSize, 0]');
tag(2) = Point3([TagSize, 0, 0]');
tag(3) = Point3([0, -TagSize, 0]');
tag(4) = Point3([-TagSize, 0, 0]');
tag(5) = Point3([TagSize, TagSize, 0]');
tag(6) = Point3([-TagSize, TagSize,0]');

for c = 3:2:length(dat.Z)
        for i = c-2:1:c
                   for k = 1:length(dat.Z{i})
                       j = dat.J{i}{k};
                       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,1}, measurementNoise, X(i), uint64(ar_tag(j,1)), dat.K));
                       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,2}, measurementNoise, X(i), uint64(ar_tag(j,2)), dat.K));
                       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,3}, measurementNoise, X(i), uint64(ar_tag(j,3)), dat.K));
                       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,4}, measurementNoise, X(i), uint64(ar_tag(j,4)), dat.K));
                       graph_container.add(BetweenFactorPoint3(uint64(ar_tag(j,1)), uint64(ar_tag(j,2)), tag(2), pointPriorNoise));
                       graph_container.add(BetweenFactorPoint3(uint64(ar_tag(j,2)), uint64(ar_tag(j,3)), tag(1), pointPriorNoise));
                       graph_container.add(BetweenFactorPoint3(uint64(ar_tag(j,3)), uint64(ar_tag(j,4)), tag(4), pointPriorNoise));
                       graph_container.add(BetweenFactorPoint3(uint64(ar_tag(j,4)), uint64(ar_tag(j,1)), tag(3), pointPriorNoise));
                       graph_container.add(BetweenFactorPoint3(uint64(ar_tag(j,1)), uint64(ar_tag(j,3)), tag(5), pointPriorNoise));
                       graph_container.add(BetweenFactorPoint3(uint64(ar_tag(j,2)), uint64(ar_tag(j,4)), tag(6), pointPriorNoise));
                        tagID=find(DetAll{i}(k,1)==LandMarksComputed(:,1));
                        initialEstimate.insert(uint64(ar_tag(LandMarksComputed(tagID,1),1)), Point3([LandMarksComputed(tagID,2),LandMarksComputed(tagID,3),0]'));
                        initialEstimate.insert(uint64(ar_tag(LandMarksComputed(tagID,1),2)), Point3([LandMarksComputed(tagID,4),LandMarksComputed(tagID,5),0]'));
                        initialEstimate.insert(uint64(ar_tag(LandMarksComputed(tagID,1),3)), Point3([LandMarksComputed(tagID,6),LandMarksComputed(tagID,7),0]'));
                        initialEstimate.insert(uint64(ar_tag(LandMarksComputed(tagID,1),4)), Point3([LandMarksComputed(tagID,8),LandMarksComputed(tagID,9),0]'));
			if i~=1
                        	estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
                        	initialEstimate.insert(X(i), estimatePose);
			end
                   end
                   
                   if i~=length(dat.Z)
                        graph_container.add(BetweenFactorPose3(X(i),X(i+1), Pose3(Rot3(eye(3)), Point3([0 0 0]')), posePriorNoise));
                   end
        end
        batchOptimizer = LevenbergMarquardtOptimizer(graph_container, initialEstimate);
        fullyOptimized = batchOptimizer.optimize();
        isam.update(graph_container, fullyOptimized);
end

%% Fine grain optimization, allowing user to iterate step by step
%isam.update(graph_container, fullyOptimized);
result = isam.calculateEstimate();
disp(result);
%% Plot results with covariance ellipses
CameraP2=[];
for i= 1:size(Pose_lock,1)
    CameraP2=[CameraP2;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z];
end
AllPosesComputed=0;
toc
figure(4),
scatter3(CameraP2(:,1),CameraP2(:,2),CameraP2(:,3));
end
