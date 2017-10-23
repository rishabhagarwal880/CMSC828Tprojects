function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC, IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the project pdf
%AllPosesComputed
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
cameraPar = cameraParameters('IntrinsicMatrix', K');
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

measurementNoiseSigma = 1.0;
%pointNoiseSigma = 0.1;
pointNoiseSigma = 0.00001;

%poseNoiseSigmas = [0.00 0.00 0.00 0 0 0]';
poseNoiseSigmas = [0.01 0.01 0.01 0.01 0.01 0.01]';

%% Create graph container and add factors to it from GTSAM library
graph_container = NonlinearFactorGraph;
[M,N]=size(dat.Z);
X=uint64([]);
for i=1:N
        X(i)=symbol('x',i);  
end


ar_tag = uint64([]);
count=1;
for j=1:length(totalLandmarks)
%     p(totalLandmarks(j))=symbol('p',totalLandmarks(j));
%     q(totalLandmarks(j))=symbol('q',totalLandmarks(j));
%     r(totalLandmarks(j))=symbol('r',totalLandmarks(j));
%     s(totalLandmarks(j))=symbol('s',totalLandmarks(j));
   %ar_tag(j,1) = totalLandmarks(j);
    for a = 1:4
        ar_tag(totalLandmarks(j),a) = uint64(symbol('p',count)); 
        count = count + 1;          
     end
end
%% Add factors for all measurements
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
%blah=1;
for i = 1:N 
   for k = 1:length(dat.Z{i})
       j = dat.J{i}{k};
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,1}, measurementNoise, X(i), uint64(ar_tag(j,1)), dat.K));
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,2}, measurementNoise, X(i), uint64(ar_tag(j,2)), dat.K));
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,3}, measurementNoise, X(i), uint64(ar_tag(j,3)), dat.K));
       graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,4}, measurementNoise, X(i), uint64(ar_tag(j,4)), dat.K));
   end
% %    
%    for k = 1:length(dat.Z{i})
%        j = dat.J{i}{k};
%        graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,1}, measurementNoise, X(i), ar_tag(j,1), dat.K));
%        graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,2}, measurementNoise, X(i), ar_tag(j,2), dat.K));
%        graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,3}, measurementNoise, X(i), ar_tag(j,3), dat.K));
%        graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k,4}, measurementNoise, X(i), ar_tag(j,4), dat.K));
%    end
  %  blah=blah+1;
end
 %  blah

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
graph_container.add(PriorFactorPose3(X(1), truth.cameras{1}.pose, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
% graph_container.add(PriorFactorPoint2(symbol('p',10), Point2(0,0), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,1), Point3([0 0 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,2), Point3([TagSize 0 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,3), Point3([TagSize TagSize 0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(ar_tag(10,4), Point3([0 TagSize 0]'), pointPriorNoise));

%  Add Gaussian priors for a pose and a landmark to constrain the system
% cameraPriorNoise  = noiseModel.Diagonal.Sigmas(cameraNoiseSigmas);
% firstCamera = SimpleCamera(truth.cameras{1}.pose, truth.K);
% graph.add(PriorFactorSimpleCamera(symbol('c',1), firstCamera, cameraPriorNoise));
% 
% pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
% graph.add(PriorFactorPoint3(symbol('p',1), truth.points{1}, pointPriorNoise));


%% New vision
n = TagSize; % if origin is 1,1 need to add 1
homosq(1,:) = [0,0]; % set up four corners of tags
homosq(2,:) = [n,0];
homosq(3,:) = [n,n];
homosq(4,:) = [0,n];
transformed_points=[0 0 0];
homo_1 = homosq;
homo_1(:,3) = 1;
homo_1 = homo_1';

for a = 1:size(DetAll,2)
    detected=DetAll{a};
    if(find(detected(:,1)==10))
        ID=10;
        tag10=true;
    else
        ID=detected(1,1);
        tag10=false;
    end
    if(tag10)
        for b = 1:size(detected,1) % loop through every detection for current frame
            detectedID = detected(b,1); % get current tag id
            if detectedID == ID % if we reach global id while looping through detections
                tagPoints = [DetAll{a}(b,2) DetAll{a}(b,3); DetAll{a}(b,4) DetAll{a}(b,5); ... % pixel points of global Id
                    DetAll{a}(b,6) DetAll{a}(b,7); DetAll{a}(b,8) DetAll{a}(b,9)];
                globalP1 = tagPoints;
                globalP1(:,3) = 1; % add z posision as 1
                globalP1 = globalP1';
                H = homography2d(globalP1,homo_1);
                H= H/H(3,3);
                [camera_rot,camera_loc]=estimateWorldCameraPose(tagPoints,homo_1', cameraPar , 'MaxNumTrials',600,'Confidence',75,'MaxReprojectionError',5);
            end
        end
        for i = 1:size(detected,1)
            detectedID=detected(i,1);
            %         if currID ~=ID
            if isempty((find(detected(i,1)==transformed_points(:,1))))
                %for l=2:2:9
                tagPoints = [DetAll{a}(i,2) DetAll{a}(i,3) ];
                intP = tagPoints;
                intP(:,3) = 1;
                intP = intP';
                intP = intP(:,1);
                finP = H*intP;
                finP = finP/finP(3);
                x = (finP(1)); % origin for global was 1,1, find x,y difference. Divide by scale that was used earlier
                y = (finP(2));
                %scatter(x,y,'b','filled')
                transformed_points = [transformed_points ; [detected(i,1) x y] ];
                %                 end
                %end
            end
        end
        Pose_lock=[Pose_lock;camera_loc];
        Pose_rota=[Pose_rota;camera_rot];
        eang=[eang;rotm2eul(camera_rot)];
    end
    if(~tag10)
        for k=1:size(detected)
            detectedID=detected(k,1);
            if(find(detectedID==transformed_points(:,1)))
                tagPoints1 = [DetAll{a}(k,2) DetAll{a}(k,3); DetAll{a}(k,4) DetAll{a}(k,5);
                    DetAll{a}(k,6) DetAll{a}(k,7); DetAll{a}(k,8) DetAll{a}(k,9)];
                temp=k;
                break
            end
        end
        globalP1 = tagPoints1;
        globalP1(:,3) = 1;
        globalP1 = globalP1';
        H = homography2d(globalP1,homo_1);
        H= H/H(3,3);
        refID=find(detected(temp,1)==transformed_points(:,1));
        xref=transformed_points(refID,2);
        yref=transformed_points(refID,3);
        homo_sq_2(1,:) = [xref,yref]; % set up four corners of tags
        homo_sq_2(2,:) = [n+xref,yref];
        homo_sq_2(3,:) = [n+xref,n+yref];
        homo_sq_2(4,:) = [xref,n+yref];
        homo_2 = homo_sq_2;
        homo_2(:,3) = 1;
        homo_2 = homo_2';
        [camera_rot,camera_loc]=estimateWorldCameraPose(tagPoints1,homo_2', cameraPar , 'MaxNumTrials',600,'Confidence',75,'MaxReprojectionError',5);
        for i = 1:size(detected,1)
            if isempty((find(detected(i,1)==transformed_points(:,1))))
                %for l=2:2:9
                tagPoints2 = [DetAll{a}(i,2) DetAll{a}(i,3) ];
                intP = tagPoints2;
                intP(:,3) = 1;
                intP = intP';
                intP = intP(:,1);
                finP = H*intP;
                finP = finP/finP(3);
                x =finP(1)+xref;% origin for global was 1,1, find x,y difference. Divide by scale that was used earlier
                y =finP(2)+yref;
                %scatter(x,y,'b','filled')
                %hold on
                transformed_points = [transformed_points ; [detected(i,1) x y] ];
                %end
            end
        end
        Pose_lock=[Pose_lock;camera_loc];
        Pose_rota=[Pose_rota;camera_rot];
        eang=[eang;rotm2eul(camera_rot)];
    end
end


%% Constraints
% PoseNoise = noiseModel.Diagonal.Sigmas([2*0.1; 2*0.1]);
% tag(1) = Point3([0, TagSize, 0]');
% tag(2) = Point3([TagSize, 0, 0]');
% tag(3) = Point3([0, -TagSize, 0]');
% tag(4) = Point3([-TagSize, 0, 0]');
% tag(5) = Point3([TagSize, TagSize, 0]');
% tag(6) = Point3([-TagSize, TagSize,0]');

for i = 1:size(totalLandmarks)
        graph_container.add(BetweenFactorPoint3(uint64(ar_tag(totalLandmarks(i),1)), uint64(ar_tag(totalLandmarks(i),2)), Point3([TagSize 0 0]'), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(uint64(ar_tag(totalLandmarks(i),2)), uint64(ar_tag(totalLandmarks(i),3)), Point3([0 TagSize 0]'), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(uint64(ar_tag(totalLandmarks(i),3)), uint64(ar_tag(totalLandmarks(i),4)), Point3([-TagSize 0 0]'), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(uint64(ar_tag(totalLandmarks(i),4)), uint64(ar_tag(totalLandmarks(i),1)), Point3([0 -TagSize 0]'), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(uint64(ar_tag(totalLandmarks(i),1)), uint64(ar_tag(totalLandmarks(i),3)), Point3([TagSize TagSize 0]'), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(uint64(ar_tag(totalLandmarks(i),2)), uint64(ar_tag(totalLandmarks(i),4)), Point3([-TagSize TagSize 0]'), pointPriorNoise));
end
for i = 1:size(X,2)-1
        graph_container.add(BetweenFactorPose3(X(i),X(i+1), Pose3(Rot3(eye(3)), Point3([0 0 0]')), posePriorNoise));
end
%% Initialize cameras and points close to ground truth
initialEstimate = Values;
% Camera Poses
% for i = 1:size(Pose_loc,1)
%     
%     estimatePose = Pose3([eang(i,1),eang(i,2),eang(i,3),Pose_loc(i,1),Pose_loc(i,2),Pose_loc(i,3)]);
%     %estimatePose = SimpleCamera.Lookat(Point3([Pose_loc(i,1),Pose_loc(i,2),Pose_loc(i,3)]'), Point3, Point3([0,0,1]'), truth.K);
%     %estimatePose = Point3([Pose_rot(i,1),Pose_rot(i,2),Pose_rot(i,3)]');
%     initialEstimate.insert(symbol('x',i), estimatePose);
% end
%pp=1;
for i = 1:size(X,2) 
   estimatePose = Pose3([eang(i,1) ,eang(i,2) ,eang(i,3), Pose_lock(i,1), Pose_lock(i,2), Pose_lock(i,3)]);
    % estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    %estimatePose = Point3(Rot3(rand(3,3))]');      ([eang(i,1) ,eang(i,2) ,eang(i,3) ,Pose_lock(i,1), Pose_lock(i,2), Pose_lock(i,3)])
   % estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    initialEstimate.insert(X(i), estimatePose);
%    pp=pp+1;
end
%pp
% Landmarks
% for i = 1:length(totalLandmarks)
%     estimateLandmark(1) = truth.landmarks{i,1}.retract([0,0,0]');%0.1*randn(3,1));
%     estimateLandmark(2) = truth.landmarks{i,2}.retract([0,0,0]');%0.1*randn(3,1));
%     estimateLandmark(3) = truth.landmarks{i,3}.retract([0,0,0]');%0.1*randn(3,1));
%     estimateLandmark(4) = truth.landmarks{i,4}.retract([0,0,0]');%0.1*randn(3,1));
%     initialEstimate.insert(p(totalLandmarks(i)), estimateLandmark(1));
%     initialEstimate.insert(q(totalLandmarks(i)), estimateLandmark(2));
%     initialEstimate.insert(r(totalLandmarks(i)), estimateLandmark(3));
%     initialEstimate.insert(s(totalLandmarks(i)), estimateLandmark(4));
% end
%c=1;
for j = 2:size(transformed_points)
    estimateLandmark1 = Point3([transformed_points(j,2) transformed_points(j,3) 0]');%0.1*randn(3,1));
    estimateLandmark2 = Point3([transformed_points(j,2)+n transformed_points(j,3) 0]');%0.1*randn(3,1));
    estimateLandmark3 = Point3([transformed_points(j,2)+n transformed_points(j,3)+n 0]');%0.1*randn(3,1));
    estimateLandmark4 = Point3([transformed_points(j,2) transformed_points(j,3)+n 0]');%0.1*randn(3,1));
    initialEstimate.insert(uint64(ar_tag(transformed_points(j,1),1)), estimateLandmark1);
    initialEstimate.insert(uint64(ar_tag(transformed_points(j,1),2)), estimateLandmark2);
    initialEstimate.insert(uint64(ar_tag(transformed_points(j,1),3)), estimateLandmark3);
    initialEstimate.insert(uint64(ar_tag(transformed_points(j,1),4)), estimateLandmark4);
   % c=c+1;
end
%c
%% Fine grain optimization, allowing user to iterate step by step
% parameters = LevenbergMarquardtParams;
% parameters.setlambdaInitial(0.001);
% parameters.setVerbosityLM('trylambda');
% parameters = LevenbergMarquardtParams;
% parameters.setlambdaInitial(1.0);
% parameters.setVerbosityLM('trylambda');
% optimizer = DoglegOptimizer(graph_container,initialEstimate);
%optimizer = LevenbergMarquardtOptimizer(graph_container, initialEstimate, parameters);
% 
% for i=1:5
%     optimizer.iterate();
% end
% result = optimizer.optimize();
%result = optimizer.values();
% % 
  optimizer = LevenbergMarquardtOptimizer(graph_container,initialEstimate);
  result = optimizer.optimizeSafely();
%     disp(result);
% optimizer = DoglegOptimizer(graph_container,initialEstimate);
% result = optimizer.optimizeSafely();
disp(result);
% parameters = LevenbergMarquardtParams;
% parameters.setlambdaInitial(1.0);
% parameters.setVerbosityLM('trylambda');

%optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);
% parameters = DoglegParams;
% parameters.setDeltaInitial(1.0);
% % parameters.setVerbosityDL('trydelta');
% optimizer = DoglegOptimizer(graph_container,initialEstimate);%,parameters);
% result = optimizer.optimizeSafely();
%marginals = Marginals(graph_container, result);
%% Plot results with covariance ellipses
% marginals = Marginals(graph_container, result);
% cla
% hold on;
% plot3DPoints(result, [], marginals);
% plot3DTrajectory(result, '*', 1, 81, marginals);
for i = 1:size(totalLandmarks, 1)
       LandMarksComputed(i,:)=([totalLandmarks(i),result.at(ar_tag(totalLandmarks(i),1)).x, result.at(ar_tag(totalLandmarks(i),1)).y,...
           result.at(ar_tag(totalLandmarks(i),2)).x, result.at(ar_tag(totalLandmarks(i),2)).y,...
           result.at(ar_tag(totalLandmarks(i),3)).x, result.at(ar_tag(totalLandmarks(i),3)).y,...
           result.at(ar_tag(totalLandmarks(i),4)).x, result.at(ar_tag(totalLandmarks(i),4)).y]);
       scatter3(result.at(ar_tag(totalLandmarks(i),1)).x, result.at(ar_tag(totalLandmarks(i),1)).y, 1);%result.at(symbol('p',totalLandmarks(i))).z)
       scatter3(result.at(ar_tag(totalLandmarks(i),2)).x, result.at(ar_tag(totalLandmarks(i),2)).y, 1);%result.at(symbol('q',totalLandmarks(i))).z)
       scatter3(result.at(ar_tag(totalLandmarks(i),3)).x, result.at(ar_tag(totalLandmarks(i),3)).y, 1);%result.at(symbol('r',totalLandmarks(i))).z)
       scatter3(result.at(ar_tag(totalLandmarks(i),4)).x, result.at(ar_tag(totalLandmarks(i),4)).y, 1);%result.at(symbol('s',totalLandmarks(i))).z)
       hold on;
end
CameraP=[];
AllPosesComputed=[];
for i= 1:size(Pose_lock,1)
    CameraP=[CameraP;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z];
    AllPosesComputed =[AllPosesComputed;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z,result.at(symbol('x',i)).rotation.quaternion'];
end
CameraP;
 
figure(2),
scatter3(CameraP(:,1),CameraP(:,2),CameraP(:,3));
end
