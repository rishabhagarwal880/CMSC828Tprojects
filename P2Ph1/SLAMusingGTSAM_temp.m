function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC)
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
m=1; Pose_loc=[]; Pose_rot=[]; eang=[];
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

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.00 0.00 0.00 0 0 0]';

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
% graph_container.add(PriorFactorPoint2(symbol('p',10), Point2(0,0), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('p',10), Point3([0,0,0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('q',10), Point3([TagSize,0,0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('r',10), Point3([TagSize,TagSize,0]'), pointPriorNoise));
graph_container.add(PriorFactorPoint3(symbol('s',10), Point3([0,TagSize,0]'), pointPriorNoise));

%  Add Gaussian priors for a pose and a landmark to constrain the system
% cameraPriorNoise  = noiseModel.Diagonal.Sigmas(cameraNoiseSigmas);
% firstCamera = SimpleCamera(truth.cameras{1}.pose, truth.K);
% graph.add(PriorFactorSimpleCamera(symbol('c',1), firstCamera, cameraPriorNoise));
% 
% pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
% graph.add(PriorFactorPoint3(symbol('p',1), truth.points{1}, pointPriorNoise));

%% Constraints
% PoseNoise = noiseModel.Diagonal.Sigmas([2*0.1; 2*0.1]);
tag(1) = Point3([0, TagSize, 0]');
tag(2) = Point3([TagSize, 0, 0]');
tag(3) = Point3([0, -TagSize, 0]');
tag(4) = Point3([-TagSize, 0, 0]');
tag(5) = Point3([TagSize, TagSize, 0]');
tag(6) = Point3([-TagSize, TagSize,0]');

for i = 1:size(totalLandmarks)
        graph_container.add(BetweenFactorPoint3(p(totalLandmarks(i)), q(totalLandmarks(i)), tag(2), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(q(totalLandmarks(i)), r(totalLandmarks(i)), tag(1), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(r(totalLandmarks(i)), s(totalLandmarks(i)), tag(4), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(s(totalLandmarks(i)), p(totalLandmarks(i)), tag(3), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(p(totalLandmarks(i)), r(totalLandmarks(i)), tag(5), pointPriorNoise));
        graph_container.add(BetweenFactorPoint3(q(totalLandmarks(i)), s(totalLandmarks(i)), tag(6), pointPriorNoise));
end

for i = 1:length(x)-1
        graph_container.add(BetweenFactorPose3(x(i), x(i+1), Pose3(Rot3(eye(3)), Point3([0 0 0]')), posePriorNoise));
end

%% New vision
n = TagSize; % if origin is 1,1 need to add 1
hsq(1,:) = [0,0]; % set up four corners of tags
hsq(2,:) = [n,0];
hsq(3,:) = [n,n];
hsq(4,:) = [0,n];
transformed=[0 0 0];
homo1 = hsq;
homo1(:,3) = 1;
homo1 = homo1';

for a = 1:size(DetAll,2)
    curr=DetAll{a};
    if(find(curr(:,1)==10))
        ID=10;
        tag10=true;
    else
        ID=curr(1,1);
        tag10=false;
    end
    if(tag10)
        for b = 1:size(curr,1) % loop through every detection for current frame
            currID = curr(b,1); % get current tag id
            if currID == ID % if we reach global id while looping through detections
                tagPoints = [DetAll{a}(b,2) DetAll{a}(b,3); DetAll{a}(b,4) DetAll{a}(b,5); ... % pixel points of global Id
                    DetAll{a}(b,6) DetAll{a}(b,7); DetAll{a}(b,8) DetAll{a}(b,9)];
                globalP1 = tagPoints;
                globalP1(:,3) = 1; % add z posision as 1
                globalP1 = globalP1';
                H = homography2d(globalP1,homo1);
                H= H/H(3,3);
                [crot,cloc]=estimateWorldCameraPose(tagPoints,homo1', cameraParams , 'MaxNumTrials',600,'Confidence',75,'MaxReprojectionError',5);
            end
        end
        for i = 1:size(curr,1)
            currID=curr(i,1);
            %         if currID ~=ID
            if isempty((find(curr(i,1)==transformed(:,1))))
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
                transformed = [transformed ; [curr(i,1) x y] ];
                %                 end
                %end
            end
        end
        Pose_loc=[Pose_loc;cloc];
        Pose_rot=[Pose_rot;crot];
        eang=[eang;rotm2eul(crot)];
    end
    if(~tag10)
        for k=1:size(curr)
            currID=curr(k,1);
            if(find(currID==transformed(:,1)))
                tagPoints1 = [DetAll{a}(k,2) DetAll{a}(k,3); DetAll{a}(k,4) DetAll{a}(k,5);
                    DetAll{a}(k,6) DetAll{a}(k,7); DetAll{a}(k,8) DetAll{a}(k,9)];
                temp=k;
                break
            end
        end
        globalP1 = tagPoints1;
        globalP1(:,3) = 1;
        globalP1 = globalP1';
        H = homography2d(globalP1,homo1);
        H= H/H(3,3);
        refID=find(curr(temp,1)==transformed(:,1));
        xref=transformed(refID,2);
        yref=transformed(refID,3);
        hsq2(1,:) = [xref,yref]; % set up four corners of tags
        hsq2(2,:) = [n+xref,yref];
        hsq2(3,:) = [n+xref,n+yref];
        hsq2(4,:) = [xref,n+yref];
        homo2 = hsq2;
        homo2(:,3) = 1;
        homo2 = homo2';
        [crot,cloc]=estimateWorldCameraPose(tagPoints1,homo2', cameraParams , 'MaxNumTrials',600,'Confidence',75,'MaxReprojectionError',5);
        for i = 1:size(curr,1)
            if isempty((find(curr(i,1)==transformed(:,1))))
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
                transformed = [transformed ; [curr(i,1) x y] ];
                %end
            end
        end
        Pose_loc=[Pose_loc;cloc];
        Pose_rot=[Pose_rot;crot];
        eang=[eang;rotm2eul(crot)];
    end
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
for i = 1:size(Pose_loc,1) 
    estimatePose = Pose3([eang(i,1)+1 ,eang(i,2)+1 ,eang(i,3)+1 ,Pose_loc(i,1)-2.3,Pose_loc(i,2)-.3,Pose_loc(i,3)-0.29]);
    % estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
    %estimatePose = Point3(Rot3(rand(3,3))(i,1),Pose_rot(i,2),Pose_rot(i,3)]');
    initialEstimate.insert(symbol('x',i), estimatePose);
end
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
for i = 2:size(transformed)
    estimateLandmark1 = Point3([transformed(i,2),transformed(i,3),0]');%0.1*randn(3,1));
    estimateLandmark2 = Point3([transformed(i,2)+n,transformed(i,3),0]');%0.1*randn(3,1));
    estimateLandmark3 = Point3([transformed(i,2)+n,transformed(i,3)+n,0]');%0.1*randn(3,1));
    estimateLandmark4 = Point3([transformed(i,2),transformed(i,3)+n,0]');%0.1*randn(3,1));
    initialEstimate.insert(p(transformed(i,1)), estimateLandmark1);
    initialEstimate.insert(q(transformed(i,1)), estimateLandmark2);
    initialEstimate.insert(r(transformed(i,1)), estimateLandmark3);
    initialEstimate.insert(s(transformed(i,1)), estimateLandmark4);
end

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
% % for i=1:5
% %     optimizer.iterate();
% % end
% result = optimizer.optimize();
%result = optimizer.values();
optimizer = LevenbergMarquardtOptimizer(graph_container,initialEstimate);
result = optimizer.optimizeSafely();
disp(result);
%marginals = Marginals(graph_container, result);
%% Plot results with covariance ellipses
% marginals = Marginals(graph_container, result);
% cla
% hold on;
% plot3DPoints(result, [], marginals);
% plot3DTrajectory(result, '*', 1, 81, marginals);
for i = 1:size(totalLandmarks, 1)
%        LandMarksComputed(i,:)=([totalLandmarks(i),result.at(symbol('p',totalLandmarks(i))).x, result.at(symbol('p',totalLandmarks(i))).y,...
%            result.at(symbol('q',totalLandmarks(i))).x, result.at(symbol('q',totalLandmarks(i))).y,...
%            result.at(symbol('r',totalLandmarks(i))).x, result.at(symbol('r',totalLandmarks(i))).y,...
%            result.at(symbol('s',totalLandmarks(i))).x, result.at(symbol('s',totalLandmarks(i))).y]);
       scatter3(result.at(symbol('p',totalLandmarks(i))).x, result.at(symbol('p',totalLandmarks(i))).y, 1);%result.at(symbol('p',totalLandmarks(i))).z)
       scatter3(result.at(symbol('q',totalLandmarks(i))).x, result.at(symbol('q',totalLandmarks(i))).y, 1);%result.at(symbol('q',totalLandmarks(i))).z)
       scatter3(result.at(symbol('r',totalLandmarks(i))).x, result.at(symbol('r',totalLandmarks(i))).y, 1);%result.at(symbol('r',totalLandmarks(i))).z)
       scatter3(result.at(symbol('s',totalLandmarks(i))).x, result.at(symbol('s',totalLandmarks(i))).y, 1);%result.at(symbol('s',totalLandmarks(i))).z)
       hold on;
end
CameraP=[];
for i= 1:size(Pose_loc,1)
    CameraP=[CameraP;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z];
end
CameraP;

% LandMarksComputed(i,:)=([totalLandmarks(i),result.at(symbol('p',totalLandmarks(i))).x, result.at(symbol('p',totalLandmarks(i))).y,...
%            result.at(symbol('q',totalLandmarks(i))).x, result.at(symbol('q',totalLandmarks(i))).y,...
%            result.at(symbol('r',totalLandmarks(i))).x, result.at(symbol('r',totalLandmarks(i))).y,...
%            result.at(symbol('s',totalLandmarks(i))).x, result.at(symbol('s',totalLandmarks(i))).y]);
toc
figure(2),
scatter3(CameraP(:,1),CameraP(:,2),CameraP(:,3));
end
