function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the project pdf

% import gtsam.*

%% Vision

% Match the Tags every for every consecutive frames
x1 = []; y1 = [];
x2 = []; y2 = [];
im = cell2mat(LeftImgs.LeftImgs(1,1));
cameraParams = cameraParameters('IntrinsicMatrix', K');
for i = 1:1%size(DetAll, 2) - 1
    for j = 1:size(DetAll{1,i}, 1)
        ID = DetAll{1,i}(j,1);
        ID2 = find(DetAll{1,i+1}(:,1)==ID);

        if ID2 ~= 0 
            x1(j,:) = [DetAll{1,i}(j,2) DetAll{1,i}(j,4) DetAll{1,i}(j,6) DetAll{1,i}(j,8)];
            y1(j,:) = [DetAll{1,i}(j,3) DetAll{1,i}(j,5) DetAll{1,i}(j,7) DetAll{1,i}(j,9)];
            x2(j,:) = [DetAll{1,i+1}(ID2,2) DetAll{1,i+1}(ID2,4) DetAll{1,i+1}(ID2,6) DetAll{1,i+1}(ID2,8)];
            y2(j,:) = [DetAll{1,i+1}(ID2,3) DetAll{1,i+1}(ID2,5) DetAll{1,i+1}(ID2,7) DetAll{1,i+1}(ID2,9)];
        else
            continue;
        end
    end
    
    for k = 1:size(x1, 1)
%         Determine Projection Matrix of Camera in Pose 1 and Pose 2
        R1 = eye(3); t1 = [1;1;1];
        P1 = K * [eye(3), [1;1;1]];

        matchedPoints1(:,1) = [x1(k,1); y1(k,1); 1];
        matchedPoints1(:,2) = [x1(k,2); y1(k,2); 1];
        matchedPoints1(:,3) = [x1(k,3); y1(k,3); 1];
        matchedPoints1(:,4) = [x1(k,4); y1(k,4); 1];

        matchedPoints2(:,1) = [x2(k,1); y2(k,1); 1];
        matchedPoints2(:,2) = [x2(k,2); y2(k,2); 1];
        matchedPoints2(:,3) = [x2(k,3); y2(k,3); 1];
        matchedPoints2(:,4) = [x2(k,4); y2(k,4); 1];

        H = homography2d(matchedPoints1, matchedPoints2);
        H = H/H(3,3);

        RT = K\H;
        Rt(:,1) = RT(:,1);
        Rt(:,2) = RT(:,2);
        Rt(:,3) = cross(Rt(:,1),Rt(:,2));
        Rt(:,4) = RT(:,3);
        P2 = K * Rt;
        
        for j = 1:size(matchedPoints1, 2)
            m1(j,:) = [matchedPoints1(1,j) matchedPoints1(2,j)];
            m2(j,:) = [matchedPoints2(1,j) matchedPoints2(2,j)];
            points3D(j,:) = triangulate_mich(P1, m1(j,:), P2, m2(j,:));
            points3D(j,:) = points3D(j,:)/points3D(j,3);
        end
        scatter3(points3D(:,1), points3D(:,2), points3D(:,3), 'filled');
        hold on
        
        [E, epipolarInliers] = estimateEssentialMatrix(...
                m1, m2, cameraParams, 'Confidence', 99.99);
        [orient, loc] = relativeCameraPose(E, cameraParams, m1, m2);
        camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);
    end
    
%     c = 1;
%     for k = 1:size(x1, 1)
%         for kk = 1:4
%             matchedPoints1(c,:) = [x1(k,kk), y1(k,kk)];
%             matchedPoints2(c,:) = [x2(k,kk), y2(k,kk)];
%             c = c + 1;
%         end
%     end   
% 
%     % Estimate the fundamental matrix
%     [E, epipolarInliers] = estimateEssentialMatrix(...
%     matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);
% 
%     % Find epipolar inliers
%     inlierPoints1 = matchedPoints1(epipolarInliers, :);
%     inlierPoints2 = matchedPoints2(epipolarInliers, :);
% 
%     % Compute the Camera Pose
%     [orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);
% 
%     % Reconstruct the 3-D Locations of Matched Points
%     camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);
%     % Compute extrinsics of the second camera
%     [R, t] = cameraPoseToExtrinsics(orient, loc);
%     camMatrix2 = cameraMatrix(cameraParams, R, t);
%     points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);
%     points3D = points3D ./ points3D(:,3);
%     ptCloud = pointCloud(points3D);
% 
%     % Visualize the camera locations and orientations
%     cameraSize = 0.05;
%     figure
% %     plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
%     hold on
%     grid on
%     plotCamera('Location', loc, 'Orientation', orient, 'Size', cameraSize, ...
%         'Color', 'b', 'Label', '2', 'Opacity', 0);
%     
%     pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
%     
% %     scatter3(points3D(:,1), points3D(:,2), points3D(:,3), 'filled')
%     1;
    
    
    
    
end











% %% Data generation
% dat.K = Cal3_S2(K(1,1), K(2,2), 0, K(1,3), K(2,3));
% truth.K = dat.K;
% totalLandmarks = DetAll{1,1}(:,1);
% for i = 1:size(DetAll, 2)
%    truth.cameras{i} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([2,2,0]'), truth.K);
%    for j = 1:size(DetAll{1,i}, 1)
%        % Only one of the corners of the Tags for now
%        dat.Z{i}{j} = Point2(DetAll{1,i}(j,2), DetAll{1,i}(j,3));
%        dat.J{i}{j} = DetAll{1,i}(j,1);
%        totalLandmarks = union(totalLandmarks, DetAll{1,i}(j,1));
%    end
% end
% c = 0;
% for i = 1:size(totalLandmarks, 1)
%     truth.landmarks{i} = Point3([c, c, 0]');
%     c = c + TagSize;
% end
% 
% measurementNoiseSigma = 1.0;
% pointNoiseSigma = 0.1;
% poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
% 
% %% Create graph container and add factors to it from GTSAM library
% graph_container = NonlinearFactorGraph;
% 
% %% Add factors for all measurements
% measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
% for i = 1:length(dat.Z)
%    for k = 1:length(dat.Z{i})
%        j = dat.J{i}{k};
%        graph_container.add(GenericProjectionFactorCal3_S2(dat.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',j), dat.K));
%        caca = GenericProjectionFactorCal3_S2(dat.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',j), dat.K);
%    end
% end
% 
% %% Add Gaussian priors for a pose and a landmark to constrain the system
% posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
% graph_container.add(PriorFactorPose3(symbol('x',1), truth.cameras{1}.pose, posePriorNoise));
% pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
% graph_container.add(PriorFactorPoint3(symbol('p',10), Point3([0,0,0]'), pointPriorNoise));
% 
% %% Initialize cameras and points close to ground truth
% initialEstimate = Values;
% % Camera Poses
% for i = 1:length(truth.cameras)
%     estimatePose = truth.cameras{i}.pose.retract(0.1*randn(6,1));
%     initialEstimate.insert(symbol('x',i), estimatePose);
% end
% % Landmarks
% for i = 1:size(totalLandmarks, 1)
%     estimateLandmark = truth.landmarks{i}.retract(0.1*randn(3,1));
%     initialEstimate.insert(symbol('p',totalLandmarks(i,1)), estimateLandmark);
% end
% % initialEstimate.print(sprintf('\nInitial estimate:\n  '));
% 
% %% Fine grain optimization, allowing user to iterate step by step
% parameters = LevenbergMarquardtParams;
% parameters.setlambdaInitial(1.0);
% parameters.setVerbosityLM('trylambda');
% 
% optimizer = LevenbergMarquardtOptimizer(graph_container, initialEstimate, parameters);
% 
% for i=1:5
%     optimizer.iterate();
% end
% 
% result = optimizer.values();
% %% Plot results with covariance ellipses
% marginals = Marginals(graph_container, result);
% cla
% hold on;
% plot3DPoints(result, [], marginals);
% plot3DTrajectory(result, '*', 1, 8, marginals);
% 
% view(3)
% colormap('hot')
% 
% % for i = 1:size(totalLandmarks, 1)
% %        scatter3(result.at(symbol('p',totalLandmarks(i))).x, result.at(symbol('p',totalLandmarks(i))).y, result.at(symbol('p',totalLandmarks(i))).z)
% %        hold on;
% % end
end
