    %function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
         %                                       IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the project pdf
addpath('../../../gtsam_toolbox')
import gtsam.*
clear all
clc
% Add factor graph
graph = NonlinearFactorGraph;

close all
params = matfile('CalibParams.mat');
data = matfile('DataMapping.mat');
tagSize = params.TagSize;
K = params.K; K1 = K;
fx = K(1,1); fy = K(2,2); s = K(1,2); x0 = K(1,3); y0 = K(2,3);
K = Cal3_S2(fx,fy,s,x0,y0);%K = Cal3_S2(fx,fy,s,x0,y0);
DetAll = data.DetAll;

% Create keys for states
stateKeys = [];
for a = 1:size(DetAll,2) % loop through every frame
    truth.cameras{a} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([0,0,1]'), K);
    stateKeys = [stateKeys ; symbol('x',a)];% add a new symbol for every state
end

% AR Tag Landmarks
allTagIds = [];
for a = 1:size(DetAll,2)
    allTagIds = [allTagIds ; DetAll{a}(:,:)]; % add first column of each frame to get all tag Ids from every frame
end
[~,uniqueIndexes] = unique(allTagIds(:,1)); % sort unique by firt column
tagIds = allTagIds(uniqueIndexes,:); % all unique tag ids
% tagIds are now [TagID, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y]

% Create keys for landmarks
tagKeys = uint64([]);
count = 1;
for a = 1:size(tagIds,1) % loop through every unique tag Id
    tagKeys(a,1) = tagIds(a,1);
    for p = 2:5
        tagKeys(a,p) = uint64(symbol('p',count)); 
        count = count + 1;          
     end
end
% tagKeys has [tagId p1Symbol p2Symbol p3Symbol p4Symbol]
% so that p4 === p3
%         ||     ||
%         p1 === p2


% Projective Measurement Noise
measurementNoiseSigma = 1.0;
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);

% Set up homorgraphy points
% Homography stuff since it sucks without it
nextToGlobal = []; 
%n = tagSize*10; % scale up
n = 1+tagSize; % if origin is 1,1 need to add 1
tagWindow(1,:) = [1, 1]; % set up four corners of tags
tagWindow(2,:) = [ n, 1];
tagWindow(3,:) = [n, n];
tagWindow(4,:) = [1, n];

% Plot
%scatter(tagWindow(:,1),tagWindow(:,2),'m','filled'); hold on

% Homography points to be transformed to
homoP1 = tagWindow;
homoP1(:,3) = 1;
homoP1 = homoP1';

% Will be true if tag10 found in first frame
using10 = false;

scatter(0,0,'r','filled'); hold on

% Loop through all frames
h = waitbar(0,'Adding Projective Measurements...');
for a = 1:size(DetAll,2) % loop through every frame
    currentDetections = DetAll{a}; % all detections on current frame
    currentStateIdSymbol = stateKeys(a); % state key
    globalInFrame = false;
    
     % IF FIRST FRAME SET WHAT GLOBAL ID WILL BE
    if a == 1 % if on first frame check if 10 is in it
        if size(find(all(bsxfun(@eq,currentDetections(:,1),10),2)),1) > 0 % check if tag 10 in first frame
            globalId = 10;
            using10 = true;
            
        else
            globalId = currentDetections(1,1); % if no tag 10 make global the first etection, have to fix it later (after loop)
        end
        nextToGlobal = [globalId 0 0];
    end
    
    % FIND OUT IF GLOBAL TAG IN THIS FRAME
    if size(find(all(bsxfun(@eq,currentDetections(:,1),globalId),2)),1) > 0 % if global Id detected in this frame
        globalInFrame = true;
    end
    
    if globalInFrame % if we found global in this frame
    
        for b = 1:size(currentDetections,1) % loop through every detection for current frame
            currentId = currentDetections(b,1); % get current tag id
            
            % IF GLOBAL DO HOMOGRAPHY FOR THIS FRAME
            if currentId == globalId % if we reach global id while looping through detections
                 tagPoints = [DetAll{a}(b,2) DetAll{a}(b,3); DetAll{a}(b,4) DetAll{a}(b,5); ... % pixel points of global Id
                              DetAll{a}(b,6) DetAll{a}(b,7); DetAll{a}(b,8) DetAll{a}(b,9)];
                 globalP1 = tagPoints;
                 globalP1(:,3) = 1; % add z posision as 1
                 globalP1 = globalP1';

                 H = homography2d(globalP1,homoP1); % Get homography matrix
                 
                 % GET POSE FROM HOMOGRAPHY
                 r1 = K1\H(:,1) / norm(K1\H(:,1));
                 r2 = K1\H(:,2) / norm(K1\H(:,2));
                 t_cw = K1\H(:,3)  / norm(K1\H(:,3));
                 r3 = cross(r1, r2);
                 R_cw = [r1 r2 r3];
                 
                 H_cr = eye(4);
                 H_wr = [(R_cw)' -(R_cw)'*t_cw; 0 0 0 1]*H_cr; 
                 pos = H_wr(1:3,end);
                 %scatter(t_cw(1), t_cw(2),12,'g','*'); hold on
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                  Hnormal = H./H(3,3);
                
                  Kinv = inv(K1);
                  
                  R12t = K1 \ Hnormal;

                  [U,S,Vt] = svd(R12t);
                  R1 = U * [1 0 0; 0 1 0 ; 0 0 det(U*Vt')] * Vt';
                  t1 = R12t(:,3) / norm(R12t(:,1));

                  r1 = R12t(:,1);
                  r2 = R12t(:,2);
                  r3 = cross(r1,r2);
                  Rot = [r1 r2 r3]; % rotation matrix
                  t = R12t(:,3); % translation vector
                  eulZYX = rotm2eul(Rot); % get 3 angles from rotation matrix
                % dpos = [dpos(1)+t(1) dpos(1)+t(2)];

                 % FIND TAGS IN CURRENT DETECTION THAT HAVE NOT BEEN FOUND ,(RELATIVE TO GLOBAL ID)
                 % USE HOMOGRAPHY TO GET THEIR GLOBAL POSITIONS (RELATIVE TO GLOBAL ID)
                 for i = 1:size(currentDetections,1) % loop through current detections
                     
                     if currentDetections(i,1) ~= globalId % dont go through global, already know global position.
                         
                         if size(find(all(bsxfun(@eq,nextToGlobal(:,1),currentDetections(i,1)),2)),1) < 1 % if we didnt find current tagId in nextToGlobal Matrix, we havent added its position relative to global yet. if it is in nextToGlobal skip over, awe already got global position for it
                             tagPoints = [DetAll{a}(i,2) DetAll{a}(i,3) ]; % get bottom left corner of tag
    
%                              if a == 2
%                                  'hold up'
%                              end
                             oldPoint = tagPoints;
                             oldPoint(:,3) = 1;
                             oldPoint = oldPoint';
                             oldPoint = oldPoint(:,1);

                             newPoint = H*oldPoint;
                             newPoint = newPoint/newPoint(3);
                             %scatter(newPoint(1),newPoint(2),'g','filled')
                             xdiff = (newPoint(1) - 1)/10; % origin for global was 1,1, find x,y difference. Divide by scale that was used earlier
                             ydiff = (newPoint(2) - 1)/10;
                             scatter(xdiff,ydiff,'b','filled')
                             nextToGlobal = [nextToGlobal ; [currentDetections(i,1) xdiff ydiff] ];  % all Ids Next to global
                         end
                     end
                 end
            end
        
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% % if current detection is global 10 do homography
%         if currentId == 10 && ~found10
%              tagPoints = [DetAll{a}(b,2) DetAll{a}(b,3); DetAll{a}(b,4) DetAll{a}(b,5); ...
%                           DetAll{a}(b,6) DetAll{a}(b,7); DetAll{a}(b,8) DetAll{a}(b,9)];
%              globalP1 = tagPoints;
%              globalP1(:,3) = 1;
%              globalP1 = globalP1';
%              
%              H = homography2d(globalP1,homoP1); % Get homography matrix
%              
%              for i = 1:size(currentDetections,1)
%                  if currentDetections(i,1) ~= 10    
%                      tagPoints = [DetAll{a}(i,2) DetAll{a}(i,3) ]; % get bottom left corner of ag
% 
%                      oldPoint = tagPoints;
%                      oldPoint(:,3) = 1;
%                      oldPoint = oldPoint';
% 
%                      oldPoint = oldPoint(:,1);
%                      newPoint = H*oldPoint;
%                      newPoint = newPoint/newPoint(3);
%                      scatter(newPoint(1),newPoint(2),'g','filled')
%                      xdiff = (newPoint(1) - 1)/10;
%                      ydiff = (newPoint(2) - 1)/10;
%                      found10 = true;
%                      
%                      %nextToGlobal = [nextToGlobal ; [currentDetections(i,1) xdiff ydiff] ];  % all Ids Next to global
%                  end
%              end
%         end
        
        % GO THROUGH ALL 4 POINTS OF EACH TAG, GIVE xy POSITION IN CAMERA FRAME, STATE SYMBOL, LANDMARK SYMBOL
        for c = 2:2:8 % loop through every point on current tag (points are x,y so step by 2)
            uxy = [currentDetections(b,c) currentDetections(b,c+1)]; % xy position of point, there are 4 points per row/tag 
           
            idIndex = find(all(bsxfun(@eq, tagKeys(:,1),currentId),2)); % index of current id in tagKeys
            pointSymbol = tagKeys(idIndex,c/2+1); % p1 is 2nd col in tagKeys, p2 is 3... c/2+1 converts 2,4,6,8 to 2,3,4,5

            graph.add(GenericProjectionFactorCal3_S2(Point2(uxy'),measurementNoise, ...
                        currentStateIdSymbol, uint64(pointSymbol),K));
            % graph.add(GenericProjectionFactorCal3DS2(Point2(uxy'),measurementNoise, ...
             %           currentStateIdSymbol, uint64(pointSymbol),K));
        end
        end
    end
    
    %%%%%%%%%%%%%%%%%%%% Global Id not in frame %%%%%%%%%%%%%%%%%%%%%%%%%5
    if ~globalInFrame
        % Same process as above (if globalInFrame) except when adding xdiff
        % and y diff of new tag id, add the xdiff,ydiff of the reference or
        % fill in "global"
        
        % Get reference global
        for i = 1:size(currentDetections,1) % loop through current detections
            if size(find(all(bsxfun(@eq, nextToGlobal(:,1) , currentDetections(i,1)),2)),1) > 0 % if current detection has ben previously found and mapped in reference to globalID
                globalReference = currentDetections(i,1); % set reference for frame (since frame doesnt have globalID in it)
                break
            end
        end
        
        for b = 1:size(currentDetections,1) % loop through every detection for current frame
            currentId = currentDetections(b,1); % get current tag id
            
            % If global reference do homography
            if currentId == globalReference % if we reach the global reference while looping through detections
                 tagPoints = [DetAll{a}(b,2) DetAll{a}(b,3); DetAll{a}(b,4) DetAll{a}(b,5); ... % pixel points of global Reference
                              DetAll{a}(b,6) DetAll{a}(b,7); DetAll{a}(b,8) DetAll{a}(b,9)];
                 globalP1 = tagPoints;
                 globalP1(:,3) = 1; % add z position as 1
                 globalP1 = globalP1';

                 H = homography2d(globalP1,homoP1); % Get homography matrix
                 H = H/H(3,3);
                 
                 % Get global reference xdiff,ydiff
                 globalReferenceIndex = find(all(bsxfun(@eq,nextToGlobal(:,1),globalReference),2));
                 xdiffReference = nextToGlobal(globalReferenceIndex,2);
                 ydiffReference = nextToGlobal(globalReferenceIndex,3);
                 
                 % GET POSE FROM HOMOGRAPHY
                 r1 = K1\H(:,1) / norm(K1\H(:,1));
                 r2 = K1\H(:,2) / norm(K1\H(:,1));
                 t_cw = K1\H(:,3)  / norm(K1\H(:,3));
                 r3 = cross(r1, r2);
                 R_cw = [r1 r2 r3];
                 
                 H_cr = eye(4);
                 H_wr = [(R_cw)' -(R_cw)'*t_cw; 0 0 0 1]*H_cr; 
                 pos = H_wr(1:3,end);
                % scatter(-1+pos(1), 0+pos(2),4,'g','*'); hold on
                 
                 % FIND TAGS IN CURRENT DETECTION THAT HAVE NOT BEEN FOUND (RELATIVE TO GLOBAL ID),
                 % USE HOMOGRAPHY TO GET THEIR GLOBAL POSITIONS (RELATIVE TO GLOBAL Reference)
                 for i = 1:size(currentDetections,1) % loop through current detections
                     
                     if curr(i,1) ~= globalReference % dont go through global, already know global position.
                         
                         if size(find(all(bsxfun(@eq,nextToGlobal(:,1),currentDetections(i,1)),2)),1) < 1 % if we didnt find current tagId in nextToGlobal Matrix, we havent added its position relative to global yet. if it is in nextToGlobal skip over, we already got global position for it
                             tagPoints = [DetAll{a}(i,2) DetAll{a}(i,3) ]; % get bottom left corner of tag
                
                             oldPoint = tagPoints;
                             oldPoint(:,3) = 1;
                             oldPoint = oldPoint';
                             oldPoint = oldPoint(:,1);

                             newPoint = H*oldPoint;
                             newPoint = newPoint/newPoint(3);
                             %scatter(newPoint(1),newPoint(2),'g','filled')
                             
                             % Get global reference xdiff,ydiff
                             globalReferenceIndex = find(all(bsxfun(@eq,nextToGlobal(:,1),globalReference),2));
                             xdiffReference = nextToGlobal(globalReferenceIndex,2);
                             ydiffReference = nextToGlobal(globalReferenceIndex,3);
                             
                             
                             xdiff = (newPoint(1) - 1)/10; % origin for globalReference was 1,1, find x,y difference. Divide by scale that was used earlier
                             ydiff = (newPoint(2) - 1)/10;
                             
                             xdiff = xdiff + xdiffReference;
                             ydiff = ydiff + ydiffReference;
                             
                             scatter(xdiff,ydiff,'b','filled')
                             nextToGlobal = [nextToGlobal ; [currentDetections(i,1) xdiff ydiff] ];
                         end
                     end
                 end
            end
            for c = 2:2:8 % loop through every point on current tag (points are x,y so step by 2)
            uxy = [currentDetections(b,c) currentDetections(b,c+1)]; % xy position of point, there are 4 points per row/tag 
           
            idIndex = find(all(bsxfun(@eq, tagKeys(:,1),currentId),2)); % index of current id in tagKeys
            pointSymbol = tagKeys(idIndex,c/2+1); % p1 is 2nd col in tagKeys, p2 is 3... c/2+1 converts 2,4,6,8 to 2,3,4,5

            graph.add(GenericProjectionFactorCal3_S2(Point2(uxy'),measurementNoise, ...
                        currentStateIdSymbol, uint64(pointSymbol),K));
            % graph.add(GenericProjectionFactorCal3DS2(Point2(uxy'),measurementNoise, ...
             %           currentStateIdSymbol, uint64(pointSymbol),K));
            end
        end
            
    end
    waitbar(a/size(DetAll,2))
end
close(h)

tic
% Noise
pointNoiseSigma = 0.00001;
poseNoiseSigmas = [0.01 0.01 0.01 0.01 0.01 0.01]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);

% Add constraints relativePose between points
% cnoise = [.001 .001 .001]';
% cnoise = noiseModel.Diagonal.Sigmas(cnoise);

% SIZE OF TAG CONSTRAINTS
% tag keys each row is a tag w/ 4 points [bottomleft bottomRight topRigh topLeft]
    r1 = [tagSize 0 0]; % relative pose bottom left --> bottom right
    r2 = [0 tagSize 0]; % relative pose bottom right --> top right
    r3 = [-tagSize 0 0]; % relative pose top right --> topleft
    r4 = [0 -tagSize 0]; % relative pose top left --> bottom left
    r5 = [tagSize tagSize 0]; % relative pose bottom left --> top right
    r6 = [-tagSize tagSize 0]; % relative pose bottom right --> top left
for a = 1:size(tagKeys,1)
    graph.add(BetweenFactorPoint3(tagKeys(a,2),tagKeys(a,3),Point3(r1'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(tagKeys(a,3),tagKeys(a,4),Point3(r2'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(tagKeys(a,4),tagKeys(a,5),Point3(r3'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(tagKeys(a,5),tagKeys(a,2),Point3(r4'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(tagKeys(a,2),tagKeys(a,4),Point3(r5'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(tagKeys(a,3),tagKeys(a,5),Point3(r6'),pointPriorNoise));
end   

% ADD PRIOR
% Prior camera position uh make random 
rot = eul2rotm([rand rand rand]); % get rotation matrix from odom
trans = [rand rand rand]'; % get translation from odom
graph.add(PriorFactorPose3(stateKeys(1), truth.cameras{1}.pose, posePriorNoise));

% Global tag prior
graph.add(PriorFactorPoint3(tagKeys(1,2), Point3(0,0,0), pointPriorNoise));
graph.add(PriorFactorPoint3(tagKeys(1,3), Point3([tagSize 0 0]'), pointPriorNoise));
graph.add(PriorFactorPoint3(tagKeys(1,4), Point3([tagSize tagSize 0]'), pointPriorNoise));
graph.add(PriorFactorPoint3(tagKeys(1,5), Point3([0 tagSize 0]'), pointPriorNoise));

% Neighbors to global priors (got this from homography)
% for i = 1:size(nextToGlobal,1)
%     tagId = nextToGlobal(i,1);
%     idIndex = find(all(bsxfun(@eq,tagKeys(:,1),tagId),2));
%     pointKey = tagKeys(idIndex,2);
%     x = nextToGlobal(i,2); y = nextToGlobal(i,3);
%     graph.add(PriorFactorPoint3(pointKey,Point3(x,y,0),pointPriorNoise));    
% end

% Initial Estimates States
initialEstimate = Values;
for a = 1:size(stateKeys,1)
     rot = eul2rotm([rand rand rand]);
     trans = [rand rand rand]';
     estimatePose = truth.cameras{a}.pose.retract(0.1*randn(6,1));
    initialEstimate.insert(stateKeys(a),estimatePose);
end

% Initial Estimates Landmarks
for a = 1:size(tagKeys,1)
    tagId = tagKeys(a,1); % The tag Id
    nextToGlobalIndex = find(all(bsxfun(@eq, nextToGlobal(:,1),tagId),2)); % the index of tag ID in nextToGLobalMatrix
    bottomLeftX = nextToGlobal(nextToGlobalIndex,2); % x location of bottom left corner of current tag
    bottomLeftY = nextToGlobal(nextToGlobalIndex,3); % y location of bottom left corner of current tag
    
    initialEstimate.insert(uint64(tagKeys(a,2)), Point3([bottomLeftX bottomLeftY 0]'));
    initialEstimate.insert(uint64(tagKeys(a,3)), Point3([bottomLeftX+tagSize bottomLeftY 0]'));
    initialEstimate.insert(uint64(tagKeys(a,4)), Point3([bottomLeftX+tagSize bottomLeftY+tagSize 0]'));
    initialEstimate.insert(uint64(tagKeys(a,5)), Point3([bottomLeftX bottomLeftY+tagSize 0]'));
%     for b = 2:5
%         initialEstimate.insert(uint64(tagKeys(a,b)), Point3([rand rand 0]'));
%     end
end

% Add Identity Between States
for a = 1:size(stateKeys,1)-1
   graph.add(BetweenFactorPose3(stateKeys(a),stateKeys(a+1),Pose3(Rot3(eye(3)),Point3([0 0 0]')),posePriorNoise));
end

toc
%% Optimize
% Fine grain optimization, allowing user to iterate step by step
close all
tic
addpath('../../../gtsam_toolbox')
import gtsam.*

parameters = LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

%optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);
% parameters = DoglegParams;
% parameters.setDeltaInitial(1.0);
% parameters.setVerbosityDL('trydelta');
 optimizer = DoglegOptimizer(graph,initialEstimate);%,parameters);


% for i=1:5
%     optimizer.iterate();
% end

% result = optimizer.values();
result = optimizer.optimize();
%for i=1:5
%    optimizer.iterate();
%end
%result = optimizer.values();
%result.print(sprintf('\nFinal result:\n  '));


%optimizer = DoglegOptimizer(graph, initialEstimate);
%result = optimizer.optimizeSafely;
marginals = Marginals(graph, result);
%cla 
%hold on
%plot3DPoints(result, [], marginals);
%plot3DTrajectory(result, '*', 1, 8, marginals);
%view(3)
%grid on


% Put state results in matrix
stateResults = [];
for n = 1:size(stateKeys,1)
    stateResults = [stateResults ; [result.at(stateKeys(n)).x result.at(stateKeys(n)).y result.at(stateKeys(n)).z]];
end

% Put landmark results in matrix
landMarkResults = [];
landMarkCenters = [];
LandMarksComputed = [];
for n = 1:size(tagKeys,1)
    for i = 2:5
        landMarkResults = [landMarkResults ; [result.at(tagKeys(n,i)).x ...
                                    result.at(tagKeys(n,i)).y result.at(tagKeys(n,i)).z]];
    end
    
    LandMarksComputed = [LandMarksComputed ; ...
                            [tagIds(n,1) result.at(tagKeys(n,2)).x result.at(tagKeys(n,2)).y ...
                                          result.at(tagKeys(n,3)).x result.at(tagKeys(n,3)).y ...
                                          result.at(tagKeys(n,4)).x result.at(tagKeys(n,4)).y ...
                                          result.at(tagKeys(n,5)).x result.at(tagKeys(n,5)).y]];
                                      
    xc = result.at(tagKeys(n,2)).x ;%+ tagSize/2;
    yc = result.at(tagKeys(n,2)).y ;%+ tagSize/2;
    landMarkCenters(n,:) = [xc yc];
end

% Plot corners of landmarks
% landMarkResults(:,3) = 0;
% scatter3(landMarkResults(:,1),landMarkResults(:,2),landMarkResults(:,3))

% Plot states
figure
scatter3(stateResults(:,1),stateResults(:,2),stateResults(:,3))

% Plot center of lanmarks
figure
%scatter(landMarkCenters(:,1),landMarkCenters(:,2),'filled')
scatter(LandMarksComputed(:,2),LandMarksComputed(:,3),'filled')
grid on
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples
%end
toc
% Add 4 prior points for the first tag (10), dont use Pose, dont use odometry
% Add prior camera position from homography
% add factors for measurements, only takes pixel values
% add constraints between corner points like distance?
% ^yes, add betweenFactorPoint3

% !synclient HorizEdgeScroll=0 HorizTwoFingerScroll=0
