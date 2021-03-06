%% You can uncomment this for your development. But you must comment this out before you submit.
% addpath(home/rishabh/repositories/CMSC828Tprojects/HW1/gtsam_toolbox);
%% Function Descriptions
%{
INPUT
       Odom
            A 3 X N matrix where N is the number of odometry measurements made

       ObservedLandMarks
            A list of structures with each structure which has Locations (a list 
            of noisy position x,y observations of each landmark) and Idx (ID of each 
            corresponding landmarks)

	StartingLocation
            (X, Y, Th) of the assumed starting location.

OUTPUT
      LandmarksComputed
            A N x 3 matrix with each row denoting ID, X, Y values where ID is 
            each landmark ID (rows sorted in ascending order by ID) and X and Y 
            are their corresponding X and Y coordinates.

      AllPosesComputed
            A 'M x 3' matrix where each row is X, Y, and Theta where X and Y are 
            the coordinates and the Theta is the orientation.
%}

%% Function Implementation
function [LandmarksComputed, AllPosesComputed] = SLAMusingGTSAM(Odom, ObservedLandMarks, StartingLocation)
import gtsam.*

%     LandmarksComputed=0;
%     AllPosesComputed=0;
    
    graph= NonlinearFactorGraph;
    [p,q]=size(Odom);
    [m,n]=size(ObservedLandMarks);
   
    for i=1:q+1
        x(i)=symbol('x',i);  
    end
    
%     for b=1:m
%             IDx{b}=table([ObservedLandMarks{b}.Idx', ObservedLandMarks{b}.Locations(:,1), ObservedLandMarks{b}.Locations(:,2)]);
%     end
    ID = ObservedLandMarks{1}.Idx;
    for b=2:m
            ID=union(ID,ObservedLandMarks{b}.Idx);
    end
    [xp,yp]=size(ID);
    for a=1:yp
         l(ID(a))=symbol('l',ID(a));
    end
    
    priorMean = Pose2(StartingLocation(1), StartingLocation(2), StartingLocation(3)); % prior at origin
    priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
    graph.add(PriorFactorPose2(x(1), priorMean, priorNoise));
    
   %odometry 
    %odometry = Pose2(2.0, 2.0, 0.0);
    odometryNoise = noiseModel.Diagonal.Sigmas([2*0.3; 2*0.3; 0.017]);
    for j=1:q
	odometry = Pose2(Odom(1, j), Odom(2, j), Odom(3, j));
        graph.add(BetweenFactorPose2(x(j), x(j+1), odometry, odometryNoise));
    end
    
    % Add bearing/range measurement factors
    degrees = pi/180;
    brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1]);
    for i=1:q+1
        [aa,bb]=size(ObservedLandMarks{i}.Idx);
        for b=1:bb
            if i==1
                    graph.add(BearingRangeFactor2D(x(i), l(ObservedLandMarks{i}.Idx(b)),...
                        Rot2(atan2(ObservedLandMarks{i}.Locations(b,2),ObservedLandMarks{i}.Locations(b,1))-StartingLocation(3)), ...
                        sqrt((ObservedLandMarks{i,1}.Locations(b,1))^2 + (ObservedLandMarks{i,1}.Locations(b,2))^2), brNoise));
                else
                    graph.add(BearingRangeFactor2D(x(i), l(ObservedLandMarks{i}.Idx(b)),...
                        Rot2(atan2(ObservedLandMarks{i}.Locations(b,2),ObservedLandMarks{i}.Locations(b,1))-(StartingLocation(3)+Odom(3,i-1))), ...
                        sqrt((ObservedLandMarks{i,1}.Locations(b,1))^2 + (ObservedLandMarks{i,1}.Locations(b,2))^2), brNoise));
            end
        end   
    end
%     graph_cont.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(8), brNoise));
%     graph_cont.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, brNoise));
%     graph_cont.add(BearingRangeFactor2D(i3, j2, Rot2(90*degrees), 2, brNoise));

    graph.print(sprintf('\nFull graph:\n'));
    
    
    % Initialize to noisy points
     initialEstimate = Values;
     initialEstimate.insert(x(1),Pose2(StartingLocation(1), StartingLocation(2), StartingLocation(3)));
     for k=1:q
         initialEstimate.insert(x(k+1),Pose2(StartingLocation(1)+Odom(1,k), StartingLocation(2)+Odom(2,k), StartingLocation(3)+Odom(3,k)));
%             initialEstimate.insert(x(k+1),Pose2(StartingLocation(1), StartingLocation(2), StartingLocation(3)));
     end
     
     
%      for u=1:m
%          for v=1:size(ObservedLandMarks{u}.Idx)
%                 if u==1
%                     initialEstimate.insert(l(ObservedLandMarks{u}.Idx(v)), Point2(ObservedLandMarks{u}.Locations(v,1)-(StartingLocation(1)), ObservedLandMarks{u}.Locations(v,2)-(StartingLocation(1))));
%                 else
%                     initialEstimate.insert(l(ObservedLandMarks{u}.Idx(v)), Point2(ObservedLandMarks{u}.Locations(v,1)-(StartingLocation(1)+Odom(1,u-1)), ObservedLandMarks{u}.Locations(v,2)-(StartingLocation(1)+Odom(2,u-1))));
%                 end
%          end
%      end
        
     
         for v=1:yp
               initialEstimate.insert(l(ID(v)), Point2(3.5 , -7.7));            
         end

     initialEstimate.print(sprintf('\nInitial estimate:\n'));

     
    % Optimize using Levenberg-Marquardt optimization with an ordering from colamd
    optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
    result = optimizer.optimizeSafely();
    result.print(sprintf('\nFinal result:\n'));

    % Plot Covariance Ellipses
    cla;hold on

    marginals = Marginals(graph, result);
    plot2DTrajectory(result, [], marginals);
    plot2DPoints(result, 'b', marginals);
    
    
    for i = 1:(size(Odom, 2) + 1)
       AllPosesComputed(:,i) = [result.at(x(i)).x, result.at(x(i)).y, result.at(x(i)).theta];
    end
    for i = 1:size(ID, 2)
       LandMarksComputed(i,:) = [ID(i), result.at(l(ID(i))).x, result.at(l(ID(i))).y];
    end
 AllPosesComputed
LandmarksComputed=LandMarksComputed
%     plot([result.at(i1).x; result.at(j1).x],[result.at(i1).y; result.at(j1).y], 'c-');
%     plot([result.at(i2).x; result.at(j1).x],[result.at(i2).y; result.at(j1).y], 'c-');
%     plot([result.at(i3).x; result.at(j2).x],[result.at(i3).y; result.at(j2).y], 'c-');
%     axis([-0.6 4.8 -1 1])
%     axis equal
%     view(2)
        
    % Refer to Factor Graphs and GTSAM Introduction 
    % https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
    % and the examples in the library in the GTSAM tookkit. See folder
    % gtsam_toolbox/gtsam_examples
    
    % A rough pipeline guide has been provided. This is not necessarily the 
    % solution, nor may or may not be complete. You will need to figure out 
    % the full and proper pipeline. But this should help you get started.

    % Create graph container and add factors to it from GTSAM library
    % Create keys for variables
    % Add prior
    % Add directly to graph
    % Add odometry
    % Add bearing/range measurement factors 
    % Initialize to noisy points
    % Optimize using any optimizer
    % Get Results

    % Computer error for your validation. We have provided you with ideal
    % pose file that you can refer to. You do not need to return this.
    
    % Return computed landmarks and poses
    
end
