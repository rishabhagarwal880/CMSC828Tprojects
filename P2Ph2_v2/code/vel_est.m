function velocity_estimation = vel_est(DetAll,TLeftImgs,K,LandMarksComputed,VelGTSAM)
Ni=length(TLeftImgs);
Int_matrix=K;
%% Calculating Image velocity
%cd /StraightLineFrames
%Assuming all the time stamps will have image files
for i=1:Ni-1
    i
    if isempty(DetAll{i})
        continue
    end
    
    %getting current image in grayscale
    current_image_file=sprintf('%d%s',i,'.jpg');
    current_RGB_image=imread(current_image_file);
    current_img=rgb2gray(current_RGB_image);
    
    %getting next image in grayscale
    next_image_file=sprintf('%d%s',i+1,'.jpg');
    next_RGB_image=imread(next_image_file);
    next_img=rgb2gray(next_RGB_image);
    
    %initializing the tracker with current image
    points=detectMinEigenFeatures(current_img);
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
    points=points.selectStrongest(2500).Location;
    initialize(pointTracker, points,current_img);
    
    %calculating image velocity
    [points1, val] = step(pointTracker, current_img);
    [points2, val] = step(pointTracker, next_img);
    %img_velocity=(points2-points1)/(TLeftImgs(i+1)-TLeftImgs(i+1));
    
    %points in camera frame
    points1_tracked = [points1';ones(1,size(points1,1))];
    points1_camera= K\points1_tracked;
    points1_camera= (points1_camera(1:3,val))';
    points2_tracked = [points2';ones(1,size(points2,1))];
    points2_camera= K\points2_tracked;
    points2_camera= (points2_camera(1:3,val))';
   
    dt =(TLeftImgs(i+1)-TLeftImgs(i));
    img_velocity=(points2_camera-points1_camera)/dt;
    
    %flow conversion
    M_conversion= zeros(2*size(points2_camera,1),6);
    camera_vel=zeros(2*size(points2_camera,1),1);
    tag_img(1,:)=[DetAll{i}(1,2), DetAll{i}(1,3)]; 
    tag_img(2,:)=[DetAll{i}(1,4), DetAll{i}(1,5)]; 
    tag_img(3,:)=[DetAll{i}(1,6), DetAll{i}(1,7)]; 
    tag_img(4,:)=[DetAll{i}(1,8), DetAll{i}(1,9)];
    id = find(LandMarksComputed(:,1) == DetAll{i}(1,1));
    tag_world(1,:)=[LandMarksComputed(id,2), LandMarksComputed(id,3)];
    tag_world(2,:)=[LandMarksComputed(id,4), LandMarksComputed(id,5)];
    tag_world(3,:)=[LandMarksComputed(id,6), LandMarksComputed(id,7)];
    tag_world(4,:)=[LandMarksComputed(id,8), LandMarksComputed(id,9)];
    cameraParams = cameraParameters('IntrinsicMatrix', K');
    [R2,T]=extrinsics(tag_img, tag_world, cameraParams);
    R =R2';
    H = [R(:,1) R(:,2) T'];
    count = 1;
    for j = 1:size(points2_camera,1)
	Z= H\((points2_camera(j,:))');
	z = 1/Z(3);
	x = points2_camera(j,1);
	y = points2_camera(j,2);
	M_conversion((count):(count+1),:) = [[-1/z 0 x/z x*y -(1+x^2) y];[0 -1/z y/z 1+y^2 -x*y -x]];
	camera_vel((count):(count+1),:) = [img_velocity(j,1);img_velocity(j,2)];
	count = count + 2;
    end
    
    % Ransac Implementation
    [up_img_flow, up_M_conversion] = ransac(3, img_velocity, .002, M_conversion, camera_vel, false);
    v = up_M_conversion\up_img_flow;
%    v2 = M_conversion\camera_vel;

    R2*v(1:3);
    vel(i,:) = R2*v(1:3);
    omg(i,:) = R2*v(4:end);
 %   vel_nr(i,:) = R2*v2(1:3);

%     
%     if isempty(DetAll{i+1})
%         i=i+1;
%     end
%     
end
x=linspace(1,Ni,Ni);
vel =[0 0 0;vel];
%vel_nr =[0 0 0;vel_nr];

% plot(x,vel(:,1),x,VelGTSAM(:,1),'--');
% plot(x,vel(:,2),x,VelGTSAM(:,2),'--');
% plot(x,vel(:,3),x,VelGTSAM(:,3),'--');

 figure
 subplot(3,1,1)       % add first plot in 2 x 1 grid
 plot(x,vel(:,1),x,VelGTSAM(:,1),'--')
 title('Velocity in X')
 
 subplot(3,1,2)       % add second plot in 2 x 1 grid
 plot(x,vel(:,2),x,VelGTSAM(:,2),':')       % plot using + markers
 title('Velocity in Y')
 
 subplot(3,1,3)       % add second plot in 2 x 1 grid
 plot(x,vel(:,3),x,VelGTSAM(:,3),'--')       % plot using + markers
 title('Velocity in Z')

%%figure
%subplot(3,1,1)       % add first plot in 2 x 1 grid
%plot(x,vel(:,1),x,vel_nr(:,1),'--')
%title('Velocity in X')

%subplot(3,1,2)       % add second plot in 2 x 1 grid
%plot(x,vel(:,2),x,vel_nr(:,2),':')       % plot using + markers
%title('Velocity in Y')

%subplot(3,1,3)       % add second plot in 2 x 1 grid
%plot(x,vel(:,3),x,vel_nr(:,3),'--')       % plot using + markers
%title('Velocity in Z')
vel
omg
velocity_estimation = vel; 
