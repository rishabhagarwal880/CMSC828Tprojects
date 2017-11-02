function velocity_estimation = vel_est(DetAll,TLeftImgs,K)
Ni=length(TLeftImgs);
Int_matrix=K;
%% Calculating Image velocity

%Assuming all the time stamps will have image files
for i=1:Ni-1
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
    points=points.Location;
    initialize(pointTracker, points,current_img);
    
    %calculating image velocity
    points1 = step(pointTracker, current_img);
    points2 = step(pointTracker, next_img);
    img_velocity=(points2-points1)/(TLeftImgs(i+1)-TLeftImgs(i+1));
    
    
    
end