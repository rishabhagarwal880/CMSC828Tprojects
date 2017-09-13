function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  an obstacle.

% CMSC 828T Proj 1 Phase 1

% Output structure: 
    % Map is a cell array containing the following:
    %   --> map{1} contains a 3D logical occupancy grid
    %   --> map{2}, Map{3} and Map{4} store the discretized axes values
    %       corresponding to the X, Y and Z axes respectively
    %   --> map{5} and map{6} store the xy_res and z_res respectively


% Open file, read data, close file. Comments in file marked by #
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);
%% START YOUR CODE HERE %%
i=length(fileDat{1});
data=fileDat(1,2:10);
limits=cell2mat(data);
bound=limits(1,1:6);
limits(1,:)=[];
blocks=limits;
n_x=((bound(4)-bound(1))/xy_res)+1;
n_y=((bound(5)-bound(2))/xy_res)+1;
n_z=((bound(6)-bound(3))/z_res)+1;
X=linspace(bound(1),bound(4),n_x);
Y=linspace(bound(2),bound(5),n_y);
Z=linspace(bound(3),bound(6),n_z);
% C=cell(length(X),length(Y),length(Z));
% for q=1:length(X)
%    for r=1:length(Y)
%       for s=1:length(Z)
%          C{q,r,s}=[X(q),Y(r),Z(s),0]; 
%       end
%        
%    end
% end
grid=zeros(length(X),length(Y),length(Z));
for j=1:i-1
    if rem(((blocks(j,1)-X(1))/xy_res),1)== 0
        x_b=((blocks(j,1)-X(1))/xy_res);
    else
        if (X(((blocks(j,1)-X(1))/xy_res)-rem(((blocks(j,1)-X(1))/xy_res),1)+1)-X(1))-(blocks(j,1)-X(1))< margin
            x_b=((blocks(j,1)-X(1))/xy_res)-rem(((blocks(j,1)-X(1))/xy_res),1)+1;
        else
            x_b=((blocks(j,1)-X(1))/xy_res)-rem(((blocks(j,1)-X(1))/xy_res),1);
        end
    end
    if rem(((blocks(j,2)-Y(1))/xy_res),1)== 0
        y_b=((blocks(j,1)-Y(1))/xy_res);
    else
        if (Y(((blocks(j,2)-Y(1))/xy_res)-rem(((blocks(j,2)-Y(1))/xy_res),1)+1)-Y(1))-(blocks(j,2)-Y(1))< margin
            y_b=((blocks(j,2)-Y(1))/xy_res)-rem(((blocks(j,2)-Y(1))/xy_res),1)+1;
        else
            y_b=((blocks(j,2)-Y(1))/xy_res)-rem(((blocks(j,2)-Y(1))/xy_res),1);
        end
    end
    if rem(((blocks(j,3)-Z(1))/z_res),1)== 0
        z_b=((blocks(j,3)-Z(1))/z_res);
    else
        if (Z(((blocks(j,3)-Z(1))/z_res)-rem(((blocks(j,3)-Z(1))/z_res),1)+1)-Z(1))-(blocks(j,3)-Z(1))< margin
            z_b=((blocks(j,3)-Z(1))/z_res)-rem(((blocks(j,3)-Z(1))/z_res),1)+1;
        else
            z_b=((blocks(j,3)-Z(1))/z_res)-rem(((blocks(j,3)-Z(1))/z_res),1);
        end
    end
    grid(x_b+1:int16(((blocks(j,4)-X(1))/xy_res))+1,y_b+1:int16((blocks(j,5)-Y(1))/xy_res)+1,z_b+1:int16((blocks(j,6)-Z(1))/z_res)+1)=1;
end
%% END YOUR CODE HERE %%
map=cell(1,6);
map{1}=grid;
map{2}=X;
map{3}=Y;
map{4}=Z;
map{5}=xy_res;
map{6}=z_res;
grid;

end
