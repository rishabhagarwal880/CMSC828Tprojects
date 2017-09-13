function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% CMSC 828T Proj 1 Phase 1

%% START YOUR CODE HERE %%
grid = map{1};
size_grid = size(grid);
 for i = 1:size_grid(1)
     for j = 1:size_grid(2)
         for k = 1:size_grid(3)
             if grid(i,j,k) == 1
                 vertices = [i j k;i+1 j k;i+1 j+1 k;i j+1 k;i j k+1;i+1 j k+1;i+1 j+1 k+1;i j+1 k+1];
                 faces = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                 patch('Vertices',vertices,'Faces',faces,...
                       'FaceVertexCData',hsv(6),'FaceColor','blue')
                 view(3)
 %                 axis vis3d
                 hold on
             end
         end
     end
 end

plot3(path(:,1), path(:,2), path(:,3), 'LineWidth', 2)

%% END YOUR CODE HERE %%

end