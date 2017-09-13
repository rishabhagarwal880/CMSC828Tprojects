function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% CMSC 828T Proj 1 Phase 1

% If the map is empty, output empty vector else, compute
if isempty(map)
    C = [];
else
    %% START YOUR CODE HERE %%
grid=map{1};
[M,N,L]=size(grid);
c=1;
for i=1:M
    for j=1:N
        for k=1:L
                mapco(c,1)=i;
                mapco(c,2)=j;
                mapco(c,3)=k;
                mapco(c,4)=grid(i,j,k);
               c=c+1;
        end
    end
end
[O,P]=size(path);
[Q,R]=size(mapco);
while true
    for i=1:O
        for j=1:Q
           if mapco(j,1:3)==path(i,1:3)
               if mapco(j,4)==1
                    C(i)=1;
               else
                   C(i)=0; 
               end
           end
        end
    end

    %% END YOUR CODE HERE %%
end
end
