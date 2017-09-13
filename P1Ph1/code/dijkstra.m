function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% CMSC 828T Proj 1 Phase 1

if nargin < 4
    astar = false;
end

%% START YOUR CODE HERE %%
source=[((start(1)-map{2}(1))/map{5})+1,((start(2)-map{3}(1))/map{5})+1,((start(3)-map{4}(1))/map{6})+1];
aim=[((goal(1)-map{2}(1))/map{5})+1,((goal(2)-map{3}(1))/map{5})+1,((goal(3)-map{4}(1))/map{6})+1];
grid=map{1};
[M,N,L]=size(grid);

c=1;
for i=1:M
    for j=1:N
        for k=1:L
            if i==source(1) && j==source(2) && k==source(3)
            else
                Nvisited(c,1)=i;
                Nvisited(c,2)=j;
                Nvisited(c,3)=k;
                Nvisited(c,4)=sqrt(((i-source(1))^2)+((j-source(2))^2)+((k-source(3))^2));
                Nvisited(c,5)=sqrt(((i-source(1))^2)+((j-source(2))^2)+((k-source(3))^2))+sqrt(((i-aim(1))^2)+((j-aim(2))^2)+((k-aim(3))^2));
                c=c+1;
            end
        end
    end
end
Nvisited=sortrows(Nvisited,4);
ref=Nvisited;
visited(1,:)=[source(1,1);source(1,2);source(1,3);0;0];
planner(1)=inf;
planner(2:length(Nvisited)+1,1)=1:length(Nvisited);
length(planner);
col(1:length(planner))=inf;
planner=[planner,col'];
planner(1,1)=0;
ncol=1:length(Nvisited);
Nvisited=[Nvisited,ncol'];
j=1;
flag=0;
l=0;
[mr,nr]=size(ref);
planner(1,2)=0;
if astar==0
while isempty(Nvisited)==0
    j=j+1;
    [M,N]=size(Nvisited);
    for i=1:M
        if Nvisited(i,4)<=sqrt(3) && planner(Nvisited(i,6)+1,2)>Nvisited(i,4) && planner(Nvisited(i,6)+1,2)==inf
             planner(Nvisited(i,6)+1,2)=Nvisited(i,4)+planner(visited(j-1,5)+1,2);
             planner(Nvisited(i,6)+1,3)=visited(j-1,5);
                
        elseif Nvisited(i,4)<=sqrt(3) && planner(Nvisited(i,6)+1,2)>Nvisited(i,4)+ planner(visited(j-1,5)+1,2) %visited(j-1,4)
            planner(Nvisited(i,6)+1,2)=Nvisited(i,4)+planner(visited(j-1,5)+1,2);
            planner(Nvisited(i,6)+1,3)=visited(j-1,5);
%         else 
%             [J,K]=min(Nvisited(:,4));
%             visited=[visited;Nvisited]
        end
    end
    [l,I]=min(Nvisited(:,4));
    [l,Ir]=min(ref(:,4));
%     if(Nvisited(I,1:3)==aim)
%         flag=1
%         break
%     end
     %Nvisited=removerows(Nvisited,'ind',I-1)
    visited=[visited; Nvisited(I,1:4),Nvisited(I,6)];%Ir];
    Nvisited(I,:)=[];
    [Q,R]=size(Nvisited);
    for k=1:Q
        Nvisited(k,4)=sqrt(((Nvisited(k,1)-visited(j,1))^2)+((Nvisited(k,2)-visited(j,2))^2)+((Nvisited(k,3)-visited(j,3))^2));
    end
    for K=1:mr
        ref(K,4)=sqrt(((ref(K,1)-visited(j,1))^2)+((ref(K,2)-visited(j,2))^2)+((ref(K,3)-visited(j,3))^2));
        if ref(K,1)==visited(j,1) && ref(K,2)==visited(j,2) && ref(K,3)==visited(j,3)
            ref(K,4)=inf;
        end
    end
%     if(flag==1)
%        break
%    end
end
planner;
index=0;
for p=1:size(ref)
    if([ref(p,1:3)]==[aim])
        index=p;
    end
end
z=20;
if index ==0
    path=[];
    num_expanded=0;
else
    path=[index];
    while(z>0)
        q=planner(index+1,3);
        if q==0
            break
        else
            path=[path;q];
            z=planner(q,3);
            index=planner(q+1,1);
        end
    end
    final_path=[];
    [s1,m2]=size(path);
    for d=1:s1
    final_path=[final_path;ref(path(d),1),ref(path(d),2),ref(path(d),3)];
    end
%     [PATH,NUM_EXPANDED]=[final_path,source];
    path=[final_path;source];
    path=flipud(path);
    [num_expanded,dd]=size(path);
end
    

%% a star %%
else
    
blahh=0;
closeset=[];
while true
    j=j+1;
    [M,N]=size(Nvisited);
    closeset=[];
    for i=1:M
        if Nvisited(i,4)<=sqrt(3)
            closeset=[closeset;Nvisited(i,:)];
        end
    end
%              planner(i+1,2)=Nvisited(i,5);
%              planner(i+1,3)=planner(j,1);
% %         elseif Nvisited(i,4)<=sqrt(3) && planner(i+1,2)>Nvisited(i,4)
% %             planner(i+1,2)=Nvisited(i,4)+visited(j-1,4);
% %             planner(i+1,3)=planner(j,1);
% %         else 
% %             [J,K]=min(Nvisited(:,4));
% %             visited=[visited;Nvisited]
%         end
%     end
    [l,I]=min(closeset(:,5));
    [H,L]=size(closeset);
    visited=[visited; closeset(I,1:5)];
    
    for v=1:H
        if(closeset(v,1:3)==aim)
            flag=1;
            break
        end
    end
    
%      Nvisited=removerows(Nvisited,'ind',I-1)
%     visited=[visited; Nvisited(I-1,1:4),0];
    [Q,R]=size(Nvisited);
    for k=1:Q
        if Nvisited(k,1)==visited(j,1) && Nvisited(k,2)==visited(j,2) && Nvisited(k,3)==visited(j,3)
            Nvisited(k,4)=inf;
        else
            Nvisited(k,4)=sqrt(((Nvisited(k,1)-visited(j,1))^2)+((Nvisited(k,2)-visited(j,2))^2)+((Nvisited(k,3)-visited(j,3))^2));
            Nvisited(k,5)=Nvisited(k,4)+sqrt(((Nvisited(k,1)-aim(1))^2)+((Nvisited(k,2)-aim(2))^2)+((Nvisited(k,3)-aim(3))^2));
        end
    end
%     for K=1:mr
%         ref(K,4)=sqrt(((ref(K,1)-visited(j,1))^2)+((ref(K,2)-visited(j,2))^2)+((ref(K,3)-visited(j,3))^2));
%         if ref(K,1)==visited(j,1) && ref(K,2)==visited(j,2) && ref(K,3)==visited(j,3)
%             ref(K,4)=inf;
%         end
    %end
               
    if(flag==1)
       break
    end
   if blahh==100000
      break 
   end
   blahh=blahh+1;
end
    visited;
    [lengt,rand]=size(visited);
    if lengt==1
        path=[];
        num_expanded=0;
        
    else
%     [PATH,NUM_EXPANDED]=[visited(:,1:3),lengt] ; 
        path=visited(:,1:3);
        num_expanded=lengt;
    end
end
%% END YOUR CODE HERE %%

end