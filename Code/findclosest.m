function [joint,distance,obstacle]=findclosest(q)
opos=load('obstaclepoints.mat');
j_pos=joint_pos(q);
closest_point=0;
closest_distance=99999;
closest_obstacle=99999*[1 1 1]';
for i=4:size(j_pos,2)
    for j=1:size(opos.obs_pts,2)
        if norm(j_pos(:,i)-opos.obs_pts(:,j))<=closest_distance
            closest_distance=norm(j_pos(:,i)-opos.obs_pts(:,j));
            closest_point=i-1;
            closest_obstacle=opos.obs_pts(:,j);
        end
    end
end
joint=closest_point;
distance=closest_distance;
obstacle=closest_obstacle;
end