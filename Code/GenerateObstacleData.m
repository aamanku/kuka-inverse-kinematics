function GenerateObstacleData
%----------------------------------------
%given points of obstacle
a=[0.65 0.2 0.92]';
b=[0.65 -0.2 0.92]';
c=[0.65 -0.2 0.42]';
d=[0.65 0.2 0.42]';
e=[-0.5 0.2 0.92]';
f=[-0.5 -0.2 0.92]';
g=[-0.5 -0.2 0.42]';
h=[-0.5 0.2 0.42]';
%------------------------------------------
obs_cor=[a b c d e f g h];
%------------------------------------------
%positions of A and B
p_A=[0.9099 0.3576 0.466]';
p_B=[0.9099 -0.4189 0.466]';
%---------------------------------------------
el=[0.65 0.2 0.92;
    -0.5 -0.2 0.42]; %corners
obs_pts=[];
p=0;
for i=linspace(0,0.65,5)%starting from 0 as ignoring lower dividing in 5 points
    for j=linspace(-0.2,0.2,5)
        for k=linspace(0.42,0.92,5)
            temp_pt=[i j k];
            if (min(abs(temp_pt-el(1,:)))<=0.00001)||(min(abs(temp_pt-el(2,:)))<=0.00001) %only including surface points
            obs_pts=[obs_pts temp_pt'];
            p=p+1;
            end
        end
    end
end
save('obstaclepoints.mat','obs_pts')
end