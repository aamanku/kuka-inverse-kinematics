function Drawplots(q)
%% Plotting joint angles
qdeg=rad2deg(q);
figure('Name','Joint Angles');
hold on;
for i=1:size(qdeg,2)
    plot(0.005*(1:size(q,1))',qdeg(:,i));
end
xlabel('Tsec')
ylabel('Angle Deg')
title('Joint Angles')
lgd=legend(strcat('1max=' , num2str(max(abs(qdeg(:,1))))),strcat('2max=' , num2str(max(abs(qdeg(:,2))))),strcat('3max=' , num2str(max(abs(qdeg(:,3))))),strcat('4max=' , num2str(max(abs(qdeg(:,4))))),strcat('5max=' , num2str(max(abs(qdeg(:,5))))),strcat('6max=' , num2str(max(abs(qdeg(:,6))))),strcat('7max=' , num2str(max(abs(qdeg(:,7))))));
title(lgd,'Absolute max')
%% Joint velocity
figure('Name','Joint Velocity');
qv=zeros([size(q,1)-1 7]);
hold on
for i=1:size(qdeg,2)
    qv(:,i)=(qdeg(1:end-1,i)-qdeg(2:end,i))./0.005;
    plot(0.005*(1:size(q,1)-1)',qv(:,i));
end
xlabel('Tsec')
ylabel('Deg/sec')
title('Joint Angular velocity')
lgd=legend(strcat('1max=' , num2str(max(abs(qv(:,1))))),strcat('2max=' , num2str(max(abs(qv(:,2))))),strcat('3max=' , num2str(max(abs(qv(:,3))))),strcat('4max=' , num2str(max(abs(qv(:,4))))),strcat('5max=' , num2str(max(abs(qv(:,5))))),strcat('6max=' , num2str(max(abs(qv(:,6))))),strcat('7max=' , num2str(max(abs(qv(:,7))))));
title(lgd,'Absolute max')
%% EndEffector Orientation Errors
figure('Name','EndEffector Orientation Error');
phi=zeros([size(q,1) 1]);
theta=phi;
psi=theta;
Td=Trfm(deg2rad([58.2686 75.3224 11.7986 45.9029 -22.1081 -31.2831 -42.3712]'));
[phid,thetad,psid]=rotmat2zyz(Td(1:3,1:3));
hold on
for i=1:size(q,1)
    T=Trfm(q(i,:)');
    [phi(i),theta(i),psi(i)]=rotmat2zyz(T(1:3,1:3));
end
curr=[phi,theta,psi];
e=[phid,thetad,psid].*ones([size(q,1) 3])-curr;
edeg=rad2deg(e);

for i=1:3
    plot(0.005*(1:size(q,1))',edeg(:,i));
end
xlabel('Tsec')
ylabel('Deg')
title('EndEffector Orientation Error')
lgd=legend(strcat('phi max=' , num2str(max(abs(edeg(:,1))))),strcat('theta max=' , num2str(max(abs(edeg(:,2))))),strcat('psi max=' , num2str(max(abs(edeg(:,3))))));
title(lgd,'Absolute max error')
%% Obstacle and EndEffector trajectory
figure('Name','Obstacle and EndEffector trajectory');
obs=load('obstaclepoints');
plot3(obs.obs_pts(1,:),obs.obs_pts(2,:),obs.obs_pts(3,:),'p');
hold on
for i=1:size(q,1)
    pos=joint_pos(q(i,:)');
    plot3(pos(1,end),pos(2,end),pos(3,end),'.');
end
plot3(0,0,0,'x')
legend('Obstacle','Trajectory','origin')
xlabel('x')
ylabel('y')
zlabel('z')
title('Obstacle and EndEffector trajectory')
end