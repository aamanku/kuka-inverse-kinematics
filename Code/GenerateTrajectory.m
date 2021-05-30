function GenerateTrajectory

q_A=deg2rad([58.2686 75.3224 11.7986 45.9029 -22.1081 -31.2831 -42.3712]');
p_A=[0.9099 0.3576 0.466]';
p_B=[0.9099 -0.4189 0.466]';

Td=Trfm(q_A);
Td(1:3,4)=p_A;

steps = 10000;
[phi,theta,psi]=rotmat2zyz(Td(1:3,1:3));Ped=[Td(1,4);Td(2,4);Td(3,4)];
XD=[Ped;phi;theta;psi];%Desired configuration 
q = zeros(7,steps);
e = zeros(6,steps);
p = zeros(3,steps);
q(:,1) = q_A;%Initial guess
obscen=[0;0.67;0.325];%%box center
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K = 0.1*eye(6);%errorgain
maxdelta=0.0001;
maxjvel=0.005;
obsgain=0.1;
influencedist=0.25;
minobsdist=0.09;
awagain=0.01;
angjlimgain=10;
poserrorthreshold=0.001;
angerrorthreshold=deg2rad(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This part reorients the robot to maximize the distance for the second part 
qtemp=q_A;
for i = 1:steps-1
    Ja = an_Jacob(q(:,i));
    pinvJ=Ja'/(Ja*Ja');
    x=forward_kine(q(:,i));
    p(:,i)=x(1:3);
    e(:,i) = XD-x;
    [cj,dist,obs]=findclosest(q(:,i));
    qdotobs=(dist<=influencedist).*((cj==3).*jointjacibian_3(q(:,i),obs)+(cj==4).*jointjacibian_4(q(:,i),obs)+(cj==5).*jointjacibian_5(q(:,i),obs)+(cj==6).*jointjacibian_6(q(:,i),obs)+(cj==7).*jointjacibian_7(q(:,i),obs))'./(abs(minobsdist-dist));
    qdotawa=(jointjacibian_3(q(:,i),obscen)+jointjacibian_4(q(:,i),obscen)+jointjacibian_5(q(:,i),obscen)+jointjacibian_6(q(:,i),obscen)+jointjacibian_7(q(:,i),obscen))';
    qdot_=angjlimgain*qdotangle(q(:,i))+obsgain*qdotobs+awagain*qdotawa;
    qdot = pinvJ*K*e(:,i)+(eye(7)-pinvJ*Ja)*qdot_;
    if max(abs(qdot))>=maxjvel
        qdd=maxjvel*qdot./max(abs(qdot));
    else
        qdd=qdot;
    end
    if max(abs(qtemp-qdot))<=maxdelta
        break;
    else
        qtemp=qdot;
    end
    q(:,i+1) = q(:,i) + qdd;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This part gives joint configurations from p_A to p_B 
j=i;
q=q(:,1:i);
e=e(:,1:i);
Td(1:3,4)=p_B;
[phi,theta,psi]=rotmat2zyz(Td(1:3,1:3));Ped=[Td(1,4);Td(2,4);Td(3,4)];
XD=[Ped;phi;theta;psi];%Desired configuration 
for i = j:steps-1+j
    Ja = an_Jacob(q(:,i));
    pinvJ=Ja'/(Ja*Ja');
    x=forward_kine(q(:,i));
    p(:,i)=x(1:3);
    e(:,i) = XD-x;
    if (  max(abs(e(1:3,i)))<poserrorthreshold) &&(  max(abs(e(4:6,i)))<angerrorthreshold)
        break;
    end
    [cj,dist,obs]=findclosest(q(:,i));    
    qdotobs=(dist<=influencedist).*((cj==3).*jointjacibian_3(q(:,i),obs)+(cj==4).*jointjacibian_4(q(:,i),obs)+(cj==5).*jointjacibian_5(q(:,i),obs)+(cj==6).*jointjacibian_6(q(:,i),obs)+(cj==7).*jointjacibian_7(q(:,i),obs))'./(abs(minobsdist-dist));
    qdotawa=(jointjacibian_3(q(:,i),obscen)+jointjacibian_4(q(:,i),obscen)+jointjacibian_5(q(:,i),obscen)+jointjacibian_6(q(:,i),obscen)+jointjacibian_7(q(:,i),obscen))';
    qdot_=angjlimgain*qdotangle(q(:,i))+obsgain*qdotobs+awagain*qdotawa;
    qdot = pinvJ*K*e(:,i)+(eye(7)-pinvJ*Ja)*qdot_;
    if max(abs(qdot))>=maxjvel
        qdd=maxjvel*qdot./max(abs(qdot));
    else
        qdd=qdot;
    end
    q(:,i+1) = q(:,i) + qdd;
end
%% Plotting and Saving
q_f=q(:,1:i);
e=e(:,1:i);
figure('Name','Pose Error');
hold on
for h=1:6
    plot(0.005*(1:size(e,2))',e(h,1:end))
end
xlabel('Tsec')
ylabel('Error')
title('EndEffector Pose Error')
file=fopen('Kulkarni_Abhijeet.txt','wt');
lgd=legend('p_x','p_y','p_z','phi','theta','psi');
title(lgd,'Coordinate');
fprintf(file,'%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n',q_f);
fclose(file);

end