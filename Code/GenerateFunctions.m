function GenerateFunctions(flg)
syms q1 q2 q3 q4 q5 q6 q7 obx oby obz real
syms T(a_i,d_i,alpha_i,theta_i) 
% flg=true;%%force generate files
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dhp=[0.34   q1  0   -0.5*pi;
     0      q2  0   0.5*pi;
     0.4    q3  0   0.5*pi;
     0      q4  0   -0.5*pi;
     0.4    q5  0   -0.5*pi;
     0      q6  0   0.5*pi;
     0.126  q7  0   0]; %DH parameters for KUKA LWR4+ robot arm
 %------------------------------------
anglelimits=deg2rad([-170 170;-120 120;-170 170;-120 120;-170 170;-120 120;-175 175]); % angle range from https://www.coboticsworld.com/wp-content/uploads/2019/05/KUKA-LWR-Brochure.pdf
 %------------------------------------
Tb_0=[0   0   1   0;
    1   0   0   0;
    0   1   0   0;
    0   0   0  1];  %%Tb_0
%---------------------------------------
q_A=deg2rad([58.2686 75.3224 11.7986 45.9029 -22.1081 -31.2831 -42.3712]');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transformation matrix; analytical jacobian; forward kinematics
if flg ||~(isfile('Trfm.m') && isfile('an_Jacob.m') && isfile('forward_kine.m'))
    A(d_i,theta_i,a_i,alpha_i)=[cos(theta_i) -sin(theta_i)*cos(alpha_i) sin(theta_i)*sin(alpha_i) a_i*cos(theta_i);sin(theta_i) cos(theta_i)*cos(alpha_i) -cos(theta_i)*sin(alpha_i) a_i*sin(theta_i);0 sin(alpha_i) cos(alpha_i) d_i;0 0 0 1];
    Trf=eye(4);%initialize Transformation matrix
    p=zeros(3,size(dhp,1)+1)+q1*0; % initializing positions matrix
    for i=1:size(dhp,1)
        Trn=(simplify(A(dhp(i,1),dhp(i,2),dhp(i,3),dhp(i,4))));
        Trf=simplify(Trf*Trn);% gives transformation matrix T0->i
        p(:,i+1)=Tb_0(1:3,1:3)*simplify(Trf(1:3,4));
    end

    Trf=Tb_0*Trf;%%transforming to get wrt b
    matlabFunction(simplify(Trf),'File','Trfm','Vars',{[q1 q2 q3 q4 q5 q6 q7]'}); %Transformation matrix
    [phi,theta,psi]=rotmat2zyz(Trf(1:3,1:3));Pe=[Trf(1,4);Trf(2,4);Trf(3,4)];
    matlabFunction((jacobian([Pe;phi;theta;psi],[q1 q2 q3 q4 q5 q6 q7]')),'File','an_Jacob','Vars',{[q1 q2 q3 q4 q5 q6 q7]'},'Optimize',false);%analytical jacobian
    matlabFunction([Pe;phi;theta;psi],'File','forward_kine','Vars',{[q1 q2 q3 q4 q5 q6 q7]'});%forward kinematics
    posj=[p(1,:);p(2,:);p(3,:)];%%joint positions
    for i=2:2:6
        posj(:,i)=(posj(:,i-1)+posj(:,i+1))./2;%%assuming joints are in the middle. using this as two joints are on the same origin in the dhparameters
    end
    
    matlabFunction(posj,'File','joint_pos','Vars',{[q1 q2 q3 q4 q5 q6 q7]'});% Gives origin positions as a function of q
    %jacobian of norm of distance between a point and joint's origin is
    %given below
    matlabFunction(jacobian(norm([obx;oby;obz]-posj(:,4)),[q1 q2 q3 q4 q5 q6 q7]'),'File','jointjacibian_3','Vars',{[q1 q2 q3 q4 q5 q6 q7]',[obx;oby;obz]});
    matlabFunction(jacobian(norm([obx;oby;obz]-posj(:,5)),[q1 q2 q3 q4 q5 q6 q7]'),'File','jointjacibian_4','Vars',{[q1 q2 q3 q4 q5 q6 q7]',[obx;oby;obz]});
    matlabFunction(jacobian(norm([obx;oby;obz]-posj(:,6)),[q1 q2 q3 q4 q5 q6 q7]'),'File','jointjacibian_5','Vars',{[q1 q2 q3 q4 q5 q6 q7]',[obx;oby;obz]});
    matlabFunction(jacobian(norm([obx;oby;obz]-posj(:,7)),[q1 q2 q3 q4 q5 q6 q7]'),'File','jointjacibian_6','Vars',{[q1 q2 q3 q4 q5 q6 q7]',[obx;oby;obz]});
    matlabFunction(jacobian(norm([obx;oby;obz]-posj(:,8)),[q1 q2 q3 q4 q5 q6 q7]'),'File','jointjacibian_7','Vars',{[q1 q2 q3 q4 q5 q6 q7]',[obx;oby;obz]});
end
% return
%% secondary objective
if flg ||~(isfile('qdotz.m'))
x=sym('x',[7 1],'real');
w=0;
for i=1:length(x)
    w=w+((x(i)-0.5*(anglelimits(i,1)+anglelimits(i,2)))/anglelimits(i,2)-anglelimits(i,1))^2;
end
w=-0.5*w/length(x);
qdz=jacobian(w,x)';
matlabFunction(qdz,'File','qdotangle','vars',{x}); % null space optimizing q_dot
end
end