clear all
close all
clc
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
joint_conf=importdata('Kulkarni_Abhijeet.txt');
pause(1)
if (clientID>-1)
    disp('connected')
    q=[0 0 0 0 0 0 0];
%     joint_conf=deg2rad([60.6814 76.9216 10.8163 50.6658 -22.5383 -28.1644 -42.7095]);
    [~,q(1)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint1',vrep.simx_opmode_blocking);
    [~,q(2)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint2',vrep.simx_opmode_blocking);
    [~,q(3)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint3',vrep.simx_opmode_blocking);
    [~,q(4)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint4',vrep.simx_opmode_blocking);
    [~,q(5)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint5',vrep.simx_opmode_blocking);
    [~,q(6)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint6',vrep.simx_opmode_blocking);
    [~,q(7)]=vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint7',vrep.simx_opmode_blocking);
    
    %
    for j=1:size(joint_conf,1)
    for i=1:7
        vrep.simxSetJointTargetPosition(clientID,q(i),joint_conf(j,i),vrep.simx_opmode_streaming);
        
    end
    pause(5/1000)
    end
else
    
    disp('tryagain...')
end
vrep.delete(); % call the destructor!