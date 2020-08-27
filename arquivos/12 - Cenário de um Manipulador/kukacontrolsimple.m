    clear; clc;
    
    disp('Program started');
    vrep=remApi('remoteApi');
    vrep.simxFinish(-1); 
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    [~,j1]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint1',vrep.simx_opmode_oneshot_wait);
    [~,j2]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint2',vrep.simx_opmode_oneshot_wait); 
    [~,j3]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint3',vrep.simx_opmode_oneshot_wait); 
    [~,j4]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint4',vrep.simx_opmode_oneshot_wait);
    [~,j5]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint5',vrep.simx_opmode_oneshot_wait);
    [~,j6]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint6',vrep.simx_opmode_oneshot_wait);
    [~,j7]=vrep.simxGetObjectHandle(clientID,'LBR4p_joint7',vrep.simx_opmode_oneshot_wait);
    
    joint_handles.j(1) = j1;
    joint_handles.j(2) = j2;
    joint_handles.j(3) = j3;
    joint_handles.j(4) = j4;
    joint_handles.j(5) = j5;
    joint_handles.j(6) = j6;
    joint_handles.j(7) = j7;
    
    startingJoints = [0, pi/3, 0, pi/2, pi/3, 0, 0]; 
    disp('Starting robot');

    vrep.simxPauseCommunication(clientID, 1); 
    for i = 1:7
        vrep.simxSetJointTargetPosition(clientID, joint_handles.j(i),...
            startingJoints(i),...
            vrep.simx_opmode_oneshot);
    end
    vrep.simxPauseCommunication(clientID, 0);