%% MAIN
% Code for drone control with remote Api communication with CoppeliaSim
%
% Arthur Nunes
%
% Need auxiliar functions: axisangle_from_R, double_integrator,
% generateFieldEquations, generateRwr and R_from_euler
% Need remote Api files: remApi.m, remoteApi.dll (Windows),
% remoteApiProto.m
clear, clc, close all

%% User inputs
targets = [-1 0 3.5
           0 0 3.5
           0 0 2
           0.2 -0.5 3];
iterations = 150;

%% API Inicialization
% First play the simulation scene
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
vrep.simxSynchronous(clientID, true);
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

[~,target_handle] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait);
[~,drone_handle] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_base', vrep.simx_opmode_oneshot_wait);

%% Variables
[~, target_pos] = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot_wait); 

[~, drone_pos] = vrep.simxGetObjectPosition(clientID, drone_handle, -1, vrep.simx_opmode_oneshot_wait); 
[~, drone_orient] = vrep.simxGetObjectOrientation(clientID, drone_handle, -1, vrep.simx_opmode_oneshot_wait); 
%[~, drone_quat] = vrep.simxGetObjectQuaternion(clientID, drone_handle, -1, vrep.simx_opmode_oneshot_wait);
[~, drone_linvel, drone_angvel] = vrep.simxGetObjectVelocity(clientID, drone_handle, vrep.simx_opmode_oneshot_wait);

inputs = [5.335, 0, 0, 0];
g = 9.81;
m = inputs(1)/g;
kbeta = 0.15;
kv = 0.15;

prev_Rwr = eye(3);

%% Simulation
for i = 1:size(targets,1)
    new_target_pos = targets(i,:);
    vrep.simxSetObjectPosition(clientID, target_handle, -1, new_target_pos, vrep.simx_opmode_oneshot_wait);
    [~, target_pos] = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot_wait);
    F = generateFieldEquations(1, target_pos');

    for t = 1:iterations
        %[~, target_pos] = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot_wait); 
        [~, drone_pos] = vrep.simxGetObjectPosition(clientID, drone_handle, -1, vrep.simx_opmode_oneshot_wait); 
        [~, drone_orient] = vrep.simxGetObjectOrientation(clientID, drone_handle, -1, vrep.simx_opmode_oneshot_wait);
        %[~, drone_quat] = vrep.simxGetObjectQuaternion(clientID, drone_handle, -1, vrep.simx_opmode_oneshot_wait);
        [~, drone_linvel, drone_angvel] = vrep.simxGetObjectVelocity(clientID, drone_handle, vrep.simx_opmode_oneshot_wait);

        ad = double_integrator(F,drone_linvel',drone_pos',kv, -1);
        Rwb = R_from_euler(drone_orient');
        zb = Rwb*[0 0 1]';
        ar = ad + g*[0 0 1]';
        Rwr = generateRwr(ar,zb);
        Re = Rwb'*Rwr;
        [n, Beta] = axisangle_from_R(Re);

        Rwr_dot = Rwr - prev_Rwr; prev_Rwr = Rwr;
        S_wr=(Rwb')*Rwr_dot*(Re');
        wr = [S_wr(3,2), S_wr(1,3), S_wr(1,2)]';

        tau = m*zb'*ar;
        w = wr +kbeta*sin(Beta)*n;
        inputs = [tau, w'];    
        vrep.simxCallScriptFunction(clientID,'Quadricopter', vrep.sim_scripttype_childscript, 'setSpeed',[], inputs, [], [], vrep.simx_opmode_blocking);
    end
end

%% End the API and the simulation
vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait); 
vrep.simxFinish(clientID); 
vrep.delete(); 
disp('Program ended');