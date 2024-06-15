clear
clc
close all

addpath('..\lie_theory\');

%% Define paramters

% Select the robot to test
robot = 'husky';
% robot = 'holonomic';

% Select the path to test
pathName = 'vertHelix';
% pathName = 'sine';
% pathName = 'horHelix';

%% Load Data

switch pathName
    case 'vertHelix'
        path = 1;
    case 'sine'
        path = 2;
    case 'horHelix'
        path = 3;
    otherwise
        printf('Path not calculated')
        quit(0)
end

switch robot
    case 'husky'
        results = load("husky_ur5e_results_full.mat"); % Load matlab results here for open loop simulation
        motorStates = getHuskyMotorFromVel(results.states.base.x(:,path),results.states.base.omega(:,path));
    case 'holonomic'
        results = load("husky_ur5e_holo_results_full.mat"); % Load matlab results here for open loop simulation
        motorStates = getHolonomicMotorFromVel(results.states.base.x(:,path),results.states.base.y(:,path),results.states.base.omega(:,path));
    otherwise
        printf('Joint values not calculated')
        quit(0)
end

jointStates = results.states.arm.q(:,:,path);
dt = results.dt;
rateObj = rateControl(1/dt);

%% Connect to CoppeliaSim

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19000, true, true, 5000,5);
if clientID <0
    disp("Failed to connect MATLAB to CoppeliaSim");
    vrep.delete;
    return;
else
    fprintf("Connection %d to remote API server is open. \n", clientID)
end

%% Get Joint Handles

jointNames = ["./shoulder_pan_joint"; "./shoulder_lift_joint";"./elbow_joint";"./wrist_1_joint";"./wrist_2_joint";"./wrist_3_joint"];

switch robot
    case 'husky'
        motorNames = ["./front_left_wheel"; "./front_right_wheel";"./rear_right_wheel"; "./rear_left_wheel"];
    case 'holonomic'
        % --------------------- TODO ------------------------ %
        motorNames = ["./link[0]/regularRotation"; "./link[1]/regularRotation"; "./link[2]/regularRotation"; "./link[3]/regularRotation"];
    otherwise
        printf('Robot not modeled')
        quit(0)
end

jointHandles = -ones(size(jointNames));
motorHandles = -ones(size(motorNames));

for i=1:length(jointHandles)
        [err1,jointHandles(i)] = vrep.simxGetObjectHandle(clientID, convertStringsToChars(jointNames(i)), vrep.simx_opmode_oneshot_wait);
end

for i=1:length(motorHandles)
        [err1,motorHandles(i)] = vrep.simxGetObjectHandle(clientID, convertStringsToChars(motorNames(i)), vrep.simx_opmode_oneshot_wait);
end

pause(3);

%% Send Values to Simulation

reset(rateObj);
for i = 1:length(jointStates)
    i

    currentJoint = jointStates(:,i);
    currentMotor = motorStates(:,i);

    setArmJointPositions(vrep,clientID,jointHandles, currentJoint);
    setArmMotorVelocities(vrep,clientID, motorHandles, currentMotor);
	
    waitfor(rateObj);
end

%% Close the Simulator Connection

setArmJointPositions(vrep,clientID,jointHandles, zeros(size(jointHandles)));
setArmMotorVelocities(vrep,clientID, motorHandles, zeros(size(motorHandles)));

pause(5)

vrep.simxFinish(clientID); %close connection
vrep.delete(); %call the destructor!
disp('Program ended');